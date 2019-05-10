using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;

public class First_order_upwind_scheme : MonoBehaviour
{
    const int WX = 192;
    const int WY = 144;
    const float DT = 1.00f;//デルタタイム
    const float MU = 0.0000001f;//粘性項μ。大きいほどぬめっとしてる
    const float arufa = MU * DT;//粘性項で使う値
    const float ar1fa = 1.0f / (1.0f + 4.0f * arufa);//粘性項で使う値
    const float ALPHA = 1.79f;//圧力計算の加速係数


    public Shader heatmapShader;///ヒートマップをレンダリングするシェーダー//
    Material heatmapMaterial;///heatmapのマテリアル heatmapShaderと紐づけされる
    public ComputeShader NSComputeShader;///NS.compute 流体の更新を行うコンピュートシェーダー 
    CommandBuffer commandb;
    ComputeBuffer YU;
    ComputeBuffer YUN;
    ComputeBuffer YV;
    ComputeBuffer YVN;
    ComputeBuffer YPN;
    ComputeBuffer GXd;
    ComputeBuffer GYd;
    ComputeBuffer YUV;
    ComputeBuffer YVU;
    ComputeBuffer DIV;
    ComputeBuffer VOR;
    ComputeBuffer kabeP;
    ComputeBuffer kabeX;
    ComputeBuffer kabeY;
    int kernelnewgrad;
    int kernelnewgrad2;
    int kernelupwind0;
    int kernelupwind1;
    int kernelpressure0;
    int kernelpressure1;
    int kerneldiv;
    int kernelrhs;
    int kernelveloc;
    int kernelnensei0;
    int kernelnensei1;
    int kernelVorticity;
    int kernelcomputebuffermemcopy_i;
    int kernelcomputebuffermemcopy_f;
    int kernelfillmem_f;
    int kernelfillmem_ui;


    uint[] kbp;//流体関連の圧力壁定義点のvalではなく種類を記憶するほう
    uint[] kbx;//流体関連のｘ速度壁定義点のvalではなく種類を記憶するほう
    uint[] kby;//流体関連のｙ速度壁定義点のvalではなく種類を記憶するほう
    float[] kkx;//流体関連のｘ速度壁定義点のval
    float[] kky;//流体関連のｙ速度壁定義点のval
    float[] kkp;//流体関連の圧力壁定義点のval


    void Start()
    {
        kbp = new uint[WX * WY];
        kbx = new uint[WX * WY];
        kby = new uint[WX * WY];
        kkx = new float[WX * WY];
        kky = new float[WX * WY];
        kkp = new float[WX * WY];
        //壁仕様
        //x速度、y速度を記憶するkabeX,kabeYは<=128で「壁」、>128で「流体」
        //圧力を格納するkabePは <=64で壁なので内部の圧力は参照されない、>64かつ<=128で参照されるかつ自身の更新がない つまり圧力固定の吸収湧出壁となる。>128では参照も書き込みもされる普通の流体部分

        //シェーダー系設定
        heatmapMaterial = new Material(heatmapShader);

        //Compute Shaderの設定
        FindKernelInit();
        InitializeComputeBuffer();//ここで流体のvram生成
        SetKernels();//カーネル全部作成＆引数セット。さらにシェーダーにcompute buffer紐づけも

        //壁初期設定
        SetWall();

        //コマンドバッファ系
        Camera cam = GameObject.Find("Main Camera").GetComponent<Camera>();//コンポーネント
        commandb = new CommandBuffer();
        commandb.name = "heatmap instanse";
        commandb.DrawProcedural(cam.cameraToWorldMatrix, heatmapMaterial, 0, MeshTopology.Points, WX * WY, 1);
        cam.AddCommandBuffer(CameraEvent.AfterForwardOpaque, commandb);

    }









    void Update()
    {
        //CFD解析本編
        for (int loop = 0; loop < 4; loop++)
        {
            CopyBufferToBuffer_f(YU, YUN);
            CopyBufferToBuffer_f(YV, YVN);

            //veloc
            NSComputeShader.Dispatch(kernelveloc, 1, WY, 1);
            //一次風上差分 移流 ここではYU,YVの値を使ってYUN,YUVを求めている
            NSComputeShader.Dispatch(kernelupwind0, 1, WY, 1);
            NSComputeShader.Dispatch(kernelupwind1, 1, WY, 1);

            /*
            //粘性//ここだけなぜか陰的実装っぽいかんじ
            CopyBufferToBuffer_f(YU, YUN);
            CopyBufferToBuffer_f(YV, YVN);//粘性
            for (int i = 0; i < 16; i++)
            {
                NSComputeShader.Dispatch(kernelnensei0, 1, WY, 1);
                NSComputeShader.Dispatch(kernelnensei1, 1, WY, 1);
            }
            */

            //DIV
            NSComputeShader.Dispatch(kerneldiv, 1, WY, 1);

            //pressure
            for (int i = 0; i < 16; i++)
            {
                NSComputeShader.Dispatch(kernelpressure0, WX * WY / 128 / 2, 1, 1);
                NSComputeShader.Dispatch(kernelpressure1, WX * WY / 128 / 2, 1, 1);
            }

            //rhs
            NSComputeShader.Dispatch(kernelrhs, 1, WY, 1);

        }//CFDmainループ終わり

        NSComputeShader.Dispatch(kernelVorticity, 1, WY, 1);

    }


    void OnRenderObject()
    {
        //commandb_ye.DrawProceduralを使ってるのでいらなくなった
    }

    void FindKernelInit()//1
    {
        kernelupwind0 = NSComputeShader.FindKernel("upwind0");
        kernelupwind1 = NSComputeShader.FindKernel("upwind1");
        kernelpressure0 = NSComputeShader.FindKernel("pressure0");
        kernelpressure1 = NSComputeShader.FindKernel("pressure1");
        kerneldiv = NSComputeShader.FindKernel("div");
        kernelrhs = NSComputeShader.FindKernel("rhs");
        kernelveloc = NSComputeShader.FindKernel("veloc");
        kernelnensei0 = NSComputeShader.FindKernel("nensei0");
        kernelnensei1 = NSComputeShader.FindKernel("nensei1");
        kernelVorticity = NSComputeShader.FindKernel("Vorticity");
        kernelcomputebuffermemcopy_i = NSComputeShader.FindKernel("computebuffermemcopy_i");
        kernelcomputebuffermemcopy_f = NSComputeShader.FindKernel("computebuffermemcopy_f");
        kernelfillmem_f = NSComputeShader.FindKernel("fillmem_f");
        kernelfillmem_ui = NSComputeShader.FindKernel("fillmem_ui");
    }

    /// コンピュートバッファの初期化
    void InitializeComputeBuffer()
    {
        YU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YUN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YVN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YPN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        GXd = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        GYd = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YUV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YVU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        DIV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        VOR = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        kabeP = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));//computebufferのstrideには4未満が指定できないらしい。OpenCLではstride=1でやっていた
        kabeX = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));
        kabeY = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));
        FillComputeBuffer();
    }

    void FillComputeBuffer()
    {
        FillMem_f(YU, YU.count, 0, 0.0f);
        FillMem_f(YUN, YUN.count, 0, 0.0f);
        FillMem_f(YV, YV.count, 0, 0.0f);
        FillMem_f(YVN, YVN.count, 0, 0.0f);
        FillMem_f(YPN, YPN.count, 0, 0.0f);
        FillMem_f(YUV, YUV.count, 0, 0.0f);
        FillMem_f(YVU, YVU.count, 0, 0.0f);
        FillMem_f(DIV, DIV.count, 0, 0.0f);
        FillMem_f(VOR, VOR.count, 0, 0.0f);
        FillMem_ui(kabeX, kabeX.count, 0, 0);
        FillMem_ui(kabeY, kabeY.count, 0, 0);
    }


    void SetKernels()
    {
        heatmapMaterial.SetBuffer("VOR", VOR);//ここで描画側のshaderとcompute bufferのVORを紐づけ
        NSComputeShader.SetFloat("DT", DT);

        NSComputeShader.SetBuffer(kernelupwind0, "YU", YU);
        NSComputeShader.SetBuffer(kernelupwind0, "YV", YV);
        NSComputeShader.SetBuffer(kernelupwind0, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelupwind0, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelupwind0, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelupwind0, "YUV", YUV);
        NSComputeShader.SetBuffer(kernelupwind0, "kabe", kabeX);

        NSComputeShader.SetBuffer(kernelupwind1, "YU", YU);
        NSComputeShader.SetBuffer(kernelupwind1, "YV", YV);
        NSComputeShader.SetBuffer(kernelupwind1, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelupwind1, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelupwind1, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelupwind1, "YUV", YUV);
        NSComputeShader.SetBuffer(kernelupwind1, "kabe", kabeY);

        NSComputeShader.SetBuffer(kernelnensei0, "YU", YU);
        NSComputeShader.SetBuffer(kernelnensei0, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelnensei0, "YV", YV);
        NSComputeShader.SetBuffer(kernelnensei0, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelnensei0, "GXd", GXd);
        NSComputeShader.SetBuffer(kernelnensei0, "GYd", GYd);
        NSComputeShader.SetBuffer(kernelnensei0, "kabeX", kabeX);
        NSComputeShader.SetBuffer(kernelnensei0, "kabeY", kabeY);
        NSComputeShader.SetFloat("arufa", arufa);
        NSComputeShader.SetFloat("ar1fa", ar1fa);

        NSComputeShader.SetBuffer(kernelnensei1, "YU", YU);
        NSComputeShader.SetBuffer(kernelnensei1, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelnensei1, "YV", YV);
        NSComputeShader.SetBuffer(kernelnensei1, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelnensei1, "GXd", GXd);
        NSComputeShader.SetBuffer(kernelnensei1, "GYd", GYd);
        NSComputeShader.SetBuffer(kernelnensei1, "kabeX", kabeX);
        NSComputeShader.SetBuffer(kernelnensei1, "kabeY", kabeY);

        NSComputeShader.SetBuffer(kernelpressure0, "DIV", DIV);
        NSComputeShader.SetBuffer(kernelpressure0, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelpressure0, "kabeP", kabeP);
        NSComputeShader.SetFloat("ALPHA", ALPHA);

        NSComputeShader.SetBuffer(kernelpressure1, "DIV", DIV);
        NSComputeShader.SetBuffer(kernelpressure1, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelpressure1, "kabeP", kabeP);

        NSComputeShader.SetBuffer(kerneldiv, "DIV", DIV);
        NSComputeShader.SetBuffer(kerneldiv, "YUN", YUN);
        NSComputeShader.SetBuffer(kerneldiv, "YVN", YVN);

        NSComputeShader.SetBuffer(kernelrhs, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelrhs, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelrhs, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelrhs, "kabeX", kabeX);
        NSComputeShader.SetBuffer(kernelrhs, "kabeY", kabeY);

        NSComputeShader.SetBuffer(kernelveloc, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelveloc, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelveloc, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelveloc, "YUV", YUV);

        NSComputeShader.SetBuffer(kernelVorticity, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelVorticity, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelVorticity, "VOR", VOR);
    }


    void SetWall()
    {
        for (int i = 0; i < WX * WY; i++)
        {
            kbp[i] = 255;
            kbx[i] = 255;
            kby[i] = 255;
            kkx[i] = 0.0f;
            kky[i] = 0.0f;
            kkp[i] = 0.0f;
        }

        for (int i = 0; i < WY; i++)//左の端の境界設定。左から流れてくるやつの設定
        {
            kbp[i * WX] = 0;//圧力定義点の圧力固定、かつPoisson方程式での参照なし。つまり壁

            kby[0 + i * WX] = 0;//速度固定の辺(上)本来下も固定する必要があるが、周回条件ありなので今回は省略
            kky[0 + i * WX] = 0.0f;//速度　辺(上)

            kbx[0 + i * WX] = 0;//速度固定の辺(左)
            kkx[0 + i * WX] = 0.0f;//速度　辺(左)
            kbx[1 + i * WX] = 0;//速度固定の辺(右)
            kkx[1 + i * WX] = 0.6f;//速度　辺(右)
        }

        for (int i = 0; i < WY; i++)//右の端の境界設定
        {
            //ここでは速度は固定しない
            kbp[WX - 1 + i * WX] = 100;//吸収(湧出)壁。これで圧力一定の吸収壁になる
        }

        //板障害物を生成
        for (int i = 0; i < 6; i++)
        {
            kbp[14 + (70 + i) * WX] = 0;//圧力定義点の圧力固定、かつPoisson方程式での参照なし。つまり壁
            kbx[14 + (70 + i) * WX] = 0;// 速度固定の辺(左)
            kbx[15 + (70 + i) * WX] = 0;// 速度固定の辺(右)
            kby[14 + (70 + i) * WX] = 0;//速度固定の辺(上)
            kby[14 + (71 + i) * WX] = 0;//速度固定の辺(下)
        }

        kabeX.SetData(kbx);
        kabeY.SetData(kby);
        kabeP.SetData(kbp);
        YUN.SetData(kkx);
        YVN.SetData(kky);
        YPN.SetData(kkp);
    }



    //Host Data
    void FillHD_I(uint[] data)
    {
        for (int i = 0; i < WX * WY; i++)
        {
            data[i] = 0;
        }
    }
    void FillHD_F(float[] data)
    {
        for (int i = 0; i < WX * WY; i++)
        {
            data[i] = 0.0f;
        }
    }



    //全部0.0fで埋める関数 at CPU
    void FillBuffer_F(ComputeBuffer data)
    {
        float[] a = new float[data.count];
        for (int i = 0; i < data.count; i++)
        {
            a[i] = 0.0f;
        }
        data.SetData(a);
    }
    //全部0(int32)で埋める関数 at CPU
    void FillBuffer_I(ComputeBuffer data)
    {
        uint[] a = new uint[data.count];
        for (int i = 0; i < data.count; i++)
        {
            a[i] = 0;
        }
        data.SetData(a);
    }


    //vram同士のコピーuint限定
    //offset次第では書き込みオーバーフローも発生する
    void CopyBufferToBuffer_i(ComputeBuffer datadst, ComputeBuffer datasrc, int size = 0, int dstoffset = 0, int srcoffset = 0)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        if (size == 0)
        {
            size = datadst.count;
        }
        NSComputeShader.SetInt("OFSET0", size);
        NSComputeShader.SetInt("OFSETDST", dstoffset);
        NSComputeShader.SetInt("OFSETSRC", srcoffset);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_i, "DATADSTI", datadst);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_i, "DATASRCI", datasrc);
        NSComputeShader.Dispatch(kernelcomputebuffermemcopy_i, (size + 63) / 64, 1, 1);
    }

    //vram同士のコピーfloat限定
    //offset次第では書き込みオーバーフローも発生する
    void CopyBufferToBuffer_f(ComputeBuffer datadst, ComputeBuffer datasrc, int size = 0, int dstoffset = 0, int srcoffset = 0)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        if (size == 0)
        {
            size = datadst.count;
        }
        NSComputeShader.SetInt("OFSET0", size);
        NSComputeShader.SetInt("OFSETDST", dstoffset);
        NSComputeShader.SetInt("OFSETSRC", srcoffset);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_f, "DATADST", datadst);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_f, "DATASRC", datasrc);
        NSComputeShader.Dispatch(kernelcomputebuffermemcopy_f, (size + 63) / 64, 1, 1);
    }

    //fill float版
    //offset次第では書き込みオーバーフローも発生する
    void FillMem_f(ComputeBuffer datadst, int size, int dstoffset, float fcolor)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        NSComputeShader.SetInt("OFSET0", size);
        NSComputeShader.SetInt("OFSETDST", dstoffset);
        NSComputeShader.SetFloat("FCOLOR", fcolor);
        NSComputeShader.SetBuffer(kernelfillmem_f, "DATADST", datadst);
        NSComputeShader.Dispatch(kernelfillmem_f, (size + 63) / 64, 1, 1);
    }
    //fill uint版
    //offset次第では書き込みオーバーフローも発生する
    void FillMem_ui(ComputeBuffer datadst, int size, int dstoffset, int uicolor)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        NSComputeShader.SetInt("OFSET0", size);
        NSComputeShader.SetInt("OFSETDST", dstoffset);
        NSComputeShader.SetInt("UICOLOR", uicolor);
        NSComputeShader.SetBuffer(kernelfillmem_ui, "DATADSTUI", datadst);
        NSComputeShader.Dispatch(kernelfillmem_ui, (size + 63) / 64, 1, 1);
    }









    void OnDisable()
    {
        // コンピュートバッファは明示的に破棄しないと怒られます
        YU.Release();
        YUN.Release();
        YV.Release();
        YVN.Release();
        GXd.Release();
        GYd.Release();
        YPN.Release();
        YUV.Release();
        YVU.Release();
        DIV.Release();
        VOR.Release();
        kabeP.Release();
        kabeX.Release();
        kabeY.Release();
    }

}