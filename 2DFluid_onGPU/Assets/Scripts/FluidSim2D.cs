using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.UI;
using UnityEngine.Rendering;

public class FluidSim2D : MonoBehaviour
{
    [SerializeField] RawImage image;
    const int WX = 192;
    const int WY = 144;
    const float DT = 1.00f;//デルタタイム
    const float MU = 0.0000000001f;//粘性項μ。大きいほどぬめっとしてる
    const float ALPHA = 1.79f;//SOR法の加速係数
    int METHOD;//CIP法なら1、一次風上差分なら0
    public Shader heatmapShader;///ヒートマップをレンダリングするシェーダー//
    public ComputeShader NSComputeShader;///NS.compute 流体の更新を行うコンピュートシェーダー 
    ComputeBuffer YU;//ここではGPUで扱う流体の物理量は大文字としておく
    ComputeBuffer YUN;
    ComputeBuffer YV;
    ComputeBuffer YVN;
    ComputeBuffer YPN;
    ComputeBuffer GXU;//CIPのみで使用
    ComputeBuffer GYU;//CIPのみで使用
    ComputeBuffer GXV;//CIPのみで使用
    ComputeBuffer GYV;//CIPのみで使用
    ComputeBuffer GXd0;//CIPのみで使用
    ComputeBuffer GYd0;//CIPのみで使用
    ComputeBuffer GXd1;//CIPのみで使用
    ComputeBuffer GYd1;//CIPのみで使用
    ComputeBuffer YUV;
    ComputeBuffer YVU;
    ComputeBuffer DIV;
    ComputeBuffer VOR;
    ComputeBuffer WallP;
    ComputeBuffer WallX;
    ComputeBuffer WallY;
    RenderTexture renderTexture;//表示の時だけ使う

    //カーネル
    int kernelnewgrad_u;
    int kernelnewgrad_v;
    int kernelcip_u;
    int kernelcip_v;
    int kernelupwind_u;
    int kernelupwind_v;
    int kernelpressure0;
    int kernelpressure1;
    int kerneldiv;
    int kernelrhs;
    int kernelveloc;
    int kernelvisc;
    int kernelVorticity;
    int kernelcomputebuffermemcopy_i;
    int kernelcomputebuffermemcopy_f;


    void Start()
    {
        METHOD = 1;//CIP法なら1、一次風上差分なら0
        if (METHOD == 0) Debug.Log("1st Order Upwinding");
        if (METHOD == 1) Debug.Log("CIP");

        //RenderTexture系
        renderTexture = new RenderTexture(WX, WY, 1, RenderTextureFormat.ARGB32);
        renderTexture.enableRandomWrite = true;
        //renderTexture.filterMode = FilterMode.Point;//読み取り時に自動補間しない
        //renderTexture.wrapMode = TextureWrapMode.Clamp;//境界条件みたいな
        renderTexture.Create();
        image.texture = renderTexture;

        //Compute Shaderの設定
        FindKernelInit();
        InitializeComputeBuffer();//ここで流体のvram生成
        SetKernels();//カーネル全部作成＆引数セット。さらにシェーダーにcompute buffer紐づけも

        //壁初期設定
        SetWall();
    }






    void Update()
    {
        //CFD解析本編
        for (int loop = 0; loop < 4; loop++)
        {
            CopyBufferToBuffer_f(YU, YUN);
            CopyBufferToBuffer_f(YV, YVN);

            //ここでy速度定義点でのx速度の近傍4平均、x速度定義点でのy速度の近傍4平均を計算
            NSComputeShader.Dispatch(kernelveloc, 1, WY, 1);

            if (METHOD == 0)
            {
                //一次風上差分 移流 ここではYU,YVの値を使ってYUN,YUVを求めている
                NSComputeShader.Dispatch(kernelupwind_u, 1, WY, 1);
                NSComputeShader.Dispatch(kernelupwind_v, 1, WY, 1);
                CopyBufferToBuffer_f(YU, YUN);//粘性計算skipするなら要らない
                CopyBufferToBuffer_f(YV, YVN);//粘性計算skipするなら要らない
            }
            else
            {
                //cip移流
                CopyBufferToBuffer_f(GXd0, GXU);
                CopyBufferToBuffer_f(GYd0, GYU);
                CopyBufferToBuffer_f(GXd1, GXV);
                CopyBufferToBuffer_f(GYd1, GYV);
                NSComputeShader.Dispatch(kernelcip_u, 1, WY, 1);
                NSComputeShader.Dispatch(kernelcip_v, 1, WY, 1);
                CopyBufferToBuffer_f(YU, YUN);
                CopyBufferToBuffer_f(YV, YVN);
            }

            //粘性。これはskipしてもよい。一次風上差分だとどうせ数値粘性が大きい
            NSComputeShader.Dispatch(kernelvisc, 1, WY, 1);

            //ダイバージェンス計算
            NSComputeShader.Dispatch(kerneldiv, 1, WY, 1);

            //ポアソン方程式ソルバー。SOR法。誤差は判定せず適当に打ち切り
            for (int i = 0; i < 16; i++)
            {
                NSComputeShader.Dispatch(kernelpressure0, WX * WY / 128 / 2, 1, 1);
                NSComputeShader.Dispatch(kernelpressure1, WX * WY / 128 / 2, 1, 1);
            }

            //求まった圧力から速度修正
            NSComputeShader.Dispatch(kernelrhs, 1, WY, 1);

            if (METHOD == 1)
            {
                NSComputeShader.Dispatch(kernelnewgrad_u, 1, WY, 1);
                NSComputeShader.Dispatch(kernelnewgrad_v, 1, WY, 1);
            }

        }//CFDmainループ終わり

        //レンダリングで赤緑表示するための渦度を計算
        NSComputeShader.Dispatch(kernelVorticity, 1, WY, 1);
    }







    void FindKernelInit()
    {
        kernelupwind_u = NSComputeShader.FindKernel("Upwind_u");
        kernelupwind_v = NSComputeShader.FindKernel("Upwind_v");
        kernelpressure0 = NSComputeShader.FindKernel("Pressure0");
        kernelpressure1 = NSComputeShader.FindKernel("Pressure1");
        kerneldiv = NSComputeShader.FindKernel("Div");
        kernelrhs = NSComputeShader.FindKernel("Rhs");
        kernelveloc = NSComputeShader.FindKernel("Veloc");
        kernelvisc = NSComputeShader.FindKernel("Visc");
        kernelVorticity = NSComputeShader.FindKernel("Vorticity");

        kernelnewgrad_u = NSComputeShader.FindKernel("Newgrad_u");
        kernelnewgrad_v = NSComputeShader.FindKernel("Newgrad_v");
        kernelcip_u = NSComputeShader.FindKernel("Cip_u");
        kernelcip_v = NSComputeShader.FindKernel("Cip_v");

        kernelcomputebuffermemcopy_i = NSComputeShader.FindKernel("ComputeBufferMemcopy_i");
        kernelcomputebuffermemcopy_f = NSComputeShader.FindKernel("ComputeBufferMemcopy_f");
    }

    /// コンピュートバッファの初期化
    void InitializeComputeBuffer()
    {
        YU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YUN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YVN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YPN = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        GXU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GYU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GXV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GYV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GXd0 = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GYd0 = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GXd1 = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        GYd1 = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));//CIPのみで使用
        YUV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        YVU = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        DIV = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        VOR = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(float)));
        WallP = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));//computebufferのstrideには4未満が指定できないらしい。OpenCLではstride=1でやっていた
        WallX = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));
        WallY = new ComputeBuffer(WX * WY, Marshal.SizeOf(typeof(uint)));
        FillComputeBuffer();
    }

    void FillComputeBuffer()
    {
        FillBuffer_F(YUN);
        FillBuffer_F(YVN);
        FillBuffer_F(YPN);
        FillBuffer_F(DIV);
        FillBuffer_F(VOR);
        FillBuffer_F(GXU);
        FillBuffer_F(GYU);
        FillBuffer_F(GXV);
        FillBuffer_F(GYV);
        FillBuffer_UI(WallX);
        FillBuffer_UI(WallY);
    }

    void SetKernels()
    {
        NSComputeShader.SetFloat("DT", DT);
        NSComputeShader.SetFloat("MU", MU);

        NSComputeShader.SetBuffer(kernelupwind_u, "YU", YU);
        NSComputeShader.SetBuffer(kernelupwind_u, "YV", YV);
        NSComputeShader.SetBuffer(kernelupwind_u, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelupwind_u, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelupwind_u, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelupwind_u, "YUV", YUV);
        NSComputeShader.SetBuffer(kernelupwind_u, "Wall", WallX);

        NSComputeShader.SetBuffer(kernelupwind_v, "YU", YU);
        NSComputeShader.SetBuffer(kernelupwind_v, "YV", YV);
        NSComputeShader.SetBuffer(kernelupwind_v, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelupwind_v, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelupwind_v, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelupwind_v, "YUV", YUV);
        NSComputeShader.SetBuffer(kernelupwind_v, "Wall", WallY);

        NSComputeShader.SetBuffer(kernelvisc, "YU", YU);
        NSComputeShader.SetBuffer(kernelvisc, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelvisc, "YV", YV);
        NSComputeShader.SetBuffer(kernelvisc, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelvisc, "WallX", WallX);
        NSComputeShader.SetBuffer(kernelvisc, "WallY", WallY);

        NSComputeShader.SetBuffer(kernelpressure0, "DIV", DIV);
        NSComputeShader.SetBuffer(kernelpressure0, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelpressure0, "WallP", WallP);
        NSComputeShader.SetFloat("ALPHA", ALPHA);

        NSComputeShader.SetBuffer(kernelpressure1, "DIV", DIV);
        NSComputeShader.SetBuffer(kernelpressure1, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelpressure1, "WallP", WallP);

        NSComputeShader.SetBuffer(kerneldiv, "DIV", DIV);
        NSComputeShader.SetBuffer(kerneldiv, "YUN", YUN);
        NSComputeShader.SetBuffer(kerneldiv, "YVN", YVN);

        NSComputeShader.SetBuffer(kernelrhs, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelrhs, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelrhs, "YPN", YPN);
        NSComputeShader.SetBuffer(kernelrhs, "WallX", WallX);
        NSComputeShader.SetBuffer(kernelrhs, "WallY", WallY);

        NSComputeShader.SetBuffer(kernelveloc, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelveloc, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelveloc, "YVU", YVU);
        NSComputeShader.SetBuffer(kernelveloc, "YUV", YUV);

        NSComputeShader.SetBuffer(kernelVorticity, "YUN", YUN);
        NSComputeShader.SetBuffer(kernelVorticity, "YVN", YVN);
        NSComputeShader.SetBuffer(kernelVorticity, "VOR", VOR);
        NSComputeShader.SetTexture(kernelVorticity, "Tex", renderTexture);

        //ここまで共通
        //ここからはCIP method関連

        NSComputeShader.SetBuffer(kernelnewgrad_u, "yn", YVN);
        NSComputeShader.SetBuffer(kernelnewgrad_u, "y", YV);
        NSComputeShader.SetBuffer(kernelnewgrad_u, "GX", GXV);
        NSComputeShader.SetBuffer(kernelnewgrad_u, "GY", GYV);
        NSComputeShader.SetBuffer(kernelnewgrad_u, "Wall", WallY);

        NSComputeShader.SetBuffer(kernelnewgrad_v, "yn", YUN);
        NSComputeShader.SetBuffer(kernelnewgrad_v, "y", YU);
        NSComputeShader.SetBuffer(kernelnewgrad_v, "GX", GXU);
        NSComputeShader.SetBuffer(kernelnewgrad_v, "GY", GYU);
        NSComputeShader.SetBuffer(kernelnewgrad_v, "Wall", WallX);

        NSComputeShader.SetBuffer(kernelcip_u, "fn", YUN);
        NSComputeShader.SetBuffer(kernelcip_u, "gxn", GXU);
        NSComputeShader.SetBuffer(kernelcip_u, "gyn", GYU);
        NSComputeShader.SetBuffer(kernelcip_u, "u", YU);
        NSComputeShader.SetBuffer(kernelcip_u, "v", YVU);
        NSComputeShader.SetBuffer(kernelcip_u, "GXd", GXd0);
        NSComputeShader.SetBuffer(kernelcip_u, "GYd", GYd0);
        NSComputeShader.SetBuffer(kernelcip_u, "Wall", WallX);

        NSComputeShader.SetBuffer(kernelcip_v, "fn", YVN);
        NSComputeShader.SetBuffer(kernelcip_v, "gxn", GXV);
        NSComputeShader.SetBuffer(kernelcip_v, "gyn", GYV);
        NSComputeShader.SetBuffer(kernelcip_v, "u", YUV);
        NSComputeShader.SetBuffer(kernelcip_v, "v", YV);
        NSComputeShader.SetBuffer(kernelcip_v, "GXd", GXd1);
        NSComputeShader.SetBuffer(kernelcip_v, "GYd", GYd1);
        NSComputeShader.SetBuffer(kernelcip_v, "Wall", WallY);
    }


    void SetWall()
    {
        //壁仕様
        //壁の種類を記憶するWallX,WallYは<=128で「壁」、>128で「流体」。bool型でもよかったかも
        //WallPは <=64で壁。圧力は参照されない
        //        >64かつ<=128で圧力固定の流体。参照されるかつ自身の圧力更新がない。見た目では流体吸収or湧出部分となる。
        //        >128では参照も書き込みもされる普通の流体部分

        uint[] wallp;//流体関連の圧力壁定義点のvalではなく種類を記憶するほう
        uint[] wallx;//流体関連のｘ速度壁定義点のvalではなく種類を記憶するほう
        uint[] wally;//流体関連のｙ速度壁定義点のvalではなく種類を記憶するほう
        float[] u;//流体関連のｘ速度壁定義点のval
        float[] v;//流体関連のｙ速度壁定義点のval
        float[] p;//流体関連の圧力壁定義点のval

        wallp = new uint[WX * WY];
        wallx = new uint[WX * WY];
        wally = new uint[WX * WY];
        u = new float[WX * WY];
        v = new float[WX * WY];
        p = new float[WX * WY];

        //全部流体で満たす
        for (int i = 0; i < WX * WY; i++)
        {
            wallp[i] = 255;
            wallx[i] = 255;
            wally[i] = 255;
            u[i] = 0.0f;
            v[i] = 0.0f;
            p[i] = 0.0f;
        }

        //左の端の境界設定。左の流体発生部分。壁から流体が湧き出しているイメージ
        for (int i = 0; i < WY; i++)
        {
            wallp[i * WX] = 0;//WallPは <=64で壁。圧力は参照されない

            wally[0 + i * WX] = 0;//1*1壁の上辺にあたる。種類は壁。本来下辺の設定も必要だが周期条件で縦に繋がってるので省略
            v[0 + i * WX] = 0.0f;//速度0固定

            wallx[0 + i * WX] = 0;//1*1壁の左辺にあたる。種類は壁
            u[0 + i * WX] = 0.0f;//速度0固定
            wallx[1 + i * WX] = 0;//1*1壁の右辺にあたる。種類は壁
            u[1 + i * WX] = 0.6f;//速度0.6固定。これが流れを生む
        }

        for (int i = 0; i < WY; i++)//右の端の境界設定。ここでは吸収を担当
        {
            //ここでは速度は固定しない
            wallp[WX - 1 + i * WX] = 100;//> 64かつ <= 128で圧力固定の流体。参照されるかつ自身の圧力更新がない。見た目では流体吸収or湧出部分となる。
        }

        //1*6の板障害物を生成
        for (int i = 0; i < 6; i++)
        {
            wallp[14 + (70 + i) * WX] = 0;//WallPは <=64で壁。圧力は参照されない
            wallx[14 + (70 + i) * WX] = 0;// 速度固定 左辺
            wallx[15 + (70 + i) * WX] = 0;// 速度固定 右辺
            wally[14 + (70 + i) * WX] = 0;//速度固定 上辺
            wally[14 + (71 + i) * WX] = 0;//速度固定 下辺
        }

        //CPU→GPUに転送
        WallX.SetData(wallx);
        WallY.SetData(wally);
        WallP.SetData(wallp);
        YUN.SetData(u);
        YVN.SetData(v);
        YPN.SetData(p);
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
    //全部0(uint32)で埋める関数 at CPU
    void FillBuffer_UI(ComputeBuffer data)
    {
        uint[] a = new uint[data.count];
        for (int i = 0; i < data.count; i++)
        {
            a[i] = 0;
        }
        data.SetData(a);
    }


    //vram同士のコピーuint限定
    //offset次第では書き込みオーバーフローも発生するため注意
    void CopyBufferToBuffer_ui(ComputeBuffer datadst, ComputeBuffer datasrc, int size = 0, int dstoffset = 0, int srcoffset = 0)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        if (size == 0)
        {
            size = datadst.count;
        }
        NSComputeShader.SetInt("SIZE", size);
        NSComputeShader.SetInt("OFFSETDST", dstoffset);
        NSComputeShader.SetInt("OFFSETSRC", srcoffset);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_i, "DATADSTI", datadst);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_i, "DATASRCI", datasrc);
        NSComputeShader.Dispatch(kernelcomputebuffermemcopy_i, (size + 63) / 64, 1, 1);
    }

    //vram同士のコピーfloat限定
    //offset次第では書き込みオーバーフローも発生するため注意
    void CopyBufferToBuffer_f(ComputeBuffer datadst, ComputeBuffer datasrc, int size = 0, int dstoffset = 0, int srcoffset = 0)//この場合のsizeはbyteではなく配列数、offsetもそう
    {
        if (size == 0)
        {
            size = datadst.count;
        }
        NSComputeShader.SetInt("SIZE", size);
        NSComputeShader.SetInt("OFFSETDST", dstoffset);
        NSComputeShader.SetInt("OFFSETSRC", srcoffset);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_f, "DATADSTF", datadst);
        NSComputeShader.SetBuffer(kernelcomputebuffermemcopy_f, "DATASRCF", datasrc);
        NSComputeShader.Dispatch(kernelcomputebuffermemcopy_f, (size + 63) / 64, 1, 1);
    }


    void OnDisable()
    {
        // コンピュートバッファは明示的に破棄しないと怒られます
        YU.Release();
        YUN.Release();
        YV.Release();
        YVN.Release();
        GXU.Release();
        GYU.Release();
        GXV.Release();
        GYV.Release();
        GXd0.Release();
        GYd0.Release();
        GXd1.Release();
        GYd1.Release();
        YPN.Release();
        YUV.Release();
        YVU.Release();
        DIV.Release();
        VOR.Release();
        WallP.Release();
        WallX.Release();
        WallY.Release();
    }

}