Shader "Heatmap" {
	Properties{
		_Intensity("色の強さ", float) = 1.0
	}

	SubShader{
		Pass{
		CGPROGRAM

		
#pragma target 5.0// シェーダーモデルは5.0を指定
#pragma vertex vert
#pragma geometry geom
#pragma fragment frag

#define WX (uint)(192)
#define WY (uint)(144)

#include "UnityCG.cginc"

	uniform fixed _Intensity;

	//渦度情報
	StructuredBuffer<float> VOR;


	// 頂点シェーダからの出力
	struct VSOut {
		float4 pos : SV_POSITION;
		float4 col : COLOR;
	};

	// 頂点シェーダ
	VSOut vert(uint id : SV_VertexID)
	{
		// idを元に、渦度情報を取得
		VSOut output;
		//位置設定
		output.pos.x = 10.0f / WY * ((float)(id%WX) - (float)(WX / 2));
		output.pos.y = 10.0f / WY * ((float)(WY / 2) - (float)(id / WX + 1));
		output.pos.z = 0;
		output.pos.w = 0;

		//vorから色設定
		float vor = VOR[id] * 9.0;
		output.col.r = clamp(-vor,0,1);
		output.col.g = clamp(vor,0,1);
		output.col.b = clamp(0.4*(abs(vor)-0.3),0,1);
		output.col.a = 1;
		return output;
	}

	
	// ジオメトリシェーダ
	[maxvertexcount(4)]
	void geom(point VSOut input[1], inout TriangleStream<VSOut> outStream)
	{
		VSOut output;
		// 全ての頂点で共通の値を計算しておく
		float4 pos = input[0].pos;
		float4 col = input[0].col;

		// 四角形になるように頂点を生産
		for (int x = 0; x < 2; x++)
		{
			for (int y = 0; y < 2; y++)
			{
				// テクスチャ座標
				float4 tex = float4(x, y, 0, 0);

				// 頂点位置を計算
				float4 pos4 = pos + 10.0*tex / WY;
				float3 worldPos = float3(pos4.x, pos4.y, -0.1);
				output.pos = mul(UNITY_MATRIX_VP, float4(worldPos, 1.0f));

				// 色
				output.col = col;

				// ストリームに頂点を追加
				outStream.Append(output);
			}
		}

		// トライアングルストリップを終了
		outStream.RestartStrip();
	}

	


	// ピクセルシェーダー
	float4 frag(VSOut i) : COLOR
	{
		return i.col*_Intensity;
	}

		ENDCG
	}



	}
}