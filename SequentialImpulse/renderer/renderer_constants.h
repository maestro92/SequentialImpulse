#ifndef RENDERER_CONSTANT_H_
#define RENDERER_CONSTANT_H_

// http://stackoverflow.com/questions/23288934/c-how-to-have-same-enum-members-name-in-different-enum-names-without-getting-e


namespace R_FULL_COLOR
{
	enum E
	{
		u_color = 0
	};
}

namespace R_FULL_TEXTURE
{
	enum E
	{
		u_texture = 0
	};
}

namespace R_MAP
{
	enum E
	{
		u_texture = 0
	};
}

namespace R_FOG_EDGE_BLUR
{
	enum E
	{
		u_texture = 0,
		u_texelSize,
		u_offset
	};
}

namespace R_FOG_FADE_UPDATE
{
	enum E
	{
		u_texture = 0
	};
}

namespace R_FOW
{
	enum E
	{
		u_fogFadeTexture = 0,
		u_noiseTexture,
		u_time,
		u_noiseSpeed,
		u_edgeShape,
		u_noiseTexSamplingLocationScale,
		u_fogMeshVertexOrigin,
		u_fogMeshVertex2UVMat
	};
}

#endif