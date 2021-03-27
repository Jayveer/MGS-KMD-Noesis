#pragma once
#include <inttypes.h>

struct KmdVert {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
};

struct KmdNVert {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
};

struct KmdUV {
	uint8_t tu;
	uint8_t tv;
};

struct KmdFace {
	uint8_t fa;
	uint8_t fb;
	uint8_t fc;
	uint8_t fd;
};

struct Vec3Long {
	int32_t x;
	int32_t y;
	int32_t z;
};

struct KmdHeader {
	uint32_t numBones;
	uint32_t numMesh;
	Vec3Long max;
	Vec3Long min;
};

struct KmdMesh {
	uint32_t flag;
	uint32_t numFace;
	Vec3Long max;
	Vec3Long min;
	Vec3Long pos;
	int32_t parent;
	int32_t unknownA;
	uint32_t numVertex;
	uint32_t vertexIndexOffset;
	uint32_t faceIndexOffset;
	uint32_t numNormals;
	uint32_t normalIndexOffset;
	uint32_t normalFaceOffset;
	uint32_t uvOffset;
	uint32_t materialOffset;
	uint32_t pad;
};