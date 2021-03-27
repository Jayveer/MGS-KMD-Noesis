#pragma once
#include <inttypes.h>

struct OarHeader {
	uint32_t magic;
	uint32_t maxJoint;
	uint32_t numMotion;
	uint32_t archiveSize; // * 2 as always
};

//num oartable = numJoints
struct ArchiveTable {
	uint16_t numFrames;
	uint16_t archiveOffset[]; // for numJoints + 1;
};