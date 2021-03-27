#pragma once
#include <vector>
#include "mgs/common/util.h"
#include "mgs/motion/oar/oar.h"
#include "noesis/plugin/pluginshare.h"

const double g_mgs1_PI = acos(-1);
const float  g_mgs1_GAME_FRAMERATE = 30.0f;

struct MoveAnimation {
    int keyframe;
    float x;
    float y;
    float z;
};

struct RotAnimation {
    int keyframe;
    float x;
    float y;
    float z;
    float w;
};

inline
float shiftRadix(const float& f, const int& exponent) {
    return f * pow(2, exponent);
}

inline
bool checkMagic(BYTE* motionFile) {
    OarHeader* header = (OarHeader*)motionFile;
    return header->magic == 0x6152414F;
}

inline
BYTE* openMotion(noeRAPI_t* rapi) {
    int len;
    char out[MAX_NOESIS_PATH];
    BYTE* marFile = rapi->Noesis_LoadPairedFile("load oar", ".oar", len, out);
    if (!marFile) return NULL;
    return checkMagic(marFile) ? marFile : NULL;
}

inline
RichQuat eulerToQuat(RichVec3 euler) {
    double cy = cos(euler[2] * 0.5);
    double sy = sin(euler[2] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[0] * 0.5);
    double sr = sin(euler[0] * 0.5);

    RichQuat qt;
    qt[0] = sr * cp * cy - cr * sp * sy;
    qt[1] = cr * sp * cy + sr * cp * sy;
    qt[2] = cr * cp * sy - sr * sp * cy;
    qt[3] = cr * cp * cy + sr * sp * sy;

    return qt;
}

inline
int32_t negateBits(int32_t val, int numBits) {
    int hi = numBits - 1;
    int32_t mx = pow(2, hi);
    return val >> hi ? val |= -mx : val;
}

inline
std::vector<RotAnimation> readRotBitstream(uint16_t* rotBitStream, const int& size, const uint32_t& numFrames) {
    int keyFrame = 0;
    std::vector<RotAnimation> ra;
    RichBitStream bs = RichBitStream(rotBitStream, size * 2);

    uint8_t xL = bs.ReadBits(4);
    uint8_t yL = bs.ReadBits(4);
    uint8_t zL = bs.ReadBits(4);

    while (keyFrame < numFrames) {
        keyFrame += bs.ReadBits(4);
        uint8_t unknown = bs.ReadBits(4);

        int32_t x = bs.ReadBits(xL); x = negateBits(x, xL);
        int32_t y = bs.ReadBits(yL); y = negateBits(y, yL);
        int32_t z = bs.ReadBits(zL); z = negateBits(z, zL);

        float fx = x / 2047.0f * g_mgs1_PI;
        float fy = y / 2047.0f * g_mgs1_PI;
        float fz = z / 2047.0f * g_mgs1_PI;

        RichVec3 v3 = { fx, fy, fz };
        RichQuat qt = eulerToQuat(v3);

        ra.push_back({ keyFrame, qt[0], qt[1], qt[2], qt[3] });
    }

    return ra;
}

inline
std::vector<MoveAnimation> readMoveBitstream(uint16_t* moveBitStream, const int& size, const uint32_t& numFrames) {
    int keyFrame = 0;
    std::vector<MoveAnimation> ma;
    RichBitStream bs = RichBitStream(moveBitStream, size * 2);

    int32_t y = bs.ReadBits(16);
    if (y & 0x800) { y |= -0x1000; }
    float originY = y;
    ma.push_back({ keyFrame, 0 , originY, 0 });

    uint8_t xL = bs.ReadBits(4);
    uint8_t yL = bs.ReadBits(4);
    uint8_t zL = bs.ReadBits(4);
    uint8_t unk = bs.ReadBits(4);

    if (!xL && !yL && !zL) {
        return ma;
    }

    while (keyFrame < numFrames) {
        keyFrame++; //not sure what determines keyframe, still need to look into it
        int32_t x = bs.ReadBits(xL); x = negateBits(x, xL);
        int32_t y = bs.ReadBits(yL); y = negateBits(y, yL);
        int32_t z = bs.ReadBits(zL); z = negateBits(z, zL);

        float fy = originY + y;
        float fx = x / 2047.0f * g_mgs1_PI;
        float fz = z / 2047.0f * g_mgs1_PI;
        
        ma.push_back({ keyFrame, fx, fy, fz });
    }

    return ma;
}

inline
noeKeyFrameData_t createTransKFData(const MoveAnimation& trans, std::vector<float>& aniData) {
    noeKeyFrameData_t data = {};
    data.dataIndex = aniData.size();
    data.time = trans.keyframe / g_mgs1_GAME_FRAMERATE;

    aniData.push_back(trans.x);
    aniData.push_back(trans.y);
    aniData.push_back(trans.z);

    return data;
}

inline
noeKeyFrameData_t createRotKFData(const RotAnimation& rot, std::vector<float>& aniData) {
    noeKeyFrameData_t data = {};
    data.dataIndex = aniData.size();
    data.time = rot.keyframe / g_mgs1_GAME_FRAMERATE;

    RichQuat quat = { rot.x, rot.y, rot.z, rot.w };
    quat.Transpose();

    aniData.push_back(quat[0]);
    aniData.push_back(quat[1]);
    aniData.push_back(quat[2]);
    aniData.push_back(quat[3]);

    return data;
}

inline
std::vector<noeKeyFrameData_t> createKFData(const std::vector<MoveAnimation>& trans, const std::vector<RotAnimation>& rot, const std::vector<MoveAnimation>& scale, std::vector<float>& aniData) {
    std::vector<noeKeyFrameData_t> kfData;

    for (int i = 0; i < trans.size(); i++) {
        noeKeyFrameData_t transData = createTransKFData(trans[i], aniData);
        kfData.push_back(transData);
    }

    for (int i = 0; i < rot.size(); i++) {
        noeKeyFrameData_t rotData = createRotKFData(rot[i], aniData);
        kfData.push_back(rotData);
    }

    for (int i = 0; i < scale.size(); i++) {
        noeKeyFrameData_t scaleData = createTransKFData(scale[i], aniData);
        kfData.push_back(scaleData);
    }

    return kfData;
}

inline
noeKeyFramedBone_t createKFBone(uint32_t boneID, modelBone_t* noeBones, int numBones, int numFrames, const std::vector<MoveAnimation>& trans, const std::vector<RotAnimation>& rot, const std::vector<MoveAnimation>& scale, std::vector<float>& aniData, std::vector<std::vector<noeKeyFrameData_t>>& kfData) {
    noeKeyFramedBone_t kfBone = {};

    int boneIdx = boneID;
    if (boneIdx == -1) return kfBone;

    kfBone.boneIndex = boneIdx;

    kfBone.scaleInterpolation = NOEKF_INTERPOLATE_LINEAR;
    kfBone.rotationInterpolation = NOEKF_INTERPOLATE_LINEAR;
    kfBone.translationInterpolation = NOEKF_INTERPOLATE_LINEAR;

    kfBone.scaleType = NOEKF_SCALE_VECTOR_3;
    kfBone.rotationType = NOEKF_ROTATION_QUATERNION_4;
    kfBone.translationType = NOEKF_TRANSLATION_VECTOR_3;

    kfBone.numScaleKeys = scale.size();
    kfBone.numRotationKeys = rot.size();
    kfBone.numTranslationKeys = trans.size();

    kfBone.maxTime = numFrames / g_mgs1_GAME_FRAMERATE;

    std::vector<noeKeyFrameData_t> nkfData = createKFData(trans, rot, scale, aniData);
    kfData.push_back(nkfData);

    int transPos = 0;
    int rotPos = (trans.size());
    int scalePos = (trans.size() + rot.size());

    if (kfBone.numScaleKeys)        kfBone.scaleKeys = &kfData[kfData.size() - 1][scalePos];
    if (kfBone.numRotationKeys)     kfBone.rotationKeys = &kfData[kfData.size() - 1][rotPos];
    if (kfBone.numTranslationKeys)  kfBone.translationKeys = &kfData[kfData.size() - 1][transPos];

    return kfBone;
}

inline
noeKeyFramedAnim_t createKFAnim(char* animName, modelBone_t* noeBones, int numBones, std::vector<noeKeyFramedBone_t>& kfBones, std::vector<float>& data) {
    noeKeyFramedAnim_t kfAnim = {};

    kfAnim.name = animName;
    kfAnim.numKfBones = kfBones.size();
    kfAnim.kfBones = kfBones.data();
    kfAnim.numBones = numBones;
    kfAnim.framesPerSecond = g_mgs1_GAME_FRAMERATE;

    kfAnim.data = data.data();
    kfAnim.numDataFloats = data.size();

    return kfAnim;
}

inline
noesisAnim_t* bindOar(uint8_t* oar, uint8_t* archiveOffset, int maxJoints, int archiveSize, noeRAPI_t* rapi, modelBone_t* noeBones, int numBones) {
    ArchiveTable* archiveTable = (ArchiveTable*)oar;

    std::vector<float> aniData;
    std::vector<noeKeyFramedBone_t> kfBones;
    std::vector <std::vector<noeKeyFrameData_t>> kfData;
    

    for (int j = 0; j < maxJoints + 1; j++) {
        std::vector<RotAnimation>  rot;
        std::vector<MoveAnimation> trans;
        std::vector<MoveAnimation> scale;

        if (j == 0) {
            int moveOffset = archiveTable->archiveOffset[0];
            uint16_t* moveBitstream = (uint16_t*)&archiveOffset[moveOffset * 2];
            trans = readMoveBitstream(moveBitstream, archiveSize, archiveTable->numFrames);
            j++;
        }

        int boneID = j - 1;
        int rotOffset = archiveTable->archiveOffset[j];
        uint16_t* rotBitstream = (uint16_t*)&archiveOffset[rotOffset * 2];
        rot = readRotBitstream(rotBitstream, archiveSize, archiveTable->numFrames); 

        noeKeyFramedBone_t kfBone = createKFBone(boneID, noeBones, numBones, archiveTable->numFrames, trans, rot, scale, aniData, kfData);
        kfBones.push_back(kfBone);
    }

    if (aniData.empty()) return NULL;

    std::string animName = "anim";
    noeKeyFramedAnim_t kfAnim = createKFAnim((char*)animName.c_str(), noeBones, numBones, kfBones, aniData);

    noesisAnim_t* anim = rapi->Noesis_AnimFromBonesAndKeyFramedAnim(noeBones, numBones, &kfAnim, true);
    return anim;
}

inline
void loadMotion(noeRAPI_t* rapi, BYTE* motionFile, modelBone_t* noeBones, int numBones) {
    OarHeader* oarHeader = (OarHeader*)motionFile;

    if (oarHeader->maxJoint > numBones) return;

    uint8_t* archiveTable = (uint8_t*)&motionFile[0x10];

    int tableEntrySize = (oarHeader->maxJoint + 2) * 2;
    int oarTableSize   = tableEntrySize * oarHeader->numMotion;
    uint8_t* archiveOffset = &motionFile[oarTableSize + 0x10];

    CArrayList<noesisAnim_t*> animList;

    for (int i = 0; i < oarHeader->numMotion; i++) {
        uint8_t* thisOar = &archiveTable[i * tableEntrySize];

        noesisAnim_t* anim = bindOar(thisOar, archiveOffset, oarHeader->maxJoint, oarHeader->archiveSize, rapi, noeBones, numBones);
        if (anim) animList.Append(anim);
    }

    noesisAnim_t* anims = rapi->Noesis_AnimFromAnimsList(animList, animList.Num());
    rapi->rpgSetExData_AnimsNum(anims, 1);
}