#pragma once
#include "mat.h"

inline
void setOrigin(KmdMesh* mesh, modelBone_t* noeBone, noeRAPI_t* rapi) {
    modelMatrix_t t = g_identityMatrix;
    g_mfn->Math_VecCopy(noeBone->mat.o, t.o);
    rapi->rpgSetTransform(&t);
}


inline
void bindVertex(KmdVert* vertices, std::vector<float>& vertexBuffer) {
    vertexBuffer.push_back(vertices->x);
    vertexBuffer.push_back(vertices->y);
    vertexBuffer.push_back(vertices->z);
}

inline
void bindNormal(KmdNVert* vertices, std::vector<float>& normalBuffer) {
    float scale = 1 / 4096.0f;
    normalBuffer.push_back(vertices->x * scale);
    normalBuffer.push_back(vertices->y * scale);
    normalBuffer.push_back(vertices->z * scale);
}

inline
void bindUV(KmdUV* uv, std::vector<float>& uvBuffer) {
    float scale = 256.0f;
    uvBuffer.push_back(uv->tu / scale);
    uvBuffer.push_back(uv->tv / scale);
}

inline
void bindSkin(int16_t parent, KmdMesh* mesh, int meshNum, std::vector<float>& weightBuffer, std::vector<uint8_t>& boneBuffer) {
    weightBuffer.push_back(1.0f);
    int boneIdx = parent == -1 ? meshNum : mesh->parent;
    boneBuffer.push_back(boneIdx);
}

inline
void bindMesh(KmdMesh* mesh, int meshNum, modelBone_t* noeBone, BYTE* fileBuffer, noeRAPI_t* rapi, CArrayList<noesisTex_t*>& texList, CArrayList<noesisMaterial_t*>& matList) {
    std::vector<float> uvBuffer, normalBuffer, vertexBuffer, weightBuffer;
    std::vector<uint8_t> boneBuffer, faceBuffer, normalFaceBuffer;

    KmdUV* uvOffset = (KmdUV*)&fileBuffer[mesh->uvOffset];
    uint8_t* faceOffset = (uint8_t*)&fileBuffer[mesh->faceIndexOffset];
    KmdVert* vertexOffset = (KmdVert*)&fileBuffer[mesh->vertexIndexOffset];
    KmdNVert* normalOffset = (KmdNVert*)&fileBuffer[mesh->normalIndexOffset];
    uint16_t* materialOffset = (uint16_t*)&fileBuffer[mesh->materialOffset];
    uint8_t* normalFaceOffset = (uint8_t*)&fileBuffer[mesh->normalFaceOffset];

    int x = 0;
    setOrigin(mesh, noeBone, rapi);

    for (int i = 0; i < mesh->numFace; i++) {

        for (int j = 0; j < 4; j++) {
            uint8_t fa = faceOffset[x];
            uint8_t na = normalFaceOffset[x] & 0x7F;

            bindUV(&uvOffset[x++], uvBuffer);
            bindVertex(&vertexOffset[fa], vertexBuffer);
            bindNormal(&normalOffset[na], normalBuffer);
            bindSkin(vertexOffset[fa].w, mesh, meshNum, weightBuffer, boneBuffer);
        }

        faceBuffer.push_back(0);
        faceBuffer.push_back(2);
        faceBuffer.push_back(1);

        if (faceOffset[x - 2] != faceOffset[x - 1]) {
            faceBuffer.push_back(0);
            faceBuffer.push_back(3);
            faceBuffer.push_back(2);
        }

        bindMat(materialOffset[i], fileBuffer, rapi, matList, texList);
        rapi->rpgBindBoneIndexBuffer(&boneBuffer[0], RPGEODATA_UBYTE, 1, 1);
        rapi->rpgBindBoneWeightBuffer(&weightBuffer[0], RPGEODATA_FLOAT, 4, 1);
        rapi->rpgBindUV1BufferSafe(&uvBuffer[0], RPGEODATA_FLOAT, 8, uvBuffer.size() * 4);
        rapi->rpgBindNormalBufferSafe(&normalBuffer[0], RPGEODATA_FLOAT, 12, normalBuffer.size() * 4);
        rapi->rpgBindPositionBufferSafe(&vertexBuffer[0], RPGEODATA_FLOAT, 12, vertexBuffer.size() * 4);
        rapi->rpgCommitTrianglesSafe(&faceBuffer[0], RPGEODATA_UBYTE, faceBuffer.size(), RPGEO_TRIANGLE, 0);
        rapi->rpgClearBufferBinds();

        uvBuffer.clear();
        faceBuffer.clear();
        boneBuffer.clear();
        normalBuffer.clear();
        weightBuffer.clear();
        vertexBuffer.clear();
    }
}