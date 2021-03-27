#pragma once
#include "mgs/model/kmd/kmd.h"
#include "noesis/plugin/pluginshare.h"

inline
void worldToParent(modelBone_t* bone) {
    RichMat43 currentMat(bone->mat);
    RichMat43 parentMat(bone->eData.parent->mat);
    bone->mat = (currentMat * parentMat).m;
}

inline
modelBone_t* bindKMDBones(KmdMesh* mesh, int numBones, noeRAPI_t* rapi) {
    modelBone_t* noeBones = rapi->Noesis_AllocBones(numBones);

    for (int i = 0; i < numBones; i++) {
        float x = mesh[i].pos.x;
        float y = mesh[i].pos.y;
        float z = mesh[i].pos.z;

        RichVec3 bonePosV3 = { x, y, z };
        memcpy_s(&noeBones[i].mat.o, 12, &bonePosV3, 12);

        if (mesh[i].parent > -1) {
            noeBones[i].eData.parent = &noeBones[mesh[i].parent];
            worldToParent(&noeBones[i]);
        }
    }

    rapi->rpgSetExData_Bones(noeBones, numBones);
    return noeBones;
}