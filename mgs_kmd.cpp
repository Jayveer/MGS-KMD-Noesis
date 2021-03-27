#include <vector>
#include "tool.h"

const char* g_pPluginName = "mgs_kmd";
const char* g_pPluginDesc = "Metal Gear Solid KMD handler by Jayveer.";

bool checkKMD(BYTE* fileBuffer, int bufferLen, noeRAPI_t* rapi) {
    return true;
}

noesisModel_t* loadKMD(BYTE* fileBuffer, int bufferLen, int& numMdl, noeRAPI_t* rapi) {
    void* ctx = rapi->rpgCreateContext();
    KmdHeader* header = (KmdHeader*)fileBuffer;
    KmdMesh*   mesh   = (KmdMesh*  )&fileBuffer[0x20];

    modelBone_t* noeBones = bindKMDBones(mesh, header->numMesh, rapi);

    CArrayList<noesisTex_t*>      texList;
    CArrayList<noesisMaterial_t*> matList;

    for (int i = 0; i < header->numMesh; i++) {
        bindMesh(&mesh[i], i, &noeBones[i], fileBuffer, rapi, texList, matList);
    }

    noesisMatData_t* md = rapi->Noesis_GetMatDataFromLists(matList, texList);
    rapi->rpgSetExData_Materials(md);

    if (g_mgs1OarPrompt && header->numBones) {
        BYTE* motionFile = openMotion(rapi);
        if (motionFile) loadMotion(rapi, motionFile, noeBones, header->numBones);
    }

    noesisModel_t* mdl = rapi->rpgConstructModel();
    if (mdl) numMdl = 1;

    rapi->rpgDestroyContext(ctx);
    return mdl;
}

bool NPAPI_InitLocal(void) {
    int fh = g_nfn->NPAPI_Register("Metal Gear Solid", ".kmd");
    if (fh < 0) return false;

    g_nfn->NPAPI_SetTypeHandler_TypeCheck(fh, checkKMD);
    g_nfn->NPAPI_SetTypeHandler_LoadModel(fh, loadKMD);

    applyTools();

    return true;
}


void NPAPI_ShutdownLocal(void) {

}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved) {
    return TRUE;
}