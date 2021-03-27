#pragma once
#include "motion.h"
#include "bone.h"
#include "mesh.h"
#include "mat.h"

bool g_mgs1OarPrompt = false;
const char* g_mgs1plugin_name = "Metal Gear Solid";

inline
int genericToolSet(bool& setting, int toolIdx) {
    setting = !setting;
    g_nfn->NPAPI_CheckToolMenuItem(toolIdx, setting);
    return 1;
}

int mgs1_anim_prompt(int toolIdx, void* user_data) {
    return genericToolSet(g_mgs1OarPrompt, toolIdx);
}

inline
int makeTool(char* toolDesc, int (*toolMethod)(int toolIdx, void* userData)) {
    int handle = g_nfn->NPAPI_RegisterTool(toolDesc, toolMethod, NULL);
    g_nfn->NPAPI_SetToolSubMenuName(handle, g_mgs1plugin_name);
    return handle;
}

inline
void applyTools() {
    makeTool("Prompt for Motion Archive", mgs1_anim_prompt);
}