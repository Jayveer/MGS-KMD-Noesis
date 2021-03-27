#pragma once
#include "mgs/common/util.h"
#include "mgs/archive/dar/dar.h"
#include "image/pcx/dr_pcx.h"

inline
int findMaterialIdx(char* matName, CArrayList<noesisMaterial_t*>& matList) {
    for (int i = 0; i < matList.Num(); i++) {
        if (!strcmp(matList[i]->name, matName))
            return i;
    }
    return -1;
}

inline
int findTextureIdx(char* texName, CArrayList<noesisTex_t*>& texList) {
    for (int i = 0; i < texList.Num(); i++) {
        if (!strcmp(texList[i]->filename, texName))
            return i;
    }
    return -1;
}

inline
uint8_t* findPcx(noeRAPI_t* rapi, uint16_t& strcode, int& size) {
    std::filesystem::path p{ rapi->Noesis_GetInputName() };
    p = p.parent_path();

    for (const std::filesystem::directory_entry& file : std::filesystem::recursive_directory_iterator(p)) {
        if (file.path().extension() == ".dar") {
            Dar dar = Dar(file.path().u8string());

            if (uint8_t* pcx = dar.findFile(strcode, 0x70, size))
                return pcx;
        }
    }

    return NULL;
}

inline
noesisTex_t* loadTexture(noeRAPI_t* rapi, uint16_t& strcode) {
    int size;
    uint8_t* texData = findPcx(rapi, strcode, size);
    if (!texData) return NULL;

    int width;
    int height;
    int components;
    uint8_t* imageData = drpcx_load_memory(texData, size, false, &width, &height, &components, 4);
    int datasize = width * height * 4;
    uint8_t* tga = makeTGA(imageData, datasize, width, height);
    drpcx_free(imageData);
    noesisTex_t* noeTexture = rapi->Noesis_LoadTexByHandler(tga, datasize + 0x12, ".tga");
    delete[] tga;
    delete[] texData;
    return noeTexture;
}

inline
void bindMat(uint16_t strcode, BYTE* fileBuffer, noeRAPI_t* rapi, CArrayList<noesisMaterial_t*>& matList, CArrayList<noesisTex_t*>& texList) {

    //set mat name
    std::string matStr = intToHexString(strcode);
    char matName[7];
    strcpy_s(matName, matStr.c_str());

    //check if material already exists
    int x = findMaterialIdx(matName, matList);

    //use existing mat if it does
    if (x > -1) {
        rapi->rpgSetMaterial(matName);
        return;
    }
    
    //create material
    noesisMaterial_t* noeMat = rapi->Noesis_GetMaterialList(1, false);
    noeMat->name = rapi->Noesis_PooledString(matName);

    //set tex name
    std::string texStr = intToHexString(strcode) + ".tga";
    char texName[11];
    strcpy_s(texName, texStr.c_str());

    //check if texture already exists
    int y = findTextureIdx(texName, texList);

    //set tex to mat if it does
    if (y > -1) {
        noeMat->texIdx = y;
        matList.Append(noeMat);
        rapi->rpgSetMaterial(matName);
        return;
    }

    //load texture
    noesisTex_t* noeTexture = loadTexture(rapi, strcode);
    if (!noeTexture) return;
    noeTexture->filename = rapi->Noesis_PooledString(texName);

    //set tex to mat
    noeMat->texIdx = texList.Num();
    matList.Append(noeMat);
    
    texList.Append(noeTexture);

    //set material
    rapi->rpgSetMaterial(noeMat->name);
}