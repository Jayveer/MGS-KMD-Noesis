#include "dar.h"

Dar::Dar(std::string filename) {
	std::ifstream fs;
	this->dataSize = std::filesystem::file_size(filename);

	fs.open(filename, std::ios::binary);
	uint8_t* p = new uint8_t[dataSize];
	fs.read((char*)p, dataSize);
	this->darData = p;
	fs.close();
}

Dar::~Dar() {
	delete[] darData;
}

uint8_t* Dar::findFile(uint16_t id, uint16_t ext, int& size) {
	int ptr = 0;

	while (ptr < dataSize) {
		DarEntry* entry = (DarEntry*)&darData[ptr];

		if (entry->strcode == id && entry->extension == ext) {
			size = entry->size;
			uint8_t* file = new uint8_t[size];
			memcpy(file, &darData[ptr + 8], size);
			return file;
		}

		ptr += (entry->size) + 8;
	}

	return NULL;
}