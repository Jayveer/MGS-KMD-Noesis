#pragma once
#include <fstream>
#include <filesystem>

struct DarEntry {
	uint16_t strcode;
	uint16_t extension;
	uint32_t size;
	uint8_t data[];
};

class Dar {
public:
	Dar(std::string filename);
	~Dar();

	uint8_t* findFile(uint16_t id, uint16_t ext, int& size);
private:
	uint8_t* darData;
	int dataSize;
};