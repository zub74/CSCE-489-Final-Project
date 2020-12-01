#pragma once
#ifndef SHAPE_H
#define SHAPE_H

#include <memory>
#include <vector>
#include <string>

#define GLEW_STATIC
#include <GL/glew.h>

class MatrixStack;
class Program;
class DeltaMesh;
class Program;

class Shape
{
public:
	Shape();
	virtual ~Shape();
	void loadObj(const std::string &filename, std::vector<float> &pos, std::vector<float> &nor, std::vector<float> &tex, bool loadNor = true, bool loadTex = true);
	void loadMesh(const std::string &meshName);
	void setProgram(std::shared_ptr<Program> p) { prog = p; }
	virtual void init();
	virtual void draw(const std::shared_ptr<Program> prog) const;
	void setTextureFilename(const std::string &f) { textureFilename = f; }
	std::string getTextureFilename() const { return textureFilename; }
	std::vector<float> getPosBuf() { return posBuf; }
	std::vector<float> getNorBuf() { return norBuf; }
	std::vector<float> getTexBuf() { return texBuf; }
	void setPosBuf(std::vector<float> b) { posBuf = b; }
	void setNorBuf(std::vector<float> b) { norBuf = b; }
	void setTexBuf(std::vector<float> b) { texBuf = b; }
	void addBlendPos(std::vector<float> blendBuf, double t);
	void addBlendNor(std::vector<float> blendBuf, double t);
	void addBlendTex(std::vector<float> blendBuf, double t);
	
protected:
	std::string meshFilename;
	std::string textureFilename;
	std::shared_ptr<Program> prog;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	GLuint posBufID;
	GLuint norBufID;
	GLuint texBufID;
};

#endif
