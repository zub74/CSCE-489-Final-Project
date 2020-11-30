#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();
	
	void load(const std::string &RESOURCE_DIR);
	void init();
	void tare();
	void reset();
	void step();
	
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;
	
	double getTime() const { return t; }
	void setAtom(bool b);
	
private:
	double t;
	double h;
	long stepCount;
	bool atom = false;
	Eigen::Vector3d grav;
	Eigen::Vector3d windForce;
	Eigen::Vector3d initialWindForce;

	
	std::shared_ptr<Shape> sphereShape;
	std::shared_ptr<Cloth> cloth;
	std::vector< std::shared_ptr<Particle> > spheres;
	std::vector< std::shared_ptr<Particle> > spheresAlt;
};

#endif
