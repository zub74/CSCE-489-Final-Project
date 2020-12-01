#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"
#include <thread> //multithread!
#include <chrono>
#include <ctime>

#define PI 3.1415

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	stepCount(0),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 5e-3;
	
	grav << 0.0, -9.8, 0; //gravity

	initialWindForce << 1.0, 0.0, 9.8; //set up the wind force 
	windForce = initialWindForce;
	
	int rows = 15;
	int cols = 15;
	double mass = 0.1;
	double stiffness = 1e1;
	Vector3d x00(-0.5, 1.48, -0.103);
	Vector3d x01(0.5, 1.48, -0.103);
	Vector3d x10(-0.75, 0.9, -0.103);
	Vector3d x11(0.75, 0.9, -0.103);
	Vector3d offset(0, 0, 0.5);
	std::shared_ptr<Cloth> sail1 = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, stiffness);
	//std::shared_ptr<Cloth> sail2 = make_shared<Cloth>(rows, cols, x00 + offset, x01 + offset, x10 + offset, x11 + offset, mass, stiffness);
	//std::shared_ptr<Cloth> sail3 = make_shared<Cloth>(rows, cols, x00 - offset, x01 - offset, x10 - offset, x11 - offset, mass, stiffness);

	sails.push_back(sail1); //add sails
	//sails.push_back(sail2);
	//sails.push_back(sail3);
	
	sphereShape = make_shared<Shape>();
	sphereShape->loadMesh(RESOURCE_DIR + "Little_Ship.obj");

	ship = make_shared<Shape>();
	ship->loadMesh(RESOURCE_DIR + "Little_Ship.obj");
	
	auto sphere = make_shared<Particle>(sphereShape);
	spheres.push_back(sphere);
	sphere->r = 0.01;
	sphere->x = Vector3d(0.0, 0.2, 0.0);

	auto smallSphere1 = make_shared<Particle>(sphereShape);
	auto smallSphere2 = make_shared<Particle>(sphereShape);
	auto smallSphere3 = make_shared<Particle>(sphereShape);
	auto smallSphere4 = make_shared<Particle>(sphereShape);
	auto smallSphere5 = make_shared<Particle>(sphereShape);
	auto smallSphere6 = make_shared<Particle>(sphereShape);
	smallSphere1->r = 0.05;
	smallSphere2->r = 0.05;
	smallSphere3->r = 0.05;
	smallSphere4->r = 0.05;
	smallSphere5->r = 0.05;
	smallSphere6->r = 0.05;
	int t = 0;
	smallSphere1->x = sphere->x + Vector3d(0.20 * sin(3 * t), 0, 0.20 * cos(3 * t));
	smallSphere2->x = sphere->x + Vector3d(-0.20 * sin(3 * t), 0, -0.20 * cos(3 * t));
	smallSphere3->x = sphere->x + Vector3d(0, 0.20 * sin(1 * t + PI / 2), -0.20 * cos(1 * t + PI / 2));
	smallSphere4->x = sphere->x + Vector3d(0, -0.20 * sin(1 * t + PI / 2), 0.20 * cos(1 * t + PI / 2));
	smallSphere5->x = sphere->x + Vector3d(0.20 * sin(2 * t + PI / 4), 0.20 * cos(2 * t + PI / 4), 0);
	smallSphere6->x = sphere->x + Vector3d(-0.20 * sin(2 * t + PI / 4), -0.20 * cos(2 * t + PI / 4), 0);
	spheresAlt.push_back(smallSphere1);
	spheresAlt.push_back(smallSphere2);
	spheresAlt.push_back(smallSphere3);
	spheresAlt.push_back(smallSphere4);
	spheresAlt.push_back(smallSphere5);
	spheresAlt.push_back(smallSphere6);
}

void Scene::setAtom(bool b) {
	if (b) {
		for (int i = 0; i < spheresAlt.size(); i++) { //add spheres to sphere vector
			spheresAlt[i]->x = spheres[0]->x; //put them inside the sphere so they don't show the past location for the first frame after reenabling
			spheres.push_back(spheresAlt[i]);
		}
		auto s = spheres[0];
		//update positions before drawing again
		spheres[1]->x = s->x + Vector3d(0.20 * sin(3 * t), 0, 0.20 * cos(3 * t));
		spheres[2]->x = s->x + Vector3d(-0.20 * sin(3 * t), 0, -0.20 * cos(3 * t));
		spheres[3]->x = s->x + Vector3d(0, 0.20 * sin(1 * t + PI / 2), -0.20 * cos(1 * t + PI / 2));
		spheres[4]->x = s->x + Vector3d(0, -0.20 * sin(1 * t + PI / 2), 0.20 * cos(1 * t + PI / 2));
		spheres[5]->x = s->x + Vector3d(0.20 * sin(2 * t + PI / 4), 0.20 * cos(2 * t + PI / 4), 0);
		spheres[6]->x = s->x + Vector3d(-0.20 * sin(2 * t + PI / 4), -0.20 * cos(2 * t + PI / 4), 0);
	}
	else {
		spheres.resize(1); //remove all additional spheres
	}

	atom = b;
}

void Scene::init()
{
	sphereShape->init();
	for (auto const& sail : sails) {
		sail->init();
	}
}

void Scene::tare()
{
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->tare();
	}
	for (auto const& sail : sails) {
		sail->tare();
	}
}

void Scene::reset()
{
	t = 0.0;
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->reset();
	}
	for (auto const& sail : sails) {
		sail->reset();
	}
}

void Scene::step()
{
	t += h;

	grav(2) = -abs(cos(t) / 10);
	
	// Move the sphere
	if(!spheres.empty()) {
		auto s = spheres.front();
		Vector3d x0 = s->x;
		//s->x(2) = 0.5 * sin(0.5 * t);
		if (atom && spheres.size() > 1) {
			spheres[1]->x = s->x + Vector3d(0.20 * sin(3 * t), 0, 0.20 * cos(3 * t));
			spheres[2]->x = s->x + Vector3d(-0.20 * sin(3 * t), 0, -0.20 * cos(3 * t));
			spheres[3]->x = s->x + Vector3d(0, 0.20 * sin(1 * t + PI / 2), -0.20 * cos(1 * t + PI / 2));
			spheres[4]->x = s->x + Vector3d(0, -0.20 * sin(1 * t + PI / 2), 0.20 * cos(1 * t + PI / 2));
			spheres[5]->x = s->x + Vector3d(0.20 * sin(2 * t + PI / 4), 0.20 * cos(2 * t + PI / 4), 0);
			spheres[6]->x = s->x + Vector3d(-0.20 * sin(2 * t + PI / 4), -0.20 * cos(2 * t + PI / 4), 0);
		}
	}
	
	// Simulate the cloth
	//spheres.clear();
	if (!(stepCount % 150)) { //every 5 steps, update rand factor
		long initialTime = 1606801198;
		time_t  timev;
		cout << time(&timev) - initialTime << endl;
		windForce = initialWindForce + (initialWindForce * (rand() % 10) / 10.0);
		//windForce << 0, 0, 0;
	}

	vector<thread> threads;
	for (int i = 1; i < sails.size(); i++) { //run every other sail on a different thread
		//sail->step(h, grav, windForce, spheres);
		thread t(&Cloth::step, sails[i], h, grav, windForce, spheres); //hot diggity darn gosh it worked https://thispointer.com/c11-start-thread-by-member-function-with-arguments/
		threads.push_back(move(t)); //no copy constructor (makes sense)
	}

	sails[0]->step(h, grav, windForce, spheres); //run first sail on this thread

	for (int i = 0; i < threads.size(); i++) {
		threads[i].join(); 
	}

	stepCount++; //increment step count (don't want to use t so I can easily mod it without fmod)
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(139.0 / 255.0 , 69.0 / 255.0, 19.0 / 255.0).data()); //saddle brown!

	MV->pushMatrix();
	MV->scale(0.01);
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	ship->draw(prog); //draw the ship
	MV->popMatrix();

	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->draw(MV, prog);
		glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(spheres[i]->x(0) * 2 + i * 0.1, spheres[i]->x(1) * 2, abs(spheres[i]->x(2) * 2)).data());
	}
	for (auto const& sail : sails) {
		sail->draw(MV, prog);
	}
}
