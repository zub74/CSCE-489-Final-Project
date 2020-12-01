#include <iostream>

#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"
#include <thread> //multithread!

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

	initialWindForce << 0.0, 0.0, 9.8; //set up the wind force 
	windForce = initialWindForce;
	
	int rows = 20;
	int cols = 20;
	double mass = 0.1;
	double stiffness = 1e1;
	Vector3d x00(-0.25, 0.5, 0.0);
	Vector3d x01(0.25, 0.5, 0.0);
	Vector3d x10(-0.3, 0.0, 0);
	Vector3d x11(0.3, 0.0, 0);
	Vector3d offset(0, 0, 0.5);
	std::shared_ptr<Cloth> sail1 = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, stiffness);
	std::shared_ptr<Cloth> sail2 = make_shared<Cloth>(rows, cols, x00 + offset, x01 + offset, x10 + offset, x11 + offset, mass, stiffness);
	std::shared_ptr<Cloth> sail3 = make_shared<Cloth>(rows, cols, x00 - offset, x01 - offset, x10 - offset, x11 - offset, mass, stiffness);

	sails.push_back(sail1); //add sails
	sails.push_back(sail2);
	sails.push_back(sail3);
	
	sphereShape = make_shared<Shape>();
	sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");
	
	auto sphere = make_shared<Particle>(sphereShape);
	spheres.push_back(sphere);
	sphere->r = 0.1;
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
	for (auto sail : sails) {
		sail->init();
	}
}

void Scene::tare()
{
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->tare();
	}
	for (auto sail : sails) {
		sail->tare();
	}
}

void Scene::reset()
{
	t = 0.0;
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->reset();
	}
	for (auto sail : sails) {
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
		s->x(2) = 0.5 * sin(0.5 * t);
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
	if (!(stepCount % 50)) { //every 5 steps, update rand factor
		cout << "updating wind" << endl;
		windForce = initialWindForce + (initialWindForce * (rand() % 10) / 10.0);
		//windForce << 0, 0, 0;
	}

	vector<thread> threads;
	for (auto sail : sails) {
		//sail->step(h, grav, windForce, spheres);
		thread t(&Cloth::step, sail, h, grav, windForce, spheres); //hot diggity darn gosh it worked https://thispointer.com/c11-start-thread-by-member-function-with-arguments/
		threads.push_back(move(t)); //no copy constructor (makes sense)
	}

	for (int i = 0; i < threads.size(); i++) { //auto constructor didn't work?
		threads[i].join(); 
	}

	stepCount++; //increment step count (don't want to use t so I can easily mod it without fmod)
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(139.0 / 255.0 , 69.0 / 255.0, 19.0 / 255.0).data()); //saddle brown!
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->draw(MV, prog);
		glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(spheres[i]->x(0) * 2 + i * 0.1, spheres[i]->x(1) * 2, abs(spheres[i]->x(2) * 2)).data());
	}
	for (auto sail : sails) {
		sail->draw(MV, prog);
	}
}
