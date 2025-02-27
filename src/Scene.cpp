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

#define PI  3.141592653589793238

using namespace std;
using namespace Eigen;


Scene::Scene() :
	t(0.0),
	h(1e-2),
	stepCount(0),
	grav(0.0, 0.0, 0.0),
	windAngle(0)
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

	initialWindForce << 0.0, 0.0, -10; //set up the wind force 
	windForce = initialWindForce;
	
	int rows = 17;
	int cols = 17;
	double mass = 0.1;
	double stiffness = 1e1;
	Vector3d x00(-0.75, 2.13 - 0.2, 0.395);
	Vector3d x01(0.75, 2.13 - 0.2, 0.395);
	Vector3d x10(-1, 1.0, 0.34);
	Vector3d x11(1, 1.0, 0.34);
	Vector3d s00(-0.025, 2.67, 2.46);
	Vector3d s01(-0.025, 0.9, 0.8);
	Vector3d s10(-0.025, 1.6, 2.45);
	Vector3d s11(-0.025, 0.8, 0.8);
	Vector3d f00(-0.70, 2.13 - 0.3, -1.045);
	Vector3d f01(0.70, 2.13 - 0.3, -1.045);
	Vector3d f10(-0.95, 1.0, -1.10);
	Vector3d f11(0.95, 1.0, -1.10);
	shared_ptr<Cloth> sail1 = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, stiffness, false);
	shared_ptr<Cloth> sail2 = make_shared<Cloth>(rows, cols, s00, s01, s10, s11, mass, stiffness * 3, false);
	shared_ptr<Cloth> sail3 = make_shared<Cloth>(rows, cols, f00, f01, f10, f11, mass, stiffness * 3, false);

	double y = 0.0;
	shared_ptr<Cloth> water = make_shared<Cloth>(rows + 2, cols + 2, Vector3d(-10,y,10), Vector3d(10, y, 10), Vector3d(-10, y, -10), Vector3d(10, y, -10), mass, stiffness / 5.0, true);

	sails.push_back(water); //add water
	sails.push_back(sail1); //add sails
	sails.push_back(sail2); //add sails
	sails.push_back(sail3); //add sails
	
	sphereShape = make_shared<Shape>();
	sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

	ship = make_shared<Shape>();
	ship->loadMesh(RESOURCE_DIR + "galleon_model_stripped_down.obj");

	compass = make_shared<Shape>();
	compass->loadMesh(RESOURCE_DIR + "sphere2.obj");

	auto mast1 = make_shared<Particle>(sphereShape);
	auto mast2 = make_shared<Particle>(sphereShape);
	auto mast3 = make_shared<Particle>(sphereShape);
	spheres.push_back(mast1);
	spheres.push_back(mast2);
	spheres.push_back(mast3);
	mast1->r = 0.08;
	mast2->r = 0.08;
	mast3->r = 0.06;
	mast1->x = Vector3d(0.0, 2, 0.47);
	mast2->x = Vector3d(0.05, 2, 1.55);
	mast3->x = Vector3d(0.0, 2, -1);

	{
		/*auto smallSphere1 = make_shared<Particle>(sphereShape);
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
		spheresAlt.push_back(smallSphere6);*/
	}
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
	ship->init(); //YOU DUMB FKIN DUMMY INITIALIZE YOUR OBJECTS GOD
	compass->init();
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
	if (angleUpdate) { //update the wind force
		windForce << 10 * sin(windAngle * PI / 180), 0, -10 * cos(windAngle * PI / 180);
		initialWindForce = windForce;
		//cout << windForce << endl;
		angleUpdate = false;
	}

	if (!(stepCount % 150)) { //every 150 steps, update rand factor
		windForce = initialWindForce + (initialWindForce * (rand() % 10) / 10.0);
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
	glUniform3fv(prog->getUniform("kdBack"), 1, Vector3f(139.0 / 255.0 , 69.0 / 255.0, 19.0 / 255.0).data()); //saddle brown!

	
	MV->pushMatrix();
	MV->translate(0, -(MV->topMatrix()[3][1] - sails[0]->center(1)), 0); //follow the water (idfk why the water y value starts at 1.465)
	MV->rotate(sin(t*3) / 5, 0, 0, 1); //boat rotation
	MV->rotate(PI / 32, 1, 0, 0); //boat rotation

	MV->pushMatrix();//draw boat scaled down
	MV->scale(0.5);
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	ship->draw(prog);
	MV->popMatrix();

	for(int i = 0; i < (int)spheres.size(); ++i) { //draw spheres (for debugging)
		//spheres[i]->draw(MV, prog);
		//glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(spheres[i]->x(0) * 2 + i * 0.1, spheres[i]->x(1) * 2, abs(spheres[i]->x(2) * 2)).data());
	}

	for (int i = 1; i < sails.size(); i++) { //draw sails
		sails[i]->draw(MV, prog);
	}
	MV->popMatrix();

	sails[0]->draw(MV, prog); //draw water
}
