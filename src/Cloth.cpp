#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double stiffness)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;
	
	// Create particles
	n = 0;
	double r = 0.02; // Used for collisions
	int nVerts = rows*cols;
	for(int i = 0; i < rows; ++i) {
		double u = i / (rows - 1.0);
		Vector3d x0 = (1 - u)*x00 + u*x10;
		Vector3d x1 = (1 - u)*x01 + u*x11;
		for(int j = 0; j < cols; ++j) {
			double v = j / (cols - 1.0);
			Vector3d x = (1 - v)*x0 + v*x1;
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			p->x = x;
			p->v << 0.0, 0.0, 0.0;
			p->m = mass/(nVerts);
			// Pin four particles (the corners)
			if((i == 0 || i == rows-1) && (j == 0 || j == cols-1)) {
				p->fixed = true;
				p->i = -1;
			} else {
				p->fixed = false;
				p->i = n;
				n += 3;
			}
		}
	}
	
	// Create x springs
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + 1;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
		}
	}
	
	// Create y springs
	for(int j = 0; j < cols; ++j) {
		for(int i = 0; i < rows-1; ++i) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
		}
	}
	
	// Create shear springs
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k00 = i*cols + j;
			int k10 = k00 + 1;
			int k01 = k00 + cols;
			int k11 = k01 + 1;
			springs.push_back(createSpring(particles[k00], particles[k11], stiffness));
			springs.push_back(createSpring(particles[k10], particles[k01], stiffness));
		}
	}
	
	// Create x bending springs
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols-2; ++j) {
			int k0 = i*cols + j;
			int k2 = k0 + 2;
			springs.push_back(createSpring(particles[k0], particles[k2], stiffness));
		}
	}
	
	// Create y bending springs
	for(int j = 0; j < cols; ++j) {
		for(int i = 0; i < rows-2; ++i) {
			int k0 = i*cols + j;
			int k2 = k0 + 2*cols;
			springs.push_back(createSpring(particles[k0], particles[k2], stiffness));
		}
	}

	// Build system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	v.resize(n);
	f.resize(n);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();

	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}

	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}
}

void Cloth::step(double h, const Vector3d &grav, const Vector3d& windForce, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	VectorXd vLastRun = v;

	typedef Eigen::Triplet<double> T;

	std::vector<T> M_;
	for (int i = 0; i < particles.size(); i++) { //set v and f
		if (!particles[i]->fixed) { //if not a fixed particle
			
			for (int index = particles[i]->i; index < particles[i]->i + 3; index++) {
				M_.push_back(T(index, index, particles[i]->m));
			}

			//Matrix3d I = Matrix3d::Identity(3, 3);
			//M.block<3, 3>(particles[i]->i, particles[i]->i) = particles[i]->m * I;

			v.segment<3>(particles[i]->i) = particles[i]->v;
			f.segment<3>(particles[i]->i) = particles[i]->m * (grav + windForce);
		}
	}
	M.setFromTriplets(M_.begin(), M_.end()); //set M

	//cout << M << endl;

	for (int i = 0; i < springs.size(); i++) { //spring forces and stiffness matrix
		auto spring = springs[i]; //current spring
		Vector3d deltaX = spring->p1->x - spring->p0->x;
		double length = deltaX.norm();
		Vector3d fs = spring->E * (length - spring->L) * (deltaX / length);
		Matrix3d I = Matrix3d::Identity(3, 3);
		double lVal = (length - spring->L) / length;
		MatrixXd x_xT = deltaX * deltaX.transpose();
		double xDot = deltaX.dot(deltaX);
		MatrixXd Ks = (spring->E / (length * length)) * ((1 - lVal) * x_xT + lVal * xDot * I); //it didn't like me doing it all on one line, so I pulled some things out as variables

		if (!spring->p0->fixed) { //if p0 not fixed
			f.segment<3>(spring->p0->i) += fs; //add force to p0

		}
		if (!spring->p1->fixed) { //if p1 not fixed
			f.segment<3>(spring->p1->i) -= fs; //subtract force from p1
		}

		
		if (!spring->p0->fixed && !spring->p1->fixed) { //if both aren't fixed
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					K(spring->p0->i + j, spring->p0->i + k) -= Ks(j, k); //plswork
					K(spring->p0->i + j, spring->p1->i + k) += Ks(j, k); //plswork
					K(spring->p1->i + j, spring->p0->i + k) += Ks(j, k); //plswork
					K(spring->p1->i + j, spring->p1->i + k) -= Ks(j, k); //plswork
				}
			}

			/*K.block<3, 3>(spring->p0->i, spring->p0->i) -= Ks; //subtract
			K.block<3, 3>(spring->p0->i, spring->p1->i) += Ks; //add
			K.block<3, 3>(spring->p1->i, spring->p0->i) += Ks; //add
			K.block<3, 3>(spring->p1->i, spring->p1->i) -= Ks; //subtract*/
		}
		else if (!spring->p0->fixed) { //else if p0 isn't fixed
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					K(spring->p0->i + j, spring->p0->i + k) -= Ks(j, k); //plswork
				}
			}
			//K.block<3, 3>(spring->p0->i, spring->p0->i) -= Ks; //subtract
		}
		else if (!spring->p1->fixed) { //esle if 01 isn't fixed
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					K(spring->p1->i + j, spring->p1->i + k) -= Ks(j, k); //plswork
				}
			}
			//K.block<3, 3>(spring->p1->i, spring->p1->i) -= Ks; //subtract
		}
		
	}

	double c = 65; //collision constant

	for (int i = 0; i < particles.size(); i++) { //sphere collisions

		auto particle = particles[i];

		for (int j = 0; j < spheres.size(); j++) { //each sphere
			auto sphere = spheres[j];
			Vector3d deltaX = particle->x - sphere->x;
			double length = deltaX.norm();
			double d = particle->r + sphere->r - length;

			if (d > 0 && !particle->fixed) { //collision with a non fixed particle
				Vector3d fc = c * d * (deltaX / length);
				f.segment<3>(particle->i) += fc; //add collision force
				Matrix3d Kc = c * d * Matrix3d::Identity(3,3);
				K.block<3, 3>(particle->i, particle->i) -= Kc; //subtract? Not sure if it should be added or subtracted (I get good results with both (or not setting it at all))
			}
		}

	}

	//solve the system
	SparseMatrix<double> A = M - (h * h) * K.sparseView();
	VectorXd b = M * v + h * f;

	ConjugateGradient< SparseMatrix<double> > cg;
	cg.setMaxIterations(25);
	cg.setTolerance(1e-6);
	cg.compute(A);
	VectorXd x = cg.solveWithGuess(b, vLastRun);

	for (int i = 0; i < particles.size(); i++) {
		if (!particles[i]->fixed) { //if not fixed 
			particles[i]->v = x.segment<3>(particles[i]->i);
			particles[i]->x += h * particles[i]->v; // I had this in a separate for loop like in the assignment page, but I don't think two different ones are needed?
		}
	}

	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
