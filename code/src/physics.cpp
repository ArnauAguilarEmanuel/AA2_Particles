#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <iostream>
#include <vector>

namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace Particles {
	extern const int maxParticles;
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
}
namespace Mesh {
	extern const int numCols;
	extern const int numRows;
	extern void updateMesh(float* array_data);
	extern void drawMesh();
}
namespace Fiber {
	extern const int numVerts;
	extern void updateFiber(float* array_data);
	extern void drawFiber();
}
namespace Cube {
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}

///GUI Parameters
float sphereMass = 0;
float sphereRadius = 1;
float spherePosition[3]{ 0, 3, 0 };

float capsuleRadius = 1;
float CapsulePositionA[3]{ -3,1,2 };
float CapsulePositionB[3]{ -3,1,-2 };

float gravity[3]{ 0, -9.81, 0 };
bool useGravity = true;

bool pause = false;
bool restart = false;

float elasticCoeficient = 1;
float frictionCoeficient = 1;
////

struct Collider {

	virtual bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) = 0;
	virtual void getPlane( glm::vec3& normal, float& d) = 0;

	void computeCollision(const glm::vec3& old_pos, const glm::vec3& old_vel, glm::vec3& new_pos, glm::vec3& new_vel) {
		float d;
		glm::vec3 normal;
		getPlane(normal, d);
		normal = glm::normalize(normal);

		float planeDistance = (normal.x * old_pos.x + normal.y * old_pos.y + normal.z * old_pos.z + d) / sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) ;
		new_pos = old_pos - (1 + elasticCoeficient) * ( normal * old_pos + d) * normal;

		new_vel = old_vel - (1 + elasticCoeficient) * (normal * old_vel) * normal;

		new_vel = new_vel - frictionCoeficient * (new_vel - (normal * new_vel) * normal);
	}
};

struct Plane {
	float d;
	glm::vec3 normal;
	glm::vec3 position;

	Plane() {};

	Plane(glm::vec3 _position, glm::vec3 _normal) : position(_position), normal(_normal) {
		d = -(position.x* normal.x + position.y* normal.y + position.z* normal.z);
	}
};

struct PlaneCol : Collider {
public:
	Plane plane;

	PlaneCol(glm::vec3 _position, glm::vec3 _normal) {
		plane = Plane(_position, _normal);
	}

	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) override {
		return 0 >= (next_pos.x * plane.normal.x + next_pos.y * plane.normal.y + next_pos.z * plane.normal.z + plane.d) /
			glm::sqrt(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y + plane.normal.z * plane.normal.z);
	}

	void getPlane(glm::vec3& normal, float& d) override{
		normal = plane.normal;
		d = plane.d;
	}
};

class Particle {
public:
	Particle(glm::vec3 _position, glm::vec3 _velocity, float _mass) : position(_position), velocity(_velocity), mass(_mass) {}
	glm::vec3 position;
	glm::vec3 old_pos;
	glm::vec3 velocity;
	glm::vec3 old_vel;
	glm::vec3 acceleration;
	float mass;
};

struct ForceActuator {
	virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0;
};







struct GravityForce : ForceActuator {
	glm::vec3 gravity;
	void updateGravity(float g[3]){
		gravity = glm::vec3(g[0], g[1], g[2]);
	}
	glm::vec3 computeForce(float mass, const glm::vec3& position) override {
		if(useGravity) return mass * gravity;
		else return glm::vec3(0);
	}
};

struct PositionalGravityForce : ForceActuator {
	float M;
	float G;
	glm::vec3 pos;
	PositionalGravityForce(float _mass, glm::vec3& _position, float _G): pos(_position), M(_mass), G(_G) {}

	glm::vec3 computeForce(float mass, const glm::vec3& position) override {
		glm::vec3 vec = position - pos;
		float length = vec.x*vec.x + vec.y *vec.y + vec.z * vec.z;
		
		float strength = ((G * M * mass) / length);

		return  strength * glm::normalize((pos - position));
	}

	void updateMass(float _mass) {
		M = _mass;
	}
	void updatePos(glm::vec3 _pos){
		pos = _pos;
	}

};

class ParticleSystem {
public:
	std::vector<Particle> particles;
	std::vector<float> particlesPositions;

	ParticleSystem(int count) {
		for (int i = 0; i < count; i++) {
			particles.push_back(Particle(
				glm::vec3((((rand() % 1000))/100.f) - 5, ((rand() % 500)/100.f) + 5, ((rand() % 1000)/100.f) - 5),///pos
				glm::vec3(((rand() % 5)-2), (rand() % 5) - 2, (rand() % 5) - 2),///vel
				1)///mass
			);//push a new particle in a random positon x and z and y
		}

		for (Particle p : particles) {
			particlesPositions.push_back(p.position.x);
			particlesPositions.push_back(p.position.y);
			particlesPositions.push_back(p.position.z);
		}
	}

	void updateParticlesPositon() {
		int i = 0;
		for (Particle p : particles) {
			particlesPositions.at(i) = p.position.x;
			particlesPositions.at(i + 1) = p.position.y;
			particlesPositions.at(i + 2) = p.position.z;
			i += 3;
		}
	}

	void updateParticles() {
		Particles::updateParticles(0, particles.size(), &particlesPositions[0]);
	}
};



ParticleSystem* sistema;
std::vector<ForceActuator*> forces;

// Boolean variables allow to show/hide the primitives
bool renderSphere = true;
bool renderCapsule = true;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();


	if (renderSphere)
		Sphere::drawSphere();
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = sistema->particles.size();
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}

	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
		Fiber::drawFiber();

	if (renderCube)
		Cube::drawCube();
}


void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Text("");

		if (ImGui::Button("Restart"))
			restart = true;
		ImGui::SameLine();
		ImGui::Checkbox("Pause", &pause);

		ImGui::StyleColorsDark();

		ImGui::Text("\nSphere parameters");
		ImGui::SliderFloat("Sphere mass", &sphereMass, 0.0f, 100.f, "ratio = %.3f");
		ImGui::InputFloat3("Sphere Position", spherePosition);
		ImGui::InputFloat("Sphere Radius", &sphereRadius);

		ImGui::Text("\nSphere parameters");
		ImGui::InputFloat3("Capusle Position A", CapsulePositionA);
		ImGui::InputFloat3("Capsule Position B", CapsulePositionB);
		ImGui::InputFloat("Capsule Radius", &capsuleRadius);

		ImGui::Text("\nConstants");
		ImGui::Checkbox("Use Gravity", &useGravity);
		ImGui::InputFloat3("Gravity", gravity);
		ImGui::SliderFloat("Elastic coefficient", &elasticCoeficient, 0.0f, 1.f, "ratio = %.3f");
		ImGui::SliderFloat("Friction coefficient", &frictionCoeficient, 0.0f, 1.f, "ratio = %.3f");


	}
	// .........................
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);

	}
}

glm::vec3 computeForces(float mass, const glm::vec3& position, const std::vector<ForceActuator*>& force_acts) {
	glm::vec3 force = glm::vec3(0);
	for (ForceActuator* f : force_acts) {
		force += f->computeForce(mass, position);

	}
	return force;
}

void euler(float dt, ParticleSystem& particles, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts) {
	for (auto& p : particles.particles) {
		p.old_pos = p.position;
		p.old_vel = p.velocity;

		p.position = p.old_pos + dt * p.old_vel;
		p.velocity = p.old_vel + dt * computeForces(p.mass, p.position, force_acts);
	}
}

PlaneCol botomPlane(glm::vec3(0), glm::vec3(0, 1, 0));

void PhysicsInit() {
	// Do your initialization code here...
	sistema = new ParticleSystem(5000);
	forces.push_back(new GravityForce());
	glm::vec3 sp = glm::vec3(spherePosition[0], spherePosition[1], spherePosition[2]);
	Sphere::updateSphere(sp, sphereRadius);
	forces.push_back(new PositionalGravityForce(10.f, sp, 6.67f));

	// ...................................
}

void PhysicsUpdate(float dt) {
	//update sphere
	glm::vec3 sp = glm::vec3(spherePosition[0], spherePosition[1], spherePosition[2]);
	Sphere::updateSphere(sp, sphereRadius);
	dynamic_cast<PositionalGravityForce*>(forces.at(1))->updatePos(sp);
	
	//Update Capsule
	glm::vec3 cpA = glm::vec3(CapsulePositionA[0], CapsulePositionA[1], CapsulePositionA[2]);
	glm::vec3 cpB = glm::vec3(CapsulePositionB[0], CapsulePositionB[1], CapsulePositionB[2]);
	Capsule::updateCapsule(cpA, cpB, capsuleRadius);


	if (!pause) {

		// Do your update code here...
		dynamic_cast<PositionalGravityForce*>(forces.at(1))->updateMass(sphereMass);
		dynamic_cast<GravityForce*>(forces.at(0))->updateGravity(gravity);

		euler(dt, *sistema, std::vector<Collider*>(), forces);
		
		for (Particle& p : sistema->particles) {
			if (botomPlane.checkCollision(p.old_pos, p.position)) {
				botomPlane.computeCollision(p.old_pos, p.old_vel, p.position, p.velocity);
			}
		}


		sistema->updateParticlesPositon();
		sistema->updateParticles();
		// ...........................
	}

	if (restart){
		restart = false;
		pause = false;
		sistema = new ParticleSystem(5000);
	}

}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}