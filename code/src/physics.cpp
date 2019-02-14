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


class Particle {
public:
	Particle(glm::vec3 _position, glm::vec3 _velocity, float _mass) : position(_position), velocity(_velocity), mass(_mass) {}
	glm::vec3 position;
	glm::vec3 velocity;
	float mass;
};

class ParticleSystem {
public:
	std::vector<Particle> particles;
	std::vector<float> particlesPositions;

	ParticleSystem(int count) {
		for (int i = 0; i < count; i++) {
			particles.push_back(Particle(
				glm::vec3((((rand() % 1000))/100.f) - 5, ((rand() % 500)/100.f) + 5, ((rand() % 1000)/100.f) - 5),///pos
				glm::vec3(((rand() % 11)), (rand() % 11), (rand() % 11)),///vel
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

void PhysicsInit() {
	// Do your initialization code here...
	sistema = new ParticleSystem(5000);
	// ...................................
}

void PhysicsUpdate(float dt) {
	// Do your update code here...

	sistema->updateParticlesPositon();
	sistema->updateParticles();
	// ...........................
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}