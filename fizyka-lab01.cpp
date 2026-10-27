#include <glad/gl.h>        
#include <GLFW/glfw3.h>
#include <PxPhysicsAPI.h>
#include <vector>
#include <cmath>
using namespace physx;

const float R = 0.5f;           // promień kuli
const float SPREZYSTOSC = 0.8f; // współczynnik odbicia (0 - brak, 1 - mocna sprężystość)
const float TLUMIENIE = 0.1f;   // tłumienie ruchu

PxPhysics* fizyka;
PxScene* scena;
PxMaterial* mat;
std::vector<PxRigidDynamic*> kule;

// Nowa kula ma tą samą sprężystość
void dodajKule(float x, float y) {
    PxRigidDynamic* b = fizyka->createRigidDynamic(PxTransform(x, y, 0));
    //wykrywanie kolizji
    PxRigidActorExt::createExclusiveShape(*b, PxSphereGeometry(R), *mat);
    //masa
    PxRigidBodyExt::updateMassAndInertia(*b, 1.f);
    b->setLinearDamping(TLUMIENIE);
    b->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z); // ruch tylko w 2D
    scena->addActor(*b);
    kule.push_back(b);
}

// Kliknięcie myszą -> nowa kula w miejscu kursora (okno 700x700, widok x:-5..5, y:-1..9)
void klik(GLFWwindow* w, int btn, int act, int) {
    if (btn == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(w, &mx, &my);
        dodajKule(-5 + 10 * (float)mx / 700, 9 - 10 * (float)my / 700);
    }
}

// Rysowanie okręgu
void kolo(float x, float y) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 32; i++)
        glVertex2f(x + R * cosf(i * 0.19635f), y + R * sinf(i * 0.19635f));
    glEnd();
}

int main() {
    // PhysX: inicjalizacja
    static PxDefaultAllocator alok;
    static PxDefaultErrorCallback err;
    PxFoundation* fund = PxCreateFoundation(PX_PHYSICS_VERSION, alok, err);
    fizyka = PxCreatePhysics(PX_PHYSICS_VERSION, *fund, PxTolerancesScale());

    PxSceneDesc sd(fizyka->getTolerancesScale());
    //grawitacja
    sd.gravity = PxVec3(0, -9.81f, 0);
    sd.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    sd.filterShader = PxDefaultSimulationFilterShader;
    scena = fizyka->createScene(sd);

    // Materiał: tarcie statyczne, tarcie dynamiczne, sprężystość
    mat = fizyka->createMaterial(0.5f, 0.5f, SPREZYSTOSC);

    // Ściany: podłoga i dwie ściany boczne
    scena->addActor(*PxCreatePlane(*fizyka, PxPlane(0, 1, 0, 0), *mat));  // y = 0
    scena->addActor(*PxCreatePlane(*fizyka, PxPlane(1, 0, 0, 4), *mat));  // x = -4
    scena->addActor(*PxCreatePlane(*fizyka, PxPlane(-1, 0, 0, 4), *mat)); // x = 4

 
    glfwInit();
    GLFWwindow* okno = glfwCreateWindow(700, 700, "Sprezystosc", nullptr, nullptr);
    glfwMakeContextCurrent(okno);
    glfwSwapInterval(1);              // 60 FPS 
    gladLoadGL(glfwGetProcAddress);   
    glfwSetMouseButtonCallback(okno, klik);
    glMatrixMode(GL_PROJECTION);
    glOrtho(-5, 5, -1, 9, -1, 1);     // widok 2D z boku

    dodajKule(0.f, 5.f);              // pierwsza kula

    while (!glfwWindowShouldClose(okno)) {
        // Symulacja fizyki
        scena->simulate(1.f / 60.f);
        scena->fetchResults(true);

        // Rysowanie
        glClear(GL_COLOR_BUFFER_BIT);
        glLineWidth(3);               // ściany
        glBegin(GL_LINE_STRIP);
        glVertex2f(-4, 9); glVertex2f(-4, 0); glVertex2f(4, 0); glVertex2f(4, 9);
        glEnd();
        glLineWidth(1);               // kule
        for (auto k : kule)
            kolo(k->getGlobalPose().p.x, k->getGlobalPose().p.y);

        glfwSwapBuffers(okno);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}