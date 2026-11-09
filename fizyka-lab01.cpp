
// Sterowanie: mysz = kierunek lufy, strzałki lewo/prawo = suwak impulsu, LPM = strzał, R = usuń piłki
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <PxPhysicsAPI.h>
#include <cmath>
#include <cstdio>
#include <vector>
using namespace physx;

const float G = 9.81f;                              // grawitacja [m/s^2]
const float R = 0.0625f, DENSITY = 245.f;           // promień piłki [m] i gęstość [kg/m^3]
const float M = DENSITY * 4.18879f * R * R * R;     // masa kuli = gęstość * 4/3*pi*R^3 (ok. 0,25 kg)
const float HX = 2.0f, HY = 1.0f, HW = 0.125f, RIM = 0.015f;   // środek obręczy, połowa jej szerokości, promień końcówek
const PxVec3 start(0.25f, 0.225f, 0);               // oś obrotu armaty
const float L = 0.30f;                              // długość lufy 
const float PMIN = 0.1f, PMAX = 3.0f;               // zakres suwaka impulsu [N*s] 
float power = 0.6f;                                 // aktualny impuls startowy [N*s]

struct Ball { PxRigidDynamic* b; float prevX, prevY; bool scored; };

void circle(float x, float y, float r) {
    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i <= 32; i++) glVertex2f(x + r * cosf(i * 6.2832f / 32), y + r * sinf(i * 6.2832f / 32));
    glEnd();
}

int main() {
    PxDefaultAllocator alloc; PxDefaultErrorCallback err;
    PxFoundation* found = PxCreateFoundation(PX_PHYSICS_VERSION, alloc, err);
    PxTolerancesScale ts; ts.length = 0.25f; ts.speed = 2.5f;     // typowe wymiary i prędkości tej sceny (progi odbić, uśpienia)
    PxPhysics* phys = PxCreatePhysics(PX_PHYSICS_VERSION, *found, ts);
    PxSceneDesc d(phys->getTolerancesScale());
    d.gravity = PxVec3(0, -G, 0);
    d.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    d.filterShader = PxDefaultSimulationFilterShader;
    PxScene* scene = phys->createScene(d);
    PxMaterial* mat = phys->createMaterial(0.5f, 0.5f, 0.7f);   // tarcie stat., dyn., sprężystość

    auto addBox = [&](float x, float y, float hx, float hy) {   // statyczne przeszkody (podłoga, tablica)
        scene->addActor(*PxCreateStatic(*phys, PxTransform(PxVec3(x, y, 0)), PxBoxGeometry(hx, hy, 0.25f), *mat));
        };
    addBox(1.5f, -0.125f, 5, 0.125f);                // podłoga (górna krawędź y=0)
    addBox(HX + HW + 0.025f, HY + 0.15f, 0.0125f, 0.175f);   // tablica
    for (int i = 0; i < 2; i++) {                    // dwa końce obręczy jako małe kule
        float s = (i == 0) ? -HW : HW;
        scene->addActor(*PxCreateStatic(*phys, PxTransform(PxVec3(HX + s, HY, 0)), PxSphereGeometry(RIM), *mat));
    }

    std::vector<Ball> balls;                         // wszystkie wystrzelone piłki zostają na scenie

 
    glfwInit();
    GLFWwindow* w = glfwCreateWindow(1000, 600, "Rzut do kosza", nullptr, nullptr);
    glfwMakeContextCurrent(w);
    gladLoadGL(glfwGetProcAddress);
    glfwSwapInterval(1);
    glMatrixMode(GL_PROJECTION); glOrtho(-0.25, 3.25, -0.125, 1.975, -1, 1);   // widok z boku

    int score = 0; bool clickWas = false;
    double prevTime = glfwGetTime(); float acc = 0; 
    while (!glfwWindowShouldClose(w)) {
        glfwPollEvents();
        double now = glfwGetTime(); float dt = fminf((float)(now - prevTime), 0.1f); prevTime = now;

        // suwak impulsu: strzałki lewo/prawo (cały zakres w ok. 4 s)
        if (glfwGetKey(w, GLFW_KEY_LEFT))  power -= (PMAX - PMIN) / 4 * dt;
        if (glfwGetKey(w, GLFW_KEY_RIGHT)) power += (PMAX - PMIN) / 4 * dt;
        power = fminf(fmaxf(power, PMIN), PMAX);

        // kąt = kierunek od armaty do kursora
        double mx, my; int ww, wh;
        glfwGetCursorPos(w, &mx, &my); glfwGetWindowSize(w, &ww, &wh);
        float cx = -0.25f + (float)mx / ww * 3.5f, cy = 1.975f - (float)my / wh * 2.1f;
        float a = fminf(fmaxf(atan2f(cy - start.y, cx - start.x), 0.05f), 1.5f);   // lufa tylko w górę (3°..86°)
        PxVec3 muzzle = start + PxVec3(L * cosf(a), L * sinf(a), 0);               // koniec rury

        // wektor impulsu P = |P|(cos a, sin a); prędkość startowa v0 = P / m
        PxVec3 impulse(power * cosf(a), power * sinf(a), 0);

        bool click = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT);
        if (click && !clickWas) {                    // STRZAŁ: nowa piłka + siła startowa (impuls)
            PxRigidDynamic* b = PxCreateDynamic(*phys, PxTransform(muzzle), PxSphereGeometry(R), *mat, DENSITY);
            b->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z |   // ruch tylko w płaszczyźnie XY
                PxRigidDynamicLockFlag::eLOCK_ANGULAR_X | PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y);
            scene->addActor(*b);
            b->addForce(impulse, PxForceMode::eIMPULSE);
            balls.push_back({ b, muzzle.x, muzzle.y, false });
        }
        clickWas = click;
        if (glfwGetKey(w, GLFW_KEY_R)) {             // R = usuń wszystkie piłki
            for (Ball& k : balls) k.b->release();
            balls.clear();
        }

        // stały krok fizyki 1/60 s; trafienie sprawdzane po każdym kroku
        for (acc += dt; acc >= 1 / 60.f; acc -= 1 / 60.f) {
            scene->simulate(1 / 60.f); scene->fetchResults(true);
            for (Ball& k : balls) {                  // środek piłki przecina poziom obręczy z góry w dół między jej końcami
                PxVec3 p = k.b->getGlobalPose().p;
                if (!k.scored && k.prevY >= HY && p.y < HY) {
                    float t = (k.prevY - HY) / (k.prevY - p.y);          // interpolacja miejsca przecięcia
                    if (fabsf(k.prevX + t * (p.x - k.prevX) - HX) < HW) { score++; k.scored = true; }
                }
                k.prevX = p.x; k.prevY = p.y;
            }
        }

        // ---------- rysowanie ----------
        glClearColor(0.6f, 0.8f, 1, 1); glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glColor3f(0.3f, 0.6f, 0.3f); glRectf(-0.25f, -0.125f, 3.25f, 0);                       // podłoga
        glColor3f(1, 1, 1); glRectf(HX + HW + 0.0125f, HY - 0.025f, HX + HW + 0.0375f, HY + 0.325f); // tablica
        glColor3f(1, 0.3f, 0); glLineWidth(4);                                                  // obręcz
        glBegin(GL_LINES); glVertex2f(HX - HW, HY); glVertex2f(HX + HW, HY); glEnd();
        circle(HX - HW, HY, RIM); circle(HX + HW, HY, RIM);                                     // końce obręczy = kule kolizji
        // koło armaty
        glColor3f(0.25f, 0.15f, 0.05f); circle(start.x, 0.1375f, 0.125f);
        glColor3f(0.55f, 0.35f, 0.15f); circle(start.x, 0.1375f, 0.105f);
        glColor3f(0.25f, 0.15f, 0.05f); glLineWidth(3); glBegin(GL_LINES);
        for (int i = 0; i < 4; i++) {
            float t = i * 0.7854f; glVertex2f(start.x + 0.105f * cosf(t), 0.1375f + 0.105f * sinf(t));
            glVertex2f(start.x - 0.105f * cosf(t), 0.1375f - 0.105f * sinf(t));
        }
        glEnd();
        // lufa: zwężająca się rura, obracana o kąt a 
        glPushMatrix(); glTranslatef(start.x, start.y, 0); glRotatef(a * 57.2958f, 0, 0, 1);
        glColor3f(0.2f, 0.2f, 0.22f);                                                           // korpus
        glBegin(GL_QUADS); glVertex2f(-0.0875f, -0.075f); glVertex2f(L, -0.05f); glVertex2f(L, 0.05f); glVertex2f(-0.0875f, 0.075f); glEnd();
        circle(-0.0875f, 0, 0.075f);                                                            // tył lufy
        glColor3f(0.4f, 0.4f, 0.45f);                                                           // połysk
        glBegin(GL_QUADS); glVertex2f(-0.05f, 0.03f); glVertex2f(L - 0.025f, 0.0225f); glVertex2f(L - 0.025f, 0.035f); glVertex2f(-0.05f, 0.05f); glEnd();
        glColor3f(0.1f, 0.1f, 0.12f);                                                           // pierścienie
        glRectf(0.0625f, -0.07f, 0.1f, 0.07f); glRectf(0.2f, -0.0625f, 0.225f, 0.0625f);
        glRectf(L - 0.0375f, -0.0675f, L + 0.0125f, 0.0675f);                                   // wylot
        glPopMatrix();
        glColor3f(0.7f, 0.55f, 0.2f); circle(start.x, start.y, 0.025f);                         // oś 
        glColor3f(0.95f, 0.5f, 0.1f);                                                           // piłki
        for (Ball& k : balls) { PxVec3 p = k.b->getGlobalPose().p; circle(p.x, p.y, R); }

        // przewidywana parabola 
        PxVec3 v = impulse / M;
        glColor3f(0, 0, 0); glPointSize(4); glBegin(GL_POINTS);
        for (float t = 0; t < 1.5f; t += 0.03f)
            glVertex2f(muzzle.x + v.x * t, muzzle.y + v.y * t - 0.5f * G * t * t);
        glEnd();

        // suwak impulsu 
        float fill = 0.25f + (power - PMIN) / (PMAX - PMIN);
        glColor3f(0.2f, 0.2f, 0.2f); glRectf(0.25f, 1.8f, 1.25f, 1.85f);
        glColor3f(0.2f, 0.8f, 0.2f); glRectf(0.25f, 1.8f, fill, 1.85f);
        glColor3f(1, 1, 1);          glRectf(fill - 0.0125f, 1.775f, fill + 0.0125f, 1.875f);

        char title[256];
        snprintf(title, 256, "[mysz: kierunek] [LPM: strzal] [strzalki L/P: sila] [R: usun pilki]  impuls %.2f N*s  v0 %.2f m/s  pilki %d  punkty %d",
            power, v.magnitude(), (int)balls.size(), score);
        glfwSetWindowTitle(w, title);
        glfwSwapBuffers(w);
    }
    return 0;
}