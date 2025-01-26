// File: main.cpp
#include "cube_model.hpp"
#include "fps_camera.hpp"
#include "plane_model.hpp"
#include "shader.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <reactphysics3d/reactphysics3d.h>

#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <sstream>
#include <vector>

// ReactPhysics3D namespace
using namespace reactphysics3d;

struct Object {
    RigidBody* rigidBody;
    Transform prevTransform;
    glm::vec3 scale;
    std::shared_ptr<Model> model;
};

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void CursorPosCallback(GLFWwindow* window, double xposIn, double yposIn);

void ProcessInput(GLFWwindow* window, float deltaTime);
void ResetWorld();
void SpawnCubes(int count);
Object CreateCube(const Vector3& position, const Quaternion& rotation, const float scale);
void Render(const Shader& shader, float interpolationFactor);
void Shoot();

PhysicsCommon PhysicsManager;
PhysicsWorld* World;
FPSCamera Camera;
std::shared_ptr<CubeModel> Cube;
std::shared_ptr<PlaneModel> Floor;
std::vector<Object> Objects;

bool FirstMouse = true;
float LastX, LastY;

struct Settings
{
    std::string WindowTitle = "OpenGL Physics";
    int WindowWidth = 800;
    int WindowHeight = 600;
    int WindowPositionX = 0;
    int WindowPositionY = 0;
    bool FullScreen = false;
    float FOV = 75.0f;
    float WorldSize = 100.0f;
    glm::vec3 LightDir = glm::normalize(glm::vec3(0.5f, 1.0f, 1.0f));
    glm::vec3 LightColor = glm::vec3(1.0f, 1.0f, 0.8f);
    glm::vec3 AmbientColor = glm::vec3(1.0f, 1.0f, 1.0f);
    float AmbientIntensity = 0.5f;
    float SpecularShininess = 32.0f;
    float SpecularIntensity = 0.5;
    int NumCubes = 100;
} Settings;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);
#endif

    // glfw window creation
    // --------------------
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    GLFWwindow* window = nullptr;
    if (Settings.FullScreen) {
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        window = glfwCreateWindow(mode->width, mode->height, Settings.WindowTitle.c_str(), monitor, nullptr);
        Settings.WindowWidth = mode->width;
        Settings.WindowHeight = mode->height;
    }
    else
    {
        window = glfwCreateWindow(Settings.WindowWidth, Settings.WindowHeight, Settings.WindowTitle.c_str(), nullptr, nullptr);
        glfwGetWindowSize(window, &Settings.WindowWidth, &Settings.WindowHeight);
        glfwGetWindowPos(window, &Settings.WindowPositionX, &Settings.WindowPositionY);
    }

    if (window == nullptr)
    {
        std::cerr << "ERROR::GLFW: Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // disable vsync
    glfwSwapInterval(0);

    glfwSetFramebufferSizeCallback(window, FramebufferSizeCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetKeyCallback(window, KeyCallback);
    glfwSetCursorPosCallback(window, CursorPosCallback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "ERROR::GLAD: Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // world setup
    World = PhysicsManager.createPhysicsWorld();
    World->setGravity({ 0.0f, -9.8f, 0.0f });

    Cube = std::make_shared<CubeModel>("assets/texture_05.png");
    Floor = std::make_shared<PlaneModel>("assets/texture_05.png", Settings.WorldSize);

    ResetWorld();

    Camera.Position = glm::vec3(0.0f, 5.0f, 0.0f);
    Camera.MovementSpeed = 10.0f;
    Camera.FOV = Settings.FOV;
    Camera.AspectRatio = static_cast<GLfloat>(Settings.WindowWidth) / static_cast<GLfloat>(Settings.WindowHeight);

    Shader defaultShader("shaders/default.vs", "shaders/default.fs");
    defaultShader.Use();
    defaultShader.SetMat4("projection", Camera.GetProjectionMatrix());
    defaultShader.SetVec3("lightDir", Settings.LightDir);
    defaultShader.SetVec3("lightColor", Settings.LightColor);
    defaultShader.SetVec3("ambientColor", Settings.AmbientColor);
    defaultShader.SetFloat("ambientIntensity", Settings.AmbientIntensity);
    defaultShader.SetFloat("specularShininess", Settings.SpecularShininess);
    defaultShader.SetFloat("specularIntensity", Settings.SpecularIntensity);

    // setup OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // game loop
    // -----------
    float currentTime = 0.0f;
    float lastTime    = 0.0f;
    float lastFPSTime = 0.0f;
    float deltaTime   = 0.0f;
    const float simulationTimeStep = 1.0f / 60.0f;
    float accumulator = 0.0f;
    float factor      = 0.0f;
    int fpsCount = 0;
    std::stringstream fps;

    while (!glfwWindowShouldClose(window))
    {
        // calculate deltaTime and FPS
        // ---------------------------
        currentTime = glfwGetTime();
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        accumulator += deltaTime;
        // fps counter
        fpsCount++;
        if ((currentTime - lastFPSTime) >= 1.0f)
        {
            fps.str(std::string());
            fps << fpsCount;
            fpsCount = 0;
            lastFPSTime = currentTime;
        }

        // input
        // -----
        ProcessInput(window, deltaTime);

        // update
        // ------
        while (accumulator >= simulationTimeStep)
        {
            // Update the Dynamics world with a constant time step
            World->update(simulationTimeStep);
            // Decrease the accumulated time
            accumulator -= simulationTimeStep;
        }
        factor = accumulator / simulationTimeStep;

        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Render(defaultShader, factor);

        // display FPS in window title
        glfwSetWindowTitle(window, (Settings.WindowTitle + " - " + std::to_string(Objects.size()) + " Objects - FPS: " + fps.str()).c_str());

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    PhysicsManager.destroyPhysicsWorld(World);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void FramebufferSizeCallback(GLFWwindow* /* window */, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever a mouse button is clicked, this callback is called
// -----------------------------------------------------------------
void MouseButtonCallback(GLFWwindow* window, int button, int action, int /* mods */)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
        Shoot();
}

// glfw: whenever a keyboard key is pressed, this callback is called
// -----------------------------------------------------------------
void KeyCallback(GLFWwindow* window, int key, int /* scancode */, int action, int /* mods */)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    else if (key == GLFW_KEY_R && action == GLFW_PRESS)
        ResetWorld();
    else if (key == GLFW_KEY_T && action == GLFW_PRESS)
        SpawnCubes(Settings.NumCubes);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void CursorPosCallback(GLFWwindow* /* window */, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (FirstMouse)
    {
        LastX = xpos;
        LastY = ypos;
        FirstMouse = false;
    }

    float xoffset = xpos - LastX;
    float yoffset = LastY - ypos; // reversed since y-coordinates go from bottom to top

    LastX = xpos;
    LastY = ypos;

    Camera.ProcessMouseMovement(xoffset, yoffset);
}

void ProcessInput(GLFWwindow* window, float deltaTime)
{
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        Camera.Move(MOVE_FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        Camera.Move(MOVE_BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        Camera.Move(MOVE_LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        Camera.Move(MOVE_RIGHT, deltaTime);
}

void SpawnCubes(int count)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<float> randPosition(-Settings.WorldSize / 2.0f, Settings.WorldSize / 2.0f);
    std::uniform_real_distribution<float> randScale(0.5f, 3.0f);

    for (int i = 0; i < count; ++i)
    {
        float scale = randScale(generator);
        Vector3 position(randPosition(generator), Settings.WorldSize, randPosition(generator));
        Objects.push_back(CreateCube(position, Quaternion::identity(), scale));
    }
}

Object CreateCube(const Vector3& position, const Quaternion& rotation, const float scale)
{
    Transform transform(position, rotation);
    BoxShape* boxShape = PhysicsManager.createBoxShape({ scale / 2.0f, scale / 2.0f, scale / 2.0f });
    RigidBody* rigidBody = World->createRigidBody(transform);
    rigidBody->addCollider(boxShape, Transform::identity());

    return Object{ .rigidBody = rigidBody, .prevTransform = transform, .scale = glm::vec3(scale), .model = Cube };
}

void ResetWorld()
{
    for (auto obj : Objects)
        World->destroyRigidBody(obj.rigidBody);
    Objects.clear();
    // floor
    Transform transform({ 0.0f, 0.0f, 0.0f }, Quaternion::identity());
    BoxShape* boxShape = PhysicsManager.createBoxShape({ Settings.WorldSize / 2.0f, 5.0f, Settings.WorldSize / 2.0f });
    RigidBody* rigidBody = World->createRigidBody(transform);
    rigidBody->setType(BodyType::STATIC);
    rigidBody->addCollider(boxShape, Transform({ 0.0f, -5.0f, 0.0f }, Quaternion::identity()));
    Objects.push_back({ .rigidBody = rigidBody, .prevTransform = transform, .scale = glm::vec3(1.0f), .model = Floor });
}

void Render(const Shader& shader, float interpolationFactor)
{
    shader.Use();
    shader.SetMat4("view", Camera.GetViewMatrix());
    shader.SetVec3("cameraPos", Camera.Position);
    for (auto& object : Objects)
    {
        const auto& currTransform = object.rigidBody->getTransform();

        Transform interpolatedTransform = Transform::interpolateTransforms(object.prevTransform, currTransform, interpolationFactor);
        const Vector3& position = interpolatedTransform.getPosition();
        const Quaternion& rotation = interpolatedTransform.getOrientation();

        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), { position.x, position.y, position.z });
        glm::mat4 rotationMatrix = glm::mat4(1.0f);
        rotationMatrix = glm::rotate(rotationMatrix, rotation.x, { 1.0f, 0.0f, 0.0f });
        rotationMatrix = glm::rotate(rotationMatrix, rotation.y, { 0.0f, 1.0f, 0.0f });
        rotationMatrix = glm::rotate(rotationMatrix, rotation.z, { 0.0f, 0.0f, 1.0f });
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), object.scale);

        glm::mat4 modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;

        shader.SetMat4("model", modelMatrix);
        object.model->Draw(shader);

        object.prevTransform = currTransform;
    }
}

void Shoot()
{
    Vector3 position(Camera.Position.x, Camera.Position.y, Camera.Position.z);
    Object obj = CreateCube(position, Quaternion::identity(), 1.0f);
    obj.rigidBody->setLinearVelocity(Vector3(Camera.Front.x, Camera.Front.y, Camera.Front.z) * 15.0f);
    Objects.push_back(obj);
}