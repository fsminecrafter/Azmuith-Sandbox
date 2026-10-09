#include <PxPhysicsAPI.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <extensions/PxFixedJoint.h>
#include <extensions/PxRigidBodyExt.h>
#if defined(AZMUITH_ENABLE_PHYSX_GPU)
#include <gpu/PxGpu.h>
#endif
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_vulkan.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace physx;
using Json = nlohmann::json;

void requireVk(VkResult result) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error("Vulkan operation failed with code " + std::to_string(result));
    }
}

struct QueueFamilies {
    uint32_t graphics = UINT32_MAX;
    uint32_t present = UINT32_MAX;
    bool complete() const { return graphics != UINT32_MAX && present != UINT32_MAX; }
};

enum class ObjectKind {
    Box,
    Sphere,
    Cylinder,
    Gear,
    Axle,
    Bearing,
    StructureCube,
    StructureLink,
    Glue
};

enum class ToolMode { Select, Box, Sphere, Cylinder, Gear, Axle, Bearing, Structure, Glue };
enum class GizmoMode { Move, Rotate, Scale };

ObjectKind objectKindForTool(ToolMode tool) {
    switch (tool) {
    case ToolMode::Box: return ObjectKind::Box;
    case ToolMode::Sphere: return ObjectKind::Sphere;
    case ToolMode::Cylinder: return ObjectKind::Cylinder;
    case ToolMode::Gear: return ObjectKind::Gear;
    case ToolMode::Axle: return ObjectKind::Axle;
    case ToolMode::Bearing: return ObjectKind::Bearing;
    case ToolMode::Structure: return ObjectKind::StructureCube;
    case ToolMode::Glue: return ObjectKind::Glue;
    case ToolMode::Select: return ObjectKind::Box;
    }
    return ObjectKind::Box;
}

const char* objectKindName(ObjectKind kind) {
    switch (kind) {
    case ObjectKind::Box: return "Box";
    case ObjectKind::Sphere: return "Sphere";
    case ObjectKind::Cylinder: return "Cylinder";
    case ObjectKind::Gear: return "Gear";
    case ObjectKind::Axle: return "Axle";
    case ObjectKind::Bearing: return "Bearing";
    case ObjectKind::StructureCube: return "Structure cube";
    case ObjectKind::StructureLink: return "Structure link";
    case ObjectKind::Glue: return "Glue";
    }
    return "Object";
}

struct GearSettings {
    int teeth = 14;
    float module = 0.12f;
    float thickness = 0.28f;
    float boreRadius = 0.22f;
};

struct Body {
    PxRigidDynamic* actor = nullptr;
    glm::vec3 color;
    std::string name;
    ObjectKind kind = ObjectKind::Box;
    PxVec3 halfExtents = PxVec3(0.5f);
    GearSettings gear;
};

struct JointRecord {
    size_t first = 0;
    size_t second = 0;
    PxTransform localFirst;
    PxTransform localSecond;
};

PxFilterFlags editorFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                                 PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                                 PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize) {
    const PxFilterFlags result = PxDefaultSimulationFilterShader(attributes0, filterData0,
        attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
    if (result == PxFilterFlag::eDEFAULT) pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
    return result;
}

struct SceneVertex {
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 normal;
};

void appendObjectTriangles(const Body& body, std::vector<SceneVertex>& vertices) {
    const PxTransform pose = body.actor->getGlobalPose();
    auto emit = [&](PxVec3 a, PxVec3 b, PxVec3 c, float brightness = 1.0f) {
        const PxVec3 worldA = pose.transform(a);
        const PxVec3 worldB = pose.transform(b);
        const PxVec3 worldC = pose.transform(c);
        PxVec3 worldNormal = (worldB - worldA).cross(worldC - worldA);
        if (worldNormal.magnitudeSquared() > 0.000001f) worldNormal.normalize();
        else worldNormal = PxVec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 color = body.color * brightness;
        const glm::vec3 normal(worldNormal.x, worldNormal.y, worldNormal.z);
        vertices.push_back({glm::vec3(worldA.x, worldA.y, worldA.z), color, normal});
        vertices.push_back({glm::vec3(worldB.x, worldB.y, worldB.z), color, normal});
        vertices.push_back({glm::vec3(worldC.x, worldC.y, worldC.z), color, normal});
    };

    if (body.kind == ObjectKind::Box || body.kind == ObjectKind::StructureCube) {
        constexpr std::array<std::array<int, 4>, 6> faces = {{{0, 1, 3, 2}, {4, 6, 7, 5},
            {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}}};
        constexpr std::array<float, 6> light = {0.80f, 0.57f, 0.70f, 0.94f, 0.63f, 0.78f};
        std::array<PxVec3, 8> corners;
        for (int index = 0; index < 8; ++index) {
            corners[index] = PxVec3((index & 1) ? body.halfExtents.x : -body.halfExtents.x,
                (index & 2) ? body.halfExtents.y : -body.halfExtents.y,
                (index & 4) ? body.halfExtents.z : -body.halfExtents.z);
        }
        for (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
            const auto& face = faces[faceIndex];
            emit(corners[face[0]], corners[face[1]], corners[face[2]], light[faceIndex]);
            emit(corners[face[0]], corners[face[2]], corners[face[3]], light[faceIndex]);
        }
        return;
    }

    if (body.kind == ObjectKind::Sphere || body.kind == ObjectKind::Glue) {
        constexpr int rings = 10;
        constexpr int segments = 18;
        const float radius = body.halfExtents.x;
        for (int ring = 0; ring < rings; ++ring) {
            const float lower = -1.5707963f + 3.1415926f * ring / rings;
            const float upper = -1.5707963f + 3.1415926f * (ring + 1) / rings;
            for (int segment = 0; segment < segments; ++segment) {
                const float angle0 = 6.2831853f * segment / segments;
                const float angle1 = 6.2831853f * (segment + 1) / segments;
                auto spherePoint = [&](float latitude, float longitude) {
                    return PxVec3(radius * std::cos(latitude) * std::cos(longitude),
                        radius * std::sin(latitude), radius * std::cos(latitude) * std::sin(longitude));
                };
                const PxVec3 a = spherePoint(lower, angle0);
                const PxVec3 b = spherePoint(upper, angle0);
                const PxVec3 c = spherePoint(upper, angle1);
                const PxVec3 d = spherePoint(lower, angle1);
                emit(a, b, c, 0.78f);
                emit(a, c, d, 0.84f);
            }
        }
        return;
    }

    if (body.kind == ObjectKind::Cylinder || body.kind == ObjectKind::Axle || body.kind == ObjectKind::StructureLink) {
        constexpr int segments = 24;
        const float halfLength = body.halfExtents.x;
        const float radius = body.halfExtents.y;
        for (int segment = 0; segment < segments; ++segment) {
            const float angle0 = 6.2831853f * segment / segments;
            const float angle1 = 6.2831853f * (segment + 1) / segments;
            const PxVec3 a(-halfLength, radius * std::cos(angle0), radius * std::sin(angle0));
            const PxVec3 b(halfLength, radius * std::cos(angle0), radius * std::sin(angle0));
            const PxVec3 c(halfLength, radius * std::cos(angle1), radius * std::sin(angle1));
            const PxVec3 d(-halfLength, radius * std::cos(angle1), radius * std::sin(angle1));
            emit(a, b, c, 0.86f);
            emit(a, c, d, 0.86f);
            emit(PxVec3(-halfLength, 0, 0), d, a, 0.65f);
            emit(PxVec3(halfLength, 0, 0), b, c, 0.96f);
        }
        return;
    }

    if (body.kind == ObjectKind::Gear || body.kind == ObjectKind::Bearing) {
        const bool gear = body.kind == ObjectKind::Gear;
        const int teeth = gear ? std::clamp(body.gear.teeth, 8, 64) : 32;
        const int segments = gear ? teeth * 4 : 32;
        const float outerRadius = gear ? body.gear.module * (teeth + 2.0f) * 0.5f : body.halfExtents.x;
        const float rootRadius = gear ? body.gear.module * (teeth - 1.5f) * 0.5f : outerRadius;
        const float innerRadius = gear ? std::min(body.gear.boreRadius, rootRadius * 0.8f) : outerRadius * 0.48f;
        const float halfHeight = gear ? body.gear.thickness * 0.5f : body.halfExtents.y;
        auto outerPoint = [&](int index, float y) {
            const float angle = 6.2831853f * index / segments;
            const float radius = gear && (index % 4 == 1 || index % 4 == 2) ? outerRadius : rootRadius;
            return PxVec3(radius * std::cos(angle), y, radius * std::sin(angle));
        };
        for (int segment = 0; segment < segments; ++segment) {
            const PxVec3 outerTop0 = outerPoint(segment, halfHeight);
            const PxVec3 outerTop1 = outerPoint(segment + 1, halfHeight);
            const PxVec3 outerBottom0 = outerPoint(segment, -halfHeight);
            const PxVec3 outerBottom1 = outerPoint(segment + 1, -halfHeight);
            const float angle0 = 6.2831853f * segment / segments;
            const float angle1 = 6.2831853f * (segment + 1) / segments;
            const PxVec3 innerTop0(innerRadius * std::cos(angle0), halfHeight, innerRadius * std::sin(angle0));
            const PxVec3 innerTop1(innerRadius * std::cos(angle1), halfHeight, innerRadius * std::sin(angle1));
            const PxVec3 innerBottom0(innerRadius * std::cos(angle0), -halfHeight, innerRadius * std::sin(angle0));
            const PxVec3 innerBottom1(innerRadius * std::cos(angle1), -halfHeight, innerRadius * std::sin(angle1));
            emit(innerTop0, outerTop0, outerTop1, 0.94f);
            emit(innerTop0, outerTop1, innerTop1, 0.94f);
            emit(innerBottom0, outerBottom1, outerBottom0, 0.62f);
            emit(innerBottom0, innerBottom1, outerBottom1, 0.62f);
            emit(outerBottom0, outerBottom1, outerTop1, 0.76f);
            emit(outerBottom0, outerTop1, outerTop0, 0.76f);
            emit(innerBottom0, innerTop0, innerTop1, 0.55f);
            emit(innerBottom0, innerTop1, innerBottom1, 0.55f);
        }
    }
}

void appendGroundPlane(std::vector<SceneVertex>& vertices) {
    const glm::vec3 normal(0.0f, 1.0f, 0.0f);
    const glm::vec3 color(0.12f, 0.15f, 0.15f);
    const glm::vec3 a(-18.0f, 0.0f, -18.0f);
    const glm::vec3 b(-18.0f, 0.0f, 18.0f);
    const glm::vec3 c(18.0f, 0.0f, 18.0f);
    const glm::vec3 d(18.0f, 0.0f, -18.0f);
    vertices.insert(vertices.end(), {{a, color, normal}, {b, color, normal}, {c, color, normal},
        {a, color, normal}, {c, color, normal}, {d, color, normal}});
}

class PhysicsWorld : public PxSimulationEventCallback {
public:
    PhysicsWorld() {
        foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, allocator_, errorCallback_);
        if (!foundation_) throw std::runtime_error("PhysX foundation initialization failed");
        physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, PxTolerancesScale());
        if (!physics_) throw std::runtime_error("PhysX initialization failed");

    #if defined(AZMUITH_ENABLE_PHYSX_GPU)
        PxCudaContextManagerDesc cudaDesc;
        cudaManager_ = PxCreateCudaContextManager(*foundation_, cudaDesc, PxGetProfilerCallback());
        if (cudaManager_ && !cudaManager_->contextIsValid()) {
            cudaManager_->release();
            cudaManager_ = nullptr;
        }
    #endif

        dispatcher_ = PxDefaultCpuDispatcherCreate(2);
        PxSceneDesc sceneDesc(physics_->getTolerancesScale());
        sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
        sceneDesc.cpuDispatcher = dispatcher_;
        sceneDesc.filterShader = editorFilterShader;
        sceneDesc.simulationEventCallback = this;
    #if defined(AZMUITH_ENABLE_PHYSX_GPU)
        if (cudaManager_) {
            sceneDesc.cudaContextManager = cudaManager_;
            sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        }
    #endif
        scene_ = physics_->createScene(sceneDesc);
    #if defined(AZMUITH_ENABLE_PHYSX_GPU)
        if (!scene_ && cudaManager_) {
            cudaManager_->release();
            cudaManager_ = nullptr;
            sceneDesc.cudaContextManager = nullptr;
            sceneDesc.flags.clear(PxSceneFlag::eENABLE_GPU_DYNAMICS);
            scene_ = physics_->createScene(sceneDesc);
        }
    #endif
        if (!scene_) throw std::runtime_error("PhysX scene initialization failed");

        material_ = physics_->createMaterial(0.55f, 0.45f, 0.18f);
        PxRigidStatic* ground = PxCreatePlane(*physics_, PxPlane(0, 1, 0, 0), *material_);
        scene_->addActor(*ground);
        reset();
    }

    ~PhysicsWorld() {
        for (PxJoint* joint : joints_) joint->release();
        if (scene_) scene_->release();
#if defined(AZMUITH_ENABLE_PHYSX_GPU)
        if (cudaManager_) cudaManager_->release();
#endif
        if (material_) material_->release();
        if (dispatcher_) dispatcher_->release();
        if (physics_) physics_->release();
        if (foundation_) foundation_->release();
    }

    void reset() {
        clearObjects();
        for (int row = 0; row < 4; ++row) {
            spawn(PxVec3(0.0f, 0.6f + row * 1.08f, 0.0f), row == 0 ? 1.0f : 1.25f);
        }
        spawn(PxVec3(-2.5f, 1.0f, 0.0f), 0.8f);
    }

    bool createStructure(PxVec3 start, PxVec3 end) {
        const PxVec3 difference = end - start;
        const float length = difference.magnitude();
        if (length < 0.75f) return false;
        const size_t first = createObject(ObjectKind::StructureCube, start, 0.38f);
        const size_t second = createObject(ObjectKind::StructureCube, end, 0.38f);
        const size_t link = createObject(ObjectKind::StructureLink, (start + end) * 0.5f, length * 0.5f);
        if (first >= bodies_.size() || second >= bodies_.size() || link >= bodies_.size()) return false;

        PxTransform linkPose = bodies_[link].actor->getGlobalPose();
        const PxVec3 localAxis(1.0f, 0.0f, 0.0f);
        const PxVec3 direction = difference.getNormalized();
        const PxVec3 rotationAxis = localAxis.cross(direction);
        const float alignment = std::clamp(localAxis.dot(direction), -1.0f, 1.0f);
        if (alignment < -0.999999f) {
            linkPose.q = PxQuat(std::acos(-1.0f), PxVec3(0.0f, 1.0f, 0.0f));
        } else if (alignment > 0.999999f) {
            linkPose.q = PxQuat(0.0f, localAxis);
        } else {
            linkPose.q = PxQuat(std::acos(alignment), rotationAxis.getNormalized());
        }
        bodies_[link].actor->setGlobalPose(linkPose);
        return createFixedJoint(first, link, PxTransform(PxIdentity), PxTransform(PxVec3(-length * 0.5f, 0, 0))) &&
            createFixedJoint(second, link, PxTransform(PxIdentity), PxTransform(PxVec3(length * 0.5f, 0, 0)));
    }

    void spawn(PxVec3 position = PxVec3(1.5f, 5.0f, 0.0f), float halfExtent = 0.5f) {
        createObject(ObjectKind::Box, position, halfExtent);
    }

    size_t createObject(ObjectKind kind, PxVec3 position, float size = 0.5f,
                        const GearSettings& gear = GearSettings{}) {
        PxTransform pose(position);
        PxRigidDynamic* actor = nullptr;
        PxVec3 halfExtents(size);
        switch (kind) {
        case ObjectKind::Sphere:
        case ObjectKind::Glue:
            actor = PxCreateDynamic(*physics_, pose, PxSphereGeometry(size), *material_, 1.0f);
            halfExtents = PxVec3(size);
            break;
        case ObjectKind::Cylinder:
        case ObjectKind::Axle:
        case ObjectKind::StructureLink:
            pose.q = PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f));
            actor = PxCreateDynamic(*physics_, pose,
                PxCapsuleGeometry(std::max(size * 0.16f, 0.08f), size), *material_, 1.0f);
            halfExtents = PxVec3(size, std::max(size * 0.16f, 0.08f), std::max(size * 0.16f, 0.08f));
            break;
        case ObjectKind::Gear: {
            const float radius = gear.module * static_cast<float>(gear.teeth) * 0.5f;
            halfExtents = PxVec3(radius, gear.thickness * 0.5f, radius);
            actor = makeCompoundRing(pose, gear.module * (gear.teeth + 2.0f) * 0.5f,
                gear.boreRadius, gear.thickness * 0.5f, gear.teeth, gear.module, true);
            break;
        }
        case ObjectKind::Bearing:
            halfExtents = PxVec3(size, std::max(size * 0.2f, 0.08f), size);
            actor = makeCompoundRing(pose, size, size * 0.42f, halfExtents.y, 0, 0.0f, false);
            break;
        case ObjectKind::Box:
        case ObjectKind::StructureCube:
            actor = PxCreateDynamic(*physics_, pose,
                PxBoxGeometry(size, size, size), *material_, 1.0f);
            halfExtents = PxVec3(size);
            break;
        }
        if (!actor) return bodies_.size();
        actor->setAngularDamping(0.12f);
        actor->setLinearDamping(0.02f);
        scene_->addActor(*actor);
        const std::array<glm::vec3, 5> palette = {
            glm::vec3(0.94f, 0.39f, 0.24f), glm::vec3(0.91f, 0.72f, 0.36f),
            glm::vec3(0.38f, 0.72f, 0.65f), glm::vec3(0.38f, 0.55f, 0.79f),
            glm::vec3(0.77f, 0.48f, 0.65f)};
        const size_t index = bodies_.size();
        bodies_.push_back({actor, palette[index % palette.size()],
            std::string(objectKindName(kind)) + " " + std::to_string(index + 1), kind, halfExtents, gear});
        return index;
    }

    void step(float dt) {
        scene_->simulate(dt);
        scene_->fetchResults(true);
        processGlueContacts();
    }

    void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
    void onWake(PxActor**, PxU32) override {}
    void onSleep(PxActor**, PxU32) override {}
    void onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 count) override {
        for (PxU32 index = 0; index < count; ++index) {
            if (pairs[index].events.isSet(PxPairFlag::eNOTIFY_TOUCH_FOUND))
                pendingContacts_.emplace_back(header.actors[0], header.actors[1]);
        }
    }
    void onTrigger(PxTriggerPair*, PxU32) override {}
    void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}

    void setGravity(float gravity) { scene_->setGravity(PxVec3(0.0f, gravity, 0.0f)); }
    float gravity() const { return scene_->getGravity().y; }
    size_t bodyCount() const { return bodies_.size(); }
    const std::vector<Body>& bodies() const { return bodies_; }
    std::vector<Body>& bodies() { return bodies_; }

    void clearObjects() {
        for (PxJoint* joint : joints_) joint->release();
        joints_.clear();
        jointRecords_.clear();
        pendingContacts_.clear();
        for (Body& body : bodies_) {
            scene_->removeActor(*body.actor);
            body.actor->release();
        }
        bodies_.clear();
    }

    bool saveScene(const std::string& path, std::string& error) const {
        try {
            Json data;
            data["version"] = 1;
            data["gravity"] = gravity();
            data["objects"] = Json::array();
            for (const Body& body : bodies_) {
                const PxTransform pose = body.actor->getGlobalPose();
                const PxVec3 velocity = body.actor->getLinearVelocity();
                const PxVec3 angularVelocity = body.actor->getAngularVelocity();
                data["objects"].push_back({
                    {"kind", static_cast<int>(body.kind)},
                    {"name", body.name},
                    {"color", {body.color.r, body.color.g, body.color.b}},
                    {"pose", transformToJson(pose)},
                    {"halfExtents", {body.halfExtents.x, body.halfExtents.y, body.halfExtents.z}},
                    {"velocity", {velocity.x, velocity.y, velocity.z}},
                    {"angularVelocity", {angularVelocity.x, angularVelocity.y, angularVelocity.z}},
                    {"gear", {{"teeth", body.gear.teeth}, {"module", body.gear.module},
                        {"thickness", body.gear.thickness}, {"boreRadius", body.gear.boreRadius}}}});
            }
            data["joints"] = Json::array();
            for (const JointRecord& joint : jointRecords_) {
                data["joints"].push_back({{"first", joint.first}, {"second", joint.second},
                    {"localFirst", transformToJson(joint.localFirst)},
                    {"localSecond", transformToJson(joint.localSecond)}});
            }
            std::ofstream file(path);
            if (!file) {
                error = "Unable to open scene file for writing: " + path;
                return false;
            }
            file << data.dump(2) << '\n';
            if (!file) {
                error = "Unable to write scene file: " + path;
                return false;
            }
            error.clear();
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        }
    }

    bool loadScene(const std::string& path, std::string& error) {
        try {
            std::ifstream file(path);
            if (!file) {
                error = "Unable to open scene file: " + path;
                return false;
            }
            Json data;
            file >> data;
            if (data.value("version", 0) != 1 || !data.contains("objects") || !data["objects"].is_array()) {
                error = "Unsupported or invalid scene file";
                return false;
            }

            clearObjects();
            setGravity(data.value("gravity", -9.81f));
            for (const Json& object : data["objects"]) {
                const int kindValue = object.at("kind").get<int>();
                if (kindValue < 0 || kindValue > static_cast<int>(ObjectKind::Glue))
                    throw std::runtime_error("Scene contains an unknown object kind");
                const ObjectKind kind = static_cast<ObjectKind>(kindValue);
                GearSettings gear;
                if (object.contains("gear")) {
                    const Json& settings = object["gear"];
                    gear.teeth = settings.value("teeth", gear.teeth);
                    gear.module = settings.value("module", gear.module);
                    gear.thickness = settings.value("thickness", gear.thickness);
                    gear.boreRadius = settings.value("boreRadius", gear.boreRadius);
                }
                const PxTransform pose = transformFromJson(object.at("pose"));
                const float size = object.at("halfExtents").at(0).get<float>();
                const size_t index = createObject(kind, pose.p, size, gear);
                if (index >= bodies_.size()) throw std::runtime_error("PhysX failed to create a scene object");
                Body& body = bodies_[index];
                body.name = object.value("name", body.name);
                if (object.contains("color") && object["color"].size() == 3) {
                    body.color = glm::vec3(object["color"][0].get<float>(), object["color"][1].get<float>(),
                        object["color"][2].get<float>());
                }
                if (object.contains("halfExtents") && object["halfExtents"].size() == 3) {
                    const Json& extents = object["halfExtents"];
                    body.halfExtents = PxVec3(extents[0].get<float>(), extents[1].get<float>(), extents[2].get<float>());
                }
                body.actor->setGlobalPose(pose);
                if (object.contains("velocity") && object["velocity"].size() == 3) {
                    const Json& velocity = object["velocity"];
                    body.actor->setLinearVelocity(PxVec3(velocity[0].get<float>(), velocity[1].get<float>(), velocity[2].get<float>()));
                }
                if (object.contains("angularVelocity") && object["angularVelocity"].size() == 3) {
                    const Json& velocity = object["angularVelocity"];
                    body.actor->setAngularVelocity(PxVec3(velocity[0].get<float>(), velocity[1].get<float>(), velocity[2].get<float>()));
                }
            }
            if (data.contains("joints") && data["joints"].is_array()) {
                for (const Json& joint : data["joints"]) {
                    const size_t first = joint.at("first").get<size_t>();
                    const size_t second = joint.at("second").get<size_t>();
                    createFixedJoint(first, second, transformFromJson(joint.at("localFirst")),
                        transformFromJson(joint.at("localSecond")));
                }
            }
            error.clear();
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        }
    }

    bool raycast(PxVec3 origin, PxVec3 direction, size_t& bodyIndex) const {
        PxRaycastBuffer hit;
        if (!scene_->raycast(origin, direction, 1000.0f, hit) || !hit.hasBlock) return false;
        for (size_t index = 0; index < bodies_.size(); ++index) {
            if (bodies_[index].actor == hit.block.actor) {
                bodyIndex = index;
                return true;
            }
        }
        return false;
    }

    void setPosition(size_t index, PxVec3 position) {
        if (index >= bodies_.size()) return;
        PxTransform pose = bodies_[index].actor->getGlobalPose();
        pose.p = position;
        bodies_[index].actor->setGlobalPose(pose);
        bodies_[index].actor->wakeUp();
    }

    void rotate(size_t index, PxVec3 axis, float radians) {
        if (index >= bodies_.size()) return;
        PxTransform pose = bodies_[index].actor->getGlobalPose();
        pose.q = PxQuat(radians, axis.getNormalized()) * pose.q;
        bodies_[index].actor->setGlobalPose(pose);
        bodies_[index].actor->wakeUp();
    }

    void scale(size_t index, float factor) {
        if (index >= bodies_.size() || factor <= 0.0f) return;
        Body& body = bodies_[index];
        if (body.kind == ObjectKind::Gear) {
            GearSettings settings = body.gear;
            settings.module *= factor;
            settings.thickness *= factor;
            settings.boreRadius *= factor;
            updateGearSettings(index, settings);
            return;
        }
        std::vector<PxShape*> shapes(body.actor->getNbShapes());
        if (!shapes.empty()) body.actor->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
        for (PxShape* shape : shapes) {
            PxTransform localPose = shape->getLocalPose();
            localPose.p *= factor;
            shape->setLocalPose(localPose);
            const PxGeometryHolder geometry(shape->getGeometry());
            if (geometry.getType() == PxGeometryType::eBOX) {
                shape->setGeometry(PxBoxGeometry(geometry.box().halfExtents * factor));
            } else if (geometry.getType() == PxGeometryType::eSPHERE) {
                shape->setGeometry(PxSphereGeometry(geometry.sphere().radius * factor));
            } else if (geometry.getType() == PxGeometryType::eCAPSULE) {
                const PxCapsuleGeometry& capsule = geometry.capsule();
                shape->setGeometry(PxCapsuleGeometry(capsule.radius * factor, capsule.halfHeight * factor));
            }
        }
        body.halfExtents *= factor;
        body.actor->wakeUp();
    }

    bool updateGearSettings(size_t index, GearSettings settings) {
        if (index >= bodies_.size() || bodies_[index].kind != ObjectKind::Gear) return false;
        Body& body = bodies_[index];
        settings.teeth = std::clamp(settings.teeth, 8, 48);
        settings.module = std::clamp(settings.module, 0.06f, 0.28f);
        settings.thickness = std::clamp(settings.thickness, 0.08f, 0.8f);
        settings.boreRadius = std::clamp(settings.boreRadius, 0.05f, 0.6f);
        std::vector<PxShape*> shapes(body.actor->getNbShapes());
        if (!shapes.empty()) body.actor->getShapes(shapes.data(), static_cast<PxU32>(shapes.size()));
        for (PxShape* shape : shapes) body.actor->detachShape(*shape);
        body.gear = settings;
        const float pitchRadius = settings.module * settings.teeth * 0.5f;
        body.halfExtents = PxVec3(pitchRadius, settings.thickness * 0.5f, pitchRadius);
        if (!populateCompoundRing(body.actor, settings.module * (settings.teeth + 2.0f) * 0.5f,
            settings.boreRadius, settings.thickness * 0.5f, settings.teeth, settings.module, true)) return false;
        PxRigidBodyExt::updateMassAndInertia(*body.actor, 1.0f);
        body.actor->wakeUp();
        return true;
    }

    PxVec3 snappedPosition(ObjectKind kind, PxVec3 position, bool enabled) const {
        if (!enabled || (kind != ObjectKind::Gear && kind != ObjectKind::Axle)) return position;
        const ObjectKind target = kind == ObjectKind::Gear ? ObjectKind::Axle : ObjectKind::Bearing;
        const PxVec3 requestedPosition = position;
        float closestDistance = 1.35f;
        for (const Body& body : bodies_) {
            if (body.kind != target) continue;
            const PxVec3 candidate = body.actor->getGlobalPose().p;
            const float distance = (candidate - requestedPosition).magnitude();
            if (distance < closestDistance) {
                closestDistance = distance;
                position = candidate;
            }
        }
        return position;
    }
        bool gpuEnabled() const {
    #if defined(AZMUITH_ENABLE_PHYSX_GPU)
        return cudaManager_ != nullptr;
    #else
        return false;
    #endif
        }

private:
    static Json transformToJson(const PxTransform& pose) {
        return Json{{"position", {pose.p.x, pose.p.y, pose.p.z}},
            {"rotation", {pose.q.x, pose.q.y, pose.q.z, pose.q.w}}};
    }

    static PxTransform transformFromJson(const Json& value) {
        const Json& position = value.at("position");
        const Json& rotation = value.at("rotation");
        return PxTransform(PxVec3(position[0].get<float>(), position[1].get<float>(), position[2].get<float>()),
            PxQuat(rotation[0].get<float>(), rotation[1].get<float>(), rotation[2].get<float>(), rotation[3].get<float>()));
    }

    bool createFixedJoint(size_t first, size_t second, PxTransform localFirst, PxTransform localSecond) {
        if (first >= bodies_.size() || second >= bodies_.size()) return false;
        PxFixedJoint* joint = PxFixedJointCreate(*physics_, bodies_[first].actor, localFirst,
            bodies_[second].actor, localSecond);
        if (!joint) return false;
        joints_.push_back(joint);
        jointRecords_.push_back({first, second, localFirst, localSecond});
        return true;
    }

    void processGlueContacts() {
        for (const auto& [actorA, actorB] : pendingContacts_) {
            size_t indexA = bodies_.size();
            size_t indexB = bodies_.size();
            for (size_t index = 0; index < bodies_.size(); ++index) {
                if (bodies_[index].actor == actorA) indexA = index;
                if (bodies_[index].actor == actorB) indexB = index;
            }
            if (indexA >= bodies_.size() || indexB >= bodies_.size()) continue;
            size_t glueIndex = bodies_.size();
            size_t otherIndex = bodies_.size();
            if (bodies_[indexA].kind == ObjectKind::Glue) {
                glueIndex = indexA;
                otherIndex = indexB;
            } else if (bodies_[indexB].kind == ObjectKind::Glue) {
                glueIndex = indexB;
                otherIndex = indexA;
            }
            if (glueIndex >= bodies_.size() || bodies_[otherIndex].kind == ObjectKind::Glue) continue;
            const bool alreadyConnected = std::any_of(jointRecords_.begin(), jointRecords_.end(),
                [glueIndex, otherIndex](const JointRecord& record) {
                    return (record.first == glueIndex && record.second == otherIndex) ||
                        (record.first == otherIndex && record.second == glueIndex);
                });
            if (alreadyConnected) continue;
            const PxTransform gluePose = bodies_[glueIndex].actor->getGlobalPose();
            const PxTransform otherPose = bodies_[otherIndex].actor->getGlobalPose();
            const PxTransform otherLocal(otherPose.transformInv(gluePose.p),
                otherPose.q.getConjugate() * gluePose.q);
            createFixedJoint(glueIndex, otherIndex, PxTransform(PxIdentity), otherLocal);
        }
        pendingContacts_.clear();
    }

    bool attachBoxShape(PxRigidDynamic* actor, PxVec3 halfExtents, const PxTransform& localPose) {
        PxShape* shape = physics_->createShape(PxBoxGeometry(halfExtents), *material_);
        if (!shape) return false;
        shape->setLocalPose(localPose);
        const bool attached = actor->attachShape(*shape);
        shape->release();
        return attached;
    }

    PxRigidDynamic* makeCompoundRing(const PxTransform& pose, float outerRadius, float innerRadius,
                                    float halfHeight, int teeth, float module, bool gear) {
        PxRigidDynamic* actor = physics_->createRigidDynamic(pose);
        if (!actor) return nullptr;
        if (!populateCompoundRing(actor, outerRadius, innerRadius, halfHeight, teeth, module, gear)) {
            actor->release();
            return nullptr;
        }
        PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);
        return actor;
    }

    bool populateCompoundRing(PxRigidDynamic* actor, float outerRadius, float innerRadius,
                              float halfHeight, int teeth, float module, bool gear) {
        const int segments = std::max(16, gear ? teeth * 2 : 20);
        const float rootRadius = gear ? outerRadius - std::max(module, 0.04f) : outerRadius;
        const float safeInner = std::clamp(innerRadius, 0.03f, rootRadius * 0.8f);
        const float radialHalf = (rootRadius - safeInner) * 0.5f;
        const float middleRadius = (rootRadius + safeInner) * 0.5f;
        const float tangentHalf = middleRadius * std::tan(3.1415926f / segments) * 1.08f;
        for (int segment = 0; segment < segments; ++segment) {
            const float angle = 6.2831853f * segment / segments;
            const PxVec3 position(middleRadius * std::cos(angle), 0.0f, middleRadius * std::sin(angle));
            const PxQuat rotation(angle, PxVec3(0.0f, 1.0f, 0.0f));
            if (!attachBoxShape(actor, PxVec3(radialHalf, halfHeight, tangentHalf), PxTransform(position, rotation))) {
                return false;
            }
        }
        if (gear) {
            const float toothHalfRadial = module * 0.42f;
            const float toothHalfTangent = module * 0.38f;
            const float toothRadius = rootRadius + toothHalfRadial * 0.6f;
            for (int tooth = 0; tooth < teeth; ++tooth) {
                const float angle = 6.2831853f * tooth / teeth;
                const PxVec3 position(toothRadius * std::cos(angle), 0.0f, toothRadius * std::sin(angle));
                const PxQuat rotation(angle, PxVec3(0.0f, 1.0f, 0.0f));
                if (!attachBoxShape(actor, PxVec3(toothHalfRadial, halfHeight, toothHalfTangent),
                    PxTransform(position, rotation))) {
                    return false;
                }
            }
        }
        return true;
    }

    PxDefaultAllocator allocator_;
    PxDefaultErrorCallback errorCallback_;
    PxFoundation* foundation_ = nullptr;
    PxPhysics* physics_ = nullptr;
    PxDefaultCpuDispatcher* dispatcher_ = nullptr;
    PxScene* scene_ = nullptr;
    PxMaterial* material_ = nullptr;
    std::vector<Body> bodies_;
    std::vector<PxJoint*> joints_;
    std::vector<JointRecord> jointRecords_;
    std::vector<std::pair<PxActor*, PxActor*>> pendingContacts_;
#if defined(AZMUITH_ENABLE_PHYSX_GPU)
    PxCudaContextManager* cudaManager_ = nullptr;
#endif
};

class VulkanApp {
public:
    explicit VulkanApp(GLFWwindow* window) : window_(window) { initialize(); }

    ~VulkanApp() {
        if (device_) vkDeviceWaitIdle(device_);
        destroyViewportResources();
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
        if (commandPool_) vkDestroyCommandPool(device_, commandPool_, nullptr);
        for (VkFramebuffer framebuffer : framebuffers_) vkDestroyFramebuffer(device_, framebuffer, nullptr);
        if (renderPass_) vkDestroyRenderPass(device_, renderPass_, nullptr);
        for (VkImageView imageView : imageViews_) vkDestroyImageView(device_, imageView, nullptr);
        if (swapchain_) vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        if (imageAvailable_) vkDestroySemaphore(device_, imageAvailable_, nullptr);
        if (renderFinished_) vkDestroySemaphore(device_, renderFinished_, nullptr);
        if (frameFence_) vkDestroyFence(device_, frameFence_, nullptr);
        if (device_) vkDestroyDevice(device_, nullptr);
        if (surface_) vkDestroySurfaceKHR(instance_, surface_, nullptr);
        if (instance_) vkDestroyInstance(instance_, nullptr);
    }

    void frame(PhysicsWorld& world, bool& playing, bool& stepOnce, float& timeScale,
               std::vector<float>& frameTimes, float& gravity) {
        vkWaitForFences(device_, 1, &frameFence_, VK_TRUE, UINT64_MAX);
        const auto now = std::chrono::steady_clock::now();
        if (autosaveEnabled_ && now - lastAutosave_ >= std::chrono::seconds(30)) {
            if (world.saveScene("scene_autosave.json", sceneStatus_))
                sceneStatus_ = "Autosaved scene_autosave.json";
            lastAutosave_ = now;
        }
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawEditor(world, playing, stepOnce, timeScale, frameTimes, gravity);
        ImGui::Render();

        uint32_t imageIndex = 0;
        VkResult acquired = vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX,
            imageAvailable_, VK_NULL_HANDLE, &imageIndex);
        if (acquired == VK_ERROR_OUT_OF_DATE_KHR) {
            recreateSwapchain();
            return;
        }
        if (acquired != VK_SUCCESS && acquired != VK_SUBOPTIMAL_KHR) requireVk(acquired);
        vkResetFences(device_, 1, &frameFence_);
        vkResetCommandBuffer(commandBuffer_, 0);

        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        requireVk(vkBeginCommandBuffer(commandBuffer_, &begin));
    #if AZMUITH_GPU_VIEWPORT
        if (gpuSceneReadyThisFrame_) {
            recordShadowPass();
            recordViewportPass();
            recordPostProcessPass();
        }
    #endif
        VkClearValue clear{};
        clear.color = {{0.055f, 0.068f, 0.075f, 1.0f}};
        VkRenderPassBeginInfo pass{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        pass.renderPass = renderPass_;
        pass.framebuffer = framebuffers_[imageIndex];
        pass.renderArea.extent = swapchainExtent_;
        pass.clearValueCount = 1;
        pass.pClearValues = &clear;
        vkCmdBeginRenderPass(commandBuffer_, &pass, VK_SUBPASS_CONTENTS_INLINE);
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer_);
        vkCmdEndRenderPass(commandBuffer_);
        requireVk(vkEndCommandBuffer(commandBuffer_));

        const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submit.waitSemaphoreCount = 1;
        submit.pWaitSemaphores = &imageAvailable_;
        submit.pWaitDstStageMask = &waitStage;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &commandBuffer_;
        submit.signalSemaphoreCount = 1;
        submit.pSignalSemaphores = &renderFinished_;
        requireVk(vkQueueSubmit(graphicsQueue_, 1, &submit, frameFence_));

        VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
        present.waitSemaphoreCount = 1;
        present.pWaitSemaphores = &renderFinished_;
        present.swapchainCount = 1;
        present.pSwapchains = &swapchain_;
        present.pImageIndices = &imageIndex;
        VkResult presented = vkQueuePresentKHR(presentQueue_, &present);
        if (presented == VK_ERROR_OUT_OF_DATE_KHR || presented == VK_SUBOPTIMAL_KHR || resized_) {
            resized_ = false;
            recreateSwapchain();
        } else if (presented != VK_SUCCESS) {
            requireVk(presented);
        }
    }

private:
    void drawEditor(PhysicsWorld& world, bool& playing, bool& stepOnce, float& timeScale,
                    std::vector<float>& frameTimes, float& gravity) {
        playing_ = playing;
        stepOnce_ = stepOnce;
        timeScale_ = timeScale;
        gravity_ = gravity;
        frameTimes_ = frameTimes;
        drawScene(world);
        playing = playing_;
        stepOnce = stepOnce_;
        timeScale = timeScale_;
        gravity = gravity_;
    }

    void drawScene(PhysicsWorld& world) {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("Azmuith Physics Lab", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

        drawTopBar(world);
        ImGui::Separator();
        ImGui::BeginChild("workspace", ImVec2(0, 0), false);
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = ImGui::GetContentRegionAvail().y;
        const float leftWidth = std::max(190.0f, width * 0.17f);
        const float rightWidth = std::max(245.0f, width * 0.22f);

        ImGui::BeginChild("hierarchy", ImVec2(leftWidth, height), true);
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "TOOLS");
        ImGui::BeginTable("tool-grid", 2, ImGuiTableFlags_SizingStretchSame);
        const std::array<std::pair<const char*, ToolMode>, 9> tools = {{
            {"Select", ToolMode::Select}, {"Box", ToolMode::Box}, {"Sphere", ToolMode::Sphere},
            {"Cylinder", ToolMode::Cylinder}, {"Gear", ToolMode::Gear}, {"Axle", ToolMode::Axle},
            {"Bearing", ToolMode::Bearing}, {"Structure", ToolMode::Structure}, {"Glue", ToolMode::Glue}}};
        for (const auto& [label, tool] : tools) {
            ImGui::TableNextColumn();
            if (toolMode_ == tool) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.29f, 0.49f, 0.44f, 1.0f));
            if (ImGui::Button(label, ImVec2(-1, 25))) toolMode_ = tool;
            if (toolMode_ == tool) ImGui::PopStyleColor();
        }
        ImGui::EndTable();
        ImGui::Spacing();
        ImGui::Checkbox("Snap placements", &snapEnabled_);
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "SCENE");
        ImGui::Separator();
        ImGui::BulletText("Environment");
        ImGui::Indent();
        ImGui::TextDisabled("Ground plane");
        for (size_t index = 0; index < world.bodies().size(); ++index) {
            const Body& body = world.bodies()[index];
            ImGui::PushID(static_cast<int>(index));
            if (ImGui::Selectable(body.name.c_str(), selectedBody_ == static_cast<int>(index)))
                selectedBody_ = static_cast<int>(index);
            ImGui::PopID();
        }
        ImGui::Unindent();
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("viewport", ImVec2(width - leftWidth - rightWidth - 24.0f, height), true,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawViewport(world);
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("inspector", ImVec2(rightWidth, height), true);
        drawInspector(world);
        ImGui::EndChild();
        ImGui::EndChild();
        ImGui::End();
        drawSettings();
    }

    void drawSettings() {
        if (!settingsOpen_) return;
        ImGui::SetNextWindowSize(ImVec2(430, 350), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Settings", &settingsOpen_)) {
            if (ImGui::BeginTabBar("settings-categories")) {
                if (ImGui::BeginTabItem("Graphics")) {
                    ImGui::Checkbox("Ambient occlusion", &ambientOcclusion_);
                    if (ambientOcclusion_) {
                        ImGui::SliderFloat("AO strength", &aoStrength_, 0.0f, 1.0f, "%.2f");
                        ImGui::SliderFloat("AO radius", &aoRadius_, 0.5f, 3.0f, "%.1f px");
                    }
                    ImGui::Checkbox("FXAA", &fxaa_);
                    ImGui::Checkbox("Smooth shadows", &smoothShadows_);
                    if (!gpuViewportReady()) {
                        ImGui::TextDisabled("GPU post-processing is unavailable; CPU viewport fallback is active.");
                    }
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
        }
        ImGui::End();
    }

    bool gpuViewportReady() const {
#if AZMUITH_GPU_VIEWPORT
        return gpuViewportAvailable_;
#else
        return false;
#endif
    }

    void initialize() {
        createInstance();
        requireVk(glfwCreateWindowSurface(instance_, window_, nullptr, &surface_));
        selectDevice();
        createDevice();
        createSwapchain();
        createRenderPass();
        createFramebuffers();
        createCommandPool();
        createSync();
        createDescriptorPool();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 4.0f;
        style.ChildRounding = 3.0f;
        style.FrameRounding = 3.0f;
        style.GrabRounding = 3.0f;
        style.WindowPadding = ImVec2(12.0f, 10.0f);
        style.FramePadding = ImVec2(8.0f, 5.0f);
        style.ItemSpacing = ImVec2(8.0f, 7.0f);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.075f, 0.087f, 0.091f, 1.0f);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.092f, 0.105f, 0.108f, 1.0f);
        style.Colors[ImGuiCol_Border] = ImVec4(0.18f, 0.22f, 0.22f, 1.0f);
        style.Colors[ImGuiCol_Header] = ImVec4(0.22f, 0.38f, 0.36f, 1.0f);
        style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.48f, 0.44f, 1.0f);
        style.Colors[ImGuiCol_Button] = ImVec4(0.16f, 0.32f, 0.30f, 1.0f);
        style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.23f, 0.43f, 0.39f, 1.0f);
        style.Colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.16f, 0.16f, 1.0f);
        ImGui_ImplGlfw_InitForVulkan(window_, true);

        ImGui_ImplVulkan_InitInfo init{};
        init.Instance = instance_;
        init.PhysicalDevice = physicalDevice_;
        init.Device = device_;
        init.QueueFamily = queueFamilies_.graphics;
        init.Queue = graphicsQueue_;
        init.PipelineCache = VK_NULL_HANDLE;
        init.DescriptorPool = descriptorPool_;
        init.RenderPass = renderPass_;
        init.Subpass = 0;
        init.MinImageCount = std::max(2u, minImageCount_);
        init.ImageCount = static_cast<uint32_t>(swapchainImages_.size());
        init.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        init.Allocator = nullptr;
        init.CheckVkResultFn = [](VkResult result) { requireVk(result); };
        if (!ImGui_ImplVulkan_Init(&init)) throw std::runtime_error("ImGui Vulkan initialization failed");
        uploadFonts();
#if AZMUITH_GPU_VIEWPORT
        try {
            createViewportPipeline();
            gpuViewportAvailable_ = true;
        } catch (const std::exception& error) {
            destroyViewportResources();
            std::fprintf(stderr, "GPU viewport unavailable, using CPU renderer: %s\n", error.what());
        }
#endif
    }

    void createInstance() {
        uint32_t extensionCount = 0;
        const char** extensions = glfwGetRequiredInstanceExtensions(&extensionCount);
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "Azmuith Physics Lab";
        app.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
        app.pEngineName = "Azmuith Sandbox";
        app.engineVersion = VK_MAKE_VERSION(0, 1, 0);
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        info.pApplicationInfo = &app;
        info.enabledExtensionCount = extensionCount;
        info.ppEnabledExtensionNames = extensions;
        requireVk(vkCreateInstance(&info, nullptr, &instance_));
    }

    QueueFamilies findQueueFamilies(VkPhysicalDevice candidate) {
        QueueFamilies families;
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, nullptr);
        std::vector<VkQueueFamilyProperties> properties(count);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &count, properties.data());
        for (uint32_t index = 0; index < count; ++index) {
            if (properties[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) families.graphics = index;
            VkBool32 canPresent = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(candidate, index, surface_, &canPresent);
            if (canPresent) families.present = index;
            if (families.complete()) break;
        }
        return families;
    }

    void selectDevice() {
        uint32_t count = 0;
        requireVk(vkEnumeratePhysicalDevices(instance_, &count, nullptr));
        if (!count) throw std::runtime_error("No Vulkan-capable GPU found");
        std::vector<VkPhysicalDevice> devices(count);
        requireVk(vkEnumeratePhysicalDevices(instance_, &count, devices.data()));
        for (VkPhysicalDevice candidate : devices) {
            QueueFamilies families = findQueueFamilies(candidate);
            if (!families.complete()) continue;
            uint32_t extensionCount = 0;
            vkEnumerateDeviceExtensionProperties(candidate, nullptr, &extensionCount, nullptr);
            std::vector<VkExtensionProperties> extensions(extensionCount);
            vkEnumerateDeviceExtensionProperties(candidate, nullptr, &extensionCount, extensions.data());
            const bool hasSwapchain = std::any_of(extensions.begin(), extensions.end(), [](const auto& extension) {
                return std::string(extension.extensionName) == VK_KHR_SWAPCHAIN_EXTENSION_NAME;
            });
            if (hasSwapchain) {
                physicalDevice_ = candidate;
                queueFamilies_ = families;
                break;
            }
        }
        if (!physicalDevice_) throw std::runtime_error("No Vulkan device supports presentation");
    }

    void createDevice() {
        const float priority = 1.0f;
        std::vector<VkDeviceQueueCreateInfo> queues;
        for (uint32_t family : {queueFamilies_.graphics, queueFamilies_.present}) {
            if (!queues.empty() && queues.front().queueFamilyIndex == family) continue;
            VkDeviceQueueCreateInfo queue{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
            queue.queueFamilyIndex = family;
            queue.queueCount = 1;
            queue.pQueuePriorities = &priority;
            queues.push_back(queue);
        }
        const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
        VkDeviceCreateInfo info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        info.queueCreateInfoCount = static_cast<uint32_t>(queues.size());
        info.pQueueCreateInfos = queues.data();
        info.enabledExtensionCount = 1;
        info.ppEnabledExtensionNames = extensions;
        requireVk(vkCreateDevice(physicalDevice_, &info, nullptr, &device_));
        vkGetDeviceQueue(device_, queueFamilies_.graphics, 0, &graphicsQueue_);
        vkGetDeviceQueue(device_, queueFamilies_.present, 0, &presentQueue_);
    }

    void createSwapchain() {
        VkSurfaceCapabilitiesKHR capabilities{};
        requireVk(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice_, surface_, &capabilities));
        uint32_t formatCount = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &formatCount, nullptr);
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &formatCount, formats.data());
        VkSurfaceFormatKHR format = formats.front();
        for (const auto& candidate : formats) {
            if (candidate.format == VK_FORMAT_B8G8R8A8_UNORM && candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                format = candidate;
                break;
            }
        }
        surfaceFormat_ = format;
        if (capabilities.currentExtent.width != UINT32_MAX) {
            swapchainExtent_ = capabilities.currentExtent;
        } else {
            int width = 0, height = 0;
            glfwGetFramebufferSize(window_, &width, &height);
            swapchainExtent_.width = std::clamp(static_cast<uint32_t>(std::max(width, 1)),
                capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
            swapchainExtent_.height = std::clamp(static_cast<uint32_t>(std::max(height, 1)),
                capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
        }
        minImageCount_ = capabilities.minImageCount;
        uint32_t imageCount = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount && imageCount > capabilities.maxImageCount)
            imageCount = capabilities.maxImageCount;
        VkSwapchainCreateInfoKHR info{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        info.surface = surface_;
        info.minImageCount = imageCount;
        info.imageFormat = format.format;
        info.imageColorSpace = format.colorSpace;
        info.imageExtent = swapchainExtent_;
        info.imageArrayLayers = 1;
        info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        uint32_t families[] = {queueFamilies_.graphics, queueFamilies_.present};
        if (families[0] != families[1]) {
            info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            info.queueFamilyIndexCount = 2;
            info.pQueueFamilyIndices = families;
        } else {
            info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }
        info.preTransform = capabilities.currentTransform;
        info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        info.presentMode = VK_PRESENT_MODE_FIFO_KHR;
        info.clipped = VK_TRUE;
        requireVk(vkCreateSwapchainKHR(device_, &info, nullptr, &swapchain_));
        uint32_t actualCount = 0;
        vkGetSwapchainImagesKHR(device_, swapchain_, &actualCount, nullptr);
        swapchainImages_.resize(actualCount);
        vkGetSwapchainImagesKHR(device_, swapchain_, &actualCount, swapchainImages_.data());
        imageViews_.resize(actualCount);
        for (uint32_t index = 0; index < actualCount; ++index) {
            VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            view.image = swapchainImages_[index];
            view.viewType = VK_IMAGE_VIEW_TYPE_2D;
            view.format = format.format;
            view.components = {VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY};
            view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            requireVk(vkCreateImageView(device_, &view, nullptr, &imageViews_[index]));
        }
    }

    void createRenderPass() {
        VkAttachmentDescription color{};
        color.format = surfaceFormat_.format;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        VkAttachmentReference reference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &reference;
        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        info.attachmentCount = 1;
        info.pAttachments = &color;
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = 1;
        info.pDependencies = &dependency;
        requireVk(vkCreateRenderPass(device_, &info, nullptr, &renderPass_));
    }

    void createFramebuffers() {
        framebuffers_.resize(imageViews_.size());
        for (size_t index = 0; index < imageViews_.size(); ++index) {
            VkFramebufferCreateInfo info{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            info.renderPass = renderPass_;
            info.attachmentCount = 1;
            info.pAttachments = &imageViews_[index];
            info.width = swapchainExtent_.width;
            info.height = swapchainExtent_.height;
            info.layers = 1;
            requireVk(vkCreateFramebuffer(device_, &info, nullptr, &framebuffers_[index]));
        }
    }

    void createCommandPool() {
        VkCommandPoolCreateInfo pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pool.queueFamilyIndex = queueFamilies_.graphics;
        requireVk(vkCreateCommandPool(device_, &pool, nullptr, &commandPool_));
        VkCommandBufferAllocateInfo allocation{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocation.commandPool = commandPool_;
        allocation.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocation.commandBufferCount = 1;
        requireVk(vkAllocateCommandBuffers(device_, &allocation, &commandBuffer_));
    }

    void createSync() {
        VkSemaphoreCreateInfo semaphore{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        requireVk(vkCreateSemaphore(device_, &semaphore, nullptr, &imageAvailable_));
        requireVk(vkCreateSemaphore(device_, &semaphore, nullptr, &renderFinished_));
        VkFenceCreateInfo fence{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fence.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        requireVk(vkCreateFence(device_, &fence, nullptr, &frameFence_));
    }

    void createDescriptorPool() {
        const std::array<VkDescriptorPoolSize, 11> sizes = {{
            {VK_DESCRIPTOR_TYPE_SAMPLER, 1000}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
            {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
            {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}}};
        VkDescriptorPoolCreateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        info.maxSets = 1000 * static_cast<uint32_t>(sizes.size());
        info.poolSizeCount = static_cast<uint32_t>(sizes.size());
        info.pPoolSizes = sizes.data();
        requireVk(vkCreateDescriptorPool(device_, &info, nullptr, &descriptorPool_));
    }

    void uploadFonts() {
        if (!ImGui_ImplVulkan_CreateFontsTexture()) {
            throw std::runtime_error("ImGui Vulkan font upload failed");
        }
    }

#if AZMUITH_GPU_VIEWPORT
    std::vector<uint32_t> readSpirv(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) throw std::runtime_error("Unable to open viewport shader: " + path);
        const std::streamsize size = file.tellg();
        if (size <= 0 || size % sizeof(uint32_t) != 0)
            throw std::runtime_error("Invalid SPIR-V shader size: " + path);
        std::vector<uint32_t> code(static_cast<size_t>(size) / sizeof(uint32_t));
        file.seekg(0);
        file.read(reinterpret_cast<char*>(code.data()), size);
        if (!file) throw std::runtime_error("Unable to read viewport shader: " + path);
        return code;
    }

    VkShaderModule createShaderModule(const std::vector<uint32_t>& code) {
        VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        info.codeSize = code.size() * sizeof(uint32_t);
        info.pCode = code.data();
        VkShaderModule module = VK_NULL_HANDLE;
        requireVk(vkCreateShaderModule(device_, &info, nullptr, &module));
        return module;
    }

    uint32_t findMemoryType(uint32_t typeBits, VkMemoryPropertyFlags properties) {
        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties);
        for (uint32_t index = 0; index < memoryProperties.memoryTypeCount; ++index) {
            if ((typeBits & (1u << index)) &&
                (memoryProperties.memoryTypes[index].propertyFlags & properties) == properties) return index;
        }
        throw std::runtime_error("No compatible Vulkan memory type found");
    }

    void createViewportImage(uint32_t width, uint32_t height, VkFormat format,
                            VkImageUsageFlags usage, VkImageAspectFlags aspect,
                            VkImage& image, VkDeviceMemory& memory, VkImageView& view) {
        VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = format;
        imageInfo.extent = {width, height, 1};
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = usage;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        requireVk(vkCreateImage(device_, &imageInfo, nullptr, &image));

        VkMemoryRequirements requirements{};
        vkGetImageMemoryRequirements(device_, image, &requirements);
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size;
        allocation.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        requireVk(vkAllocateMemory(device_, &allocation, nullptr, &memory));
        requireVk(vkBindImageMemory(device_, image, memory, 0));

        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange = {aspect, 0, 1, 0, 1};
        requireVk(vkCreateImageView(device_, &viewInfo, nullptr, &view));
    }

    void createViewportRenderPass() {
        std::array<VkAttachmentDescription, 2> attachments{};
        attachments[0].format = VK_FORMAT_R8G8B8A8_UNORM;
        attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        attachments[1].format = VK_FORMAT_D32_SFLOAT;
        attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

        VkAttachmentReference colorReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depthReference{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorReference;
        subpass.pDepthStencilAttachment = &depthReference;

        std::array<VkSubpassDependency, 2> dependencies{};
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
            VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
            VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        info.attachmentCount = static_cast<uint32_t>(attachments.size());
        info.pAttachments = attachments.data();
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = static_cast<uint32_t>(dependencies.size());
        info.pDependencies = dependencies.data();
        requireVk(vkCreateRenderPass(device_, &info, nullptr, &viewportRenderPass_));
    }

    void createShadowRenderPass() {
        VkAttachmentDescription depth{};
        depth.format = VK_FORMAT_D32_SFLOAT;
        depth.samples = VK_SAMPLE_COUNT_1_BIT;
        depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        VkAttachmentReference depthReference{0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.pDepthStencilAttachment = &depthReference;
        std::array<VkSubpassDependency, 2> dependencies{};
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        info.attachmentCount = 1;
        info.pAttachments = &depth;
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = static_cast<uint32_t>(dependencies.size());
        info.pDependencies = dependencies.data();
        requireVk(vkCreateRenderPass(device_, &info, nullptr, &shadowRenderPass_));
    }

    void createPostRenderPass() {
        VkAttachmentDescription color{};
        color.format = VK_FORMAT_R8G8B8A8_UNORM;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference colorReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorReference;
        std::array<VkSubpassDependency, 2> dependencies{};
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        info.attachmentCount = 1;
        info.pAttachments = &color;
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = static_cast<uint32_t>(dependencies.size());
        info.pDependencies = dependencies.data();
        requireVk(vkCreateRenderPass(device_, &info, nullptr, &postRenderPass_));
    }

    VkDescriptorSetLayout createImageSetLayout(uint32_t imageBindings) {
        std::vector<VkDescriptorSetLayoutBinding> bindings(imageBindings);
        for (uint32_t index = 0; index < imageBindings; ++index) {
            bindings[index].binding = index;
            bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bindings[index].descriptorCount = 1;
            bindings[index].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        info.bindingCount = static_cast<uint32_t>(bindings.size());
        info.pBindings = bindings.data();
        VkDescriptorSetLayout layout = VK_NULL_HANDLE;
        requireVk(vkCreateDescriptorSetLayout(device_, &info, nullptr, &layout));
        return layout;
    }

    VkDescriptorSet allocateImageSet(VkDescriptorSetLayout layout) {
        VkDescriptorSetAllocateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        info.descriptorPool = descriptorPool_;
        info.descriptorSetCount = 1;
        info.pSetLayouts = &layout;
        VkDescriptorSet set = VK_NULL_HANDLE;
        requireVk(vkAllocateDescriptorSets(device_, &info, &set));
        return set;
    }

    void createShadowTarget() {
        createViewportImage(shadowExtent_.width, shadowExtent_.height, VK_FORMAT_D32_SFLOAT,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT, shadowImage_, shadowMemory_, shadowView_);
        VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerInfo.magFilter = VK_FILTER_NEAREST;
        samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.maxLod = 0.0f;
        requireVk(vkCreateSampler(device_, &samplerInfo, nullptr, &shadowSampler_));
        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = shadowRenderPass_;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = &shadowView_;
        framebufferInfo.width = shadowExtent_.width;
        framebufferInfo.height = shadowExtent_.height;
        framebufferInfo.layers = 1;
        requireVk(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &shadowFramebuffer_));
        shadowSet_ = allocateImageSet(shadowSetLayout_);
        VkDescriptorImageInfo image{shadowSampler_, shadowView_, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = shadowSet_;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &image;
        vkUpdateDescriptorSets(device_, 1, &write, 0, nullptr);
    }

    VkPipeline createViewportPipeline(VkShaderModule vertexShader, VkShaderModule fragmentShader,
                                      VkPrimitiveTopology topology) {
        std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertexShader;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragmentShader;
        stages[1].pName = "main";

        VkVertexInputBindingDescription binding{0, sizeof(SceneVertex), VK_VERTEX_INPUT_RATE_VERTEX};
        std::array<VkVertexInputAttributeDescription, 3> attributes = {{
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(SceneVertex, position)},
            {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(SceneVertex, color)},
            {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(SceneVertex, normal)}}};
        VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        vertexInput.vertexBindingDescriptionCount = 1;
        vertexInput.pVertexBindingDescriptions = &binding;
        vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
        vertexInput.pVertexAttributeDescriptions = attributes.data();

        VkPipelineInputAssemblyStateCreateInfo assembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        assembly.topology = topology;
        VkPipelineViewportStateCreateInfo viewport{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        depth.depthTestEnable = VK_TRUE;
        depth.depthWriteEnable = topology == VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST ? VK_TRUE : VK_FALSE;
        depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        blend.attachmentCount = 1;
        blend.pAttachments = &blendAttachment;
        const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dynamic.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamic.pDynamicStates = dynamicStates.data();

        VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        info.stageCount = static_cast<uint32_t>(stages.size());
        info.pStages = stages.data();
        info.pVertexInputState = &vertexInput;
        info.pInputAssemblyState = &assembly;
        info.pViewportState = &viewport;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &multisample;
        info.pDepthStencilState = &depth;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = viewportPipelineLayout_;
        info.renderPass = viewportRenderPass_;
        info.subpass = 0;
        VkPipeline pipeline = VK_NULL_HANDLE;
        requireVk(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline));
        return pipeline;
    }

    void createViewportVertexBuffer(VkDeviceSize capacity) {
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = capacity;
        bufferInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        requireVk(vkCreateBuffer(device_, &bufferInfo, nullptr, &viewportVertexBuffer_));
        VkMemoryRequirements requirements{};
        vkGetBufferMemoryRequirements(device_, viewportVertexBuffer_, &requirements);
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size;
        allocation.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        requireVk(vkAllocateMemory(device_, &allocation, nullptr, &viewportVertexMemory_));
        requireVk(vkBindBufferMemory(device_, viewportVertexBuffer_, viewportVertexMemory_, 0));
        requireVk(vkMapMemory(device_, viewportVertexMemory_, 0, capacity, 0, &viewportVertexMapped_));
        viewportVertexCapacity_ = capacity;
    }

    void createViewportPipeline() {
        createViewportRenderPass();
        createShadowRenderPass();
        createPostRenderPass();
        shadowSetLayout_ = createImageSetLayout(1);
        postSetLayout_ = createImageSetLayout(2);
        createShadowTarget();
        VkPushConstantRange pushConstant{};
        pushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pushConstant.offset = 0;
        pushConstant.size = sizeof(glm::mat4) * 2;
        VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        layoutInfo.setLayoutCount = 1;
        layoutInfo.pSetLayouts = &shadowSetLayout_;
        layoutInfo.pushConstantRangeCount = 1;
        layoutInfo.pPushConstantRanges = &pushConstant;
        requireVk(vkCreatePipelineLayout(device_, &layoutInfo, nullptr, &viewportPipelineLayout_));

        VkPushConstantRange shadowPush{};
        shadowPush.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        shadowPush.size = sizeof(glm::mat4);
        VkPipelineLayoutCreateInfo shadowLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        shadowLayout.pushConstantRangeCount = 1;
        shadowLayout.pPushConstantRanges = &shadowPush;
        requireVk(vkCreatePipelineLayout(device_, &shadowLayout, nullptr, &shadowPipelineLayout_));

        VkPushConstantRange postPush{};
        postPush.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        postPush.size = sizeof(float) * 6;
        VkPipelineLayoutCreateInfo postLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        postLayout.setLayoutCount = 1;
        postLayout.pSetLayouts = &postSetLayout_;
        postLayout.pushConstantRangeCount = 1;
        postLayout.pPushConstantRanges = &postPush;
        requireVk(vkCreatePipelineLayout(device_, &postLayout, nullptr, &postPipelineLayout_));

        const auto vertexCode = readSpirv(std::string(AZMUITH_SHADER_DIR) + "/viewport.vert.spv");
        const auto fragmentCode = readSpirv(std::string(AZMUITH_SHADER_DIR) + "/viewport.frag.spv");
        VkShaderModule vertexShader = createShaderModule(vertexCode);
        VkShaderModule fragmentShader = createShaderModule(fragmentCode);
        try {
            viewportPipeline_ = createViewportPipeline(vertexShader, fragmentShader,
                VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            gridPipeline_ = createViewportPipeline(vertexShader, fragmentShader,
                VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
        } catch (...) {
            vkDestroyShaderModule(device_, vertexShader, nullptr);
            vkDestroyShaderModule(device_, fragmentShader, nullptr);
            throw;
        }
        vkDestroyShaderModule(device_, vertexShader, nullptr);
        vkDestroyShaderModule(device_, fragmentShader, nullptr);

        createShadowPipeline();
        createPostPipeline();

        VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.maxLod = 1.0f;
        requireVk(vkCreateSampler(device_, &samplerInfo, nullptr, &viewportSampler_));
        samplerInfo.magFilter = VK_FILTER_NEAREST;
        samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        requireVk(vkCreateSampler(device_, &samplerInfo, nullptr, &depthSampler_));
        createViewportVertexBuffer(4 * 1024 * 1024);
    }

    void createShadowPipeline() {
        const auto vertexCode = readSpirv(std::string(AZMUITH_SHADER_DIR) + "/shadow.vert.spv");
        VkShaderModule vertexShader = createShaderModule(vertexCode);
        VkPipelineShaderStageCreateInfo stage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        stage.stage = VK_SHADER_STAGE_VERTEX_BIT;
        stage.module = vertexShader;
        stage.pName = "main";
        VkVertexInputBindingDescription binding{0, sizeof(SceneVertex), VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attribute{0, 0, VK_FORMAT_R32G32B32_SFLOAT,
            offsetof(SceneVertex, position)};
        VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        vertexInput.vertexBindingDescriptionCount = 1;
        vertexInput.pVertexBindingDescriptions = &binding;
        vertexInput.vertexAttributeDescriptionCount = 1;
        vertexInput.pVertexAttributeDescriptions = &attribute;
        VkPipelineInputAssemblyStateCreateInfo assembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo viewport{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        raster.depthClampEnable = VK_FALSE;
        raster.rasterizerDiscardEnable = VK_FALSE;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        raster.depthBiasEnable = VK_TRUE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo depth{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        depth.depthTestEnable = VK_TRUE;
        depth.depthWriteEnable = VK_TRUE;
        depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        VkPipelineColorBlendStateCreateInfo blend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        const std::array<VkDynamicState, 3> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_DEPTH_BIAS};
        VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dynamic.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamic.pDynamicStates = dynamicStates.data();
        VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        info.stageCount = 1;
        info.pStages = &stage;
        info.pVertexInputState = &vertexInput;
        info.pInputAssemblyState = &assembly;
        info.pViewportState = &viewport;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &multisample;
        info.pDepthStencilState = &depth;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = shadowPipelineLayout_;
        info.renderPass = shadowRenderPass_;
        VkResult result = vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &info, nullptr, &shadowPipeline_);
        vkDestroyShaderModule(device_, vertexShader, nullptr);
        requireVk(result);
    }

    void createPostPipeline() {
        const auto vertexCode = readSpirv(std::string(AZMUITH_SHADER_DIR) + "/post.vert.spv");
        const auto fragmentCode = readSpirv(std::string(AZMUITH_SHADER_DIR) + "/post.frag.spv");
        VkShaderModule vertexShader = createShaderModule(vertexCode);
        VkShaderModule fragmentShader = createShaderModule(fragmentCode);
        std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertexShader;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragmentShader;
        stages[1].pName = "main";
        VkPipelineInputAssemblyStateCreateInfo assembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo viewport{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        blend.attachmentCount = 1;
        blend.pAttachments = &blendAttachment;
        const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dynamic.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamic.pDynamicStates = dynamicStates.data();
        VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        info.stageCount = static_cast<uint32_t>(stages.size());
        info.pStages = stages.data();
        info.pInputAssemblyState = &assembly;
        info.pViewportState = &viewport;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &multisample;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = postPipelineLayout_;
        info.renderPass = postRenderPass_;
        VkResult result = vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &info, nullptr, &postPipeline_);
        vkDestroyShaderModule(device_, vertexShader, nullptr);
        vkDestroyShaderModule(device_, fragmentShader, nullptr);
        requireVk(result);
    }

    void destroyViewportTarget() {
        if (viewportTextureSet_) {
            ImGui_ImplVulkan_RemoveTexture(viewportTextureSet_);
            viewportTextureSet_ = VK_NULL_HANDLE;
        }
        if (postSet_) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &postSet_);
            postSet_ = VK_NULL_HANDLE;
        }
        if (viewportFramebuffer_) vkDestroyFramebuffer(device_, viewportFramebuffer_, nullptr);
        if (postFramebuffer_) vkDestroyFramebuffer(device_, postFramebuffer_, nullptr);
        if (postView_) vkDestroyImageView(device_, postView_, nullptr);
        if (postImage_) vkDestroyImage(device_, postImage_, nullptr);
        if (postMemory_) vkFreeMemory(device_, postMemory_, nullptr);
        if (viewportDepthView_) vkDestroyImageView(device_, viewportDepthView_, nullptr);
        if (viewportDepthImage_) vkDestroyImage(device_, viewportDepthImage_, nullptr);
        if (viewportDepthMemory_) vkFreeMemory(device_, viewportDepthMemory_, nullptr);
        if (viewportColorView_) vkDestroyImageView(device_, viewportColorView_, nullptr);
        if (viewportColorImage_) vkDestroyImage(device_, viewportColorImage_, nullptr);
        if (viewportColorMemory_) vkFreeMemory(device_, viewportColorMemory_, nullptr);
        viewportFramebuffer_ = VK_NULL_HANDLE;
        postFramebuffer_ = VK_NULL_HANDLE;
        postView_ = VK_NULL_HANDLE;
        postImage_ = VK_NULL_HANDLE;
        postMemory_ = VK_NULL_HANDLE;
        viewportDepthView_ = VK_NULL_HANDLE;
        viewportDepthImage_ = VK_NULL_HANDLE;
        viewportDepthMemory_ = VK_NULL_HANDLE;
        viewportColorView_ = VK_NULL_HANDLE;
        viewportColorImage_ = VK_NULL_HANDLE;
        viewportColorMemory_ = VK_NULL_HANDLE;
        viewportExtent_ = {};
    }

    void ensureViewportTarget(uint32_t width, uint32_t height) {
        if (viewportExtent_.width == width && viewportExtent_.height == height && viewportFramebuffer_) return;
        destroyViewportTarget();
        createViewportImage(width, height, VK_FORMAT_R8G8B8A8_UNORM,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, viewportColorImage_, viewportColorMemory_, viewportColorView_);
        createViewportImage(width, height, VK_FORMAT_D32_SFLOAT,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_IMAGE_ASPECT_DEPTH_BIT,
            viewportDepthImage_, viewportDepthMemory_, viewportDepthView_);
        const std::array<VkImageView, 2> attachments = {viewportColorView_, viewportDepthView_};
        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = viewportRenderPass_;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = width;
        framebufferInfo.height = height;
        framebufferInfo.layers = 1;
        requireVk(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &viewportFramebuffer_));
        createViewportImage(width, height, VK_FORMAT_R8G8B8A8_UNORM,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT, postImage_, postMemory_, postView_);
        framebufferInfo.renderPass = postRenderPass_;
        framebufferInfo.pAttachments = &postView_;
        framebufferInfo.attachmentCount = 1;
        requireVk(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &postFramebuffer_));

        postSet_ = allocateImageSet(postSetLayout_);
        const std::array<VkDescriptorImageInfo, 2> images = {{
            {viewportSampler_, viewportColorView_, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
            {depthSampler_, viewportDepthView_, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL}}};
        std::array<VkWriteDescriptorSet, 2> writes{};
        for (uint32_t binding = 0; binding < writes.size(); ++binding) {
            writes[binding].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[binding].dstSet = postSet_;
            writes[binding].dstBinding = binding;
            writes[binding].descriptorCount = 1;
            writes[binding].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[binding].pImageInfo = &images[binding];
        }
        vkUpdateDescriptorSets(device_, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
        viewportTextureSet_ = ImGui_ImplVulkan_AddTexture(viewportSampler_, postView_,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        viewportExtent_ = {width, height};
    }

    void ensureViewportVertexCapacity(VkDeviceSize requiredBytes) {
        if (requiredBytes <= viewportVertexCapacity_) return;
        if (viewportVertexMapped_) vkUnmapMemory(device_, viewportVertexMemory_);
        if (viewportVertexBuffer_) vkDestroyBuffer(device_, viewportVertexBuffer_, nullptr);
        if (viewportVertexMemory_) vkFreeMemory(device_, viewportVertexMemory_, nullptr);
        viewportVertexBuffer_ = VK_NULL_HANDLE;
        viewportVertexMemory_ = VK_NULL_HANDLE;
        viewportVertexMapped_ = nullptr;
        VkDeviceSize capacity = std::max<VkDeviceSize>(viewportVertexCapacity_, 4 * 1024 * 1024);
        while (capacity < requiredBytes) capacity *= 2;
        createViewportVertexBuffer(capacity);
    }

    void buildViewportGeometry(PhysicsWorld& world, ImVec2 size) {
        viewportGridVertices_.clear();
        viewportSceneVertices_.clear();
        const glm::vec3 gridColor(0.19f, 0.26f, 0.26f);
        const glm::vec3 xAxisColor(0.34f, 0.62f, 0.56f);
        const glm::vec3 zAxisColor(0.67f, 0.39f, 0.31f);
        auto addLine = [&](glm::vec3 from, glm::vec3 to, glm::vec3 color) {
            viewportGridVertices_.push_back({from, color, glm::vec3(0.0f, 1.0f, 0.0f)});
            viewportGridVertices_.push_back({to, color, glm::vec3(0.0f, 1.0f, 0.0f)});
        };
        for (int coordinate = -12; coordinate <= 12; ++coordinate) {
            const float value = static_cast<float>(coordinate);
            const glm::vec3 lineColor = coordinate == 0 ? xAxisColor : gridColor;
            const glm::vec3 crossColor = coordinate == 0 ? zAxisColor : gridColor;
            addLine({value, 0.0f, -12.0f}, {value, 0.0f, 12.0f}, lineColor);
            addLine({-12.0f, 0.0f, value}, {12.0f, 0.0f, value}, crossColor);
        }

        appendGroundPlane(viewportSceneVertices_);
        for (const Body& body : world.bodies()) {
            appendObjectTriangles(body, viewportSceneVertices_);
        }

        viewportViewProjection_ = viewProjectionFor(size);
        const glm::vec3 target(0.0f, 1.7f, 0.0f);
        const glm::vec3 lightPosition = target + glm::vec3(18.0f, 28.0f, 16.0f);
        glm::mat4 lightProjection = glm::ortho(-24.0f, 24.0f, -24.0f, 24.0f, 1.0f, 90.0f);
        lightProjection[1][1] *= -1.0f;
        viewportLightViewProjection_ = lightProjection * glm::lookAt(lightPosition, target, glm::vec3(0, 1, 0));

        const VkDeviceSize gridBytes = viewportGridVertices_.size() * sizeof(SceneVertex);
        const VkDeviceSize sceneBytes = viewportSceneVertices_.size() * sizeof(SceneVertex);
        ensureViewportVertexCapacity(gridBytes + sceneBytes);
        std::memcpy(viewportVertexMapped_, viewportGridVertices_.data(), static_cast<size_t>(gridBytes));
        if (sceneBytes > 0) {
            std::memcpy(static_cast<char*>(viewportVertexMapped_) + gridBytes,
                viewportSceneVertices_.data(), static_cast<size_t>(sceneBytes));
        }
    }

    bool drawGpuViewport(PhysicsWorld& world, ImVec2 size, ImVec2 origin) {
        try {
            const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
            const uint32_t width = std::max(1u, static_cast<uint32_t>(size.x * scale.x));
            const uint32_t height = std::max(1u, static_cast<uint32_t>(size.y * scale.y));
            ensureViewportTarget(width, height);
            ImGui::Image(reinterpret_cast<ImTextureID>(viewportTextureSet_), size);
            if (ImGui::IsItemHovered() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                const ImVec2 delta = ImGui::GetIO().MouseDelta;
                cameraYaw_ += delta.x * 0.006f;
                cameraPitch_ = std::clamp(cameraPitch_ - delta.y * 0.006f, -0.2f, 1.15f);
            }
            if (ImGui::IsItemHovered()) {
                cameraDistance_ = std::clamp(cameraDistance_ - ImGui::GetIO().MouseWheel * 0.8f, 6.0f, 30.0f);
            }
            const glm::mat4 viewProjection = viewProjectionFor(size);
            handleViewportInteraction(world, origin, size, viewProjection,
                ImGui::IsItemHovered(), ImGui::GetWindowDrawList());
            buildViewportGeometry(world, size);
            gpuSceneReadyThisFrame_ = true;
            ImDrawList* draw = ImGui::GetWindowDrawList();
            draw->AddRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(66, 94, 91, 220));
            draw->AddText(ImVec2(origin.x + 14, origin.y + 12), IM_COL32(187, 205, 202, 220),
                "VULKAN VIEWPORT  |  PHYSX SCENE");
            draw->AddText(ImVec2(origin.x + 14, origin.y + size.y - 28), IM_COL32(126, 150, 148, 220),
                "RMB orbit   Scroll zoom");
            return true;
        } catch (const std::exception& error) {
            std::fprintf(stderr, "GPU viewport target failed, using CPU renderer: %s\n", error.what());
            destroyViewportResources();
            gpuViewportAvailable_ = false;
            gpuSceneReadyThisFrame_ = false;
            ImGui::SetCursorScreenPos(origin);
            return false;
        }
    }

    void recordShadowPass() {
        VkClearValue clear{};
        clear.depthStencil = {1.0f, 0};
        VkRenderPassBeginInfo pass{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        pass.renderPass = shadowRenderPass_;
        pass.framebuffer = shadowFramebuffer_;
        pass.renderArea.extent = shadowExtent_;
        pass.clearValueCount = 1;
        pass.pClearValues = &clear;
        vkCmdBeginRenderPass(commandBuffer_, &pass, VK_SUBPASS_CONTENTS_INLINE);
        VkViewport viewport{0.0f, 0.0f, static_cast<float>(shadowExtent_.width),
            static_cast<float>(shadowExtent_.height), 0.0f, 1.0f};
        VkRect2D scissor{{0, 0}, shadowExtent_};
        vkCmdSetViewport(commandBuffer_, 0, 1, &viewport);
        vkCmdSetScissor(commandBuffer_, 0, 1, &scissor);
        vkCmdSetDepthBias(commandBuffer_, 1.25f, 0.0f, 1.75f);
        if (smoothShadows_) {
            const VkDeviceSize sceneOffset = viewportGridVertices_.size() * sizeof(SceneVertex);
            vkCmdBindVertexBuffers(commandBuffer_, 0, 1, &viewportVertexBuffer_, &sceneOffset);
            vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
            vkCmdPushConstants(commandBuffer_, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                0, sizeof(glm::mat4), &viewportLightViewProjection_[0][0]);
            vkCmdDraw(commandBuffer_, static_cast<uint32_t>(viewportSceneVertices_.size()), 1, 0, 0);
        }
        vkCmdEndRenderPass(commandBuffer_);
    }

    void recordViewportPass() {
        std::array<VkClearValue, 2> clears{};
        clears[0].color = {{0.075f, 0.095f, 0.10f, 1.0f}};
        clears[1].depthStencil = {1.0f, 0};
        VkRenderPassBeginInfo pass{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        pass.renderPass = viewportRenderPass_;
        pass.framebuffer = viewportFramebuffer_;
        pass.renderArea.extent = viewportExtent_;
        pass.clearValueCount = static_cast<uint32_t>(clears.size());
        pass.pClearValues = clears.data();
        vkCmdBeginRenderPass(commandBuffer_, &pass, VK_SUBPASS_CONTENTS_INLINE);
        VkViewport viewport{0.0f, 0.0f, static_cast<float>(viewportExtent_.width),
            static_cast<float>(viewportExtent_.height), 0.0f, 1.0f};
        VkRect2D scissor{{0, 0}, viewportExtent_};
        vkCmdSetViewport(commandBuffer_, 0, 1, &viewport);
        vkCmdSetScissor(commandBuffer_, 0, 1, &scissor);
        const VkDeviceSize sceneOffset = viewportGridVertices_.size() * sizeof(SceneVertex);
        vkCmdBindVertexBuffers(commandBuffer_, 0, 1, &viewportVertexBuffer_, &sceneOffset);
        struct CameraPush {
            glm::mat4 viewProjection;
            glm::mat4 lightViewProjection;
        } camera{viewportViewProjection_, viewportLightViewProjection_};
        vkCmdPushConstants(commandBuffer_, viewportPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
            0, sizeof(CameraPush), &camera);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, viewportPipelineLayout_,
            0, 1, &shadowSet_, 0, nullptr);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, viewportPipeline_);
        vkCmdDraw(commandBuffer_, static_cast<uint32_t>(viewportSceneVertices_.size()), 1,
            0, 0);
        const VkDeviceSize gridOffset = 0;
        vkCmdBindVertexBuffers(commandBuffer_, 0, 1, &viewportVertexBuffer_, &gridOffset);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, gridPipeline_);
        vkCmdDraw(commandBuffer_, static_cast<uint32_t>(viewportGridVertices_.size()), 1, 0, 0);
        vkCmdEndRenderPass(commandBuffer_);
    }

    void recordPostProcessPass() {
        VkClearValue clear{};
        clear.color = {{0.075f, 0.095f, 0.10f, 1.0f}};
        VkRenderPassBeginInfo pass{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        pass.renderPass = postRenderPass_;
        pass.framebuffer = postFramebuffer_;
        pass.renderArea.extent = viewportExtent_;
        pass.clearValueCount = 1;
        pass.pClearValues = &clear;
        vkCmdBeginRenderPass(commandBuffer_, &pass, VK_SUBPASS_CONTENTS_INLINE);
        VkViewport viewport{0.0f, 0.0f, static_cast<float>(viewportExtent_.width),
            static_cast<float>(viewportExtent_.height), 0.0f, 1.0f};
        VkRect2D scissor{{0, 0}, viewportExtent_};
        vkCmdSetViewport(commandBuffer_, 0, 1, &viewport);
        vkCmdSetScissor(commandBuffer_, 0, 1, &scissor);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, postPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, postPipelineLayout_,
            0, 1, &postSet_, 0, nullptr);
        const std::array<float, 6> settings = {
            ambientOcclusion_ ? 1.0f : 0.0f,
            fxaa_ ? 1.0f : 0.0f,
            aoStrength_,
            aoRadius_,
            1.0f / static_cast<float>(viewportExtent_.width),
            1.0f / static_cast<float>(viewportExtent_.height)};
        vkCmdPushConstants(commandBuffer_, postPipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(settings), settings.data());
        vkCmdDraw(commandBuffer_, 3, 1, 0, 0);
        vkCmdEndRenderPass(commandBuffer_);
    }

    void destroyViewportResources() {
        destroyViewportTarget();
        if (shadowSet_) vkFreeDescriptorSets(device_, descriptorPool_, 1, &shadowSet_);
        if (shadowFramebuffer_) vkDestroyFramebuffer(device_, shadowFramebuffer_, nullptr);
        if (shadowView_) vkDestroyImageView(device_, shadowView_, nullptr);
        if (shadowImage_) vkDestroyImage(device_, shadowImage_, nullptr);
        if (shadowMemory_) vkFreeMemory(device_, shadowMemory_, nullptr);
        if (shadowSampler_) vkDestroySampler(device_, shadowSampler_, nullptr);
        if (depthSampler_) vkDestroySampler(device_, depthSampler_, nullptr);
        if (postPipeline_) vkDestroyPipeline(device_, postPipeline_, nullptr);
        if (shadowPipeline_) vkDestroyPipeline(device_, shadowPipeline_, nullptr);
        if (viewportSampler_) vkDestroySampler(device_, viewportSampler_, nullptr);
        if (gridPipeline_) vkDestroyPipeline(device_, gridPipeline_, nullptr);
        if (viewportPipeline_) vkDestroyPipeline(device_, viewportPipeline_, nullptr);
        if (postPipelineLayout_) vkDestroyPipelineLayout(device_, postPipelineLayout_, nullptr);
        if (shadowPipelineLayout_) vkDestroyPipelineLayout(device_, shadowPipelineLayout_, nullptr);
        if (viewportPipelineLayout_) vkDestroyPipelineLayout(device_, viewportPipelineLayout_, nullptr);
        if (postSetLayout_) vkDestroyDescriptorSetLayout(device_, postSetLayout_, nullptr);
        if (shadowSetLayout_) vkDestroyDescriptorSetLayout(device_, shadowSetLayout_, nullptr);
        if (postRenderPass_) vkDestroyRenderPass(device_, postRenderPass_, nullptr);
        if (shadowRenderPass_) vkDestroyRenderPass(device_, shadowRenderPass_, nullptr);
        if (viewportRenderPass_) vkDestroyRenderPass(device_, viewportRenderPass_, nullptr);
        if (viewportVertexMapped_) vkUnmapMemory(device_, viewportVertexMemory_);
        if (viewportVertexBuffer_) vkDestroyBuffer(device_, viewportVertexBuffer_, nullptr);
        if (viewportVertexMemory_) vkFreeMemory(device_, viewportVertexMemory_, nullptr);
        viewportSampler_ = VK_NULL_HANDLE;
        depthSampler_ = VK_NULL_HANDLE;
        shadowSampler_ = VK_NULL_HANDLE;
        shadowSet_ = VK_NULL_HANDLE;
        shadowFramebuffer_ = VK_NULL_HANDLE;
        shadowView_ = VK_NULL_HANDLE;
        shadowImage_ = VK_NULL_HANDLE;
        shadowMemory_ = VK_NULL_HANDLE;
        shadowPipeline_ = VK_NULL_HANDLE;
        shadowPipelineLayout_ = VK_NULL_HANDLE;
        shadowSetLayout_ = VK_NULL_HANDLE;
        shadowRenderPass_ = VK_NULL_HANDLE;
        postPipeline_ = VK_NULL_HANDLE;
        postPipelineLayout_ = VK_NULL_HANDLE;
        postSetLayout_ = VK_NULL_HANDLE;
        postRenderPass_ = VK_NULL_HANDLE;
        gridPipeline_ = VK_NULL_HANDLE;
        viewportPipeline_ = VK_NULL_HANDLE;
        viewportPipelineLayout_ = VK_NULL_HANDLE;
        viewportRenderPass_ = VK_NULL_HANDLE;
        viewportVertexMapped_ = nullptr;
        viewportVertexBuffer_ = VK_NULL_HANDLE;
        viewportVertexMemory_ = VK_NULL_HANDLE;
        viewportVertexCapacity_ = 0;
    }
#else
    bool drawGpuViewport(PhysicsWorld&, ImVec2, ImVec2) { return false; }
    void recordViewportPass() {}
    void destroyViewportResources() {}
#endif

    void recreateSwapchain() {
        int width = 0, height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        while (width == 0 || height == 0) {
            glfwWaitEvents();
            glfwGetFramebufferSize(window_, &width, &height);
        }
        vkDeviceWaitIdle(device_);
        for (VkFramebuffer framebuffer : framebuffers_) vkDestroyFramebuffer(device_, framebuffer, nullptr);
        for (VkImageView view : imageViews_) vkDestroyImageView(device_, view, nullptr);
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        framebuffers_.clear();
        imageViews_.clear();
        swapchainImages_.clear();
        createSwapchain();
        createFramebuffers();
    }

    static ImVec2 project(glm::vec3 point, const glm::mat4& viewProjection, ImVec2 origin, ImVec2 size) {
        glm::vec4 clip = viewProjection * glm::vec4(point, 1.0f);
        if (clip.w <= 0.001f) return ImVec2(-10000.0f, -10000.0f);
        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        return ImVec2(origin.x + (ndc.x * 0.5f + 0.5f) * size.x,
                      origin.y + (0.5f - ndc.y * 0.5f) * size.y);
    }

    glm::mat4 viewProjectionFor(ImVec2 size) const {
        const glm::vec3 target(0.0f, 1.7f, 0.0f);
        const glm::vec3 eye = target + glm::vec3(
            cameraDistance_ * std::cos(cameraPitch_) * std::sin(cameraYaw_),
            cameraDistance_ * std::sin(cameraPitch_),
            cameraDistance_ * std::cos(cameraPitch_) * std::cos(cameraYaw_));
        glm::mat4 projection = glm::perspective(glm::radians(48.0f),
            std::max(size.x, 1.0f) / std::max(size.y, 1.0f), 0.1f, 100.0f);
        projection[1][1] *= -1.0f;
        return projection * glm::lookAt(eye, target, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    bool screenRay(ImVec2 mouse, ImVec2 origin, ImVec2 size, const glm::mat4& viewProjection,
                   PxVec3& rayOrigin, PxVec3& rayDirection) const {
        if (size.x <= 1.0f || size.y <= 1.0f) return false;
        const float x = 2.0f * (mouse.x - origin.x) / size.x - 1.0f;
        const float y = 1.0f - 2.0f * (mouse.y - origin.y) / size.y;
        const glm::mat4 inverseViewProjection = glm::inverse(viewProjection);
        glm::vec4 nearPoint = inverseViewProjection * glm::vec4(x, y, 0.0f, 1.0f);
        glm::vec4 farPoint = inverseViewProjection * glm::vec4(x, y, 1.0f, 1.0f);
        if (std::abs(nearPoint.w) < 0.0001f || std::abs(farPoint.w) < 0.0001f) return false;
        const glm::vec3 nearWorld = glm::vec3(nearPoint) / nearPoint.w;
        const glm::vec3 farWorld = glm::vec3(farPoint) / farPoint.w;
        const glm::vec3 direction = glm::normalize(farWorld - nearWorld);
        rayOrigin = PxVec3(nearWorld.x, nearWorld.y, nearWorld.z);
        rayDirection = PxVec3(direction.x, direction.y, direction.z);
        return true;
    }

    bool groundPoint(ImVec2 mouse, ImVec2 origin, ImVec2 size, const glm::mat4& viewProjection,
                     PxVec3& point) const {
        PxVec3 rayOrigin, rayDirection;
        if (!screenRay(mouse, origin, size, viewProjection, rayOrigin, rayDirection) ||
            std::abs(rayDirection.y) < 0.0001f) return false;
        const float distance = -rayOrigin.y / rayDirection.y;
        if (distance < 0.0f) return false;
        point = rayOrigin + rayDirection * distance;
        return true;
    }

    static float pointSegmentDistance(ImVec2 point, ImVec2 start, ImVec2 end) {
        const float dx = end.x - start.x;
        const float dy = end.y - start.y;
        const float lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < 0.001f) return std::hypot(point.x - start.x, point.y - start.y);
        const float t = std::clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared, 0.0f, 1.0f);
        return std::hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy));
    }

    float placementSize() const {
        if (toolMode_ == ToolMode::Glue) return 0.18f;
        if (toolMode_ == ToolMode::Axle) return 1.1f;
        if (toolMode_ == ToolMode::Bearing) return 0.55f;
        if (toolMode_ == ToolMode::Gear) return gearSettings_.module * gearSettings_.teeth * 0.5f;
        return 0.5f;
    }

    float placementHeight() const {
        if (toolMode_ == ToolMode::Gear) return gearSettings_.thickness * 0.5f;
        if (toolMode_ == ToolMode::Bearing) return 0.12f;
        if (toolMode_ == ToolMode::Cylinder || toolMode_ == ToolMode::Axle) return 0.62f;
        if (toolMode_ == ToolMode::Glue) return 0.18f;
        return 0.5f;
    }

    void drawGizmo(PhysicsWorld& world, const glm::mat4& viewProjection,
                   ImVec2 origin, ImVec2 size, ImDrawList* draw) {
        if (selectedBody_ < 0 || selectedBody_ >= static_cast<int>(world.bodyCount())) return;
        const PxVec3 position = world.bodies()[selectedBody_].actor->getGlobalPose().p;
        const ImVec2 center = project(glm::vec3(position.x, position.y, position.z), viewProjection, origin, size);
        const std::array<glm::vec3, 3> axes = {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)};
        const std::array<ImU32, 3> colors = {IM_COL32(230, 95, 78, 255), IM_COL32(105, 205, 143, 255), IM_COL32(95, 157, 235, 255)};
        for (int axis = 0; axis < 3; ++axis) {
            const glm::vec3 endWorld(position.x, position.y, position.z);
            const ImVec2 end = project(endWorld + axes[axis] * 1.35f, viewProjection, origin, size);
            draw->AddLine(center, end, colors[axis], 2.5f);
            draw->AddCircleFilled(end, 5.0f, colors[axis], 12);
        }
        const char* mode = gizmoMode_ == GizmoMode::Move ? "MOVE" :
            gizmoMode_ == GizmoMode::Rotate ? "ROTATE" : "SCALE";
        draw->AddText(ImVec2(center.x + 10, center.y + 10), IM_COL32(225, 231, 223, 235), mode);
    }

    void handleViewportInteraction(PhysicsWorld& world, ImVec2 origin, ImVec2 size,
                                   const glm::mat4& viewProjection, bool hovered, ImDrawList* draw) {
        if (!ImGui::IsAnyItemActive()) {
            if (ImGui::IsKeyPressed(ImGuiKey_E)) gizmoMode_ = GizmoMode::Move;
            if (ImGui::IsKeyPressed(ImGuiKey_R)) gizmoMode_ = GizmoMode::Rotate;
            if (ImGui::IsKeyPressed(ImGuiKey_T)) gizmoMode_ = GizmoMode::Scale;
        }

        const ImVec2 mouse = ImGui::GetIO().MousePos;
        PxVec3 mouseGround;
        const bool hasGroundPoint = groundPoint(mouse, origin, size, viewProjection, mouseGround);
        const bool pressed = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
        if (toolMode_ != ToolMode::Select) {
            if (hasGroundPoint) {
                const float sizeValue = placementSize();
                PxVec3 preview = mouseGround;
                preview.y = placementHeight();
                preview = world.snappedPosition(objectKindForTool(toolMode_), preview, snapEnabled_);
                const ImVec2 center = project(glm::vec3(preview.x, preview.y, preview.z), viewProjection, origin, size);
                const ImVec2 radiusPoint = project(glm::vec3(preview.x + sizeValue, preview.y, preview.z), viewProjection, origin, size);
                const float pixelRadius = std::clamp(std::abs(radiusPoint.x - center.x), 7.0f, 90.0f);
                const ImU32 ghost = IM_COL32(139, 220, 184, 150);
                if (toolMode_ == ToolMode::Sphere || toolMode_ == ToolMode::Glue)
                    draw->AddCircle(center, pixelRadius, ghost, 24, 2.0f);
                else if (toolMode_ == ToolMode::Gear) {
                    std::vector<ImVec2> teeth;
                    const int count = std::max(gearSettings_.teeth * 2, 16);
                    for (int index = 0; index < count; ++index) {
                        const float angle = 6.2831853f * index / count;
                        const float radial = pixelRadius * (index % 2 == 0 ? 1.0f : 0.82f);
                        teeth.emplace_back(center.x + std::cos(angle) * radial, center.y + std::sin(angle) * radial);
                    }
                    draw->AddPolyline(teeth.data(), static_cast<int>(teeth.size()), ghost, ImDrawFlags_Closed, 2.0f);
                } else {
                    draw->AddRect(ImVec2(center.x - pixelRadius, center.y - pixelRadius),
                        ImVec2(center.x + pixelRadius, center.y + pixelRadius), ghost, 2.0f, 0, 2.0f);
                }
                if (toolMode_ == ToolMode::Structure && structureStartPending_) {
                    const ImVec2 start = project(glm::vec3(structureStart_.x, structureStart_.y, structureStart_.z),
                        viewProjection, origin, size);
                    draw->AddLine(start, center, ghost, 3.0f);
                    draw->AddCircleFilled(start, 6.0f, ghost, 12);
                }

                if (pressed) {
                    if (toolMode_ == ToolMode::Structure) {
                        if (!structureStartPending_) {
                            structureStart_ = PxVec3(preview.x, placementHeight(), preview.z);
                            structureStartPending_ = true;
                        } else {
                            world.createStructure(structureStart_, PxVec3(preview.x, placementHeight(), preview.z));
                            structureStartPending_ = false;
                        }
                    } else {
                        const ObjectKind kind = objectKindForTool(toolMode_);
                        const size_t index = world.createObject(kind, preview, sizeValue, gearSettings_);
                        if (index < world.bodyCount()) selectedBody_ = static_cast<int>(index);
                    }
                }
            }
            draggingBody_ = false;
            draggingGizmo_ = false;
            return;
        }

        if (pressed) {
            gizmoAxis_ = -1;
            if (selectedBody_ >= 0 && selectedBody_ < static_cast<int>(world.bodyCount())) {
                const PxVec3 position = world.bodies()[selectedBody_].actor->getGlobalPose().p;
                const ImVec2 center = project(glm::vec3(position.x, position.y, position.z), viewProjection, origin, size);
                const std::array<glm::vec3, 3> axes = {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)};
                for (int axis = 0; axis < 3; ++axis) {
                    const ImVec2 end = project(glm::vec3(position.x, position.y, position.z) + axes[axis] * 1.35f,
                        viewProjection, origin, size);
                    if (pointSegmentDistance(mouse, center, end) < 9.0f) {
                        gizmoAxis_ = axis;
                        draggingGizmo_ = true;
                        break;
                    }
                }
            }
            if (gizmoAxis_ < 0) {
                PxVec3 rayOrigin, rayDirection;
                size_t hitIndex = 0;
                if (screenRay(mouse, origin, size, viewProjection, rayOrigin, rayDirection) &&
                    world.raycast(rayOrigin, rayDirection, hitIndex)) {
                    selectedBody_ = static_cast<int>(hitIndex);
                    draggingBody_ = hasGroundPoint;
                    dragOffset_ = world.bodies()[hitIndex].actor->getGlobalPose().p - mouseGround;
                } else {
                    selectedBody_ = -1;
                    draggingBody_ = false;
                }
            }
        }

        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && hasGroundPoint && draggingBody_ &&
            selectedBody_ >= 0 && selectedBody_ < static_cast<int>(world.bodyCount())) {
            PxVec3 target = mouseGround + dragOffset_;
            target = world.snappedPosition(world.bodies()[selectedBody_].kind, target, snapEnabled_);
            world.setPosition(static_cast<size_t>(selectedBody_), target);
        }
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && draggingGizmo_ && gizmoAxis_ >= 0 &&
            selectedBody_ >= 0 && selectedBody_ < static_cast<int>(world.bodyCount())) {
            const std::array<PxVec3, 3> axes = {PxVec3(1, 0, 0), PxVec3(0, 1, 0), PxVec3(0, 0, 1)};
            const float delta = ImGui::GetIO().MouseDelta.x;
            if (gizmoMode_ == GizmoMode::Move && hasGroundPoint) {
                const PxVec3 position = world.bodies()[selectedBody_].actor->getGlobalPose().p;
                const ImVec2 center = project(glm::vec3(position.x, position.y, position.z), viewProjection, origin, size);
                const ImVec2 end = project(glm::vec3(position.x, position.y, position.z) +
                    glm::vec3(axes[gizmoAxis_].x, axes[gizmoAxis_].y, axes[gizmoAxis_].z) * 1.35f,
                    viewProjection, origin, size);
                const glm::vec2 screenAxis(end.x - center.x, end.y - center.y);
                const float axisLength = std::max(glm::length(screenAxis), 1.0f);
                const float projectedDelta = glm::dot(glm::vec2(ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y),
                    screenAxis) / axisLength;
                PxVec3 target = position + axes[gizmoAxis_] * (projectedDelta * 0.012f);
                target = world.snappedPosition(world.bodies()[selectedBody_].kind, target, snapEnabled_);
                world.setPosition(static_cast<size_t>(selectedBody_), target);
            } else if (gizmoMode_ == GizmoMode::Rotate) {
                world.rotate(static_cast<size_t>(selectedBody_), axes[gizmoAxis_], delta * 0.012f);
            } else if (gizmoMode_ == GizmoMode::Scale) {
                world.scale(static_cast<size_t>(selectedBody_), std::exp(delta * 0.01f));
            }
        }
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            draggingBody_ = false;
            draggingGizmo_ = false;
            gizmoAxis_ = -1;
        }
        if (hovered) drawGizmo(world, viewProjection, origin, size, draw);
    }

    void drawViewport(PhysicsWorld& world) {
        ImVec2 size = ImGui::GetContentRegionAvail();
        ImVec2 origin = ImGui::GetCursorScreenPos();
#if AZMUITH_GPU_VIEWPORT
        gpuSceneReadyThisFrame_ = false;
        if (gpuViewportAvailable_ && drawGpuViewport(world, size, origin)) return;
#endif
        ImGui::InvisibleButton("scene-canvas", size, ImGuiButtonFlags_MouseButtonRight);
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(19, 27, 29, 255));
        if (ImGui::IsItemHovered() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            ImVec2 delta = ImGui::GetIO().MouseDelta;
            cameraYaw_ += delta.x * 0.006f;
            cameraPitch_ = std::clamp(cameraPitch_ - delta.y * 0.006f, -0.2f, 1.15f);
        }
        if (ImGui::IsItemHovered()) cameraDistance_ = std::clamp(cameraDistance_ - ImGui::GetIO().MouseWheel * 0.8f, 6.0f, 30.0f);

        const glm::mat4 viewProjection = viewProjectionFor(size);

        for (int coordinate = -12; coordinate <= 12; ++coordinate) {
            ImU32 color = coordinate == 0 ? IM_COL32(90, 145, 138, 190) : IM_COL32(52, 69, 70, 125);
            ImVec2 a = project(glm::vec3(static_cast<float>(coordinate), 0, -12), viewProjection, origin, size);
            ImVec2 b = project(glm::vec3(static_cast<float>(coordinate), 0, 12), viewProjection, origin, size);
            ImVec2 c = project(glm::vec3(-12, 0, static_cast<float>(coordinate)), viewProjection, origin, size);
            ImVec2 d = project(glm::vec3(12, 0, static_cast<float>(coordinate)), viewProjection, origin, size);
            draw->AddLine(a, b, color, coordinate == 0 ? 1.5f : 1.0f);
            draw->AddLine(c, d, color, coordinate == 0 ? 1.5f : 1.0f);
        }

        struct Face { float depth; std::array<ImVec2, 3> points; ImU32 color; };
        std::vector<Face> faces;
        std::vector<SceneVertex> sceneVertices;
        for (const Body& body : world.bodies()) {
            appendObjectTriangles(body, sceneVertices);
        }
        for (size_t vertex = 0; vertex + 2 < sceneVertices.size(); vertex += 3) {
            std::array<ImVec2, 3> screen;
            float depth = 0.0f;
            for (int corner = 0; corner < 3; ++corner) {
                const glm::vec3 position = sceneVertices[vertex + corner].position;
                screen[corner] = project(position, viewProjection, origin, size);
                const glm::vec4 clip = viewProjection * glm::vec4(position, 1.0f);
                depth += clip.z / std::max(clip.w, 0.001f);
            }
            depth /= 3.0f;
            const glm::vec3 color = sceneVertices[vertex].color;
            const ImU32 packedColor = ImGui::ColorConvertFloat4ToU32(ImVec4(color.r, color.g, color.b, 1.0f));
            faces.push_back({depth, screen, packedColor});
        }
        std::sort(faces.begin(), faces.end(), [](const Face& a, const Face& b) { return a.depth > b.depth; });
        for (const Face& face : faces) {
            draw->AddTriangleFilled(face.points[0], face.points[1], face.points[2], face.color);
            draw->AddTriangle(face.points[0], face.points[1], face.points[2], IM_COL32(20, 28, 29, 190), 1.0f);
        }
        draw->AddText(ImVec2(origin.x + 14, origin.y + 12), IM_COL32(187, 205, 202, 220),
            "CPU FALLBACK  |  PHYSX SCENE");
        draw->AddText(ImVec2(origin.x + 14, origin.y + size.y - 28), IM_COL32(126, 150, 148, 220), "RMB orbit   Scroll zoom");
        handleViewportInteraction(world, origin, size, viewProjection,
            ImGui::IsItemHovered(), draw);
    }

    void drawTopBar(PhysicsWorld& world) {
        ImGui::TextColored(ImVec4(0.83f, 0.88f, 0.84f, 1.0f), "AZMUITH");
        ImGui::SameLine();
        ImGui::TextDisabled("/  PHYSICS LAB");
        ImGui::SameLine(0, 28);
        if (ImGui::Button("Reset")) world.reset();
        ImGui::SameLine();
        if (ImGui::Button("Settings")) settingsOpen_ = !settingsOpen_;
        ImGui::SameLine();
        if (ImGui::Button("Add cube")) world.spawn();
        ImGui::SameLine(0, 20);
        ImGui::TextDisabled("%zu rigid bodies", world.bodyCount());
        ImGui::SameLine(0, 18);
        ImGui::TextColored(world.gpuEnabled() ? ImVec4(0.43f, 0.78f, 0.62f, 1.0f) : ImVec4(0.76f, 0.68f, 0.43f, 1.0f),
            world.gpuEnabled() ? "PHYSX GPU" : "PHYSX CPU");
    }

    void drawInspector(PhysicsWorld& world) {
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "SCENE FILE");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##scene-path", scenePath_, sizeof(scenePath_));
        if (ImGui::Button("Save", ImVec2(76, 0))) {
            if (world.saveScene(scenePath_, sceneStatus_)) sceneStatus_ = "Scene saved";
            lastAutosave_ = std::chrono::steady_clock::now();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load", ImVec2(76, 0))) {
            if (world.loadScene(scenePath_, sceneStatus_)) {
                sceneStatus_ = "Scene loaded";
                selectedBody_ = world.bodyCount() ? 0 : -1;
                structureStartPending_ = false;
                draggingBody_ = false;
                draggingGizmo_ = false;
            }
        }
        ImGui::SameLine();
        ImGui::Checkbox("Autosave", &autosaveEnabled_);
        if (!sceneStatus_.empty()) ImGui::TextWrapped("%s", sceneStatus_.c_str());
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "GIZMO");
        if (ImGui::Button("Move [E]", ImVec2(75, 0))) gizmoMode_ = GizmoMode::Move;
        ImGui::SameLine();
        if (ImGui::Button("Rotate [R]", ImVec2(82, 0))) gizmoMode_ = GizmoMode::Rotate;
        ImGui::SameLine();
        if (ImGui::Button("Scale [T]", ImVec2(75, 0))) gizmoMode_ = GizmoMode::Scale;
        ImGui::Separator();
        Body* selectedGear = selectedBody_ >= 0 && selectedBody_ < static_cast<int>(world.bodyCount()) &&
            world.bodies()[selectedBody_].kind == ObjectKind::Gear ? &world.bodies()[selectedBody_] : nullptr;
        if (toolMode_ == ToolMode::Gear || selectedGear) {
            ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), selectedGear ? "GEAR INSPECTOR" : "GEAR GENERATOR");
            GearSettings& settings = selectedGear ? selectedGear->gear : gearSettings_;
            bool settingsChanged = false;
            ImGui::SetNextItemWidth(-1);
            settingsChanged |= ImGui::SliderInt("Teeth", &settings.teeth, 8, 48);
            ImGui::SetNextItemWidth(-1);
            settingsChanged |= ImGui::SliderFloat("Module", &settings.module, 0.06f, 0.28f, "%.2f");
            ImGui::SetNextItemWidth(-1);
            settingsChanged |= ImGui::SliderFloat("Thickness", &settings.thickness, 0.08f, 0.8f, "%.2f m");
            ImGui::SetNextItemWidth(-1);
            settingsChanged |= ImGui::SliderFloat("Bore radius", &settings.boreRadius, 0.05f, 0.6f, "%.2f m");
            if (settingsChanged && selectedGear)
                world.updateGearSettings(static_cast<size_t>(selectedBody_), settings);
            ImGui::Separator();
        }
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "SIMULATION");
        ImGui::Separator();
        if (ImGui::Button(playing_ ? "Pause" : "Play", ImVec2(78, 0))) playing_ = !playing_;
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(70, 0))) stepOnce_ = true;
        ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("Time scale", &timeScale_, 0.1f, 2.0f, "%.2fx");
        ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("Gravity", &gravity_, -20.0f, 0.0f, "%.2f m/s^2");
        world.setGravity(gravity_);

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "SELECTION");
        ImGui::Separator();
        if (world.bodyCount() == 0) {
            ImGui::TextDisabled("No bodies in scene");
            return;
        }
        if (selectedBody_ < 0 || selectedBody_ >= static_cast<int>(world.bodyCount())) {
            ImGui::TextDisabled("No object selected");
            return;
        }
        const Body& body = world.bodies()[selectedBody_];
        ImGui::Text("%s", body.name.c_str());
        PxTransform pose = body.actor->getGlobalPose();
        ImGui::TextDisabled("%s collider", objectKindName(body.kind));
        ImGui::Spacing();
        ImGui::Text("Position");
        ImGui::Text("X  %7.3f", pose.p.x);
        ImGui::Text("Y  %7.3f", pose.p.y);
        ImGui::Text("Z  %7.3f", pose.p.z);
        ImGui::Spacing();
        ImGui::Text("Mass       %.2f kg", body.actor->getMass());
        ImGui::Text("Linear speed  %.2f m/s", body.actor->getLinearVelocity().magnitude());
        ImGui::Text("State      %s", body.actor->isSleeping() ? "Sleeping" : "Active");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.56f, 0.70f, 0.73f, 1), "FRAME TIME");
        ImGui::Separator();
        if (!frameTimes_.empty()) {
            ImGui::PlotLines("##frametime", frameTimes_.data(), static_cast<int>(frameTimes_.size()),
                0, nullptr, 0.0f, 33.0f, ImVec2(-1, 58));
        }
    }

    GLFWwindow* window_;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    QueueFamilies queueFamilies_;
    VkQueue graphicsQueue_ = VK_NULL_HANDLE;
    VkQueue presentQueue_ = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkSurfaceFormatKHR surfaceFormat_{};
    VkExtent2D swapchainExtent_{};
    uint32_t minImageCount_ = 2;
    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> imageViews_;
    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> framebuffers_;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkSemaphore imageAvailable_ = VK_NULL_HANDLE;
    VkSemaphore renderFinished_ = VK_NULL_HANDLE;
    VkFence frameFence_ = VK_NULL_HANDLE;
#if AZMUITH_GPU_VIEWPORT
    bool gpuViewportAvailable_ = false;
    bool gpuSceneReadyThisFrame_ = false;
    VkRenderPass viewportRenderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout viewportPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline viewportPipeline_ = VK_NULL_HANDLE;
    VkPipeline gridPipeline_ = VK_NULL_HANDLE;
    VkRenderPass shadowRenderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout shadowPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline shadowPipeline_ = VK_NULL_HANDLE;
    VkImage shadowImage_ = VK_NULL_HANDLE;
    VkDeviceMemory shadowMemory_ = VK_NULL_HANDLE;
    VkImageView shadowView_ = VK_NULL_HANDLE;
    VkFramebuffer shadowFramebuffer_ = VK_NULL_HANDLE;
    VkSampler shadowSampler_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout shadowSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSet shadowSet_ = VK_NULL_HANDLE;
    VkRenderPass postRenderPass_ = VK_NULL_HANDLE;
    VkPipelineLayout postPipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline postPipeline_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout postSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorSet postSet_ = VK_NULL_HANDLE;
    VkImage postImage_ = VK_NULL_HANDLE;
    VkDeviceMemory postMemory_ = VK_NULL_HANDLE;
    VkImageView postView_ = VK_NULL_HANDLE;
    VkFramebuffer postFramebuffer_ = VK_NULL_HANDLE;
    VkSampler depthSampler_ = VK_NULL_HANDLE;
    VkImage viewportColorImage_ = VK_NULL_HANDLE;
    VkDeviceMemory viewportColorMemory_ = VK_NULL_HANDLE;
    VkImageView viewportColorView_ = VK_NULL_HANDLE;
    VkImage viewportDepthImage_ = VK_NULL_HANDLE;
    VkDeviceMemory viewportDepthMemory_ = VK_NULL_HANDLE;
    VkImageView viewportDepthView_ = VK_NULL_HANDLE;
    VkFramebuffer viewportFramebuffer_ = VK_NULL_HANDLE;
    VkSampler viewportSampler_ = VK_NULL_HANDLE;
    VkDescriptorSet viewportTextureSet_ = VK_NULL_HANDLE;
    VkBuffer viewportVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory viewportVertexMemory_ = VK_NULL_HANDLE;
    void* viewportVertexMapped_ = nullptr;
    VkExtent2D viewportExtent_{};
    VkExtent2D shadowExtent_{1024, 1024};
    VkDeviceSize viewportVertexCapacity_ = 0;
    glm::mat4 viewportViewProjection_{1.0f};
    glm::mat4 viewportLightViewProjection_{1.0f};
    std::vector<SceneVertex> viewportGridVertices_;
    std::vector<SceneVertex> viewportSceneVertices_;
#endif
    bool resized_ = false;
    float cameraYaw_ = 0.72f;
    float cameraPitch_ = 0.43f;
    float cameraDistance_ = 14.0f;
    int selectedBody_ = 0;
    ToolMode toolMode_ = ToolMode::Select;
    GizmoMode gizmoMode_ = GizmoMode::Move;
    GearSettings gearSettings_;
    bool snapEnabled_ = true;
    bool structureStartPending_ = false;
    PxVec3 structureStart_ = PxVec3(0.0f);
    bool draggingBody_ = false;
    PxVec3 dragOffset_ = PxVec3(0.0f);
    int gizmoAxis_ = -1;
    bool draggingGizmo_ = false;
    bool settingsOpen_ = false;
    bool ambientOcclusion_ = true;
    bool fxaa_ = true;
    bool smoothShadows_ = true;
    float aoStrength_ = 0.65f;
    float aoRadius_ = 1.5f;
    bool playing_ = true;
    bool stepOnce_ = false;
    float timeScale_ = 1.0f;
    float gravity_ = -9.81f;
    std::vector<float> frameTimes_;
    char scenePath_[260] = "scene.json";
    bool autosaveEnabled_ = true;
    std::chrono::steady_clock::time_point lastAutosave_ = std::chrono::steady_clock::now();
    std::string sceneStatus_;
};

} // namespace

int main() {
    try {
        if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        GLFWwindow* window = glfwCreateWindow(1440, 900, "Azmuith Physics Lab", nullptr, nullptr);
        if (!window) throw std::runtime_error("Window creation failed");

        PhysicsWorld world;
        VulkanApp app(window);
        bool playing = true;
        bool stepOnce = false;
        float timeScale = 1.0f;
        float gravity = -9.81f;
        std::vector<float> frameTimes(120, 16.0f);
        auto previous = std::chrono::steady_clock::now();
        float accumulator = 0.0f;

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - previous).count();
            previous = now;
            elapsed = std::min(elapsed, 0.1f);
            frameTimes.push_back(elapsed * 1000.0f);
            if (frameTimes.size() > 120) frameTimes.erase(frameTimes.begin());
            if (playing) accumulator += elapsed * timeScale;
            constexpr float fixedStep = 1.0f / 60.0f;
            int steps = 0;
            while ((accumulator >= fixedStep || stepOnce) && steps < 8) {
                world.step(fixedStep);
                if (stepOnce) {
                    stepOnce = false;
                    accumulator = 0.0f;
                    break;
                }
                accumulator -= fixedStep;
                ++steps;
            }
            app.frame(world, playing, stepOnce, timeScale, frameTimes, gravity);
        }
        glfwDestroyWindow(window);
        glfwTerminate();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Azmuith Physics Lab: %s\n", error.what());
        glfwTerminate();
        return 1;
    }
    return 0;
}