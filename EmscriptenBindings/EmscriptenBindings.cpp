//
// Created by Oasis on 2023/3/21.
//

#include <emscripten.h>
#include <emscripten/bind.h>
//#include <PsSocket.h>

#include <chrono>

#include "binding/ActorBinding.h"
#include "binding/ControllerBinding.h"
#include "binding/CookingBinding.h"
#include "binding/JointBinding.h"
#include "binding/MathBinding.h"
//#include "binding/PVDBinding.h"
#include "binding/SceneBinding.h"
#include "binding/ShapeBinding.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

//----------------------------------------------------------------------------------------------------------------------
#define PVD_HOST "127.0.0.1"	

PxPhysics* CreateDefaultPhysics(PxFoundation &fun, PxTolerancesScale &scale){
        return PxCreatePhysics(PX_PHYSICS_VERSION,fun,scale,false,NULL,NULL);
}

PxPhysics* CreatePVDPhysics(PxFoundation &fun, PxTolerancesScale &scale,bool trackOutstandingAllocations,PxPvd* pvd){
        return PxCreatePhysics(PX_PHYSICS_VERSION,fun,scale,trackOutstandingAllocations,pvd,NULL);
}

bool InitDefaultExtensions(PxPhysics &physics){
        return PxInitExtensions(physics,NULL);
}

#if PX_DEBUG || PX_PROFILE || PX_CHECKED
//PxPvdTransport* CreatepvdTransport(int port, unsigned int timeoutInMilliseconds,PxPvd* pvd, PxPvdInstrumentationFlags flags){        
bool CreatepvdTransport(int port, unsigned int timeoutInMilliseconds,PxPvd* pvd){        
        PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, port, timeoutInMilliseconds);
        return pvd->connect(*transport,PxPvdInstrumentationFlag::eALL);  
}       
#endif


EMSCRIPTEN_BINDINGS(physx) {
    constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);
//    constant("PVD_HOST", PVD_HOST);
    //pvd
    #if PX_DEBUG || PX_PROFILE || PX_CHECKED
    function("PxCreatePvd",&PxCreatePvd,allow_raw_pointers());
    function("CreatepvdTransport",&CreatepvdTransport,allow_raw_pointers());
    class_<PxPvd>("PxPvd").function("connect", &PxPvd::connect);
    class_<PxPvdTransport>("PxPvdTransport");
    enum_<PxPvdInstrumentationFlag::Enum>("PxPvdInstrumentationFlag")
        .value("eDEBUG",PxPvdInstrumentationFlag::Enum::eDEBUG)
        .value("ePROFILE",PxPvdInstrumentationFlag::Enum::ePROFILE)
        .value("eMEMORY",PxPvdInstrumentationFlag::Enum::eMEMORY)
        .value("eALL",PxPvdInstrumentationFlag::Enum::eALL);
    #endif
    // Global functions
    // These are generally system/scene level initialization
    function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());
    function("PxInitExtensions", &PxInitExtensions,allow_raw_pointers());
    function("PxCloseExtensions", &PxCloseExtensions);
    function("PxDefaultCpuDispatcherCreate", &PxDefaultCpuDispatcherCreate, allow_raw_pointers());
    //function("PxCreatePhysics",optional_override([]()), allow_raw_pointers());
    function("CreateDefaultPhysics",&CreateDefaultPhysics,allow_raw_pointers());
    function("CreatePVDPhysics",&CreatePVDPhysics,allow_raw_pointers());
    function("InitDefaultExtensions",&InitDefaultExtensions);
    class_<PxAllocatorCallback>("PxAllocatorCallback");
    class_<PxDefaultAllocator, base<PxAllocatorCallback>>("PxDefaultAllocator").constructor<>();
    class_<PxTolerancesScale>("PxTolerancesScale")
            .constructor<>()
            .property("speed", &PxTolerancesScale::speed)
            .property("length", &PxTolerancesScale::length);

    class_<PxFoundation>("PxFoundation").function("release", &PxFoundation::release);

    /** PhysXPhysics ✅ */
    class_<PxPhysics>("PxPhysics")
            .function("release", &PxPhysics::release)
            .function("getTolerancesScale", &PxPhysics::getTolerancesScale)
            .function("createScene", &PxPhysics::createScene, allow_raw_pointers())
            .function("createShape",
                      select_overload<PxShape *(const PxGeometry &, const PxMaterial &, bool, PxShapeFlags)>(
                              &PxPhysics::createShape),
                      allow_raw_pointers())
            .function("createMaterial", &PxPhysics::createMaterial, allow_raw_pointers())
            .function("createRigidDynamic", &PxPhysics::createRigidDynamic, allow_raw_pointers())
            .function("createRigidStatic", &PxPhysics::createRigidStatic, allow_raw_pointers())
            .function("createFixedJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxFixedJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                    actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createRevoluteJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxRevoluteJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                       actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createSphericalJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxSphericalJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                        actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createDistanceJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxDistanceJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                       actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createPrismaticJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxPrismaticJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                        actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createD6Joint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxD6JointCreate(physics, actor0, PxTransform(localPosition0, localRotation0), actor1,
                                                 PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers());  // ✅

    class_<PxErrorCallback>("PxErrorCallback");
    class_<PxDefaultErrorCallback, base<PxErrorCallback>>("PxDefaultErrorCallback").constructor<>();

    class_<PxCpuDispatcher>("PxCpuDispatcher");
    class_<PxBVHStructure>("PxBVHStructure");
    class_<PxBaseTask>("PxBaseTask");
    class_<PxDefaultCpuDispatcher, base<PxCpuDispatcher>>("PxDefaultCpuDispatcher");

    class_<PxPairFlags>("PxPairFlags");
    class_<PxFilterFlags>("PxFilterFlags");
    enum_<PxPairFlag::Enum>("PxPairFlag");
    enum_<PxFilterFlag::Enum>("PxFilterFlag");
}

namespace emscripten {
namespace internal {
// Physx uses private destructors all over the place for its own reference counting
// embind doesn't deal with this well, so we have to override the destructors to keep them private
// in the bindings
// See: https://github.com/emscripten-core/emscripten/issues/5587
template <>
void raw_destructor<PxFoundation>(PxFoundation *) { /* do nothing */
}

template <>
void raw_destructor<PxBVHStructure>(PxBVHStructure *) { /* do nothing */
}
#if PX_DEBUG || PX_PROFILE || PX_CHECKED
template <>
void raw_destructor<PxPvd>(PxPvd *) { /* do nothing */
}

template <>
void raw_destructor<PxPvdTransport>(PxPvdTransport *) { /* do nothing */
}

template <>
void raw_destructor<PxPvdSceneClient>(PxPvdSceneClient *) { /* do nothing */
}
#endif
}  // namespace internal
}  // namespace emscripten
