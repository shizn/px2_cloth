#pragma once
#include<NxCooking.h>
#include<NxPhysics.h>

#include "MyFluid.h"
#include "ParticleFactory.h"
#include<stdio.h>
#include "Stream.h"

NxScene *gScene = NULL;

NxArray<MyFluid*> gMyFluids;

// Fluid globals
NxFluid* gFluid = NULL;
const int max_particles = 30000;
// Fluid Update 
struct ParticleUpdateSDK
{
    NxVec3	force;
    NxU32   flags;
};
static ParticleUpdateSDK* gUpdates = NULL;

// Fluid particle globals
NxVec3 gParticleBuffer[10000];
NxU32 gParticleBufferCap = 10000;
NxU32 gParticleBufferNum = 0;
#define REST_PARTICLES_PER_METER 10
#define KERNEL_RADIUS_MULTIPLIER 1.8
//#define MOTION_LIMIT_MULTIPLIER (3*KERNEL_RADIUS_MULTIPLIER)
#define MOTION_LIMIT_MULTIPLIER 3
#define PACKET_SIZE_MULTIPLIER 8



NxActor* CreateSphere(const NxVec3& pos, const NxReal radius, const NxReal density);
NxActor* CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal density);
NxActor* CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal density);
NxFluid* CreateFluid(const NxVec3& pos, NxU32 sideNum, NxReal distance, NxScene* scene);
int main(int argc, char **argv)
{
	//init and PVD
	bool initialized = false;
	NxPhysicsSDK *physicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
	if (!physicsSDK)
		return 0;
	else
		physicsSDK->getFoundationSDK().getRemoteDebugger()->connect("localhost", 5425);
	physicsSDK->setParameter(NX_CONTINUOUS_CD, true);

	initialized = true;

	//create a scene
	bool sceneInit = false;
	NxSceneDesc sceneDesc;
	sceneDesc.gravity.set(0, -9.81f, 0);
	gScene = physicsSDK->createScene(sceneDesc);

	if (gScene != NULL)
		sceneInit = true;

	//create a plane
	{
		NxActorDesc actorDesc;
		NxPlaneShapeDesc planeDesc;
		//planeDesc.normal = NxVec3(0, 0, 1);
		//planeDesc.d = -10.0f;

		actorDesc.shapes.pushBack(&planeDesc);
		gScene->createActor(actorDesc);
	}

	//create material
	NxMaterial *defaultMaterial = gScene->getMaterialFromIndex(0);
	defaultMaterial->setRestitution(0.3f);
	defaultMaterial->setStaticFriction(0.5f);
	defaultMaterial->setDynamicFriction(0.5f);



	//create a box
	{
		NxActorDesc actorDesc;
		NxBodyDesc bodyDesc;
		bodyDesc.angularDamping = 0.5;
		bodyDesc.linearVelocity = NxVec3(1, 0, 0);
		actorDesc.body = &bodyDesc;

		NxBoxShapeDesc boxDesc;
		boxDesc.dimensions = NxVec3(2.0f, 3.0f, 4.0f);
		actorDesc.shapes.pushBack(&boxDesc);
		actorDesc.density = 10.0f;
		actorDesc.globalPose.t = NxVec3(10.0f, 10.0f, 10.0f);
		gScene->createActor(actorDesc)->userData = NULL;
	}

	//create a cloth
	{
		// Create the objects in the scene
		NxActor* sphere1 = CreateSphere(NxVec3(-1, 0, -0.5), 1, 10);
		NxActor* box1 = CreateBox(NxVec3(1, 0, -1), NxVec3(1, 1, 1), 10);
		NxActor* box2 = CreateBox(NxVec3(0, 6.5, 0), NxVec3(5, 0.5, 0.5), 10);
		NxActor* box3 = CreateBox(NxVec3(0, 6.5, -7), NxVec3(5, 0.5, 0.5), 10);

		box2->setLinearDamping(5);
		box3->setLinearDamping(5);
		NxD6JointDesc d6Desc;
		d6Desc.actor[0] = NULL;
		d6Desc.actor[1] = box2;
		NxVec3 globalAnchor(0, 7, 0);
		d6Desc.localAnchor[0] = globalAnchor;
		box2->getGlobalPose().multiplyByInverseRT(globalAnchor, d6Desc.localAnchor[1]);
		box2->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

		box3->getGlobalPose().multiplyByInverseRT(globalAnchor, d6Desc.localAnchor[1]);
		box3->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

		d6Desc.localAxis[0] = NxVec3(1, 0, 0);
		d6Desc.localNormal[0] = NxVec3(0, 1, 0);
		d6Desc.localAxis[1] = NxVec3(1, 0, 0);
		d6Desc.localNormal[1] = NxVec3(0, 1, 0);

		d6Desc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
		d6Desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
		d6Desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
		d6Desc.xMotion = NX_D6JOINT_MOTION_FREE;
		d6Desc.yMotion = NX_D6JOINT_MOTION_FREE;
		d6Desc.zMotion = NX_D6JOINT_MOTION_FREE;

		NxJoint* d6Joint = gScene->createJoint(d6Desc);

		NxClothDesc clothDesc;
		clothDesc.globalPose.t = NxVec3(4, 7, 0);
		clothDesc.thickness = 0.2;
		//clothDesc.density = 1;
		clothDesc.bendingStiffness = 0.5;
		clothDesc.stretchingStiffness = 1;
		//clothDesc.dampingCoefficient = 0.5;
		clothDesc.friction = 0.5;
		//clothDesc.collisionResponseCoefficient = 1;
		//clothDesc.attachmentResponseCoefficient = 1;
		//clothDesc.solverIterations = 5;
		//clothDesc.flags |= NX_CLF_STATIC;
		//clothDesc.flags |= NX_CLF_DISABLE_COLLISION;
		//clothDesc.flags |= NX_CLF_VISUALIZATION;
		//clothDesc.flags |= NX_CLF_GRAVITY;
		clothDesc.flags |= NX_CLF_BENDING;
		//clothDesc.flags |= NX_CLF_BENDING_ORTHO;
		clothDesc.flags |= NX_CLF_DAMPING;
		//clothDesc.flags |= NX_CLF_COMDAMPING;
		clothDesc.flags |= NX_CLF_COLLISION_TWOWAY;

		clothDesc.flags &= ~NX_CLF_HARDWARE;
		clothDesc.flags |= NX_CLF_FLUID_COLLISION;
		clothDesc.selfCollisionThickness = 10.0f;


		NxReal w = 8;
		NxReal h = 7;
		NxReal d = 0.05;
		NxClothMeshDesc meshDesc;
		bool mInitDone = false;

		int numX = (int)(w / d) + 1;
		int numY = (int)(h / d) + 1;


		meshDesc.numVertices = (numX + 1) * (numY + 1);
		meshDesc.numTriangles = numX*numY * 2;
		meshDesc.pointStrideBytes = sizeof(NxVec3);
		meshDesc.triangleStrideBytes = 3 * sizeof(NxU32);
		meshDesc.vertexMassStrideBytes = sizeof(NxReal);
		meshDesc.vertexFlagStrideBytes = sizeof(NxU32);
		meshDesc.points = (NxVec3*)malloc(sizeof(NxVec3)*meshDesc.numVertices);
		meshDesc.triangles = (NxU32*)malloc(sizeof(NxU32)*meshDesc.numTriangles * 3);
		meshDesc.vertexMasses = 0;
		meshDesc.vertexFlags = 0;
		meshDesc.flags = 0;

		int i, j;
		NxVec3 *p = (NxVec3*)meshDesc.points;
		for (i = 0; i <= numY; i++) {
			for (j = 0; j <= numX; j++) {
				p->set(-d*j, 0.0f, -d*i);
				p++;
			}
		}

		NxU32 *id = (NxU32*)meshDesc.triangles;
		for (i = 0; i < numY; i++) {
			for (j = 0; j < numX; j++) {
				NxU32 i0 = i * (numX + 1) + j;
				NxU32 i1 = i0 + 1;
				NxU32 i2 = i0 + (numX + 1);
				NxU32 i3 = i2 + 1;
				if ((j + i) % 2) {
					*id++ = i0; *id++ = i2; *id++ = i1;
					*id++ = i1; *id++ = i2; *id++ = i3;
				}
				else {
					*id++ = i0; *id++ = i2; *id++ = i3;
					*id++ = i0; *id++ = i3; *id++ = i1;
				}
			}
		}
		// if we want tearing we must tell the cooker
		// this way it will generate some space for particles that will be generated during tearing
		if (meshDesc.flags & NX_CLF_TEARABLE)
			meshDesc.flags |= NX_CLOTH_MESH_TEARABLE;


		//cooking
		static NxCookingInterface *gCooking = 0;
		gCooking = NxGetCookingLib(NX_PHYSICS_SDK_VERSION);
		gCooking->NxInitCooking();

		gCooking->NxCookClothMesh(meshDesc, UserStream("e:\\cooked.bin", false));

		//Meshdata Buffers
		NxMeshData mReceiveBuffers;
		// here we setup the buffers through which the SDK returns the dynamic cloth data
		// we reserve more memory for vertices than the initial mesh takes
		// because tearing creates new vertices
		// the SDK only tears cloth as long as there is room in these buffers
		NxU32 numVertices = meshDesc.numVertices;
		NxU32 numTriangles = meshDesc.numTriangles;

		NxU32 maxVertices = 2 * numVertices;
		mReceiveBuffers.verticesPosBegin = (NxVec3*)malloc(sizeof(NxVec3)*maxVertices);
		mReceiveBuffers.verticesNormalBegin = (NxVec3*)malloc(sizeof(NxVec3)*maxVertices);
		mReceiveBuffers.verticesPosByteStride = sizeof(NxVec3);
		mReceiveBuffers.verticesNormalByteStride = sizeof(NxVec3);
		mReceiveBuffers.maxVertices = maxVertices;
		mReceiveBuffers.numVerticesPtr = (NxU32*)malloc(sizeof(NxU32));

		// the number of triangles is constant, even if the cloth is torn
		NxU32 maxIndices = 3 * numTriangles;
		mReceiveBuffers.indicesBegin = (NxU32*)malloc(sizeof(NxU32)*maxIndices);
		mReceiveBuffers.indicesByteStride = sizeof(NxU32);
		mReceiveBuffers.maxIndices = maxIndices;
		mReceiveBuffers.numIndicesPtr = (NxU32*)malloc(sizeof(NxU32));

		// the parent index information would be needed if we used textured cloth
		NxU32 maxParentIndices = maxVertices;
		mReceiveBuffers.parentIndicesBegin = (NxU32*)malloc(sizeof(NxU32)*maxParentIndices);
		mReceiveBuffers.parentIndicesByteStride = sizeof(NxU32);
		mReceiveBuffers.maxParentIndices = maxParentIndices;
		mReceiveBuffers.numParentIndicesPtr = (NxU32*)malloc(sizeof(NxU32));

		// init the buffers in case we want to draw the mesh 
		// before the SDK as filled in the correct values
		*mReceiveBuffers.numVerticesPtr = 0;
		*mReceiveBuffers.numIndicesPtr = 0;



		clothDesc.clothMesh = physicsSDK->createClothMesh(UserStream("e:\\cooked.bin", true));
		clothDesc.meshData = mReceiveBuffers;

		NxCloth *mCloth;
		mCloth = gScene->createCloth(clothDesc);
		mCloth->attachToShape(*box2->getShapes(), NX_CLOTH_ATTACHMENT_TWOWAY);
		mCloth->attachToShape(*box3->getShapes(), NX_CLOTH_ATTACHMENT_TWOWAY);
	}

	//create fluid 1
	{
        //fluid = CreateFluid(NxVec3(0, 12, -3.5), 15, 0.1, gScene);
    }
    //create fluid 2
    {
        //Create a set of initial particles
        ParticleSDK*	initParticles = new ParticleSDK[max_particles];
        unsigned initParticlesNum = 0;

        //Create particle filled sphere in buffer.
        NxVec3 fluidPos(0, 2, 0);
        NxVec3 offsetPos(0, 12, -3.5);
        float distance = 0.1f;
        //#ifdef __PPCGEKKO__	
        //	unsigned sideNum = 12;
        //#else
        unsigned sideNum = 16;
        //#endif
        
        //Setup structure to pass initial particles.
        NxParticleData initParticleData;
        initParticlesNum = 0;
        initParticleData.numParticlesPtr = &initParticlesNum;
        initParticleData.bufferPos = &initParticles[0].position.x;
        initParticleData.bufferPosByteStride = sizeof(ParticleSDK);
        initParticleData.bufferVel = &initParticles[0].velocity.x;
        initParticleData.bufferVelByteStride = sizeof(ParticleSDK);

        CreateParticleSphere(initParticleData, max_particles, false, offsetPos, NxVec3(0, 0, 0), 0.0f, distance, sideNum);

        //Setup fluid descriptor
        NxFluidDesc fluidDesc;
        fluidDesc.maxParticles = max_particles;
        fluidDesc.kernelRadiusMultiplier = 2.0f;
        fluidDesc.restParticlesPerMeter = 10.0f;
        fluidDesc.motionLimitMultiplier = 3.0f;
        fluidDesc.packetSizeMultiplier = 8;
        fluidDesc.collisionDistanceMultiplier = 0.1;
        fluidDesc.stiffness = 50.0f;
        fluidDesc.viscosity = 40.0f;
        fluidDesc.restDensity = 1000.0f;
        fluidDesc.damping = 0.0f;
        fluidDesc.restitutionForStaticShapes = 0.0f;
        fluidDesc.dynamicFrictionForStaticShapes = 0.05f;
        fluidDesc.simulationMethod = NX_F_SPH;
        fluidDesc.flags &= ~NX_FF_HARDWARE;

        //Add initial particles to fluid creation.
        fluidDesc.initialParticleData = initParticleData;

        //Create user fluid.
        //- create NxFluid in NxScene
        //- setup the buffers to read from data from the SDK
        //- set NxFluid::userData field to MyFluid instance
        bool trackUserData = false;
        bool provideCollisionNormals = false;
        MyFluid* fluid = new MyFluid(gScene, fluidDesc, trackUserData, provideCollisionNormals, NxVec3(0.4f, 0.5f, 0.9f), 0.03f);
        assert(fluid);
        gMyFluids.pushBack(fluid);
    }
	//simulate
	for (int i = 0; i < 3000;i++)
	{
		gScene->simulate(1.0f / 60.f);
		gScene->flushStream();
		//GetPhysicsResults
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

        // update fluid status
        if (i == 400)
        {
            MyFluid* fluid = gMyFluids[0];
            const ParticleSDK* particles = fluid->getParticles();
            unsigned particlesNum = fluid->getParticlesNum();
            if (!gUpdates)
            {
                gUpdates = new ParticleUpdateSDK[max_particles];
            }
            for (unsigned i = 0; i < particlesNum; i++)
            {
                ParticleUpdateSDK& update = gUpdates[i];
                NxVec3& force = update.force;
                force.set(0, 0, 0);
                NxU32& flags = update.flags;
                if (i >= particlesNum/2)
                {
                    flags = 0;
                    flags |= NX_FP_DELETE;
                }
                else
                {
                    flags = 0;
                }

            }
            //在这里更改粒子的属性
            NxParticleUpdateData updateData;
            updateData.bufferFlag = &gUpdates[0].flags;
            updateData.bufferFlagByteStride = sizeof(ParticleUpdateSDK);
            fluid->getNxFluid()->updateParticles(updateData);
        }
	}
	

	//release
	if (physicsSDK != NULL)
	{
		if (gScene != NULL)
			physicsSDK->releaseScene(*gScene);
		gScene = NULL;
		NxReleasePhysicsSDK(physicsSDK);
		physicsSDK = NULL;
	}
	return 1;
}



NxActor* CreateSphere(const NxVec3& pos, const NxReal radius, const NxReal density)
{
	assert(0 != gScene);

	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = radius;
	sphereDesc.localPose.t = NxVec3(0, radius, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return gScene->createActor(actorDesc);
}

NxActor* CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal density)
{
	assert(0 != gScene);

	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(boxDim.x, boxDim.y, boxDim.z);
	boxDesc.localPose.t = NxVec3(0, boxDim.y, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return gScene->createActor(actorDesc);
}

NxFluid* CreateFluid(const NxVec3& pos, NxU32 sideNum, NxReal distance, NxScene* scene)
{
	// Create a set of particles
	gParticleBufferNum = 0;
	NxReal rad = sideNum*distance*0.5f;
	for (NxU32 i = 0; i < sideNum; i++)
	{
		for (NxU32 j = 0; j < sideNum; j++)
		{
			for (NxU32 k = 0; k < sideNum; k++)
			{
				NxVec3 p = NxVec3(i*distance, j*distance, k*distance);
				if (p.distance(NxVec3(rad, rad, rad)) < rad)
				{
					p += pos - NxVec3(rad, rad, rad);
					gParticleBuffer[gParticleBufferNum++] = p;
				}
			}
		}
	}
    // 先需要初始化大量粒子的参数
	// Set structure to pass particles, and receive them after every simulation step
	NxParticleData particles;
	//particles.maxParticles			= gParticleBufferCap;
	particles.numParticlesPtr = &gParticleBufferNum;
	particles.bufferPos = &gParticleBuffer[0].x;
	particles.bufferPosByteStride = sizeof(NxVec3);

    // 然后初始化流体的各项参数
	// Create a fluid descriptor
	NxFluidDesc fluidDesc;
	fluidDesc.maxParticles = gParticleBufferCap;
	fluidDesc.kernelRadiusMultiplier = KERNEL_RADIUS_MULTIPLIER;
	fluidDesc.restParticlesPerMeter = REST_PARTICLES_PER_METER;
	fluidDesc.motionLimitMultiplier = MOTION_LIMIT_MULTIPLIER;
	fluidDesc.packetSizeMultiplier = PACKET_SIZE_MULTIPLIER;
	fluidDesc.stiffness = 50;
	fluidDesc.viscosity = 22;
	fluidDesc.restDensity = 1000;
	fluidDesc.damping = 0;
	fluidDesc.restitutionForStaticShapes = 0.4;
	fluidDesc.dynamicFrictionForStaticShapes = 0.3;
	fluidDesc.collisionResponseCoefficient = 0.5f;
	fluidDesc.collisionDistanceMultiplier = 0.1f;
	fluidDesc.simulationMethod = NX_F_SPH; //NX_F_NO_PARTICLE_INTERACTION;
    // 将流体需要的粒子与上述初始化的粒子关联起来
	fluidDesc.initialParticleData = particles;
	fluidDesc.particlesWriteData = particles;
	fluidDesc.flags &= ~NX_FF_HARDWARE;
	fluidDesc.flags |= NX_FF_COLLISION_TWOWAY;

	NxFluid* fl = gScene->createFluid(fluidDesc);
	assert(fl != NULL);
	return fl;
}
