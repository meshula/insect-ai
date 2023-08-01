
#include "PMath.h"
#include "InsectAI.h"
#include "Clock.h"

#include "demo.h"

#include "raylib.h"

#define MAXDEMO 7

using PMath::randf;
using InsectAI::LightSensor;
using InsectAI::CollisionSensor;
using InsectAI::Actuator;
using InsectAI::Function;
using InsectAI::Switch;

enum GameMode {
    kNone, kDragging
};

void DrawUserPrompts(const char* name, const char* mouse, const char* keys)
{
    DrawText(name, 15, 20, 20, WHITE);
    DrawText(mouse, 15, 40, 20, WHITE);
    DrawText(keys, 15, 60, 20, WHITE);
}

Demo::Demo() :
	m_pNN(0),
	mCurrentDemo(0), m_AICount(0), m_DemoName(0), 
	mMousex(0), mMousey(0), mDemoMode(0)
{
	ClearAll();
}

Demo::~Demo() { 
	delete m_pNN;
}

void Demo::BuildTestBrain(InsectAI::Vehicle* pVehicle, uint32 brainType) {
	LightSensor* pLightSensor;
	CollisionSensor* pCollisionSensor;
	Actuator* pMotor;

	static uint32 funcs[3] = {Function::kBuffer, Function::kInvert, Function::kSigmoid};

	bool directional = (brainType > 3 && brainType < 8);

	switch (brainType) {

		// light activated or light seeking
		case 0:
		case 4:
			{	
				pVehicle->AllocBrain(1, 1);
				pLightSensor = new LightSensor(directional, mMaxBoundH);
				pMotor = new Actuator(Actuator::kMotor);
                pMotor->SetInput(pLightSensor);
				pVehicle->AddSensor(pLightSensor);
				pVehicle->AddActuator(pMotor);
			}
			break;

		// light activated or light seeking with a transfer function
		case 1:
		case 2:
		case 3:
		case 5:
		case 6:
		case 7:
			{
				pVehicle->AllocBrain(2, 1);
				pLightSensor = new LightSensor(directional, mMaxBoundH);
				Function* pFunc = new Function(funcs[(brainType-1) & 3]);
                pFunc->AddInput(pLightSensor);

				pMotor = new Actuator(Actuator::kMotor);
                pMotor->SetInput(pFunc);

				pVehicle->AddSensor(pFunc);
				pVehicle->AddSensor(pLightSensor);
				pVehicle->AddActuator(pMotor);
			}
			break;

		// light seeking, with collision avoidance
		case 8:
			{
				pVehicle->AllocBrain(3, 1);
				pLightSensor = new LightSensor(true, mMaxBoundH);		// directional light sensor
				pCollisionSensor = new CollisionSensor(mMaxBoundH * 0.1f);
				Switch* pSwitch = new Switch();
					pSwitch->SetControl(pCollisionSensor);
					pSwitch->SetInputs(pLightSensor, pCollisionSensor);

				pMotor = new Actuator(Actuator::kMotor);
					pMotor->SetInput(pSwitch);

				pVehicle->AddSensor(pCollisionSensor);
				pVehicle->AddSensor(pSwitch);
				pVehicle->AddSensor(pLightSensor);
				pVehicle->AddActuator(pMotor);
			}
			break;
	}
}


static void DrawDart(PMath::Vec2f p, float scale, float rotation, Color c) {
    Vector2 v[4] = { { 0.0f, 0.75f}, {-0.5f, -0.75f}, {0, -0.35f}, {0.5f, -0.75f} };

	float radians = rotation * 2.f * kPi / 360.f;
	// rotate and scale the points by rotation degrees
	for (int i = 0; i < 4; ++i) {
		float x = v[i].x * scale;
		float y = v[i].y * scale;
		v[i].x = x * cosf(radians) - y * sinf(radians);
		v[i].y = x * sinf(radians) + y * cosf(radians);
	}
	for (int i = 0; i < 4; ++i) {
		v[i].x += p[0];
		v[i].y += p[1];
	}
    
    for (int i = 0; i < 4; ++i) {
        DrawLineEx(v[i], v[(i+1)%4], 2, c);
    }
}

// static
void Demo::DrawCircle(PMath::Vec2f p, float r, Color c) {
    ::DrawRing((Vector2) { p[0], p[1]}, r, r + 1, 0, 360, 20, c);
}

// s
void Demo::DrawFilledCircle(PMath::Vec2f p, float r, Color c) {
    ::DrawCircle(p[0], p[1], r, c);
}

void Demo::ClearAll() {
	RemoveAllProxies();
	m_Engine.RemoveAllEntities();
	m_AICount = 0;
}

void Demo::CreateDemoZero_Zero() {
    m_DemoName = "Light Sensitive - linear response";
    ClearAll();
    DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
    m_State[m_AICount].m_Kind = kLight;
    m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
    m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
    DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
    m_State[m_AICount].m_Kind = kVehicle;
    m_State[m_AICount].m_Vehicle = pVehicle;
    m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
    BuildTestBrain(pVehicle, 0);
    pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
    m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDefaultDemo() {
	//CreateDemoZero_Zero();
    //CreateDemoZero_One();
    //CreateDemoTwo();
    CreateDemoThree();
}

void Demo::CreateDemoZero_One() {
	m_DemoName = "Light Sensitive with Buffer - delayed response";

    ClearAll();
    DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
    m_State[m_AICount].m_Kind = kLight;
    m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
    m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
    DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
    m_State[m_AICount].m_Vehicle = pVehicle;
    m_State[m_AICount].m_Kind = kVehicle;
    m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
    BuildTestBrain(pVehicle, 1);
    pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
    m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoZero_Two() {
	m_DemoName = "Light Sensitive with Inverter";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 2);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoZero_Three() {
	m_DemoName = "Light Sensitive with Threshold";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 3);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoOne() {
	m_DemoName = "Light Sensitive Comparison";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 0);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 1);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 2);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 3);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoTwo_Zero() {
	m_DemoName = "Light Seeking - linear response";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 4);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoTwo() {
	m_DemoName = "Light Seeking Comparison";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = 0.5f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = 0.5f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 4);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 5);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 6);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
	pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
		m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		BuildTestBrain(pVehicle, 7);
		pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
		m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateLightSeekingAvoider() {
	DemoVehicle* pVehicle = new DemoVehicle(&m_State[m_AICount]);;
		m_State[m_AICount].m_Vehicle = pVehicle;
	m_State[m_AICount].m_Kind = kVehicle;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
	BuildTestBrain(pVehicle, 8);
	pVehicle->mMaxSpeed = randf(0.8f, 1.0f);
	m_AI[m_AICount++] = m_Engine.AddEntity(pVehicle);
}

void Demo::CreateDemoThree() {
	m_DemoName = "Light Seeking with Collision Avoidance";
	ClearAll();
	DemoLight* pLight = new DemoLight(&m_State[m_AICount]);
		m_State[m_AICount].m_Kind = kLight;
		m_State[m_AICount].m_Rotation = k0;
    m_State[m_AICount].m_Position[0] = (0.8f * randf() * mMaxBoundH) + 0.1f * mMaxBoundH;
    m_State[m_AICount].m_Position[1] = (0.8f * randf() * mMaxBoundV) + 0.1f * mMaxBoundV;
		m_AI[m_AICount++] = m_Engine.AddEntity(pLight);

	for (int i = 0; i < 2; ++i) {
		CreateLightSeekingAvoider();
	}
}

void Demo::Reset() {
	ClearAll();
	mDemoMode = kNone;
	mPotentialPick = -1;
	mCurrentPick = -1;
}

bool Demo::HandleKey(int key) {
	bool handled = false;
	int i;

   	switch (key) {
		case (int) ' ':
			handled = true;
			Reset();
			++mCurrentDemo;
			if (mCurrentDemo > MAXDEMO)
				mCurrentDemo = 0;

			switch (mCurrentDemo) {
				case 0:		CreateDemoZero_Zero();		break;
				case 1:		CreateDemoZero_One();		break;
				case 2:		CreateDemoZero_Two();		break;
				case 3:		CreateDemoZero_Three();		break;
				case 4:		CreateDemoOne();			break;
				case 5:		CreateDemoTwo_Zero();		break;
				case 6:		CreateDemoTwo();			break;
				case 7:		CreateDemoThree();			break;
			}

			AddAllProxies();
			break;

		case (int) 'h':
			handled = true;
			mShowBrains = !mShowBrains;
			break;

		case (int) '=':
			// double the number of entities each time
			if (mCurrentDemo == 7) {
				int count = m_Engine.GetEntityCount() - 1; // subtract 1 for the sun
				if (count < 100) {
					for (i = 0; i < count; ++i) {
						CreateLightSeekingAvoider();
					}

					RemoveAllProxies();
					AddAllProxies();
				}
			}
			break;
	}
	return handled;
}

void Demo::DragEntity(int id) {
	PMath::Vec3f pos;
    pos[0] = mMousex;
    pos[1] = mMousey;
	//ConvertWindowCoordsToOrthoGL(mMousex, mMousey, pos[0], pos[1]);
	pos[2] = k0;
	PMath::Vec3fSet(m_State[id].m_Position, pos);
}

void Demo::ChoosePotentialPick() {
	float x, y;
    x = mMousex;
    y = mMousey;
	//ConvertWindowCoordsToOrthoGL(mMousex, mMousey, x, y);
	mPotentialPick = FindClosestEntity(x, y, mMaxBoundH * 0.1f);
}

void Demo::MouseMotion(int x, int y) {
	mMousex = (float) x; 
	mMousey = (float) y;

	if (mDemoMode == kDragging) {
		DragEntity(mCurrentPick);
	}
}

void Demo::MouseClick(eMouseButton button) {
	switch (button) {
		case eLeft:
			if (mPotentialPick != -1) {
				mDemoMode = kDragging;
				mCurrentPick = mPotentialPick;
			}
			break;
        default:
            break;
	}
}

void Demo::MouseUnclick(eMouseButton button) {
	switch (button) {
		case eLeft:
			mDemoMode = kNone;
			mCurrentPick = -1;
			break;
        default:
            break;
	}
}

void Demo::WrapAround(float left, float right, float bottom, float top) {
	for (int i = 0; i < m_AICount; ++i) {
		PhysState* pState = &m_State[i];

		if (pState->m_Position[0] > right)			pState->m_Position[0] = left;
		else if (pState->m_Position[0] < left)		pState->m_Position[0] = right;
		if (pState->m_Position[1] > top)			pState->m_Position[1] = bottom;
		else if (pState->m_Position[1] < bottom)	pState->m_Position[1] = top;
	}
}

void Demo::AddAllProxies() {
	if (m_pNN) {
		for (int i = 0; i < m_AICount; ++i) {
			m_pNN->AddProxy(&m_State[i]);
		}
	}
}

void Demo::RemoveAllProxies() {
	if (m_pNN) {
		for (int i = 0; i < m_AICount; ++i) {
				m_pNN->RemoveProxy(&m_State[i]);
		}
	}
}

void Demo::SetWindowSize(int width, int height, bool fullScreen) {
	//VOpenGLMain::SetWindowSize(width, height, fullScreen);				// mRight and mTop will get set
	PMath::Vec3f origin;
	origin[0] = origin[1] = origin[2] = k0;
	PMath::Vec3f dimensions;
	dimensions[0] = dimensions[1] = dimensions[2] = 2.5f;

	RemoveAllProxies();
	delete m_pNN;
	m_pNN = new NearestNeighbours(origin, dimensions, 10, 10, 10);
	AddAllProxies();
}


void Demo::Update(float dt) {
	RenderEntities();
	DrawUserPrompts(m_DemoName, "click to drag", (mCurrentDemo != 7) ? "keys: h, space" : "keys: h, space, =");

	ChoosePotentialPick();

	if (mCurrentPick >= 0 && mPotentialPick >= 0) {
		HighlightEntity(mPotentialPick, 1.0f, 0.7f, 0.2f, 0.2f);		// highlighted object
	}

	if (mCurrentPick >= 0) {
		HighlightEntity(mCurrentPick, 1.1f, 0.7f, 0.7f, 0.7f);		// highlighted object
	}

    DrawRectangleLines(mMaxBoundH - 200, 20, 180, 240, GREEN);
    if (mCurrentPick >= 0) {
        mInspectingPick = mCurrentPick;
    }
        
    if (mInspectingPick >= 0) {
        InsectAI::Entity* entity = m_Engine.GetEntity(m_AI[mInspectingPick]);
        if (entity) {
            int x = mMaxBoundH - 190;
            int left_x = x;
            int y = 30;
            DrawText(entity->name(), x, y, 15, WHITE);
            if (entity->GetKind() & InsectAI::kKindAgent) {
                y += 20;
                x += 8;
                InsectAI::Agent* pAgent = (InsectAI::Agent*) entity;
                int sc = pAgent->GetSensorCount();
                if (sc > 0) {
                    DrawText("SENSORS", x, y, 15, BLACK);
                    y += 20;
                    x += 8;
                    for (int i = 0; i < sc; ++i) {
                        InsectAI::Sensor* sens = pAgent->GetSensor(i);
                        DrawText(sens->name(), x,y, 15, WHITE);
                        float w = sens->mActivation * 40;
                        DrawRectangle(left_x - w + 8, y, w, 20, RED);
                        DrawRectangleLines(left_x - 40 + 8, y, 40, 20, WHITE);
                        y += 20;
                    }
                    x -= 8;
                }
                int ac = pAgent->GetActuatorCount();
                if (ac > 0) {
                    DrawText("ACTUATORS", x, y, 15, BLACK);
                    x += 8;
                    y += 20;
                    for (int i = 0; i < ac; ++i) {
                        InsectAI::Actuator* act = pAgent->GetActuator(i);
                        DrawText(act->name(), x,y, 15, WHITE);
                        float w = act->mActivation * 40;
                        DrawRectangle(left_x - w + 8, y, w, 10, RED);
                        DrawRectangleLines(left_x - 40 + 8, y, 40, 10, WHITE);
                        w = act->mSteeringActivation * 40;
                        DrawRectangle(left_x - w + 8, y + 10, w, 10, RED);
                        DrawRectangleLines(left_x - 40 + 8, y + 10, 40, 10, WHITE);
                        y += 20;
                    }
                    x -= 8;
                }
                x -= 8;
            }
        }
    }
    
	m_Engine.UpdateEntities(dt, this);

	MoveEntities();

	WrapAround(0.0f, mMaxBoundH, 0.0f, mMaxBoundV);
}

static void RenderCollisionSensor(CollisionSensor* pSensor, PMath::Vec2f pos, float scale) {
	// draw the collision activation
    Color color = { (unsigned char) (pSensor->mActivation * 255),
                    (unsigned char) (pSensor->mActivation * 255), 0, 255 };
	DrawCircleV((Vector2) {pos[0], pos[1]}, scale, color);

	// draw the sensor
	color = { 192, 192, 63, 255 };
	DrawCircleLines(pos[0], pos[1], scale, color);

	// draw the dart
	color = { 255, 192, 192, 255 };
	DrawDart(pos, scale * 0.5f, 90.0f, color);
}


static void RenderLightSensor(LightSensor* pLS, PMath::Vec2f pos, float scale) {
	// draw the sensor
	Color color = { 192, 192, 63, 255 };
	DrawCircleLines(pos[0], pos[1], 0.015f, color);
	if (pLS->mbDirectional) {
		// draw steering activation
		color = { 192, 192, 63, 255 };
		DrawLineEx((Vector2) {pos[0], pos[1]},
                   (Vector2) {pos[0] + pLS->mSteeringActivation * scale, pos[1]},
                   1.f, color);

		// draw the light activation
		color = { (unsigned char) (pLS->mActivation * 255),
                  (unsigned char) (pLS->mActivation * 255), 0, 255 };
		DrawCircleV((Vector2) {pos[0], pos[1]}, scale, color);
	}
	else {
		// draw the light activation
		color = { (unsigned char) (pLS->mActivation * 255),
                  (unsigned char) (pLS->mActivation * 255), 0, 255 };
		DrawRectangle(pos[0] - scale, pos[1] - scale, scale, scale, color);
	}
}

void RenderFunction(Function* pFunction, PMath::Vec2f p, float scale) {
	Demo::DrawFilledCircle(p, scale, BLACK);
	Demo::DrawCircle(p, scale, WHITE);

	switch (pFunction->mFunction) {
		case Function::kBuffer: {
			PMath::Vec2f pnts[4] = {
				{ -0.5f, 0.2f},
				{ +0.5f, 0.2f},
				{ +0.5f, -0.2f},
				{ -0.5f, -0.2f}
			};
			for (int i = 0; i < 4; ++i) {
				pnts[i][0] *= scale;
				pnts[i][1] *= scale;
				pnts[i][0] += p[0];
				pnts[i][1] += p[1];
			}
			for (int i = 0; i < 3; ++i)
				DrawLineEx((Vector2){pnts[i][0], pnts[i][1]}, (Vector2){pnts[i+1][0], pnts[i+1][1]}, 1.0f, WHITE);
		}
		break;
		case Function::kInvert: {
			PMath::Vec2f pnts[4] = {
				{ -0.5f, 0.2f},
				{ +0.5f, 0.2f},
			};
			for (int i = 0; i < 2; ++i) {
				pnts[i][0] *= scale;
				pnts[i][1] *= scale;
				pnts[i][0] += p[0];
				pnts[i][1] += p[1];
			}
			for (int i = 0; i < 1; ++i)
				DrawLineEx((Vector2){pnts[i][0], pnts[i][1]}, (Vector2){pnts[i+1][0], pnts[i+1][1]}, 1.0f, WHITE);
			break;
		}
		case Function::kSigmoid: {
			PMath::Vec2f pnts[5] = {
				{ -0.5f, 0.2f},
				{ +0.1f, 0.4f},
				{ +0.0f, 0.0f},
				{ -0.15f, -0.4f},
				{ -0.5f, -0.5f}
			};
			for (int i = 0; i < 5; ++i) {
				pnts[i][0] *= scale;
				pnts[i][1] *= scale;
				pnts[i][0] += p[0];
				pnts[i][1] += p[1];
			}
			for (int i = 0; i < 4; ++i)
				DrawLineEx((Vector2){pnts[i][0], pnts[i][1]}, (Vector2){pnts[i+1][0], pnts[i+1][1]}, 1.0f, WHITE);
			break;
		}
	}

	p[1] -= 0.9f;
	Color col;
	if (pFunction->mActivation >= 0.0f) {
		// yellow for positive
		col.r = pFunction->mActivation * 255;
		col.g = pFunction->mActivation * 255;
		col.b = 0;
		col.a = 255;
	}
	else {
		// red for negative
		col.r = pFunction->mActivation * 255;
		col.g = 0;
		col.b = 0;
		col.a = 255;
	}

	::DrawRectangle(p[0] - 0.5f * scale, p[1] - 0.5f * scale, scale, scale, col);
	::DrawRectangleLines(p[0] - 0.5f * scale, p[1] - 0.5f * scale, scale, scale, WHITE);
}

void RenderSwitch(InsectAI::Switch* pSwitch, PMath::Vec2f pos, float scale) {
	Demo::DrawFilledCircle(pos, scale, BLACK);
	Demo::DrawCircle(pos, scale, WHITE);

	// draw the switch
	float top = (pSwitch->mpSwitch->mActivation <= 0.5f) ? -0.5f : 0.5f;
	DrawLineEx((Vector2){pos[0], pos[1] + top}, (Vector2){pos[0], pos[1] - 0.5f}, 1.0f, WHITE);

	// draw the activation
	pos[1] -= 0.9f;
	Color col;
	if (pSwitch->mActivation >= 0.0f) {
		// yellow for positive
		col.r = pSwitch->mActivation * 255;
		col.g = pSwitch->mActivation * 255;
		col.b = 0;
		col.a = 255;
	}
	else {
		// red for negative
		col.r = pSwitch->mActivation * 255;
		col.g = 0;
		col.b = 0;
		col.a = 255;
	}
	DrawRectangle(pos[0] - 0.5f * scale, pos[1] - 0.5f * scale, scale, scale, col);
}


static void RenderSensor(InsectAI::Sensor* pSensor, PMath::Vec2f p, float scale) {
    if (pSensor->GetKind() == LightSensor::GetStaticKind())
        RenderLightSensor((LightSensor*) pSensor, p, scale);
	else if (pSensor->GetKind() == Switch::GetStaticKind())
        RenderSwitch((Switch*) pSensor, p, scale);
	else if (pSensor->GetKind() == CollisionSensor::GetStaticKind())
        RenderCollisionSensor((CollisionSensor*) pSensor, p, scale);
	else if (pSensor->GetKind() == Function::GetStaticKind())
        RenderFunction((Function*) pSensor, p, scale);
}

static void RenderSensor3(InsectAI::Sensor* pSensor, PMath::Vec3f p, float scale) {
    PMath::Vec2f pos = {p[0], p[1]};
    RenderSensor(pSensor, pos, scale);
}

static void RenderActuator(Actuator* pActuator, PMath::Vec2f pos, float scale) {
	Demo::DrawFilledCircle(pos, scale, BLACK);
	Demo::DrawCircle(pos, scale, WHITE);

	// draw an M for "motor"
	DrawLineEx((Vector2){pos[0] + 0.5f * scale, pos[1] + 0.5f * scale},
	           (Vector2){pos[0] + 0.5f * scale, pos[1] - 0.5f * scale}, 1.0f, WHITE);
	DrawLineEx((Vector2){pos[0] + 0.5f * scale, pos[1] - 0.5f * scale},
			   (Vector2){pos[0] + 0.0f * scale, pos[1] + 0.3f * scale}, 1.0f, WHITE);
	DrawLineEx((Vector2){pos[0] + 0.0f * scale, pos[1] + 0.3f * scale},
			   (Vector2){pos[0] - 0.5f * scale, pos[1] - 0.5f * scale}, 1.0f, WHITE);
	DrawLineEx((Vector2){pos[0] - 0.5f * scale, pos[1] - 0.5f * scale},
			   (Vector2){pos[0] - 0.5f * scale, pos[1] + 0.5f * scale}, 1.0f, WHITE);
}

static void RenderActuatorConnections(InsectAI::Actuator* pActuator) {
/*	if (pActuator->mpInput != 0) {
		glColor3f(0.7f, 0.0f, 0.0f);
		glBegin(GL_LINE_STRIP);
			glVertex2f(k0, k0);
			glVertex2f(k0, 0.05f);
		glEnd();
	}
*/
}

static void RenderSensorConnections(InsectAI::Sensor* pSensor) {
/*	if (pSensor->GetKind() == Switch::GetStaticKind()) {
		Switch* pSwitch = (Switch*) pSensor;

		float x = pSwitch->m_Position[0];
		float y = pSwitch->m_Position[1];

		glColor3f(0.7f, 0.0f, 0.0f);
		glBegin(GL_LINES);
			glVertex2f(pSwitch->mpA->m_Position[0],	pSwitch->mpA->m_Position[1]);
			glVertex2f(x, y);
			glVertex2f(pSwitch->mpB->m_Position[0],	pSwitch->mpB->m_Position[1]);
			glVertex2f(x, y);

			glColor3f(0.0f, 0.7f, 0.0f);
			glVertex2f(pSwitch->mpB->m_Position[0], pSwitch->mpB->m_Position[1]);
			glVertex2f(x + 0.025f, y);
			glVertex2f(x + 0.025f, y);
			glVertex2f(x, y);
		glEnd();
	}
	else if (pSensor->GetKind() == Function::GetStaticKind()) {
		Function* pFunction = (Function*) pSensor;
		int count = (int) pFunction->mInputs.size();
		glColor3f(0.7f, 0.0f, 0.0f);
		glBegin(GL_LINES);
		for (int i = 0; i < count; ++i ) {
			glVertex2f(pFunction->mInputs[i]->m_Position[0], pFunction->mInputs[i]->m_Position[1]);
			glVertex2f(pFunction->m_Position[0], pFunction->m_Position[1]);
		}
		glEnd();
	}
*/
}

void Demo::RenderVehicle(DemoVehicle* pVehicle, PhysState* pState) {
	if (pVehicle == 0)
		return;

    // draw a circle
    auto p = pState->GetPosition();
    //DrawFilledCircle((PMath::Vec2f) { p[0], p[1] }, mMaxBoundH / 64.f, YELLOW);
    //Demo::DrawCircle((PMath::Vec2f) { p[0], p[1] }, mMaxBoundH / 64.f, BLACK);

    float scale = 20.f;
    
    if (mShowBrains) {
/*        for (i = 0; i < pVehicle->GetSensorCount(); ++i) {
            RenderSensorConnections(pVehicle->m_Sensors[i]);
        }

        for (i = 0; i < pVehicle->GetActuatorCount(); ++i) {
            RenderActuatorConnections(pVehicle->m_Actuators[i]);
        }
*/

        float x = 0.f;
        for (int i = 0; i < pVehicle->GetSensorCount(); ++i) {
            auto p = pVehicle->m_pState->GetPosition();
            RenderSensor(pVehicle->GetSensor(i), (PMath::Vec2f) { p[0] + x, p[1] + scale }, scale * 0.5f);
            x += scale;
        }

        x = 0;
        for (int i = 0; i < pVehicle->GetActuatorCount(); ++i) {
            auto p = pVehicle->m_pState->GetPosition();
            RenderActuator(pVehicle->GetActuator(i),(PMath::Vec2f) { p[0] + x, p[1] + scale * 2.f }, scale * 0.5f);
            x += scale;
        }
    }
    
	Color c = { 63, 63, 63, 255 };
    DrawDart((PMath::Vec2f) { p[0], p[1] },
             scale, pState->m_Rotation * (-360.0f / (2.0f * kPi)), c);
}

void Demo::RenderLight(PhysState const*const pState) {
	if (pState == 0)
		return;

	// draw a circle
    auto p = pState->GetPosition();
    DrawFilledCircle((PMath::Vec2f) { p[0], p[1] }, mMaxBoundH / 64.f, YELLOW);
    Demo::DrawCircle((PMath::Vec2f) { p[0], p[1] }, mMaxBoundH / 64.f, BLACK);
}


void Demo::RenderEntities()
{
	for (int i = 0; i < m_AICount; ++i) {
		// render agent here
		if (m_State[i].m_Kind == kVehicle) {
			RenderVehicle(m_State[i].m_Vehicle, &m_State[i]);
		}
		else if (m_State[i].m_Kind == kLight) {
			RenderLight(&m_State[i]);
		}
	}
}

void Demo::HighlightEntity(int id, float radius, float red, float green, float blue)
{
	radius *= 0.02f;
	Demo::DrawCircle(m_State[id].m_Position, radius, WHITE);
}

static void MoveVehicle(DemoVehicle* pVehicle, PhysState* pState)
{
    const float kSteeringSpeed = 0.0025f;

	// connect actuators to physics
	for (int i = 0; i < pVehicle->GetActuatorCount(); ++i) {
		Actuator* pActuator = pVehicle->GetActuator(i);
		switch (pActuator->GetKind()) {
			case Actuator::kMotor:
				pState->m_Rotation += kSteeringSpeed * pActuator->mSteeringActivation;
				//mRotation = 0.25f * kPi;
				if (pState->m_Rotation < 0.0f) pState->m_Rotation += 2.0f * kPi;
				else if (pState->m_Rotation > 2.0f * kPi) pState->m_Rotation -= 2.0f * kPi;

                const float forceScale = 1.f; // @TODO make this physical
				float activation = forceScale * pActuator->mActivation;
                // rotation of zero moves forward on y axis
				pState->m_Position[0] += pVehicle->mMaxSpeed * sinf(pState->m_Rotation) * activation;
				pState->m_Position[1] += pVehicle->mMaxSpeed * cosf(pState->m_Rotation) * activation;
				break;
		}
	}
}

void Demo::MoveEntities()
{
	for (int i = 0; i < m_AICount; ++i) {
		// render agent here
		if (m_State[i].m_Kind == kVehicle) {
			MoveVehicle(m_State[i].m_Vehicle, &m_State[i]);
			m_pNN->UpdateProxy(&m_State[i]);
		}
	}
}

int Demo::FindClosestEntity(float x, float y, float maxDistance)
{
	int best = -1;

	maxDistance *= maxDistance;
	float bestDistance = 1.0e7f;

	for (int i = 0; i < m_AICount; ++i) {
		float distSquared = m_State[i].DistanceSquared(x, y);

		if (distSquared < bestDistance && distSquared < maxDistance) {
			best = i;
			bestDistance = distSquared;
		}
	}

	return best;
}


InsectAI::DynamicState* Demo::GetNearest(InsectAI::Entity* pE, uint32 filter)
{
    InsectAI::DynamicState* pRetVal = 0;
    if (filter != 0) {
        Real nearest = 1.0e6f;
        Real radius;

        PhysState* pState = (PhysState*) pE->GetDynamicState();

        // if it's a light, simply search the entire database for the closest light
        // (lq is not that fast when the search radius is similar to the size of the database)
        if ((filter & kLight) != 0) {
            for (int i = 0; i < m_AICount; ++i) {
                if ((m_State[i].m_Kind & filter) != 0) {
                    if (m_State[i].m_Vehicle != pE) {
                        Real distSquared = pState->DistanceSquared(&m_State[i]);
                        if (distSquared < nearest) {
                            nearest = distSquared;
                            pRetVal = &m_State[i];
                        }
                    }
                }
            }
        }
        else {
            if (!m_pNN) {
                fprintf(stderr, "nearest neighbour object not initialized\n");
                exit(EXIT_FAILURE);
            }
            radius = 150;//0.15f;    // this should really account for 2 * maximum velocity of a bug
            PhysState* pNearest = (PhysState*) m_pNN->FindNearestNeighbour(pState->GetPosition(), radius, filter, pState);
            return pNearest;

            // brute force search for comparison
            for (int i = 0; i < m_AICount; ++i) {
                if ((m_State[i].m_Kind & filter) != 0) {
                    if (m_State[i].m_Vehicle != pE) {
                        Real distSquared = pState->DistanceSquared(&m_State[i]);
                        if (distSquared < nearest) {
                            nearest = distSquared;
                            pRetVal = &m_State[i];
                        }
                    }
                }
            }
            return pNearest;
        }

    }
    return pRetVal;
}



int main(int argc, char **argv) 
{  
	int done = 0;
	TimeVal prevTime;
	TimeVal newTime;

	fprintf(stdout, "Starting Insect AI demo\n");

    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1000;
    const int screenHeight = 600;

    SetTargetFPS(60);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Demo of an ALife Architecture - by Nick Porcino");

	Clock myClock;
	myClock.update();
	prevTime = myClock.getSimulationTime();

	int width = screenWidth;
	int height = screenHeight;

	Demo* pDemo = new Demo();
    pDemo->mMaxBoundH = width;
    pDemo->mMaxBoundV = height;
	pDemo->Reset();
    pDemo->m_pNN = new NearestNeighbours((PMath::Vec3f) {0,0,0},
                                         (PMath::Vec3f) {(float)width, (float)height, 0},
                                          10, 10, 10);
    pDemo->CreateDefaultDemo();
    
    bool mouseDown = false;

    while (!WindowShouldClose()) {   // Detect window close button or ESC key
        
        if (IsKeyPressed(KEY_H))
            pDemo->HandleKey('h');
        else if (IsKeyPressed(KEY_ESCAPE))
            break;
        else if (IsKeyPressed(KEY_SPACE))
            pDemo->HandleKey(' ');
        else if (IsKeyPressed(KEY_EQUAL))
            pDemo->HandleKey('=');
        
        Vector2 mousePos = GetMousePosition();
        pDemo->MouseMotion((int) mousePos.x, (int) mousePos.y);
        
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (!mouseDown)
                pDemo->MouseClick(eLeft);
            mouseDown = true;
        }
        else {
            if (!mouseDown)
                pDemo->MouseUnclick(eLeft);
            mouseDown = false;
        }

        myClock.update();
		newTime = myClock.getSimulationTime();
		float dt = (float)(newTime - prevTime);
		prevTime = newTime;

        BeginDrawing();
        ClearBackground(GRAY);

		pDemo->Update(dt);

        EndDrawing();
	}
	return EXIT_SUCCESS;
}
