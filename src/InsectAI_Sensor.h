
#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <vector>

namespace InsectAI {

/// @class	Sensor
/// @brief	Virtual base class for all sensors
///			Sensors are sensitive to particular kinds of agents
class Sensor {
public:
    Sensor() : m_Kind(0), m_SensedAgent(0), mbChooseClosest(false), mActivation(0.0f), mSteeringActivation(0.0f) {
    }

    virtual ~Sensor() { }

    enum ESensorWidth { kNearest, kAverage };

    void Reset();

    virtual void			Update(float dt) { }

    virtual void			Sense(DynamicState* pOriginState, DynamicState* pSenseeState) = 0;

            uint32			GetSensedAgentKind() const { return m_SensedAgent; }
            uint32			GetKind() const { return m_Kind; }
    virtual	ESensorWidth	GetSensorWidth() const = 0;

    virtual const char* name() const = 0;

    bool					mbDirectional;
    bool					mbInternalSensor;
    bool					mbClearEachFrame;
    bool					mbChooseClosest;						///< if false, accumulate. if true, choose closest
    float					mClosestDistance;
    float					mActivation;
    float					mSteeringActivation;

protected:
	uint32					m_Kind;									///< RTTI
	uint32					m_SensedAgent;							///< the kind of agent that can be sensed
};

/// @class	LightSensor
/// @brief	The LightSensor can sense Light Agents
class LightSensor : public Sensor {
    float mSensitiveRadius;
public:
	explicit LightSensor(bool directional, float radius)
    : Sensor()
    , mSensitiveRadius(radius) {
		m_Kind = GetStaticKind();
		m_SensedAgent = Light::GetStaticKind();
		mbDirectional = directional;
		mbClearEachFrame = true;
		mbInternalSensor = false;
	}

    static const  char* static_name() { return "Light Sensor"; }
    virtual const char* name() const override { return static_name(); }
    
    virtual ~LightSensor() = default;
    
	static uint32 GetStaticKind() { return 'Lght'; }
	virtual ESensorWidth GetSensorWidth() const override { return kNearest; }

	virtual void Sense(DynamicState* pOriginState, DynamicState* pSenseeState) override;
};

/// @class	CollisionSensor
/// @brief	Sensitive to collisions versus Vehicles
class CollisionSensor : public Sensor {
    float mSensitiveRadius;
public:
	explicit CollisionSensor(float radius);
    virtual ~CollisionSensor() = default;
	static uint32 GetStaticKind() { return 'Clld'; }

    static const  char* static_name() { return "Collision Sensor"; }
    virtual const char* name() const override { return static_name(); }

	virtual void Sense(DynamicState* pOriginState, DynamicState* pSenseeState) override;
	virtual ESensorWidth GetSensorWidth() const override { return kNearest; }
};


/// @class	Switch
/// @brief	A switch toggles to on when it's activation level exceeds 1/2
class Switch : public Sensor {
public:
	Switch();
	virtual ~Switch();
	static uint32 GetStaticKind() { return 'Swch'; }

	void SetControl(Sensor* pS) { mpSwitch = pS; }
	void SetInputs(Sensor* pA, Sensor* pB) { mpA = pA; mpB = pB; }
	virtual ESensorWidth GetSensorWidth() const override { return kNearest; }

	virtual void Sense(DynamicState* pOriginState, DynamicState* pSenseeState) override { }
	virtual void Update(float dt) override {
		bool useA = mpSwitch->mActivation <= 0.5f;
		mActivation = useA ? mpA->mActivation : mpB->mActivation;
		mSteeringActivation = useA ? mpA->mSteeringActivation : mpB->mSteeringActivation;
	}

    static const  char* static_name() { return "Switch"; }
    virtual const char* name() const override { return static_name(); }

	Sensor* mpSwitch;
	Sensor* mpA;
	Sensor* mpB;
};

/// @class	Function
/// @brief	a function is a sensor that senses one or more other sensors
class Function : public Sensor {
public:
	enum {
		kBuffer	= 'buff',
		kInvert	= 'nvrt',
		kSigmoid = 'sigm'
	};

	Function(uint32 func);
	virtual ~Function();
	static uint32 GetStaticKind() { return 'Func'; }
	virtual ESensorWidth GetSensorWidth() const override { return kNearest; }

	void AddInput(Sensor* pSensor) {
		mInputs.push_back(pSensor);
	}

	virtual void Update(float dt) override;

    virtual const char* name() const override {
        switch (mFunction) {
            case kBuffer: return "Buffer function";
            case kInvert: return "Inverter function";
            case kSigmoid: return "Sigmoid function";
            default: return "Unknown function";
        }
    }

	virtual void Sense(DynamicState* pOriginState, DynamicState* pSenseeState) override { }
	uint32 mFunction;
	std::vector <Sensor*> mInputs;
};

} // end namespace InsectAI

#endif
