#ifndef __ACTUATOR_H_
#define __ACTUATOR_H_

namespace InsectAI {

	class Sensor;

	/// @class	Actuator
	/// @brief	Actuators are things like steering wheels and motors
	class Actuator {
	public:
		enum { kMotor = 'Motr', kSteering = 'Ster' };

		Actuator(uint32 kind);
		virtual ~Actuator();

		void	Update(float dt);
		void	SetInput(Sensor* pSensor)	{ mpInput = pSensor; }
		uint32	GetKind() const				{ return mKind; }

        
		Sensor*					mpInput;
		float					mActivation, mSteeringActivation;

        virtual const char* name() const {
            switch (mKind) {
                case kMotor: return "Motor";
                case kSteering: return "Steering";
                default: return "Unknown Actuator";
            }
        }

        
	protected:
		uint32					mKind;
	};

} // end namespace InsectAI

#endif
