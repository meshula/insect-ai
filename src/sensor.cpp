

#include "PMath.h"
#include "InsectAI.h"

#include <math.h>

using PMath::randf;

namespace InsectAI {
	void Sensor::Reset() {
		if (mbClearEachFrame) {
			mActivation = 0.0f;
			mSteeringActivation = 0.0f;
		}
		mClosestDistance = 1.0e6f;
	}

	CollisionSensor::CollisionSensor(float radius)
    : Sensor()
    , mSensitiveRadius(radius)
    {
		m_Kind = GetStaticKind();
		m_SensedAgent = Vehicle::GetStaticKind();
		mbDirectional = true;
		mbClearEachFrame = true;
		mbInternalSensor = false;
		mbChooseClosest = true;
	}


	void CollisionSensor::Sense(DynamicState* pFrom, DynamicState* pTo) {
		PMath::Vec3f temp;
		PMath::Vec3fSet(temp, pTo->GetPosition());
		PMath::Vec3fSubtract(temp, pFrom->GetPosition());

        float distance = PMath::Vec2fLength(temp) / mSensitiveRadius;

		// if close enough to collide
        if (distance < 20) {//0.1f) {
			float activation = PMath::Max(0.0f, 1.0f - distance);
			activation *= activation;
			activation = PMath::Min(1.0f, activation);

			// if closer than something else we're avoiding
			if (distance < mClosestDistance) {
				float steeringActivation;

				PMath::Vec2fRotate(temp, pFrom->GetHeading());

				// if the possible collidee is in front of us, or simply very very close
				if ((distance < 0.5f) || (temp[1] > 0.0f)) {
					PMath::Vec3fScale(temp, k1 / distance);
					steeringActivation = -temp[0] + randf(0.0f, 0.1f);	// a tiny bit of noise

					temp[0] = 0.0f;
					temp[1] = 1.0f;
					PMath::Vec2fRotate(temp, pFrom->GetHeading());

					PMath::Vec2f temp2;
					temp2[0] = 0.0f;
					temp2[1] = 1.0f;
					PMath::Vec2fRotate(temp2, pTo->GetHeading());

					// if the collidee is heading towards us or simply very very close
					float dot = PMath::Vec2fDot(temp2, temp);
					if ((distance < 0.5f) || (dot < 0.0f)) {
						mClosestDistance = distance;
						mActivation = activation;
						mSteeringActivation = steeringActivation;
					}
				}
			}
		}
	}

void LightSensor::Sense(DynamicState* pFrom, DynamicState* pTo) {
	PMath::Vec3f temp;
	PMath::Vec3fSet(temp, pTo->GetPosition());
	PMath::Vec3fSubtract(temp, pFrom->GetPosition());

	float distance = PMath::Vec2fLength(temp) / mSensitiveRadius;
	float activation = PMath::Max(0.0f, 1.0f - distance);
	activation *= activation;
	activation = PMath::Min(1.0f, activation);

    float steeringActivation = 0.f;
	if (mbDirectional) {
        // remove sensitive radius, and scale by the distance metric
        float scaledActivation = PMath::Max(0.0f, 1.0f - distance);
        if (scaledActivation > 0) {
            PMath::Vec2fRotate(temp, pFrom->GetHeading());        // already normalized, so length won't change
            steeringActivation = temp[0] * scaledActivation + randf(0.0f, 0.1f);    // a tiny bit of noise
        }
	}

	if (mbChooseClosest) {
		if (distance < mClosestDistance) {
			mClosestDistance = distance;
			mActivation = activation;
			mSteeringActivation = steeringActivation;
		}
	}
	else {
		mSteeringActivation = steeringActivation;
		mActivation += activation;
	}
}



} // end namespace InsectAI
