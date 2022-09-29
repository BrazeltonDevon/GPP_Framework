//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, { 0,1,0,1 });


	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	float distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	//if (distanceToTarget > m_FleeRadius)
	//{
	//	return SteeringOutput(Elite::ZeroVector2, 0.0f, false);
	//}

	auto steering = Seek::CalculateSteering(deltaT, pAgent);
	steering.LinearVelocity *= -1.0f;

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetLinearVelocity(), 5.0f, { 0,1,0,1 });

	return steering;
}

//ARRIVE
//****
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	const float maxSpeed = pAgent->GetMaxLinearSpeed();
	const float arrivalRadius = m_targetRadius;
	const float slowRadius = m_slowRadius;

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	const float distance = steering.LinearVelocity.Magnitude();
	if (distance < arrivalRadius)
	{
		steering.LinearVelocity = Elite::Vector2{ 0,0 };
		return steering;
	}

	Elite::Vector2 velocity = steering.LinearVelocity;

	velocity.Normalize();
	if (distance < slowRadius)
	{
		velocity *= maxSpeed * distance / slowRadius;
	}
	else
	{
		velocity *= maxSpeed;
	}

	steering.LinearVelocity = velocity;

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, { 0,1,0,1 });
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), arrivalRadius, { 1,0,0,1 }, 0);
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), slowRadius, { 0,0,1,1 }, 0);
	}


	return steering;
}

//FACE
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	Elite::Vector2 lineBetween = m_Target.Position - pAgent->GetPosition();
	float agentOrientation = float(pAgent->GetRotation()); // - E_PI2

	Elite::Vector2 agentDebugOrientation{};
	agentDebugOrientation.x = lineBetween.Magnitude() * cosf(agentOrientation);
	agentDebugOrientation.y = lineBetween.Magnitude() * sinf(agentOrientation);

	float angleBetween = Elite::AngleBetween(lineBetween, agentDebugOrientation);
	double angleBetweenDegrees = (angleBetween * (180.0 / 3.141592653589793238463));

	float slowdownAngle{ 4.0f };
	float arrivalAngle{ 0.1f };

	pAgent->SetAutoOrient(false);

	if (angleBetweenDegrees > arrivalAngle)
	{
		//steering.AngularVelocity = -pAgent->GetMaxAngularSpeed() * float(angleBetweenDegrees) / slowdownAngle;
		steering.AngularVelocity = -pAgent->GetMaxAngularSpeed() * angleBetween / slowdownAngle;
	}
	else if (angleBetweenDegrees < arrivalAngle)
	{
		steering.AngularVelocity = -pAgent->GetMaxAngularSpeed() * angleBetween / slowdownAngle;
	}
	else
	{
		steering.AngularVelocity = 0;
	}

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	{
		//std::cout << "Angle between target and agent: " << angleBetweenDegrees << std::endl;

		//DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetDirection(), 10.0f, { 0,1,0,1 });
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), lineBetween.Magnitude(), { 0,0,1,1 }, -0.8f);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), lineBetween, lineBetween.Magnitude(), { 0,1,0,1 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), agentDebugOrientation, lineBetween.Magnitude(), { 0,1,0,1 });
	}
	// lineBetween.Magnitude()

	return steering;
}

//WANDER
//******

SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	Elite::Vector2 targetPos{};
	Elite::Vector2 circleCenter{ m_OffsetDistance * pAgent->GetDirection().GetNormalized() };
	circleCenter += pAgent->GetPosition();

	auto randomGen = Elite::randomFloat(-m_MaxAngleChange, m_MaxAngleChange);
	auto randomAngle = randomGen * deltaT;

	// Add the random angle to the wander angle, making a new target on the circle
	m_WanderAngle += randomAngle;

	float xValue = m_Radius * cosf(m_WanderAngle);
	float yValue = m_Radius * sinf(m_WanderAngle);
	targetPos.x = circleCenter.x + xValue;
	targetPos.y = circleCenter.y + yValue;

	m_Target = targetPos;
	steering = Seek::CalculateSteering(deltaT, pAgent);

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, { 1,0,0,1 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), circleCenter, Elite::Vector2{circleCenter - pAgent->GetPosition()}.Normalize(), { 0,1,0,1 });
		DEBUGRENDERER2D->DrawCircle(circleCenter, m_Radius, { 0,0,1,1 }, -0.8f);
		DEBUGRENDERER2D->DrawPoint(targetPos, 5.0f, { 1,0,0,1 });
	}



	return steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	//// find future position of target
	Elite::Vector2 targetPos{ m_Target.Position + m_Target.LinearVelocity };

	steering.LinearVelocity = targetPos - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * 0.7f;

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	{
		// my linear velocity
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, { 0,1,0,1 });
		// enemy predicted position
		DEBUGRENDERER2D->DrawDirection(m_Target.Position, targetPos - m_Target.Position, 5.0f, { 1,0,0,1 });
	}

	return steering;

}

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	float distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTarget > m_FleeRadius)
	{
		return SteeringOutput(Elite::ZeroVector2, 0.0f, false);
	}

	//// find future position of target
	Elite::Vector2 targetPos{ m_Target.Position + m_Target.LinearVelocity };

	steering.LinearVelocity = targetPos - pAgent->GetPosition();
	steering.LinearVelocity *= -1;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * 1.2f;

	//DEBUG LINE
	if (pAgent->CanRenderBehavior())
	{
		// my linear velocity
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, { 0,1,0,1 });
		// enemy predicted position
		DEBUGRENDERER2D->DrawDirection(m_Target.Position, targetPos - m_Target.Position, 5.0f, { 1,0,0,1 });
	}

	return steering;

}

