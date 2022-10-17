#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


// All combine into one steering behavior
//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	// seek neighborhood center
	Elite::Vector2 center{ m_pFlock->GetAverageNeighborPos() };
	m_Target = center;
	SteeringOutput steering = Seek::CalculateSteering(deltaT, pAgent);

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, Elite::Color(1, 0, 0), 0.89999f);
	}

	return steering;
}


//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	// flee based on distance from neighbors
	auto neighbors = m_pFlock->GetNeighbors();
	Elite::Vector2 linVect{};
	int amountNr = m_pFlock->GetNrOfNeighbors();
	float searchRadius = m_pFlock->GetNeighborhoodRadius();
	Elite::Vector2 differenceVector{};

	for (auto neighbor : neighbors)
	{
		// for each neighbor
		differenceVector = neighbor->GetPosition() - pAgent->GetPosition();
		//float scalar = searchRadius - differenceVector.Magnitude();
		float scalar = differenceVector.Magnitude();

		linVect += differenceVector.GetNormalized() * scalar;
	}

	//linVect.x /= amountNr;
	//linVect.y /= amountNr;

	steering.LinearVelocity = linVect.GetNormalized();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	steering.LinearVelocity *= -1.0f;

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, Elite::Color(1, 1, 0), 0.89999f);
	}

	return steering;

}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	// seek neighborhood average velocity
	Elite::Vector2 center{ m_pFlock->GetAverageNeighborVelocity() };
	m_Target = center;
	SteeringOutput steering = Seek::CalculateSteering(deltaT, pAgent);

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.0f, Elite::Color(0, 0, 1), 0.89999f);
	}

	return steering;
}
