#include "stdafx.h"
#include "Flock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"
#include "../SpacePartitioning/SpacePartitioning.h"

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize, 
	float worldSize, 
	SteeringAgent* pAgentToEvade, 
	bool trimWorld)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld { trimWorld }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
{
	m_Agents.resize(m_FlockSize);

	m_Neighbors.resize(m_Agents.size());

	m_pEvadeBehavior = new Evade();
	m_pSeekBehavior = new Seek();
	m_pWanderBehavior = new Wander();
	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);
	m_pVelMatchBehavior = new VelocityMatch(this);

	m_NrCellRows = 15;
	m_NrCellColumns = 15;
	m_pCellSpace = new CellSpace(m_WorldSize, m_WorldSize, m_NrCellRows, m_NrCellColumns, m_FlockSize);
	m_CellSize = m_NeighborhoodRadius;

	SetPrioritySteering();
	float maxLin{ 20.0f };
	float maxAng{ 10.0f };

	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetMass(1.0f);
	m_pAgentToEvade->SetMaxLinearSpeed(maxLin);
	m_pAgentToEvade->SetMaxAngularSpeed(maxAng);
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetPosition({ 50.0f,60.0f });
	m_pAgentToEvade->SetSteeringBehavior(m_pWanderBehavior);
	m_pAgentToEvade->SetBodyColor(Elite::Color(1, 0, 0));

	for (int i{}; i < m_FlockSize; i++)
	{
		m_Agents[i] = new SteeringAgent();
		m_Agents[i]->SetAutoOrient(true);
		m_Agents[i]->SetPosition({ Elite::randomVector2(0,m_WorldSize) });
		m_Agents[i]->SetMass(1.0f);
		m_Agents[i]->SetMaxLinearSpeed(maxLin);
		m_Agents[i]->SetMaxAngularSpeed(maxAng);
		m_Agents[i]->SetSteeringBehavior(m_pPrioritySteering);
	}

	// Reserve space for old positions, they are set to default values
	// that way they can be accessed with an iterator
	m_oldPositions.resize(m_Agents.size());

}

Flock::~Flock()
{
	// TODO: clean up any additional data


	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);

	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);

	SAFE_DELETE(m_pAgentToEvade);

	SAFE_DELETE(m_pCellSpace);

	for (auto pAgent : m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();
}

void Flock::Update(float deltaT)
{
	// TODO: update the flock
	// loop over all the agents
		// register its neighbors	(-> memory pool is filled with neighbors of the currently evaluated agent)
		// update it				(-> the behaviors can use the neighbors stored in the pool, next iteration they will be the next agent's neighbors)
		// trim it to the world
	m_pAgentToEvade->Update(deltaT);
	m_pEvadeBehavior->SetTarget(m_pAgentToEvade->GetPosition());

	//for (auto agent : m_Agents)
	for (size_t i = 0; i < m_Agents.size(); i++)
	{
		SteeringAgent* agent = m_Agents[i];

		m_NrOfNeighbors = 0;
		m_Neighbors.clear();

		// Register neighbors 
		if (!m_UsePartitioning)
			RegisterNeighbors(agent);
		else
		{
			m_pCellSpace->RegisterNeighbors(agent, m_CellSize);
			auto cellneighbors = m_pCellSpace->GetNeighbors();
			
			for (auto neighbor : cellneighbors)
			{
				m_Neighbors.push_back(neighbor);
				++m_NrOfNeighbors;
			}

		}

		// Update agents
		agent->Update(deltaT);

		// TRIM TO WORLD*********
		if (m_TrimWorld)
		{
			//m_pCellSpace->SetSpaceSize(m_WorldSize, m_WorldSize);
			agent->TrimToWorld(Elite::Vector2(0, 0), Elite::Vector2(m_WorldSize, m_WorldSize));
		}

		// Update cell of agent
		if (m_UsePartitioning)
		{
			Elite::Vector2 oldPos{ m_oldPositions[i] };
			m_pCellSpace->UpdateAgentCell(agent, oldPos);
			// set old position to current position (gets the next loop ready)
			m_oldPositions[i] = agent->GetPosition();
		}
	}

	if(m_TrimWorld)
		m_pAgentToEvade->TrimToWorld(Elite::Vector2(0, 0), Elite::Vector2(m_WorldSize, m_WorldSize));
		
}

void Flock::Render(float deltaT)
{
	// TODO: render the flock
	/*for (auto agent : m_Agents)
	{
		if (agent)
		{
			agent->Render(deltaT);
		}
	}*/

	m_pAgentToEvade->Render(deltaT);

	std::vector<Elite::Vector2> points =
	{
		{ 0, m_WorldSize },
		{ m_WorldSize, m_WorldSize },
		{ m_WorldSize, 0 },
		{ 0, 0 }
	};
	DEBUGRENDERER2D->DrawPolygon(&points[0], 4, { 1,0,0,1 }, 0.4f);

	// RENDER CELLSPACE
	if (m_RenderDebug && m_UsePartitioning)
	{
		m_pCellSpace->SetDebug(true);
		m_pCellSpace->RenderCells();
	}


	//DEBUG LINE
	if (m_Agents.back()->CanRenderBehavior())
	{

			for (auto agent : m_Neighbors)
			{
				DEBUGRENDERER2D->DrawSolidCircle(agent->GetPosition(), 1.0f, { 0.f,0.f }, { 0.f,1.f,0.f }, -0.8f);
				DEBUGRENDERER2D->DrawCircle(m_Agents.back()->GetPosition(), m_NeighborhoodRadius, Elite::Color(0, 1, 0), 0.87f);
				DEBUGRENDERER2D->DrawDirection(m_Agents.back()->GetPosition(), m_Agents.back()->GetLinearVelocity(), 5.0f, { 0,1,0,1 });
			}


	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming - Brazelton, Devon", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();

	ImGui::Checkbox("Trim World", &m_TrimWorld);
	if (m_TrimWorld)
	{
		ImGui::SliderFloat("Trim Size", &m_WorldSize, 0.f, 200.f, "%.1");
	}
	ImGui::Spacing();
	ImGui::Spacing();
	// CAN USE SPACIAL PARTITIONING
	ImGui::Checkbox("Use Partitioning", &m_UsePartitioning);

	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.0f, 1.0f, "%.2");
	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.0f, 1.0f, "%.2");
	ImGui::SliderFloat("Velocity Match", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.0f, 1.0f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.0f, 1.0f, "%.2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.0f, 1.0f, "%.2");

	ImGui::Spacing();

	ImGui::Checkbox("Render Debug", &m_RenderDebug);
	m_Agents.back()->SetRenderBehavior(m_RenderDebug);

	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	// for every agent

		for (auto otherAgent : m_Agents)
		{
			// loop over the agents other than the one currently being evaluated
			if (pAgent != otherAgent)
			{
				// if the agent is not the one being evaluated
				Elite::Vector2 otherPos = otherAgent->GetPosition(), myPos{ pAgent->GetPosition() };

				Elite::Vector2 distanceVector = otherPos - myPos;
				float distance = distanceVector.Magnitude();

				if (distance <= m_NeighborhoodRadius)
				{
					// is in range
					m_Neighbors.push_back(otherAgent);
					++m_NrOfNeighbors;
					// add to vector container of neighbors and to amount of neighbors
				}
			}
		}

}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	// TODO: Get average neighbor pos
	Elite::Vector2 avgPos{};

	// add all the position values and divide by the amount of values
	for (SteeringAgent* neighbor : m_Neighbors)
	{
		if (neighbor)
		{
			// go through all the neighbors and add to a var
			Elite::Vector2 pos{ neighbor->GetPosition() };
			avgPos.x += pos.x;
			avgPos.y += pos.y;
		}
	}
	avgPos.x /= m_NrOfNeighbors;
	avgPos.y /= m_NrOfNeighbors;

	return Elite::Vector2{ avgPos };
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	// TODO: Get average neighbor velocity
	Elite::Vector2 avgVelocity{};

	// add all the position values and divide by the amount of values
	for (auto neighbor : m_Neighbors)
	{
		// go through all the neighbors and add to a var
		Elite::Vector2 linVelo{ neighbor->GetLinearVelocity() };
		avgVelocity.x += linVelo.x;
		avgVelocity.y += linVelo.y;
	}
	avgVelocity.x /= m_NrOfNeighbors;
	avgVelocity.y /= m_NrOfNeighbors;

	return Elite::Vector2{ avgVelocity };

	return Elite::Vector2{};
}

void Flock::SetTarget_Seek(TargetData target)
{
	// TODO: Set target for seek behavior
	m_pSeekBehavior->SetTarget(target);
}


float* Flock::GetWeight(ISteeringBehavior* pBehavior) 
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}

void Flock::SetPrioritySteering()
{
	// Priority Steering
	// 
	// replace wander with blended behavior
	// blended(wander, seek,...)
	std::vector<BlendedSteering::WeightedBehavior> weightedSteeringBehaviors;

	weightedSteeringBehaviors.push_back({ m_pCohesionBehavior, 0.1f });
	weightedSteeringBehaviors.push_back({ m_pSeparationBehavior, 0.1f });
	weightedSteeringBehaviors.push_back({ m_pVelMatchBehavior, 0.1f });
	weightedSteeringBehaviors.push_back({ m_pWanderBehavior, 0.9f });
	weightedSteeringBehaviors.push_back({ m_pSeekBehavior, 0.1f });
	m_pBlendedSteering = new BlendedSteering(weightedSteeringBehaviors);

	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior, m_pBlendedSteering });
}