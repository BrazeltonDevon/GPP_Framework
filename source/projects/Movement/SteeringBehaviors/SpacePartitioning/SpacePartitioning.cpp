#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
	, m_CellWidth{ m_SpaceWidth / cols }
	, m_CellHeight{ m_SpaceHeight / rows }
{
	float xPos{}, yPos{ m_SpaceHeight };


	for (int i = 0; i < rows; i++)
	{
		// for every row

		// go down by 1 cell height at beginning of every row
		yPos -= m_CellHeight;
		xPos = 0;

		for (int j = 0; j < cols; j++)
		{
			// if not the first column cell, add width of a cell to position until end
			if (j > 0) xPos += m_CellWidth;
			// for every cell in the row add a cell to the vector
			m_Cells.push_back(Cell{ xPos, yPos, m_CellWidth, m_CellHeight });
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	Elite::Vector2 agentPos = agent->GetPosition();
	int index = PositionToIndex(agentPos);

	if (index > 0) m_Cells[index].agents.push_back(agent);

}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	int oldIndex = PositionToIndex(oldPos);
	int newIndex = PositionToIndex(agent->GetPosition());
	// remove old
	// 
	// was ==
	if (oldIndex != newIndex)
	{
		if (oldIndex >= 0)
		{
			m_Cells.at(oldIndex).agents.remove(agent);
			// once removed, update cell to new index
			AddAgent(agent);
		}
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius)
{
	// THIS CODE HAPPENS FOR ALL AGENTS IN UPDATE LOOP

	// local cache of cellspace, not the actual neighbors stored in the flock
	m_NrOfNeighbors = 0;
	m_Neighbors.clear();

	Elite::Vector2 bottomLeftPos;
	bottomLeftPos.x = agent->GetPosition().x - (queryRadius); // (radius /2)
	bottomLeftPos.y = agent->GetPosition().y - (queryRadius); // (radius /2) ;

	Elite::Rect boundingRect{ bottomLeftPos, queryRadius * 2, queryRadius * 2 };

	Elite::Color green{ 0.0f,1.0f,0.0f };

	for (size_t i = 0; i < m_Cells.size(); i++)
	{
		// get distance of this cell to the agent
		// if within query radius, cell is within range
		// take all agents within those cells and add
		// them to m_Neighbors
		// add ++nrNeighbors
		// if within neighborhood radius
		if (Elite::IsOverlapping(m_Cells[i].boundingBox, boundingRect))
		{
			// for every agent in the cell within radius
			//for (auto otherAgent : m_Cells[i].agents)
			for(auto size_t = m_Cells[i].agents.begin(); size_t != m_Cells[i].agents.end(); size_t++)
			{
				auto otherAgent = *size_t;
				// if not myself
				if (agent != otherAgent)
				{
					//Elite::Vector2 otherPos = otherAgent->GetPosition(), myPos{ agent->GetPosition() };
					//
					//Elite::Vector2 distanceVector = otherPos - myPos;
					//float distance = distanceVector.Magnitude();
					//
					//if (distance <= queryRadius)
					//{
					//	// is in range
					//	m_Neighbors.push_back(otherAgent);
					//	++m_NrOfNeighbors;
					//	// add to vector container of neighbors and to amount of neighbors
					//}

					m_Neighbors.push_back(otherAgent);
					++m_NrOfNeighbors;
				}

			}


			// ALSO DEBUG*********
			// if can render debug, color the cells in the neighborhood
			if (agent->CanRenderBehavior())
			{
				Elite::Polygon* tempPoly = new Elite::Polygon(m_Cells[i].GetRectPoints());

				DEBUGRENDERER2D->DrawPolygon(tempPoly, green, -0.87f);

				SAFE_DELETE(tempPoly);
			}
			// *********

		}
	}

	// DEBUG RENDER RADIUS AND HIGHLIGHT NEIGHBORS
	if (m_CanDebug && agent->CanRenderBehavior())
	{
		Cell boundingBox{ boundingRect.bottomLeft.x, boundingRect.bottomLeft.y, boundingRect.width, boundingRect.height };

		Elite::Polygon* boundRectPolygon{ new Elite::Polygon(boundingBox.GetRectPoints()) };

		// Draws the bounding box
		DEBUGRENDERER2D->DrawPolygon(boundRectPolygon, green);

		//for (SteeringAgent* neighbor : m_Neighbors)
		//{
		//	// highlights all the neighbors
		//	DEBUGRENDERER2D->DrawSolidCircle(neighbor->GetPosition(), 1.0f, { 0.f,0.f }, { 0.f,1.f,0.f }, -0.8f);
		//}

		SAFE_DELETE(boundRectPolygon);
	}

}

void CellSpace::RenderCells() const
{
	Elite::Color red{ 1.0f,0.0f,0.0f };

	for (Cell cellObject : m_Cells)
	{

		std::vector<Elite::Vector2>points = cellObject.GetRectPoints();

		DEBUGRENDERER2D->DrawSegment(points[0], points[1], red);
		DEBUGRENDERER2D->DrawSegment(points[1], points[2], red);
		DEBUGRENDERER2D->DrawSegment(points[2], points[3], red);
		DEBUGRENDERER2D->DrawSegment(points[3], points[0], red);

		DEBUGRENDERER2D->DrawString(points[1], std::to_string(cellObject.agents.size()).c_str());
	}

}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	for (size_t i = 0; i < m_Cells.size(); i++)
	{
		Elite::Rect cellBounds = m_Cells[i].boundingBox;

		// test for x bounds
		if (pos.x >= cellBounds.bottomLeft.x && pos.x <= cellBounds.bottomLeft.x + cellBounds.width)
		{
			// if is in x bounds
			// test for y bounds
			if (pos.y >= cellBounds.bottomLeft.y && pos.y <= cellBounds.bottomLeft.y + cellBounds.height)
			{
				// if is in y bounds
				// SUCCESS
				return i;
			}
		}
	}

	return -1;
	// return 0;
}