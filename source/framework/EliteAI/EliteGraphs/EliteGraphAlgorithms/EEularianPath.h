#pragma once
#include <stack>

namespace Elite
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	template <class T_NodeType, class T_ConnectionType>
	class EulerianPath
	{
	public:

		EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		Eulerianity IsEulerian() const;
		std::vector<T_NodeType*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(int startIdx, std::vector<bool>& visited) const;
		bool IsConnected() const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template<class T_NodeType, class T_ConnectionType>
	inline EulerianPath<T_NodeType, T_ConnectionType>::EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template<class T_NodeType, class T_ConnectionType>
	inline Eulerianity EulerianPath<T_NodeType, T_ConnectionType>::IsEulerian() const
	{
		// If the graph is not connected, there can be no Eulerian Trail
		if (IsConnected() == false)
			return Eulerianity::notEulerian;

		// Count nodes with odd degree 
		auto activeNodes = m_pGraph->GetAllNodes();
		int oddCount = 0;
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			// checks if is an odd amount
			if (connections.size() & 1)
				oddCount++;
		}

		// A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddCount > 2)
			return Eulerianity::notEulerian;

		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// An Euler trail can be made, but only starting and ending in these 2 nodes
		if (oddCount == 2 && activeNodes.size() != 2)
			return Eulerianity::semiEulerian;

		// A connected graph with no odd nodes is Eulerian
		return Eulerianity::eulerian;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline std::vector<T_NodeType*> EulerianPath<T_NodeType, T_ConnectionType>::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges (do not change original graph)
		auto graphCopy = m_pGraph->Clone();
		auto path = vector<T_NodeType*>();
		int nrOfNodes = graphCopy->GetNrOfNodes();
		stack<int> nodeStack;
		int currentIndex{};


		// Check if there can be an Euler path
		// If this graph is not eulerian, return the empty path
		// Else we need to find a valid starting index for the algorithm
		if (eulerianity == Eulerianity::notEulerian) return path;

		// Start algorithm loop

		// 1.----------------------
		// see the amount of degrees aka connections that the nodes have
		// if all nodes have an even amount of connections, choose any
		// otherwise if there are exactly 2 nodes with an odd amount of connections choose one of them
		// otherwise no euler path/circuit exists

		// odd count == 2

		auto activeNodes = graphCopy->GetAllNodes();

		if (eulerianity == Eulerianity::semiEulerian) {


			for (auto node : activeNodes)
			{
				auto connections = graphCopy->GetNodeConnections(node);
				// checks if is an odd amount
				if (connections.size() & 1)
					currentIndex = node->GetIndex();
			}

		};

		if (eulerianity == Eulerianity::eulerian)
		{
			currentIndex = activeNodes.at(0)->GetIndex();
		}

		// 2.----------------------
		// if current node has no neighbors
		// add it to the path
		// remove the last node from the stack
		// set the current node to this one

		auto currentNode = activeNodes.at(currentIndex);

		auto connections = graphCopy->GetNodeConnections(currentNode);

		// ALGORITHM LOOP
		while (connections.size() > 0 || nodeStack.size() > 0)
		{
			if (connections.size() < 1)
			{
				// if has no neighbors
				path.push_back(currentNode);

				// if the stack isn't empty, pop the last element
				//if (nodeStack.size() > 0)
				//{
				currentIndex = nodeStack.top();
				currentNode = activeNodes.at(currentIndex);
				nodeStack.pop();
				//}

			}
			else
			{
				// does have neighbors

				// add it to the stack
				nodeStack.push(currentIndex);

				auto neighborIndex = connections.front()->GetTo();

				// take a neighbor
				graphCopy.get()->RemoveConnection(connections.front());

				currentNode = activeNodes.at(neighborIndex);

				connections = graphCopy->GetNodeConnections(currentNode);

			}
		}

		// ELSE if it does have neighbors
		// add the node to the stack
		// take any of its neighbors
		// remove the edge between selected neighbor and that node
		// set that neighbor as the current node



		// repeat 2. until the current node has no more connections
		// and the stack is empty (while loop)
		// obtained path will be in reverse
		std::reverse(path.begin(), path.end());

		return path;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline void EulerianPath<T_NodeType, T_ConnectionType>::VisitAllNodesDFS(int startIdx, std::vector<bool>& visited) const
	{
		// mark the visited node
		visited[startIdx] = true;


		// recursively visit any valid connected nodes that were not visited before
		for (T_ConnectionType* connection : m_pGraph->GetNodeConnections(startIdx))
			if (visited[connection->GetTo()] == false)
				VisitAllNodesDFS(connection->GetTo(), visited);


	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool EulerianPath<T_NodeType, T_ConnectionType>::IsConnected() const
	{
		auto activeNodes = m_pGraph->GetAllNodes();
		vector<bool> visited(m_pGraph->GetNrOfNodes(), false);

		if (activeNodes.size() > 1 && m_pGraph->GetAllConnections().size() == 0)
			return false;

		// find a valid starting node that has connections
		int connectedIdx = invalid_node_index;
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			if (connections.size() != 0)
			{
				connectedIdx = node->GetIndex();
				break;
			}
		}

		// if no valid node could be found, return false
		if (connectedIdx == invalid_node_index)
			return false;

		// start a depth-first-search traversal from the node that has at least one connection
		VisitAllNodesDFS(connectedIdx, visited);

		// if a node was never visited, this graph is not connected
		for (auto node : activeNodes)
		{
			if (visited[node->GetIndex()] == false)
				return false;
		}

		return true;
	}

}