// Digraph.hpp
//
// ICS 46 Spring 2016
// Project #4: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <functional>
#include <list>
#include <map>
#include <algorithm>
#include <stack>
#include <queue>
#include <utility>
#include <vector>
#include <limits>
#include <iostream>


// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException
{
public:
    DigraphException(const std::string& reason): reason_{reason} { }

    std::string reason() const { return reason_; }

private:
    std::string reason_;
};

class Compare
{
public:
	bool operator()(std::pair<int, int> a, std::pair<int, int> b){ return a.second > b.second;}
};



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a template
// struct.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a template struct.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph();

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;
    //prints the graph
    void print();

    //edgeweight function for distance
    static double distance(const EdgeInfo&);
    //edgeweight function for time
    static double time(const EdgeInfo& einfo);
private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.

    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> graph;
    //checks if the graph has the vertex
 	bool has_vertex(int vertex);

 	//depth traversal for isStronglyConnected at the vertex
	bool depthFirstTraversal(int vertex) const;
	
    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.
};



// You'll need to define the member functions of your Digraph class
// template here.

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(){
}

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d){
	graph = d.graph;
	
}

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph(){
}

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d){
	if(this != &d){
		graph = d.graph;
	}
	return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const{
	std::vector<int> mykeys;
	for(auto it : graph)
		mykeys.push_back(it.first);
	return mykeys;
}

template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const{
	std::vector<std::pair<int, int>> temp_vertices;
	for(auto it:graph){
		for(auto _it: it.second.edges)
			temp_vertices.push_back(std::pair<int, int>(_it.fromVertex, _it.toVertex));
	}
	return temp_vertices;
}

template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const{
	try{
		std::vector<std::pair<int, int>> temp_vertices;
		bool found = false;
		for(auto element:graph){
			for(auto edge: element.second.edges){
				if(edge.fromVertex == vertex){
					temp_vertices.push_back(std::make_pair(edge.fromVertex, edge.toVertex));
					found = true;
				}
			}
		}
		if(!found)
			throw DigraphException("DigraphException(L283): vertex not in graph");
		else
			return temp_vertices;
	}catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}

template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const{
	try{
		for(auto it: graph){
			if(it.first == vertex)
				return it.second.vinfo;
		}
		throw DigraphException("DigraphException(L298): vertex not in graph");
	}catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}

template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const{
	try{
		for(auto it: graph){
			for(auto _it: it.second.edges){
				if((_it.fromVertex == fromVertex) && (_it.toVertex == toVertex))
					return _it.einfo;
			}
		}
		throw DigraphException("edge does not exist");
	}catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo){
	// std::cout << "testing" << std::endl;
	try{
		if(has_vertex(vertex)){
			throw DigraphException("Vertex already exists");
		}

		DigraphVertex<VertexInfo, EdgeInfo> entry;
		entry.vinfo = vinfo;
		graph.insert(std::make_pair(vertex, entry));
	}catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}

template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::has_vertex(int vertex){
	for(auto element : graph){
		if(element.first == vertex)
			return true;
	}
	return false;
}

template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::print(){
	for(auto element : graph){
		std::cout << "VERTEX:  " << element.first << std::endl;
		for(auto edge: element.second.edges){
			std::cout << "(" << edge.fromVertex << "->" << edge.toVertex << ")" << std::endl;
		}
	}
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo){
	try{
		if(!has_vertex(fromVertex) || !has_vertex(toVertex))
			throw DigraphException("DigraphException(L359): Vertex does not exist");

		DigraphEdge<EdgeInfo>* new_edge = new DigraphEdge<EdgeInfo>{fromVertex, toVertex, einfo};
		
		for(auto edge : graph[fromVertex].edges){
			// std::cout << "fromV: " << fromVertex << std::endl;
			// std::cout << "toV: " << toVertex << std::endl;
			if((edge.fromVertex == fromVertex) && (edge.toVertex == toVertex))
				throw DigraphException("DigraphException(L367): Edge already exists");
		}
		graph[fromVertex].edges.push_back(*new_edge);
		graph[toVertex].edges.push_back(*new_edge);
	}
	catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
	
}

template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex){
	try{
		if(!has_vertex(vertex))
			throw DigraphException("DigraphException(L382): Vertex does not exist");
		for(auto edge : graph[vertex].edges){
			removeEdge(edge.fromVertex, edge.toVertex);
		}
		graph.erase(vertex);
	}catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}

template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex) {
	try{
		if(!has_vertex(fromVertex) || !has_vertex(toVertex))
			throw DigraphException("DigraphException(L396): Vertex is not in graph");

		for(typename std::list<DigraphEdge<EdgeInfo>>::iterator it = graph[fromVertex].edges.begin(); it != graph[fromVertex].edges.end();){
			if(it->fromVertex == fromVertex){
				it = graph[fromVertex].edges.erase(it);
			}
			else{
				it++;
			}
		}
	}
	catch(DigraphException &e){
		std::cout << e.reason() << std::endl;
	}
}

template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const{
	return vertices().size();

}

template <typename VertexInfo, typename EdgeInfo>
int  Digraph<VertexInfo, EdgeInfo>::edgeCount() const{
	int edge_count = 0;
	for(auto it:graph){
		for(auto _it:it.second.edges){
			edge_count++;
		}
	}
	return edge_count;
}

template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const {
	int edge_count = 0;
	for(auto it:graph){
		if(it.first == vertex){
			for(auto _it: it.second.edges)
				edge_count++;
		}
	}
	return edge_count;
}

template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::depthFirstTraversal(int vertex) const {

		std::stack<int> stack; //its my stack
		std::map<int, bool> visited;
		std::vector<int> results;
		std::vector<int> keys(vertices());
		int current;
		bool edge_push;

		stack.push(vertex);
		for(auto key : keys){
			visited[key] = false;
		}

		while (!stack.empty()) {
			current = stack.top();
			edge_push = false;
			results.push_back(current); // collect element
			visited[current] = true; // mark as read
			for(auto edge : graph.at(current).edges){
				if(edge.fromVertex == current && !visited[edge.toVertex]){
					edge_push = true;
					stack.push(edge.toVertex);
				}
			}
			if(edge_push == false)
				stack.pop();
		}

		std::sort(results.begin(), results.end());
		results.erase(unique(results.begin(), results.end()), results.end());
		std::sort(keys.begin(), keys.end());
		return results == keys;
}

template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const{
	for(auto element : graph){
		if(!depthFirstTraversal(element.first)){
			return false;
		}
	}
	return true;
}

template <typename VertexInfo, typename EdgeInfo>
double Digraph<VertexInfo, EdgeInfo>::distance(const EdgeInfo& einfo){
	return einfo.miles;
}

template <typename VertexInfo, typename EdgeInfo>
double Digraph<VertexInfo, EdgeInfo>::time(const EdgeInfo& einfo){
	double mpr = einfo.milesPerHour;
	double miles = einfo.miles;
	return miles/mpr;
}

template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const{
	
	std::map <int, bool> remaining_vertices;
	std::map <int, int> distances;
	std::map <int, int> predecessor; //(v , parent)
	std::vector<std::pair<int,int>> testing;
	
	std::priority_queue<std::pair<int,int>, std::vector<std::pair<int,int>>, Compare> pq;

	for(auto element : graph){
		remaining_vertices[element.first] = false;
		if(element.first == startVertex){
			predecessor[element.first] = 0;
			distances[element.first] = 0;
			continue;
		}
		distances[element.first] = std::numeric_limits<int>::max();
		predecessor[element.first] = 0;
	}
	pq.push(std::make_pair(startVertex, distances[startVertex]));


	while(!pq.empty()){
		std::pair<int,int> vertex = pq.top();
		pq.pop(); //deque after accessing it
		if(!remaining_vertices[vertex.first])
		{
			remaining_vertices[vertex.first] = true;
			for(auto edge : graph.at(vertex.first).edges)
			{
				if(edge.fromVertex == vertex.first)
				{
					if(distances[edge.toVertex] > (distances[edge.fromVertex] + edgeWeightFunc(edge.einfo)))
					{
						// std::cout << "VERTEX.first = " << vertex.first << std::endl;
						distances[edge.toVertex] = distances[edge.fromVertex] + edgeWeightFunc(edge.einfo);
						predecessor[edge.toVertex] = vertex.first;
						pq.push(std::make_pair(edge.toVertex, distances[edge.toVertex]));
					}
				}
			}
		}
	}
	return predecessor;
}

#endif // DIGRAPH_HPP

