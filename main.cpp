// main.cpp
//
// ICS 46 Spring 2016
// Project #4: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include <iostream>
#include "InputReader.hpp"
#include "RoadMapReader.hpp"
#include "RoadMapWriter.hpp"
#include "TripReader.hpp"
#include <iomanip>
#include <list>

//get_shortest_map for time or distance
std::map<int,int> get_shortest_map(std::map<int,int> child, int start, bool choice, RoadMap& map){
	if(choice == true)
		child = map.findShortestPaths(start, map.distance);
	else
		child = map.findShortestPaths(start, map.time);
	return child;
}

//prints shortest map
void print_map(std::map<int,int> map){
	for(auto key : map)
		std::cout << key.first << "->" << key.second << std::endl;
}

//format printing time
void print_time(int hours, int minutes, double secondsRemainder){
	if(hours > 0)
		std::cout << hours << " hours ";
	if(minutes > 0)
		std::cout << minutes << " mins ";
	std::cout << std::setprecision(2) << std::fixed << secondsRemainder << " secs";
}


void print_route(std::list<int> d_path, const RoadMap map, Trip cur){
	std::vector<int> v{std::begin(d_path), std::end(d_path)};
	std::cout << "Shortest Distance from " << map.vertexInfo(cur.startVertex) << "to " << map.vertexInfo(cur.endVertex) <<  std::endl;
	std::cout << "  Begin at " <<  map.vertexInfo(v[0]) << std::endl;
	double total_distance = 0.0;
	for(int i = 1; i < v.size(); i++){
		std::cout << "  Continue to " << map.vertexInfo(v[i]) << "(";
		std::cout << std::fixed << std::setprecision(1) << map.edgeInfo(v[i-1], v[i]).miles;
		std::cout << " miles)" << std::endl;
		total_distance += map.edgeInfo(v[i-1], v[i]).miles;
	}
	std::cout << "Total Distance: " << total_distance << " miles" << std::endl;
}

//handles print route time
void print_route_time(std::list<int> t_path, const RoadMap map, Trip cur){
	std::vector<int> v{std::begin(t_path), std::end(t_path)};
	std::cout << "Shortest Driving time from " << map.vertexInfo(cur.startVertex) << "to " << map.vertexInfo(cur.endVertex) <<  std::endl;
	std::cout << "  Begin at " <<  map.vertexInfo(v[0]) << std::endl;
	int total_hours = 0;
	int total_minutes = 0;
	double total_seconds = 0.00;
	for(int i = 1; i < v.size(); i++){
		double distance = map.edgeInfo(v[i-1], v[i]).miles;
		double rate = map.edgeInfo(v[i-1], v[i]).milesPerHour;
		double time = distance/rate;
		int hours = time;
		double minutesRemainder = (time - hours)*60;
		int minutes = minutesRemainder;
		double secondsRemainder = (minutesRemainder - minutes) * 60;
		int seconds = secondsRemainder;
		std::cout << "  Continue to " << map.vertexInfo(v[i]) << "(" << std::fixed << std::setprecision(1) << distance ;
		std::cout <<  " miles @ " << rate << "mph " <<  "= "; 
		print_time(hours, minutes, secondsRemainder);
		std::cout << ")" << std::endl;
		total_hours += hours;
		total_minutes += minutes;
		total_seconds += secondsRemainder;
	}

	while(total_seconds >= 60.00){
		total_minutes++;
		total_seconds  = total_seconds - 60.00;
	}
	std::cout << "Total time: ";
	print_time(total_hours, total_minutes, total_seconds);
}


int main()
{
	InputReader reader{std::cin};
	RoadMapReader readMap;
	RoadMap map = readMap.readRoadMap(reader);
	RoadMapWriter write;
	write.writeRoadMap(std::cout, map);

	TripReader trips;
	std::vector<Trip> all_trips = trips.readTrips(reader);
	std::map<int, int> d_child;
	std::map<int, int> t_child;

	bool choice;
	std::cout << std::endl;
	for(int i = 0; i < all_trips.size(); i++){
		choice = false;
		if(all_trips[i].metric == TripMetric::Distance){
			choice = true;
			std::list<int> d_path;
			d_child = get_shortest_map(d_child, all_trips[i].startVertex, choice, map );
			int cur = all_trips[i].endVertex;
			d_path.push_front(cur);
			while(cur != all_trips[i].startVertex){
				d_path.push_front(d_child[cur]);
				cur = d_child[cur];
			}
			print_route(d_path, map, all_trips[i]);
		}

		if(all_trips[i].metric == TripMetric::Time){
			std::list<int> t_path;
			t_child = get_shortest_map(t_child, all_trips[i].startVertex, choice, map);
			int cur = all_trips[i].endVertex;
			t_path.push_front(cur);
			while(cur != all_trips[i].startVertex){
				t_path.push_front(t_child[cur]);
				cur = t_child[cur];
			}
			print_route_time(t_path, map, all_trips[i]);
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
	return 0;
}

	