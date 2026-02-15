#include<bits/stdc++.h>
#include<cmath>
using namespace std;

double INF = 1e18;

struct Node {
    double lon, lat;
    
    bool operator<(const Node& other) const {
        if(lon != other.lon) return lon < other.lon;
        return lat < other.lat;
    }
};

struct Edge {
    int to;
    double dist;
    int mode;
};

map<Node, int> nodeToID;
vector<Node> idToNode;
map<int, string> idToStationName;
vector<vector<Edge>> graph;

int getID(Node p) {
    if(nodeToID.count(p)) {
        return nodeToID[p];
    }
    
    int id = nodeToID.size();
    nodeToID[p] = id;
    idToNode.push_back(p);
    graph.resize(id + 1);
    return id;
}

double haversine(Node p1, Node p2) {
    const double R = 6371.0;
    const double PI = 3.14159265358979323846;
    
    double lat1InRad = (p1.lat * PI) / 180.0;
    double lat2InRad = (p2.lat * PI) / 180.0;
    
    double latDiff = p2.lat - p1.lat;
    double lonDiff = p2.lon - p1.lon;
    
    double latDiffInRad = (latDiff * PI) / 180.0;
    double lonDiffInRad = (lonDiff * PI) / 180.0;
    
    double a = sin(latDiffInRad / 2) * sin(latDiffInRad / 2) +
               cos(lat1InRad) * cos(lat2InRad) *
               sin(lonDiffInRad / 2) * sin(lonDiffInRad / 2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return R * c;
}

double getCost(double distance, int mode) {
    if(mode == 1) return distance * 20.0;
    if(mode == 2) return distance * 5.0;
    if(mode == 3) return distance * 7.0;
    if(mode == 4) return distance * 10.0;
    return 0.0;
}

double getSpeed(int mode) {
    if(mode == 1) return 20.0;
    if(mode == 2) return 15.0;
    if(mode == 3) return 10.0;
    if(mode == 4) return 12.0;
    return 20.0;
}

double getNextDeparture(double arrivalTime, int mode) {
    if(mode == 1) return arrivalTime;
    
    if(mode == 2) {
        const double START_TIME = 60.0;
        const double END_TIME = 1380.0;
        const double INTERVAL = 5.0;
        
        if(arrivalTime < START_TIME) return START_TIME;
        if(arrivalTime >= END_TIME) return INF;
        
        double minutesSinceStart = arrivalTime - START_TIME;
        double intervalsPassed = ceil(minutesSinceStart / INTERVAL);
        double nextDeparture = START_TIME + intervalsPassed * INTERVAL;
        
        if(nextDeparture > END_TIME) return INF;
        return nextDeparture;
    }
    
    if(mode == 3) {
        const double START_TIME = 420.0;
        const double END_TIME = 1320.0;
        const double INTERVAL = 20.0;
        
        if(arrivalTime < START_TIME) return START_TIME;
        if(arrivalTime >= END_TIME) return INF;
        
        double minutesSinceStart = arrivalTime - START_TIME;
        double intervalsPassed = ceil(minutesSinceStart / INTERVAL);
        double nextDeparture = START_TIME + intervalsPassed * INTERVAL;
        
        if(nextDeparture > END_TIME) return INF;
        return nextDeparture;
    }
    
    if(mode == 4) {
        const double START_TIME = 360.0;
        const double END_TIME = 1380.0;
        const double INTERVAL = 10.0;
        
        if(arrivalTime < START_TIME) return START_TIME;
        if(arrivalTime >= END_TIME) return INF;
        
        double minutesSinceStart = arrivalTime - START_TIME;
        double intervalsPassed = ceil(minutesSinceStart / INTERVAL);
        double nextDeparture = START_TIME + intervalsPassed * INTERVAL;
        
        if(nextDeparture > END_TIME) return INF;
        return nextDeparture;
    }
    
    return arrivalTime;
}

pair<vector<int>, double> dijkstraCheapestWithDeadline(int startNode, int endNode, double startTime, double deadline) {
    int n = graph.size();
    
    vector<double> minCost(n, INF);
    vector<double> arrivalTime(n, INF);
    vector<int> parent(n, -1);
    
    minCost[startNode] = 0.0;
    arrivalTime[startNode] = startTime;
    
    priority_queue<tuple<double, double, int>, vector<tuple<double, double, int>>, greater<tuple<double, double, int>>> pq;
    pq.push({0.0, startTime, startNode});
    
    while(!pq.empty()) {
        auto [currentCost, currentTime, id] = pq.top();
        pq.pop();
        
        if(currentCost > minCost[id]) continue;
        
        if(currentTime >= deadline) continue;
        
        if(id == endNode) break;
        
        for(auto edge : graph[id]) {
            int neighborID = edge.to;
            double edgeDist = edge.dist;
            int mode = edge.mode;
            
            double edgeCost = getCost(edgeDist, mode);
            double speed = getSpeed(mode);
            double travelTime = (edgeDist / speed) * 60.0;
            
            double departureTime = currentTime;
            if(mode != 1) {
                departureTime = getNextDeparture(currentTime, mode);
                if(departureTime == INF) continue;
            }
            
            double newArrivalTime = departureTime + travelTime;
            
            if(newArrivalTime >= deadline) continue;
            
            double newCost = currentCost + edgeCost;
            
            if(newCost < minCost[neighborID] || 
               (newCost == minCost[neighborID] && newArrivalTime < arrivalTime[neighborID])) {
                minCost[neighborID] = newCost;
                arrivalTime[neighborID] = newArrivalTime;
                parent[neighborID] = id;
                pq.push({newCost, newArrivalTime, neighborID});
            }
        }
    }
    
    vector<int> path;
    if(minCost[endNode] == INF) {
        return {path, INF};
    }
    
    for(int curr = endNode; curr != -1; curr = parent[curr]) {
        path.push_back(curr);
    }
    
    reverse(path.begin(), path.end());
    return {path, minCost[endNode]};
}

void load_roadmap(string filename) {
    ifstream file(filename);
    if(!file.is_open()) {
        cerr << "Could not open file: " << filename << endl;
        exit(1);
    }
    
    string line;
    while(getline(file, line)) {
        if(line.empty()) continue;
        
        stringstream ss(line);
        string token;
        vector<string> tokens;
        
        while(getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if(tokens.size() < 5) continue;
        
        vector<Node> roadPoints;
        for(int i = 1; i < tokens.size() - 2; i += 2) {
            double lon = stod(tokens[i]);
            double lat = stod(tokens[i + 1]);
            roadPoints.push_back({lon, lat});
        }
        
        if(roadPoints.empty()) continue;
        
        for(int k = 0; k < roadPoints.size() - 1; k++) {
            Node u = roadPoints[k];
            Node v = roadPoints[k + 1];
            
            int uID = getID(u);
            int vID = getID(v);
            double w = haversine(u, v);
            
            graph[uID].push_back({vID, w, 1});
            graph[vID].push_back({uID, w, 1});
        }
    }
}

void loadTransit(string filename, int modeID) {
    ifstream file(filename);
    if(!file.is_open()) {
        cerr << "Could not open file: " << filename << endl;
        exit(1);
    }
    
    string line;
    while(getline(file, line)) {
        if(line.empty()) continue;
        
        stringstream ss(line);
        string token;
        vector<string> tokens;
        
        while(getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if(tokens.size() < 5) continue;
        
        string startStationName = tokens[tokens.size() - 2];
        string endStationName = tokens[tokens.size() - 1];
        
        vector<Node> roadPoints;
        for(int i = 1; i < tokens.size() - 2; i += 2) {
            double lon = stod(tokens[i]);
            double lat = stod(tokens[i + 1]);
            roadPoints.push_back({lon, lat});
        }
        
        if(roadPoints.empty()) continue;
        
        int firstNodeID = getID(roadPoints.front());
        int lastNodeID = getID(roadPoints.back());
        idToStationName[firstNodeID] = startStationName;
        idToStationName[lastNodeID] = endStationName;
        
        double totalDistance = 0.0;
        for(int k = 0; k < roadPoints.size() - 1; k++) {
            totalDistance += haversine(roadPoints[k], roadPoints[k + 1]);
        }
        
        graph[firstNodeID].push_back({lastNodeID, totalDistance, modeID});
    }
}

int getNearestNodeID(Node target) {
    if(nodeToID.count(target)) {
        return nodeToID[target];
    }
    
    int closestID = -1;
    double minDist = INF;
    
    for(int i = 0; i < idToNode.size(); i++) {
        double dist = haversine(target, idToNode[i]);
        if(dist < minDist) {
            minDist = dist;
            closestID = i;
        }
    }
    
    return closestID;
}

string formatNode(int id) {
    stringstream ss;
    ss << fixed << setprecision(6);
    
    if(idToStationName.count(id)) {
        ss << idToStationName[id] << " ";
    }
    
    ss << "(" << idToNode[id].lon << ", " << idToNode[id].lat << ")";
    return ss.str();
}

string formatTime(double minutes) {
    int totalMinutes = (int)round(minutes);
    int hours = (totalMinutes / 60) % 24;
    int mins = totalMinutes % 60;
    
    string period = (hours >= 12) ? "PM" : "AM";
    int displayHour = hours % 12;
    if(displayHour == 0) displayHour = 12;
    
    stringstream ss;
    ss << displayHour << ":" << setfill('0') << setw(2) << mins << " " << period;
    return ss.str();
}

double parseTime(string timeStr) {
    int hour, minute;
    char colon, ampm[3];
    sscanf(timeStr.c_str(), "%d:%d %s", &hour, &minute, ampm);
    
    if(strcmp(ampm, "PM") == 0 && hour != 12) hour += 12;
    if(strcmp(ampm, "AM") == 0 && hour == 12) hour = 0;
    
    return hour * 60.0 + minute;
}

int main() {
    load_roadmap("Roadmap-Dhaka.csv");
    loadTransit("Routemap-DhakaMetroRail.csv", 2);
    loadTransit("Routemap-BikolpoBus.csv", 3);
    loadTransit("Routemap-UttaraBus.csv", 4);
    
    Node source, destination;
    string startTimeStr, deadlineStr;
    
    cin >> source.lon >> source.lat;
    cin >> destination.lon >> destination.lat;
    cin.ignore();
    getline(cin, startTimeStr);
    getline(cin, deadlineStr);
    
    double startTime = parseTime(startTimeStr);
    double deadline = parseTime(deadlineStr);
    
    int startID = getNearestNodeID(source);
    int endID = getNearestNodeID(destination);
    
    double walkToStart = haversine(source, idToNode[startID]);
    double walkFromEnd = haversine(idToNode[endID], destination);
    
    double walkTimeToStart = (walkToStart / 5.0) * 60.0;
    double walkTimeFromEnd = (walkFromEnd / 5.0) * 60.0;
    
    double timeAtStartNode = startTime + walkTimeToStart;
    double adjustedDeadline = deadline - walkTimeFromEnd;
    
    pair<vector<int>, double> result = dijkstraCheapestWithDeadline(startID, endID, timeAtStartNode, adjustedDeadline);
    vector<int> shortestPath = result.first;
    double totalCost = result.second;
    
    if(shortestPath.empty()) {
        cout << "No path found that reaches destination before deadline." << endl;
        return 0;
    }
    
    cout << "\nProblem No: 6\n";
    cout << "Source: (" << fixed << setprecision(6) << source.lon << ", " << source.lat << ")\n";
    cout << "Destination: (" << fixed << setprecision(6) << destination.lon << ", " << destination.lat << ")\n";
    cout << "Starting time at source: " << startTimeStr << "\n";
    cout << "Destination reaching time: " << deadlineStr << "\n\n";
    
    double currentTime = startTime;
    double totalTripDistance = 0.0;
    double calculatedTotalCost = 0.0;
    
    if(walkToStart > 1e-6) {
        double endWalkTime = currentTime + walkTimeToStart;
        cout << formatTime(currentTime) << " - " << formatTime(endWalkTime) 
             << ", Cost: ৳0.00: Walk from Source (" << fixed << setprecision(6) 
             << source.lon << ", " << source.lat << ") to " << formatNode(startID) << ".\n";
        totalTripDistance += walkToStart;
        currentTime = endWalkTime;
    }
    
    int currentMode = -1;
    int segmentStartID = shortestPath[0];
    double segmentDistance = 0.0;
    double segmentCost = 0.0;
    double segmentStartTime = currentTime;
    
    for(int i = 0; i < shortestPath.size() - 1; i++) {
        int u = shortestPath[i];
        int v = shortestPath[i + 1];
        
        double edgeDist = 0.0;
        int edgeMode = 1;
        double minEdgeCost = INF;
        
        for(auto edge : graph[u]) {
            if(edge.to == v) {
                double eCost = getCost(edge.dist, edge.mode);
                if(eCost < minEdgeCost) {
                    minEdgeCost = eCost;
                    edgeDist = edge.dist;
                    edgeMode = edge.mode;
                }
            }
        }
        
        if(currentMode == -1) currentMode = edgeMode;
        
        if(edgeMode != currentMode) {
            double speed = getSpeed(currentMode);
            double travelTime = (segmentDistance / speed) * 60.0;
            double endTime = segmentStartTime + travelTime;
            
            string modeName;
            if(currentMode == 1) modeName = "Car";
            else if(currentMode == 2) modeName = "Metro";
            else if(currentMode == 3) modeName = "Bikolpo Bus";
            else if(currentMode == 4) modeName = "Uttara Bus";
            
            cout << formatTime(segmentStartTime) << " - " << formatTime(endTime)
                 << ", Cost: ৳" << fixed << setprecision(2) << segmentCost 
                 << ": Ride " << modeName << " from " << formatNode(segmentStartID) 
                 << " to " << formatNode(u) << ".\n";
            
            totalTripDistance += segmentDistance;
            calculatedTotalCost += segmentCost;
            currentTime = endTime;
            
            currentMode = edgeMode;
            segmentStartID = u;
            
            if(edgeMode != 1) {
                double departure = getNextDeparture(currentTime, edgeMode);
                if(departure > currentTime) {
                    segmentStartTime = departure;
                } else {
                    segmentStartTime = currentTime;
                }
            } else {
                segmentStartTime = currentTime;
            }
            
            segmentDistance = edgeDist;
            segmentCost = minEdgeCost;
        } else {
            segmentDistance += edgeDist;
            segmentCost += minEdgeCost;
        }
    }
    
    double speed = getSpeed(currentMode);
    double travelTime = (segmentDistance / speed) * 60.0;
    double endTime = segmentStartTime + travelTime;
    
    string finalModeName;
    if(currentMode == 1) finalModeName = "Car";
    else if(currentMode == 2) finalModeName = "Metro";
    else if(currentMode == 3) finalModeName = "Bikolpo Bus";
    else if(currentMode == 4) finalModeName = "Uttara Bus";
    
    int lastNode = shortestPath.back();
    cout << formatTime(segmentStartTime) << " - " << formatTime(endTime)
         << ", Cost: ৳" << fixed << setprecision(2) << segmentCost 
         << ": Ride " << finalModeName << " from " << formatNode(segmentStartID) 
         << " to " << formatNode(lastNode) << ".\n";
    
    totalTripDistance += segmentDistance;
    calculatedTotalCost += segmentCost;
    currentTime = endTime;
    
    if(walkFromEnd > 1e-6) {
        double endWalkTime = currentTime + walkTimeFromEnd;
        cout << formatTime(currentTime) << " - " << formatTime(endWalkTime)
             << ", Cost: ৳0.00: Walk from " << formatNode(endID) 
             << " to Destination (" << fixed << setprecision(6) 
             << destination.lon << ", " << destination.lat << ").\n";
        totalTripDistance += walkFromEnd;
        currentTime = endWalkTime;
    }
    
    cout << "\nTotal Distance: " << fixed << setprecision(3) << totalTripDistance << " km\n";
    cout << "Total Cost: ৳" << fixed << setprecision(2) << calculatedTotalCost << "\n";
    cout << "Arrival Time: " << formatTime(currentTime) << "\n";
    
    if(currentTime < deadline) {
        cout << "Status: Arrived before deadline ✓\n";
    } else {
        cout << "Status: Warning - might be close to deadline\n";
    }
    
    ofstream kmlFile("route6.kml");
    kmlFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kmlFile << "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n";
    kmlFile << "<Document>\n";
    kmlFile << "<Placemark>\n";
    kmlFile << "<name>route6.kml</name>\n";
    kmlFile << "<LineString>\n";
    kmlFile << "<tessellate>1</tessellate>\n";
    kmlFile << "<coordinates>\n";
    kmlFile << fixed << setprecision(6) << source.lon << "," << source.lat << ",0\n";
    for(int id : shortestPath) {
        kmlFile << fixed << setprecision(6) << idToNode[id].lon << "," << idToNode[id].lat << ",0\n";
    }
    kmlFile << fixed << setprecision(6) << destination.lon << "," << destination.lat << ",0\n";
    kmlFile << "</coordinates>\n";
    kmlFile << "</LineString>\n";
    kmlFile << "</Placemark>\n";
    kmlFile << "</Document>\n";
    kmlFile << "</kml>\n";
    kmlFile.close();
    
    cout << "\nRoute saved to route6.kml\n";
    
    return 0;
}
