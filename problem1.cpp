#include<bits/stdc++.h>
#include<cmath>
using namespace std;


double INF = 1e18;

struct Node
{
    double lon,lat;

    bool operator<(const Node& other) const {
        if(lon!=other.lon)return lon < other.lon;
        return lat<other.lat;
    }
};


map<Node,int>nodeToID;
vector<Node>idToNode;



vector<vector<pair<int, double>>> graph;


int getID(Node p){
    
    if(nodeToID.count(p)){
        return nodeToID[p];
    }

    int id = nodeToID.size();
    nodeToID[p] = id;
    idToNode.push_back(p);

    graph.resize(id+1);
    return id;
}






double haversine(Node p1, Node p2){

    const double R = 6371.0;
    const double PI = 3.14159265358979323846;


    double lat1InRad = ((p1.lat)*PI)/180.0;
    double lat2InRad = ((p2.lat)*PI)/180.0;

    double latDiff = p2.lat - p1.lat;  
    double lonDiff = p2.lon - p1.lon;
    
    double latDiffInRad = (latDiff*PI)/180.0;
    double lonDiffInRad = (lonDiff*PI)/180.0;


    double a = sin(latDiffInRad/2)*sin(latDiffInRad/2)+
               cos(lat1InRad)*cos(lat2InRad)* 
               sin(lonDiffInRad/2)*sin(lonDiffInRad/2);

    double c = 2*atan2(sqrt(a),sqrt(1-a));

    return R*c;

}




vector<int> dijkstra(int startNode, int endNode){

    int n = graph.size();

    
    
    vector<double>dist(n,INF);
    vector<int>parent(n,-1);
    dist[startNode]=0.0;



    //priority_queue<1,2,3>
    //1 = what the queue is storing (pair in this case),
    //    pairs are sorted by the first element
    //2 = container, using vector to hold the pairs in memory
    //3 = comperator, by default its max heap
    //    using greater<> reverses the behaviour



    priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> pq;

    
    pq.push({0.0, startNode});


    while(!pq.empty()){
        
        double d = pq.top().first;
        int id = pq.top().second;
        pq.pop();
        // Dijkstra guarantees:
        // When a node is popped from the min-heap,
        // its shortest distance is finalized.
        
        if(id==endNode){
            break;
        }


        for(auto edge : graph[id]){
        
            int neighborID = edge.first;
            double edgeToNeighbor = edge.second;
    

            if(dist[id] + edgeToNeighbor < dist[neighborID]){
                dist[neighborID]=dist[id]+edgeToNeighbor;
                parent[neighborID]=id;
                pq.push({dist[neighborID],neighborID});
            }
        
        
        
        }
    }


    vector<int>path;
    if(dist[endNode] == INF){
        return path;
    }

    for(int curr=endNode;curr!=-1;curr=parent[curr]){
        path.push_back(curr);
    }

    reverse(path.begin(),path.end());
    return path;
}





void load_roadmap(string filename){

    ifstream file(filename);
    if(!file.is_open()){
        cerr<<"could not open file lol "<<filename<<endl;
        exit(1);
    }

   
    string line;
    while(getline(file,line)){

        if(line.empty()){
            continue;
        }


        stringstream ss(line);
        string token;
        vector<string>tokens;



        while(getline(ss,token,',')){
            tokens.push_back(token);
        }
        if(tokens.size() < 5){
            continue;
        }



        vector<Node>roadPoints;
        for(int i=1;i<tokens.size()-2;i+=2){
            double lon = stod(tokens[i]);
            double lat = stod(tokens[i+1]);
            roadPoints.push_back({lon,lat});
        }

        if (roadPoints.empty()){
            continue;
        }


        for(int k=0; k<roadPoints.size()-1; k++){
            Node u = roadPoints[k];
            Node v = roadPoints[k+1];

            int uID = getID(u);
            int vID = getID(v);
            double w = haversine(u,v);

            graph[uID].push_back({vID,w});
            graph[vID].push_back({uID,w});

            
        }
    }

}






int getNearestNodeID(Node target){

    if(nodeToID.count(target)){
        return nodeToID[target];
    }

    int closetsID = -1;
    double minDist = INF;

    for(int i=0;i<idToNode.size();i++){
        double dist = haversine(target,idToNode[i]);
        if(dist<minDist){
            minDist=dist;
            closetsID=i;
        }
    }

    return closetsID;

}





int main(){

    load_roadmap("Roadmap-Dhaka.csv");

    // cout<<"Total distinct intersections: "<<idToNode.size()<<endl;




    Node source,destination;
    cin>>source.lon>>source.lat;
    cin>>destination.lon>>destination.lat;


    int startID = getNearestNodeID(source);
    int endID = getNearestNodeID(destination);

  
    double walkToStart = haversine(source,idToNode[startID]);
    double walkFromEnd = haversine(idToNode[endID],destination);


    vector<int> shortestPath;
    shortestPath = dijkstra(startID, endID);



    if(shortestPath.empty()){
        cout << "No path found between these two points." << endl;
    } 
    else{
        cout << "\nProblem No: 1\n";
        cout << "Source: (" << fixed << setprecision(6) << source.lon << ", " << source.lat << ")\n";
        cout << "Destination: (" << fixed << setprecision(6) << destination.lon << ", " << destination.lat << ")\n\n";

        if (walkToStart > 1e-6) {
            cout << "Cost: ৳0.00: Walk from Source (" << source.lon << ", " << source.lat 
                 << ") to (" << idToNode[startID].lon << ", " << idToNode[startID].lat << ").\n";
        }

        double totalDrivingDistance = 0.0;
        for (int i = 0; i < shortestPath.size() - 1; i++) {
            int u = shortestPath[i];
            int v = shortestPath[i+1];
            totalDrivingDistance += haversine(idToNode[u], idToNode[v]);
        }
        

        int firstCarNode = shortestPath.front();
        int lastCarNode = shortestPath.back();
        
        cout << "Shortest Distance : "<<fixed<<setprecision(6)<<totalDrivingDistance<< "km : Ride Car from (" 
             <<idToNode[firstCarNode].lon<< ", " <<idToNode[firstCarNode].lat << ") to (" 
             <<idToNode[lastCarNode].lon<< ", " <<idToNode[lastCarNode].lat << ").\n";

        if (walkFromEnd > 1e-6) {
            cout << "Cost: ৳0.00: Walk from (" << idToNode[endID].lon << ", " << idToNode[endID].lat 
                 << ") to Destination (" << destination.lon << ", " << destination.lat << ").\n";
        }

        //Print the KML Coordinates
        // cout << "\nCoordinates for KML:" << endl;
        // cout << fixed << setprecision(6) << source.lon << "," << source.lat << ",0" << endl;
        // for (int id : shortestPath) {
        //     Node p = idToNode[id];
        //     cout << fixed << setprecision(6) << p.lon << "," << p.lat << ",0" << endl;
        // }
        cout << fixed << setprecision(6) << destination.lon << "," << destination.lat << ",0" << endl;
    }

    return 0;
}