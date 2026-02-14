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




struct Edge{
    int to;
    double dist;
    int mode;
};







map<Node,int>nodeToID;
vector<Node>idToNode;
map<int,string>idToStationName;



vector<vector<Edge>> graph;


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



double getCost(double distance,int mode){

    if(mode==1){
        return distance*20.0;
    }

    if(mode==2){
        return distance*5.0;
    }

    return 0.0;

}






pair<vector<int>,double> dijkstra(int startNode, int endNode){

    int n = graph.size();
    vector<double>minCost(n,INF);
    minCost[startNode]=0.0;
    vector<int>parent(n,-1);





    priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> pq;
    pq.push({0.0, startNode});

    while(!pq.empty()){
        
        double currentCost = pq.top().first;
        int id = pq.top().second;
        pq.pop();



        if(currentCost >minCost[id]){
            continue;
        }
        if(id==endNode){
            break;
        }




        for(auto edge : graph[id]){
        
            int neighborID = edge.to;
            double edgeToNeighbor = edge.dist;
            int mode = edge.mode;

         
            double edgeCost = getCost(edgeToNeighbor,mode);
    

            if(minCost[id] + edgeCost < minCost[neighborID]){
                minCost[neighborID]=minCost[id]+edgeCost;
                parent[neighborID]=id;
                pq.push({minCost[neighborID],neighborID});
            }
        
        
        
        }
    }


    vector<int>path;
    if(minCost[endNode] == INF){
        return {path,INF};
    }

    for(int curr=endNode;curr!=-1;curr=parent[curr]){
        path.push_back(curr);
    }

    reverse(path.begin(),path.end());
    return {path,minCost[endNode]};
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

            graph[uID].push_back({vID,w,1});
            graph[vID].push_back({uID,w,1});

            
        }
    }

}





void loadTransit(string filename, int modeID){


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
        string startStationName = tokens[tokens.size() - 2];
        string endStationName = tokens[tokens.size() - 1];
        vector<Node>roadPoints;
        for(int i=1;i<tokens.size()-2;i+=2){
            double lon = stod(tokens[i]);
            double lat = stod(tokens[i+1]);
            roadPoints.push_back({lon,lat});
        }

        if (roadPoints.empty()){
            continue;
        }
        //the first co-ordinate of each line is the
            //location of the source metro
        //the last co-ordinate of the roadPoints vector is the
            //location of the destination metro
        int firstNodeID = getID(roadPoints.front());
        int lastNodeID = getID(roadPoints.back());
        idToStationName[firstNodeID]=startStationName;
        idToStationName[lastNodeID]=endStationName;
      
      








        for(int k=0; k<roadPoints.size()-1; k++){
            Node u = roadPoints[k];
            Node v = roadPoints[k+1];

            int uID = getID(u);
            int vID = getID(v);
            double w = haversine(u,v);

            graph[uID].push_back({vID,w,modeID});
            graph[vID].push_back({uID,w,modeID});
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
    loadTransit("Routemap-DhakaMetroRail.csv", 2); 
    // loadTransit("Routemap-BikolpoBus.csv", 3);     
    // loadTransit("Routemap-UttaraBus.csv", 4);       
   
   
    
    cout<<"Total distinct intersections: "<<idToNode.size()<<endl;




    Node source,destination;
    cin>>source.lon>>source.lat;
    cin>>destination.lon>>destination.lat;


    int startID = getNearestNodeID(source);
    int endID = getNearestNodeID(destination);

  
    double walkToStart = haversine(source,idToNode[startID]);
    double walkFromEnd = haversine(idToNode[endID],destination);


    vector<int> shortestPath;
    pair<vector<int>,double> result = dijkstra(startID, endID);
    shortestPath = result.first;
    double totalCost = result.second;



    if(shortestPath.empty()){
        cout << "No path found between these two points " << endl;
    } 
    else{


        cout << "Cheaptes path found with " << shortestPath.size() << " steps." << endl;
        cout<<"Total Cost : "<<endl<<fixed<<setprecision(2)<<totalCost<<endl;
        cout << "Coordinates for KML:" << endl;
        cout << fixed << setprecision(6) << source.lon << "," << source.lat << ",0" << endl;
        




        //car route
        for (int id : shortestPath) {
            Node p = idToNode[id];
            cout<<fixed<<setprecision(6)<<p.lon<<","<<p.lat <<",0"<<endl;
        }

        //exact end point (Walking)
        cout<<fixed<<setprecision(6)<<destination.lon<<","<< destination.lat << ",0" << endl;
        cout<<"Walking to destination: " <<walkFromEnd<<" km"<<endl;
    }

    return 0;
}