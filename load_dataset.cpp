#include<bits/stdc++.h>
#include<cmath>
using namespace std;


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