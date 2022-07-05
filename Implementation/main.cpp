#include <bits/stdc++.h>
#include "rapidxml.hpp"
#define INF DBL_MAX

using namespace std;
using namespace rapidxml;

typedef struct
{
    long long int id;
    double lat;
    double lon;
} NodeData1;

typedef struct
{
    string name;
    long long int id;
    double lat;
    double lon;
} NodeData;

typedef struct
{
    vector<long long int> waynodes;
} way;

struct hash_pair
{
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2> &p) const
    {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

xml_document<> doc;
xml_node<> *root_node = NULL;

bool comp(pair<long double, NodeData> n1, pair<long double, NodeData> n2)
{
    return (n1.first < n2.first);
}
bool comp1(pair<long double, NodeData1> n1, pair<long double, NodeData1> n2)
{
    return (n1.first < n2.first);
}
long double dist(NodeData n1, NodeData n2)
{
    double dLat = (n2.lat - n1.lat) * M_PI / 180.0;
    double dLon = (n2.lon - n1.lon) * M_PI / 180.0;
    n1.lat = (n1.lat) * M_PI / 180.0;
    n2.lat = (n2.lat) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(n1.lat) * cos(n2.lat);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}
long double dist1(NodeData1 n1, NodeData1 n2)
{
    double dLat = (n2.lat - n1.lat) * M_PI / 180.0;
    double dLon = (n2.lon - n1.lon) * M_PI / 180.0;
    n1.lat = (n1.lat) * M_PI / 180.0;
    n2.lat = (n2.lat) * M_PI / 180.0;
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(n1.lat) * cos(n2.lat);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}

void closestdistances(vector<pair<long double, NodeData>> &distance, int index, vector<NodeData> node_details)
{
    for (int i = 0; i < node_details.size(); i++)
    {
        if (i == index)
            continue;
        long double d;
        // d = sqrt((node_details[i].lat - node_details[index].lat) * (node_details[i].lat - node_details[index].lat) + (node_details[i].lon - node_details[index].lon) * (node_details[i].lon - node_details[index].lon));
        d = dist(node_details[i], node_details[index]);
        pair<long double, NodeData> new_pair;
        new_pair = make_pair(d, node_details[i]);
        distance.push_back(new_pair);
    }
}
void closestdistances1(vector<pair<long double, NodeData1>> &distance1, int index, vector<NodeData1> node_details)
{
    for (int i = 0; i < node_details.size(); i++)
    {
        if (i == index)
            continue;
        long double d;
        // d = sqrt((node_details[i].lat - node_details[index].lat) * (node_details[i].lat - node_details[index].lat) + (node_details[i].lon - node_details[index].lon) * (node_details[i].lon - node_details[index].lon));
        d = dist1(node_details[i], node_details[index]);
        pair<long double, NodeData1> new_pair;
        new_pair = make_pair(d, node_details[i]);
        distance1.push_back(new_pair);
    }
}

long long int mindistance(unordered_map<long long int, double> distFromid, unordered_map<long long int, bool> sptset)
{
    double min_dist = INT_MAX;
    long long int minid;
    for (auto it : distFromid)
    {
        if (sptset[it.first] == false && it.second <= min_dist)
        {
            min_dist = it.second;
            minid = it.first;
        }
    }
    return minid;
}
void printpath(long long int id_start, long long int id_end, unordered_map<long long int, double> distFromid, unordered_map<long long int, long long int> Parentsrc)
{
    if (Parentsrc[id_end] == -1)
    {
        cout << id_start;
        return;
    }
    else
    {
        printpath(id_start, Parentsrc[id_end], distFromid, Parentsrc);
        cout << " -> " << id_end;
    }
}
bool dijkstra_Priorityqueue(long long int id_start, long long int id_end, unordered_map<pair<long long int, long long int>, double, hash_pair> graph, unordered_map<long long int, vector<long long int>> Ways_from_node)
{
    string str1 = "node";
    string str2 = "way";
    unordered_map<long long int, double> distFromid;
    unordered_map<long long int, long long int> Parentsrc;
    unordered_map<long long int, bool> sptset;
    for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
    {
        if (Node->name() == str1)
        {
            if (stoll(Node->first_attribute("id")->value()) == id_start)
                distFromid[stoll(Node->first_attribute("id")->value())] = 0;
            else
            {
                distFromid[stoll(Node->first_attribute("id")->value())] = INF;
            }
        }
        sptset[stoll(Node->first_attribute("id")->value())] = false;
        Parentsrc[stoll(Node->first_attribute("id")->value())] = -1;
    }
    priority_queue<pair<double, long long int>, vector<pair<double, long long int>>, greater<pair<double, long long int>>> pq;
    pq.push(make_pair(0, id_start));
    while (!pq.empty())
    {
        pair<double, long long int> temppair;
        temppair = pq.top();
        pq.pop();
        if (sptset[temppair.second] == true)
            continue;
        sptset[temppair.second] = true;
        for (auto it : Ways_from_node[temppair.second])
        {
            long long int cnid = it;

            //
            double dist0 = graph[make_pair(temppair.second, it)];
            if (distFromid[it] > dist0 + distFromid[temppair.second])
            {
                distFromid[it] = dist0 + distFromid[temppair.second];
                Parentsrc[it] = temppair.second;
                pq.push(make_pair(distFromid[it], it));
            }
            if (cnid == id_end)
            {
                cout << "Distance of destination of id : " << id_end << " from the starting point of id : " << id_start << " is ->  " << distFromid[id_end] << " kms ." << endl;
                cout << "Path will be gives as : " << endl;
                printpath(id_start, id_end, distFromid, Parentsrc);
                return true;
            }
        }
    }
    // cout << distFromid[id_end];
    return false;
}


int main()
{

    ifstream theFile("../map.osm");
    vector<char> file((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
    file.push_back('\0');

    // Parse the file
    doc.parse<0>(&file[0]);

    // Find out the root node
    root_node = doc.first_node("osm");
    cout << "Press 1 for part 1 : " << endl;
    cout << "Press 2 for part 2 : " << endl;
    cout << "Press 3 for part 3 : " << endl;
    int a;
    cout << "Enter your choice : ";
    cin >> a;
    while (a != 0)
    {
        if (a == 1)
        {
            cout << "Counting the number of nodes and path in map.osm... " << endl;
            // Iterate over the nodes
            string str1 = "node";
            int count = 0;
            for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str1)
                    count++;
            }
            cout << " Number of nodes : " << count << endl;
            int count1 = 0;
            string str2 = "way";
            for (xml_node<> *Node = root_node->first_node("way"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str2)
                    count1++;
            }
            cout << " Number of ways : " << count1 << endl;
            string s1 = "name";
            vector<NodeData> node_details;
            for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str1)
                {
                    for (xml_node<> *tag_Node = Node->first_node("tag"); tag_Node; tag_Node = tag_Node->next_sibling())
                    {
                        if (tag_Node->first_attribute("k")->value() == s1)
                        {
                            NodeData new_node;
                            new_node.name = tag_Node->first_attribute("v")->value();
                            new_node.id = stoll(Node->first_attribute("id")->value());
                            new_node.lat = stod(Node->first_attribute("lat")->value());
                            new_node.lon = stod(Node->first_attribute("lon")->value());
                            node_details.push_back(new_node);
                        }
                    }
                }
            }
            string s;
            cout << "Enter the string : ";
            cin >> s;
            while (s.compare("exit") != 0)
            {
                long long int flag = 0;
                for (int i = 0; i < node_details.size(); i++)
                {
                    if (node_details[i].name.find(s) < node_details[i].name.size())
                    {
                        flag++;
                        if (flag == 1)
                            cout << "Here is the following suggestion for given substring :" << endl;
                        cout << "\tSuggestion : " << flag << endl;
                        cout << "\tName : " << node_details[i].name << " , ID : " << node_details[i].id << endl;
                        cout << "\tlatttuide : " << node_details[i].lat << " , longituide : " << node_details[i].lon << endl
                             << endl;
                    }
                }
                if (flag == 0)
                    cout << "\tThere is no suggestion for given substring , enter the correct string." << endl
                         << endl;
                else
                    cout << endl;
                cout << "Enter the string : ";
                cin >> s;
                // if( s == "\0")break ;
            }
        }
        else if (a == 2)
        {
            string str1 = "node";
            string s1 = "name";
            vector<NodeData> node_details;
            vector<NodeData1> node_details1;
            for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str1)
                {
                    for (xml_node<> *tag_Node = Node->first_node("tag"); tag_Node; tag_Node = tag_Node->next_sibling())
                    {
                        if (tag_Node->first_attribute("k")->value() == s1)
                        {
                            NodeData new_node;
                            new_node.name = tag_Node->first_attribute("v")->value();
                            new_node.id = stoll(Node->first_attribute("id")->value());
                            new_node.lat = stod(Node->first_attribute("lat")->value());
                            new_node.lon = stod(Node->first_attribute("lon")->value());
                            node_details.push_back(new_node);
                        }
                    }
                }
            }
            for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str1)
                {
                    NodeData1 new_node;
                    // new_node.name = tag_Node->first_attribute("v")->value();
                    new_node.id = stoll(Node->first_attribute("id")->value());
                    new_node.lat = stod(Node->first_attribute("lat")->value());
                    new_node.lon = stod(Node->first_attribute("lon")->value());
                    node_details1.push_back(new_node);
                }
            }
            int choice;
            cout << "Press 1 to find the closesets nodes for a perticular node of unique id ." << endl;
            cout << "Press 0 for escape. " << endl;
            cout << "Enter your choice : ";
            cin >> choice;
            while (choice != 0)
            {
                long long int id1;
                cout << "Enter the id of node : ";
                cin >> id1;
                int k;
                cout << "Enter the number of nodes : ";
                cin >> k;
                vector<pair<long double, NodeData>> distance;
                vector<pair<long double, NodeData1>> distance1;
                cout << "Press 1 for searching closest nodes form nodes which have name also ." << endl;
                cout << "Press 2 for searching closest nodes without names ." << endl;
                int ch;
                cout << "Enter your choice : ";
                cin >> ch;
                if (ch == 1)
                {
                    int index = 0;
                    for (int i = 0; i < node_details.size(); i++)
                    {
                        if (node_details[i].id == id1)
                            break;
                        index++;
                    }
                    if (index >= node_details.size())
                    {
                        cout << "There is no node present in data of id : " << id1 << endl;
                        continue;
                    }
                    closestdistances(distance, index, node_details);
                    sort(distance.begin(), distance.end(), comp);
                    cout << " Here is the " << k << " closest nodes from the node : " << node_details[index].name << "of id : " << node_details[index].id << endl;
                    int a = 0;
                    for (int i = 0; i < k; i++)
                    {
                        // if( a > 7)break ;
                        cout << " Node " << ++a << " :" << endl;
                        cout << "\tName : " << distance[i].second.name << " , id : " << distance[i].second.id << endl;
                        cout << "\tDistance : " << distance[i].first << endl
                             << endl;
                    }
                }
                else if (ch == 2)
                {
                    int index = 0;
                    for (int i = 0; i < node_details1.size(); i++)
                    {
                        if (node_details1[i].id == id1)
                            break;

                        index++;
                    }
                    if (index >= node_details1.size())
                    {
                        cout << "There is no node present in data of id : " << id1 << endl;
                        continue;
                    }
                    closestdistances1(distance1, index, node_details1);
                    sort(distance1.begin(), distance1.end(), comp1);
                    cout << " Here is the " << k << " closest nodes from the node of id : " << node_details1[index].id << endl;
                    int a = 0;
                    for (int i = 0; i < k; i++)
                    {
                        // if( a > 7)break ;
                        cout << " Node " << ++a << " :" << endl;
                        cout << "\tID : " << distance1[i].second.id << endl;
                        cout << "\tDistance : " << distance1[i].first << endl
                             << endl;
                    }
                }
                cout << "Press 1 to find the closeset nodes for a perticular node of unique id ." << endl;
                cout << "Press 0 for escape. " << endl;
                cout << "Enter your choice : ";
                cin >> choice;
            }
        }

        else if (a == 3)
        {
            string str1 = "node";
            string str2 = "way";
            unordered_map<long long int, NodeData1> node_id;
            for (xml_node<> *Node = root_node->first_node("node"); Node; Node = Node->next_sibling())
            {
                if (Node->name() == str1)
                {

                    NodeData1 new_node;
                    // new_node.name = tag_Node->first_attribute("v")->value();
                    new_node.id = stoll(Node->first_attribute("id")->value());
                    new_node.lat = stod(Node->first_attribute("lat")->value());
                    new_node.lon = stod(Node->first_attribute("lon")->value());
                    // node_id.insert(new_node.id , new_node);
                    node_id[new_node.id] = new_node;
                }
            }
            // cout<<"1";
            string str3 = "nd";
            unordered_map<long long int, way> Ways_details;
            for (xml_node<> *Node = root_node->first_node("way"); Node; Node = Node->next_sibling())
            {
                // int index = 0 ;
                if (Node->name() == str2)
                {
                    // cout << "1";
                    long long int wayid;
                    wayid = stoll(Node->first_attribute("id")->value());
                    // cout << "1!";
                    for (xml_node<> *nd_tag = Node->first_node("nd"); nd_tag; nd_tag = nd_tag->next_sibling())
                    {
                        if (nd_tag->name() == str3)
                        {
                            long long int tempid;
                            tempid = stoll(nd_tag->first_attribute("ref")->value());
                            Ways_details[wayid].waynodes.push_back(tempid);
                        }
                    }
                }
            }
            // cout << "2";
            unordered_map<long long int, vector<long long int>> Ways_from_node;
            unordered_map<pair<long long int, long long int>, double, hash_pair> graph;
            unordered_map<long long int, way>::iterator it;
            for (it = Ways_details.begin(); it != Ways_details.end(); it++)
            {
                for (int i = 0; i < it->second.waynodes.size() - 1; i++)
                {
                    Ways_from_node[it->second.waynodes[i]].push_back(it->second.waynodes[i + 1]);
                    // graph[it->second.waynodes[i]].
                    pair<long long int, long long int> new_pair;
                    pair<long long int, long long int> new_pair1;
                    pair<long long int, long long int> new_pair0;
                    double tempdist = dist1(node_id[it->second.waynodes[i]], node_id[it->second.waynodes[i + 1]]);
                    // new_pair = make
                    new_pair = make_pair(it->second.waynodes[i], it->second.waynodes[i + 1]);
                    new_pair1 = make_pair(it->second.waynodes[i + 1], it->second.waynodes[i]);
                    new_pair0 = make_pair(it->second.waynodes[i], it->second.waynodes[i]);
                    graph[new_pair] = tempdist;
                    graph[new_pair1] = tempdist;
                    // graph[it->second.waynodes[i + 1]].push_back(new_pair1) ;
                    graph[new_pair0] = 0;
                }
            }

            cout << "Press 1 to find the distance between two nodes : ." << endl;
            cout << "Press 0 for escape. " << endl;
            int ch1;
            cout << "Enter your choice : ";
            cin >> ch1;
            while (ch1 != 0)
            {
                cout << "Enter id of starting node : ";
                long long int id_start;
                cin >> id_start;
                cout << "Enter id of destination node : ";
                long long int id_end;
                cin >> id_end;

                // dijkstra(id_start, id_end, graph, Ways_from_node);
                bool functreturn = dijkstra_Priorityqueue(id_start, id_end, graph, Ways_from_node);
                if (functreturn == false)
                {
                    cout << "There is no path between them " << endl;
                }
                cout << endl
                     << endl;
                cout << "Press 1 to find the distance between two nodes : ." << endl;
                cout << "Press 0 for escape. " << endl;
                cout << "Enter your choice : ";
                cin >> ch1;
            }
        }
        cout << "Press 1 for part 1 : " << endl;
        cout << "Press 2 for part 2 : " << endl;
        cout << "Press 3 for part 3 : " << endl;
        cout << "Press 0 for exit : " << endl;
        cout << "Enter your choice : ";
        cin >> a;
    }
    return 0;
}