#include "PriorityQueue.h"

using namespace std;

void PriorityQueue::push(pair<SearchNode*, double> &p)
{
    //Push the item to Priority Queue
    data.emplace_back(p);
}

SearchNode* PriorityQueue::pop()
{
    //Returns and removes the least cost item.
    vector<double> costs;
    for (int i = 0; i < data.size(); i++)
    {//Copy the costs of node to cost container
        double c = -data[i].second;
        costs.emplace_back(c);
        //printf("costs: %f\n", c);
    }

    auto lessCostNode = max_element(costs.begin(), costs.end()); //Store the max element index number
    int index = distance(costs.begin(), lessCostNode);   //Store the least cost node index number
   
    SearchNode *poppedNode = data[index].first;   //Current least code node
    //printf("popped Node: ");
    //poppedNode->printInfo();
    data.erase(data.begin() + index);       //Remove the least cost node from data container

    return (poppedNode);
}

bool PriorityQueue::isEmpty()
{
    //Return true if data is empty
    return (data.size() == 0);
}

void PriorityQueue::clear()
{//Clear the vector and deallocate memory for SearchNode objects
    for (auto& pair:data)
    {
        delete pair.first; //Deallocate memory for SearchNode object
    }
    data.clear(); //Clear the vector
}