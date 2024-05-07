#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H_

#include "SearchNode.h"
#include <vector>
#include <utility> // for std::pair
#include <algorithm> // for std::max_element

class PriorityQueue
{
    private:
        std::vector<std::pair<SearchNode*, double>> data;

    public:
        PriorityQueue() {};

        ~PriorityQueue() {};

        void push(std::pair<SearchNode*, double> &p);

        SearchNode* pop();

        bool isEmpty();

        void clear();

};

#endif