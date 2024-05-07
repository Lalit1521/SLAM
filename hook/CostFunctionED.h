#ifndef _COST_FUNCTION_ED_H_
#define _COST_FUNCTION_ED_H_

#include "CostFunction.h"

class CostFunctionED : public CostFunction
{
public:
    CostFunctionED() {}

    ~CostFunctionED() {}

    virtual double calValue(double tx, double ty, double th);
};

#endif