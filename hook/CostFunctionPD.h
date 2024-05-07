#ifndef _COST_FUNCTION_PD_H_
#define _COST_FUNCTION_PD_H_

#include "CostFunction.h"

class CostFunctionPD : public CostFunction
{
public:
    CostFunctionPD() {}

    ~CostFunctionPD() {}

    virtual double calValue(double tx, double ty, double th);
};

#endif