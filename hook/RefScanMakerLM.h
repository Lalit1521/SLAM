#ifndef _REF_SCAN_MAKER_LM_H_
#define _REF_SCAN_MAKER_LM_H_

#include "RefScanMaker.h"

class RefScanMakerLM : public RefScanMaker
{
    public:
        RefScanMakerLM() {}

        ~RefScanMakerLM() {}

        virtual const Scan2D *makeRefScan();
};

#endif