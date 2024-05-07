#include "RefScanMakerLM.h"

using namespace std;

const Scan2D *RefScanMakerLM::makeRefScan()
{
    vector<LPoint2D> &refLps = refScan.lps;
    refLps.clear();

    const vector<LPoint2D> &localMap = pcmap->localMap;
    for (size_t i = 0; i< localMap.size(); i++)
    {
        const LPoint2D &rp = localMap[i];
        refLps.emplace_back(rp);
    }

    return (&refScan);
}