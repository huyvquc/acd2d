//------------------------------------------------------------------------------
// Vertex Clique Cover (VCC) Convex Decomposition Wrapper for acd2d
//------------------------------------------------------------------------------

#ifndef _ACD2D_VCC_H_
#define _ACD2D_VCC_H_

#include <vector>
#include <list>
#include "acd2d_data.h"

namespace acd2d {

struct VccDecompositionResult {
    std::vector<std::vector<Point2d>> regions;
    double computation_time_sec;
    bool is_extension;
};

class VccWrapper {
public:
    static VccDecompositionResult ComputeVccDecomposition(const cd_polygon& poly, bool use_extension = false);
};

} // namespace acd2d

#endif // _ACD2D_VCC_H_
