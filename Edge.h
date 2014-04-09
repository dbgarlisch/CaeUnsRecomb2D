/****************************************************************************
*
* CAEP Plugin example - PwCaeGridWrite implementation
*
* Proprietary software product of Pointwise, Inc.
* Copyright (c) 1995-2013 Pointwise, Inc.
* All rights reserved.
*
***************************************************************************/

#ifndef _EDGE_H_
#define _EDGE_H_

#include "CaeUnsGridModel.h"

#include <set>
#include <vector>

const PWP_FLOAT Boundary = 200.0;
const PWP_FLOAT Concave  = 300.0;

class Edge {
public:
    Edge(PWP_UINT32 i0, PWP_UINT32 i1, PWP_FLOAT quality = Boundary);

    ~Edge();

    bool operator<  (const Edge & rhs) const;
    bool operator== (const Edge & rhs) const;

    PWP_UINT32      i0_;        // vertex 0
    PWP_UINT32      i1_;        // vertex 1
    CaeUnsElement   el0_;       // tri 0
    CaeUnsElement   el1_;       // tri 1
    PWP_FLOAT       quality_;   // quality of recombined quad
};

typedef std::set<Edge>              Edges;
typedef Edges::iterator             EdgeIter;
typedef Edges::const_iterator       EdgeCIter;
typedef std::pair<EdgeIter, bool>   EdgePair;

#endif
