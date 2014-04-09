/****************************************************************************
 *
 * CAEP Plugin example - PwCaeGridWrite implementation
 *
 * Proprietary software product of Pointwise, Inc.
 * Copyright (c) 1995-2013 Pointwise, Inc.
 * All rights reserved.
 *
 ***************************************************************************/

#include "Edge.h"
#include "apiGridModel.h"


Edge::Edge(PWP_UINT32 i0, PWP_UINT32 i1, PWP_FLOAT quality) :
    i0_(i0 < i1 ? i0 : i1),
    i1_(i0 < i1 ? i1 : i0),
    quality_(quality)
{
}


Edge::~Edge()
{
}


bool
Edge::operator< (const Edge & rhs) const
{
    if (i0_ < rhs.i0_) {
        return true;
    }
    else if (i0_ > rhs.i0_) {
        return false;
    }
    else if (i1_ < rhs.i1_) {
        return true;
    }
    return false;
}


bool
Edge::operator== (const Edge & rhs) const
{
    return (i0_ == rhs.i0_) && (i1_ == rhs.i1_);
}
