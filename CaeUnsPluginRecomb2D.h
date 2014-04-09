/****************************************************************************
 *
 * class CaeUnsPluginRecomb2D
 *
 * Proprietary software product of Pointwise, Inc.
 * Copyright (c) 1995-2013 Pointwise, Inc.
 * All rights reserved.
 *
 ***************************************************************************/

#ifndef _CAEUNSPLUGINRECOMB2D_H_
#define _CAEUNSPLUGINRECOMB2D_H_

#include <vector>

#include "CaePlugin.h"
#include "CaeUnsGridModel.h"
#include "Edge.h"

#include "cml.h"


//***************************************************************************
//***************************************************************************
//***************************************************************************

class CaeUnsPluginRecomb2D : public CaeUnsPlugin, public CaeFaceStreamHandler {
public:
    CaeUnsPluginRecomb2D(CAEP_RTITEM *pRti, PWGM_HGRIDMODEL model,
        const CAEP_WRITEINFO *pWriteInfo);

    ~CaeUnsPluginRecomb2D();

    static bool create(CAEP_RTITEM &rti);
    static void destroy(CAEP_RTITEM &rti);

private:

    bool        mergeElements(const CaeUnsElement &elem0, const CaeUnsElement &elem1,
                    PWP_UINT32 i0, PWP_UINT32 i1, PWGM_ELEMDATA &quad);
    bool        vertToVec3(const CaeUnsVertex &vertex, vector3 &v3);
    PWP_FLOAT   maxIncludedAngle(const PWGM_ELEMDATA &quad);

    bool        writeVertices();
    bool        writeElements();
    bool        postInfo();
    void        loadEdge(CaeUnsElement &elem, const CaeUnsVertex &v0,
                    const CaeUnsVertex &v1);
    vector3&    midPoint(PWP_UINT32 i0, PWP_UINT32 i1, vector3 &midPt);
    bool        isNewEdge(const Edge &e, EdgeIter &it);
    bool        recombine(const CaeUnsBlock &block);

    static bool qualitySort(EdgeIter it1, EdgeIter it2);

    static bool mergeElements(const PWGM_ELEMDATA &e0, const PWGM_ELEMDATA &e1,
                    PWP_UINT32 i0, PWP_UINT32 i1, PWGM_ELEMDATA &eMerged);

    static bool isConvex(const vector3 &e0, const vector3 &e1,
                    const vector3 &e2, const vector3 &e3);

    void        debugLogMaxInclAngle(PWP_FLOAT angle);
    void        debugDump();

private: // base class virtual methods

    virtual bool        beginExport();
    virtual PWP_BOOL    write();

    // face streaming handlers
    virtual PWP_UINT32 streamBegin(const PWGM_BEGINSTREAM_DATA &data);
    virtual PWP_UINT32 streamFace(const PWGM_FACESTREAM_DATA &data);
    virtual PWP_UINT32 streamEnd(const PWGM_ENDSTREAM_DATA &data);

private:
    typedef std::vector<PWGM_ELEMDATA>  RecombinedElems;
    typedef std::vector<EdgeIter>       RecomEdges;
    typedef std::set<CaeUnsElement>     UnsElementSet;

    //! The debug dump file
    PwpFile         dumpFile_;

    //! Max included angle histogram
    PWP_UINT32      maxAngleLog_[10];

    //! The max included angle allowed for combined quads.
    PWP_FLOAT       maxInclAngle_;

    //! Number of elements in source grid.
    PWP_UINT32      origElemCnt_;

    //! Number of recombined quads.
    PWP_UINT32      recombQuadCnt_;

    //! All unique tri edges
    Edges           edges_;

    //! Collection of iters into edges_. Quads formed across these edges are of
    //! sufficient quality.
    RecomEdges      recombEdges_;

    //! Collection of recombined elements.
    RecombinedElems recombElems_;
};

#endif // _CAEUNSPLUGINRECOMB2D_H_
