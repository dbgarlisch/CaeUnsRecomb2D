/****************************************************************************
 *
 * class CaeUnsPluginRecomb2D
 *
 * Proprietary software product of Pointwise, Inc.
 * Copyright (c) 1995-2013 Pointwise, Inc.
 * All rights reserved.
 *
 ***************************************************************************/

#include <cstring>
#include <set>

#include "CaeUnsGridModel.h"
#include "CaeUnsPluginRecomb2D.h"
#include "condtypes.h"
#include "Edge.h"

const char *attrMaxIncludedAngle    = "MaxIncludedAngle";
const char *attrDebugDump           = "DebugDump";


struct hElemLess {
    bool operator()(PWGM_HELEMENT lhs, PWGM_HELEMENT rhs) const
    {
        if (PWGM_HELEMENT_PID(lhs) == PWGM_HELEMENT_PID(rhs)) {
            if (PWGM_HELEMENT_ID(lhs) < PWGM_HELEMENT_ID(rhs)) {
                return true;
            }
        }
        else if (PWGM_HELEMENT_PID(lhs) < PWGM_HELEMENT_PID(rhs)) {
            return true;
        }
        return false;
    }
};


//***************************************************************************
//***************************************************************************
//***************************************************************************

CaeUnsPluginRecomb2D::CaeUnsPluginRecomb2D(CAEP_RTITEM *pRti, PWGM_HGRIDMODEL
        model, const CAEP_WRITEINFO *pWriteInfo) :
    CaeUnsPlugin(pRti, model, pWriteInfo),
    dumpFile_(),
    maxInclAngle_(100.0),
    origElemCnt_(0),
    recombQuadCnt_(0),
    edges_(),
    recombEdges_(),
    recombElems_()
{
    PWP_BOOL doDump;
    model_.getAttribute(attrDebugDump, doDump);
    model_.getAttribute(attrMaxIncludedAngle, maxInclAngle_);

    char filename[512];
    strcpy(filename, writeInfo_.fileDest);
    strcat(filename, ".dump");
    if (!doDump) {
        // remove dump file if it exists from previous run
        pwpFileDelete(filename);
    }
    else {
        dumpFile_.open(filename, pwpWrite | pwpAscii);
        sendInfoMsg("debug dump file:", 0);
        sendInfoMsg(filename, 0);
        if (!dumpFile_.isOpen()) {
            sendErrorMsg("debug dump file open failed!", 0);
        }
        else {
            fprintf(dumpFile_, "maxInclAngle %g\n", maxInclAngle_);
        }
    }

    char msg[512];
    sprintf(msg, "Recombination maxInclAngle = %g", maxInclAngle_);
    sendInfoMsg(msg, 0);

    setProgressMajorSteps(2);
}


CaeUnsPluginRecomb2D::~CaeUnsPluginRecomb2D()
{
}


static PWGM_ELEMCOUNTS&
operator+=(PWGM_ELEMCOUNTS &lhs, const PWGM_ELEMCOUNTS &rhs)
{
    for (PWP_INT ii=0; ii < PWGM_ELEMTYPE_SIZE; ++ii) {
        lhs.count[ii] += rhs.count[ii];
    }
    return lhs;
}


bool
CaeUnsPluginRecomb2D::beginExport()
{
    memset(maxAngleLog_, 0, sizeof(maxAngleLog_));

    CaeUnsBlock block(model_);
    PWGM_ELEMCOUNTS totalCounts = {{0}};
    PWGM_ELEMCOUNTS counts;
    while (block.isValid()) {
        origElemCnt_ += block.elementCount(&counts);
        totalCounts += counts;
        ++block;
    }

    // Approximate number of steps:
    //   Each block elem is touched twice; Once here and once in recombine().
    //   Plus guess for total num edges that will end up in recombEdges_ for
    //   all blocks.
    PWP_UINT32 steps = origElemCnt_ * 2;
    steps += (PWGM_ECNT_Tri(totalCounts) * 3) / 2; // assume all
    bool ret = progressBeginStep(steps);
    PWGM_ELEMDATA eData;
    block.moveFirst(model_);
    while (ret && block.isValid()) {
        block.elementCount(&counts);
        // Get approx number of unique tri edges. This ignores boundary
        // edges and assumes all edges are used twice.
        PWP_UINT32 numTriEdges = (PWGM_ECNT_Tri(counts) * 3) / 2;
        if (0 != numTriEdges) {
            // Guesstimate the fraction of numTriEdges that will be
            // combined. Farfield, delauney tris will have max included
            // angle around 120. Give maxInclAngle_ values above 120 little
            // weight since the number of quads combined above 120 will be
            // relatively small.
            PWP_FLOAT div = 120.0 - 90.0 + 3.0;
            // Given,
            //      90 <= maxInclAngle_ <= 180
            // We get,
            //      0.0 <= frac <= 2.72
            // Any maxInclAngle_ above (div + 90) results in a frac > 1.
            PWP_FLOAT frac = PWP_FLOAT((maxInclAngle_ - 90.0) / div);
            // rescale frac to range 0.05 ... 1.0
            frac = PWP_FLOAT(0.05 + 0.95 * (frac > 1.0 ? 1.0 : frac));
            // Try to reserve a reasonable size for recombEdges_ to prevent
            // a realloc. Only tris are combined. We ignore quads.
            recombEdges_.reserve(PWP_UINT32(numTriEdges * frac));
            CaeUnsElement blkElem(block);
            while (blkElem.data(eData)) {
                if (!progressIncrement()) {
                    ret = false;
                    break;
                }
                if (PWGM_ELEMTYPE_TRI == eData.type) {
                    CaeUnsVertex v0(eData.vert[0]);
                    CaeUnsVertex v1(eData.vert[1]);
                    CaeUnsVertex v2(eData.vert[2]);
                    loadEdge(blkElem, v0, v1);
                    loadEdge(blkElem, v1, v2);
                    loadEdge(blkElem, v2, v0);
                }
                ++blkElem;
            }
        }
        if (ret) {
            recombine(block);
            debugDump();
        }
        ++block;
        // all done with these - release memory
        edges_.clear();
        recombEdges_.clear();
    }
    return progressEndStep() && ret;
}


bool
CaeUnsPluginRecomb2D::mergeElements(const CaeUnsElement &elem0, const CaeUnsElement &elem1,
    PWP_UINT32 i0, PWP_UINT32 i1, PWGM_ELEMDATA &quad)
{
    PWGM_ELEMDATA eData0;
    PWGM_ELEMDATA eData1;
    return elem0.data(eData0) && elem1.data(eData1) && mergeElements(eData0,
                eData1, i0, i1, quad);
}


bool
CaeUnsPluginRecomb2D::vertToVec3(const CaeUnsVertex &vertex, vector3 &v3)
{
    PWGM_VERTDATA vd;
    bool ret = vertex.dataMod(vd);
    if (ret) {
        v3.set(vd.x, vd.y, vd.z);
    }
    return ret;
}


PWP_FLOAT
CaeUnsPluginRecomb2D::maxIncludedAngle(const PWGM_ELEMDATA &quad)
{
    PWP_FLOAT ret = 180.0;
    vector3 p0;
    vector3 p1;
    vector3 p2;
    vector3 p3;
    if (vertToVec3(quad.vert[0], p0) && vertToVec3(quad.vert[1], p1) &&
            vertToVec3(quad.vert[2], p2) && vertToVec3(quad.vert[3], p3)) {
        // right handed, unit edge vectors
        vector3 v01 = (p1 - p0).normalize();
        vector3 v12 = (p2 - p1).normalize();
        vector3 v23 = (p3 - p2).normalize();
        vector3 v30 = (p0 - p3).normalize();
        if (!isConvex(v01, v12, v23, v30)) {
            ret = Concave;
        }
        else {
            // Given vectors A and B (of length a and b);
            //
            //      A dot B = a * b * cos(angle)
            //
            // If A and B are unit vectors,
            //
            //      A dot B = 1 * 1 * cos(angle) = cos(angle)

            // Compute dot product of unit vecs to determine included angle.
            // Only works if shape is convex. hence the check above.
            vector3 v03 = (p3 - p0).normalize();
            PWP_FLOAT q = PWP_FLOAT(cml::acos_safe(cml::dot(v01, v03)));
            ret = q;

            vector3 v10 = (p0 - p1).normalize();
            q = PWP_FLOAT(cml::acos_safe(cml::dot(v12, v10)));
            if (ret < q) {
                ret = q;
            }

            vector3 v21 = (p1 - p2).normalize();
            q = PWP_FLOAT(cml::acos_safe(cml::dot(v23, v21)));
            if (ret < q) {
                ret = q;
            }

            vector3 v32 = (p2 - p3).normalize();
            q = PWP_FLOAT(cml::acos_safe(cml::dot(v30, v32)));
            if (ret < q) {
                ret = q;
            }
            ret = cml::deg(ret); // radians to degrees
        }
    }
    return ret;
}


bool
CaeUnsPluginRecomb2D::mergeElements(const PWGM_ELEMDATA &e0, const
    PWGM_ELEMDATA &e1, PWP_UINT32 i0, PWP_UINT32 i1, PWGM_ELEMDATA &eMerged)
{
    eMerged.vertCnt = 0;
    eMerged.type = PWGM_ELEMTYPE_SIZE;
    PWP_INT32 n;
    PWP_INT32 n0;
    PWP_INT32 n1;
    PWP_INT32 n2;
    for (n = 0; n < 3; ++n) {
        if (e0.index[n] == i0 || e0.index[n] == i1) {
            n0 = (n + 1) % 3;
            if (e0.index[n0] == i0 || e0.index[n0] == i1) {
                n1 = (n0 + 1) % 3;
                n2 = (n0 + 2) % 3;
                eMerged.index[0] = e0.index[n0];
                eMerged.vert[0] = e0.vert[n0];
                eMerged.index[1] = e0.index[n1];
                eMerged.vert[1] = e0.vert[n1];
                eMerged.index[2] = e0.index[n2];
                eMerged.vert[2] = e0.vert[n2];
                break;
            }
        }
    }
    for (n = 0; n < 3; ++n) {
        if (e1.index[n] == i0 || e1.index[n] == i1) {
            n0 = (n + 1) % 3;
            if (e1.index[n0] == i0 || e1.index[n0] == i1) {
                n1 = (n0 + 1) % 3;
                eMerged.index[3] = e1.index[n1];
                eMerged.vert[3] = e1.vert[n1];
                eMerged.vertCnt = 4;
                eMerged.type = PWGM_ELEMTYPE_QUAD;
                break;
            }
        }
    }
    return 4 == eMerged.vertCnt;
}


bool
CaeUnsPluginRecomb2D::isConvex(const vector3 &e0, const vector3 &e1,
    const vector3 &e2, const vector3 &e3)
{
    // compute normal vec for each edge pair.
    vector3 n01 = cml::cross(e0, e1);
    vector3 n12 = cml::cross(e1, e2);
    vector3 n23 = cml::cross(e2, e3);
    vector3 n30 = cml::cross(e3, e0);
    // if dot product between two normals is < 0, the normals are opposing.
    // This will only happen if quad is concave.
    return (cml::dot(n01, n12) >= 0) && (cml::dot(n01, n23) >= 0) &&
        (cml::dot(n01, n30) >= 0);
}


PWP_BOOL
CaeUnsPluginRecomb2D::write()
{
    // http://geuz.org/gmsh/doc/texinfo/gmsh.html#MSH-ASCII-file-format
    // $MeshFormat
    // version-number file-type data-size
    // $EndMeshFormat
    // $Nodes
    // number-of-nodes
    // node-number x-coord y-coord z-coord
    // ...
    // $EndNodes
    // $Elements
    // number-of-elements
    // elm-number elm-type number-of-tags < tag > ... node-number-list
    // ...
    // $EndElements
    //
    // where,
    //    version-number
    //      is a real number equal to 2.2
    //    file-type
    //      is an integer equal to 0 in the ASCII file format.
    //    data-size
    //      is an integer equal to the size of the floating point numbers
    //      used in the file (currently only data-size = sizeof(double) is
    //      supported).
    //    number-of-nodes
    //      is the number of nodes in the mesh.
    //    node-number
    //      is the number (index) of the n-th node in the mesh; node-number
    //      must be a postive (non-zero) integer. The node-numbers do not
    //      necessarily have to form a dense nor an ordered sequence.
    //    x-coord y-coord z-coord
    //      are the floating point values giving the X, Y and Z coordinates
    //      of the n-th node.
    //    number-of-elements
    //      is the number of elements in the mesh.
    //    elm-number
    //      is the number (index) of the n-th element in the mesh;
    //      elm-number must be a postive (non-zero) integer. The
    //      elm-numbers do not necessarily have to form a dense nor an
    //      ordered sequence.
    //    elm-type
    //      defines the geometrical type of the n-th element:
    //          1 = 2-node line.
    //          2 = 3-node triangle.
    //          3 = 4-node quadrangle.

    fputs("$MeshFormat\n", fp());
    fputs("2.2 0 8\n", fp());
    fputs("$EndMeshFormat\n", fp());
    return writeVertices() && writeElements() && postInfo();
}


bool
CaeUnsPluginRecomb2D::writeVertices()
{
    // $Nodes
    // number-of-nodes
    // node-number x-coord y-coord z-coord
    // ...
    // $EndNodes
    PWP_UINT32 vCnt = model_.vertexCount();
    if (progressBeginStep(vCnt)) {
        fputs("$Nodes\n", fp());
        fprintf(fp(), "%lu\n", (unsigned long)vCnt);
        PWGM_VERTDATA v;
        CaeUnsVertex vertex(model_);
        while (vertex.dataMod(v)) {
            fprintf(fp(), "%lu %+.12e %+.12e %+.12e\n",
                (unsigned long)vertex.index() + 1, v.x, v.y, v.z);
            ++vertex;
        }
        fputs("$EndNodes\n", fp());
    }
    return progressEndStep();
}


bool
CaeUnsPluginRecomb2D::writeElements()
{
    // $Elements
    // number-of-elements
    // elm-number elm-type number-of-tags < tag > ... node-number-list
    // ...
    // $EndElements
    //
    // where,
    //    elm-type
    //      defines the geometrical type of the n-th element:
    //          1 = 2-node line.
    //          2 = 3-node triangle.
    //          3 = 4-node quadrangle.
    fputs("$Elements\n", fp());
    fprintf(fp(), "%lu\n", (unsigned long)recombElems_.size());
    unsigned long ndx = 1;
    RecombinedElems::const_iterator it = recombElems_.begin();
    for (; it != recombElems_.end(); ++it, ++ndx) {
        if (4 == it->vertCnt) {
            fprintf(fp(), "%lu 3 0 %lu %lu %lu %lu\n", ndx,
                (unsigned long)it->index[0] + 1,
                (unsigned long)it->index[1] + 1,
                (unsigned long)it->index[2] + 1,
                (unsigned long)it->index[3] + 1);
        }
        else if (3 == it->vertCnt) {
            fprintf(fp(), "%lu 2 0 %lu %lu %lu\n", ndx,
                (unsigned long)it->index[0] + 1,
                (unsigned long)it->index[1] + 1,
                (unsigned long)it->index[2] + 1);
        }
        else {
            unsigned long gmshElemType = 0; // bogus
            fprintf(fp(), "%lu %lu 0", ndx, gmshElemType);
            for (PWP_UINT32 ii = 0; ii < it->vertCnt; ++ii) {
                fprintf(fp(), " %lu", (unsigned long)it->index[ii] + 1);
            }
            fputs("\n", fp());
        }
    }
    fputs("$EndElements\n", fp());
    return true;
}


bool
CaeUnsPluginRecomb2D::postInfo()
{
    char msg[512];
    PWP_UINT32 triCnt = (PWP_UINT32)recombElems_.size() - recombQuadCnt_;
    sprintf(msg, "%lu elements recombined into %lu elements "
                 "(%lu tris, %lu quads)", (unsigned long)origElemCnt_,
        (unsigned long)(triCnt + recombQuadCnt_), (unsigned long)triCnt,
        (unsigned long)recombQuadCnt_);
    sendInfoMsg(msg, 0);

    sendInfoMsg("Max Included Angle Distribution:", 0);
    const PWP_UINT32 numBins = sizeof(maxAngleLog_) / sizeof(maxAngleLog_[0]) - 1;
    const double binSize = (90.0 / numBins);
    double minAngle = 90.0;
    for (PWP_UINT32 ii = 0; ii < numBins; ++ii) {
        sprintf(msg, "%7.1f &lt;= %7ld &lt; %7.1f", minAngle, (unsigned long)maxAngleLog_[ii],
            minAngle + binSize);
        sendInfoMsg(msg, 0);
        minAngle = minAngle + binSize;
    }
    sprintf(msg, "Concave : %ld", (unsigned long)maxAngleLog_[numBins]);
    sendInfoMsg(msg, 0);

    return true;
}


void
CaeUnsPluginRecomb2D::loadEdge(CaeUnsElement &element, const CaeUnsVertex &v0,
    const CaeUnsVertex &v1)
{
    PWGM_VERTDATA vd0;
    PWGM_VERTDATA vd1 = { 0.0 }; // init to silence bogus winOS warning
    if (v0.dataMod(vd0) && v1.dataMod(vd1) && (vd0.i != vd1.i)) {
        Edge e(vd0.i, vd1.i);
        e.el0_ = element;
        EdgeIter it;
        if (!isNewEdge(e, it)) {
            Edge &e = *((Edge*)&(*it));
            e.el1_ = element;
            PWGM_ELEMDATA quad;
            if (mergeElements(it->el0_, it->el1_, it->i0_, it->i1_, quad)) {
                PWP_FLOAT quadMaxInclAngle = maxIncludedAngle(quad);
                debugLogMaxInclAngle(quadMaxInclAngle);
                if (quadMaxInclAngle <= maxInclAngle_) {
                    // quality = deviation from the ideal of 90 degrees
                    e.quality_ = PWP_FLOAT(quadMaxInclAngle - 90.0);
                    recombEdges_.push_back(it);
                }
            }
        }
    }
}


vector3&
CaeUnsPluginRecomb2D::midPoint(PWP_UINT32 i0, PWP_UINT32 i1, vector3 &midPt)
{
    vector3 p0;
    vector3 p1;
    midPt.set(0, 0, 0);
    if (vertToVec3(CaeUnsVertex(model_, i0), p0) &&
            vertToVec3(CaeUnsVertex(model_, i1), p1)) {
        midPt = (p0 + p1) / 2.0;
    }
    return midPt;
}


bool
CaeUnsPluginRecomb2D::isNewEdge(const Edge &e, EdgeIter &it)
{
    EdgePair ret = edges_.insert(e);
    it = ret.first;
    return ret.second;
}


bool
CaeUnsPluginRecomb2D::qualitySort(EdgeIter it1, EdgeIter it2)
{
    return it1->quality_ < it2->quality_;
}


bool
CaeUnsPluginRecomb2D::recombine(const CaeUnsBlock &block)
{
    bool ret = block.isValid();
    if (ret) {
        PWP_UINT32 blkOrigElemCnt = block.elementCount();
        PWP_UINT32 startEdgesSize = (PWP_UINT32)recombEdges_.size();
        PWP_UINT32 startSize = (PWP_UINT32)recombElems_.size();
        // Expand recombElems_ to make sure it has enough capacity.
        RecombinedElems::size_type resSize = startSize;
        // Adjust size assuming 100% of edges are combined
        resSize += startEdgesSize;
        // Add in the number of original elements that will not be combined.
        resSize += (blkOrigElemCnt - startEdgesSize);
        recombElems_.reserve(resSize);
        // Tracks which tris have been combined into quads
        std::set<PWGM_HELEMENT, hElemLess> lockedTris;
        // sort recombEdges_ by edge quality. Edges with highest quality are
        // combined first.
        std::sort(recombEdges_.begin(), recombEdges_.end(), qualitySort);
        PWGM_ELEMDATA quad;
        RecomEdges::iterator it = recombEdges_.begin();
        for (; it != recombEdges_.end(); ++it) {
            if (!progressIncrement()) {
                ret = false;
                break;
            }
            Edge &e = *((Edge *)&(*(*it)));
            if (lockedTris.end() != lockedTris.find(e.el0_)) {
                continue; // el0_ already recombined
            }
            if (lockedTris.end() != lockedTris.find(e.el1_)) {
                continue; // el1_ already recombined
            }
            if (!mergeElements(e.el0_, e.el1_, e.i0_, e.i1_, quad)) {
                continue; // bad!
            }
            // mark tris as recombined
            lockedTris.insert(e.el0_);
            lockedTris.insert(e.el1_);
            recombElems_.push_back(quad);
        }

        if (ret) {
            PWP_UINT32 blkQuadElemCnt = (PWP_UINT32)recombElems_.size() -
                startSize;
            recombQuadCnt_ += blkQuadElemCnt;
            PWP_UINT32 blkNewElemCnt = blkOrigElemCnt - blkQuadElemCnt;
            PWGM_ELEMDATA eData;
            CaeUnsElement element(block);
            while (element.isValid()) {
                // only push elements that are NOT found in lockedTris
                if ((lockedTris.find(element) == lockedTris.end()) &&
                        element.data(eData)) {
                    recombElems_.push_back(eData);
                }
                ++element;
                if (!progressIncrement()) {
                    ret = false;
                    break;
                }
            }
            ret = ret && (blkNewElemCnt == (recombElems_.size() - startSize));
        }
    }
    return ret;
}


void
CaeUnsPluginRecomb2D::debugLogMaxInclAngle(PWP_FLOAT angle)
{
    // last bin is for Concave and other bad quads
    const PWP_UINT32 numBins = sizeof(maxAngleLog_) / sizeof(maxAngleLog_[0]) - 1;
    // convert angle to index
    PWP_UINT32 binNdx = (PWP_UINT32)((angle - 90.0) / (90.0 / numBins));
    if (binNdx < numBins) {
        ++maxAngleLog_[binNdx];
    }
    else {
        ++maxAngleLog_[numBins]; // Concave
    }
}


void
CaeUnsPluginRecomb2D::debugDump()
{
    if (dumpFile_.isOpen()) {
        vector3 midPt;
        EdgeIter it = edges_.begin();
        for (; it != edges_.end(); ++it) {
            if ((it->quality_ > maxInclAngle_) && (Concave != it->quality_)) {
                continue;
            }
            midPoint(it->i0_, it->i1_, midPt);
            fprintf(dumpFile_, "%10.5g %10.5g %10.5g %10.5g\n", midPt[0],
                midPt[1], midPt[2], it->quality_);
        }
    }
}


PWP_UINT32
CaeUnsPluginRecomb2D::streamBegin(const PWGM_BEGINSTREAM_DATA &data)
{
    char msg[512];
    sprintf(msg, "STREAM BEGIN: %lu", (unsigned long)data.totalNumFaces);
    sendInfoMsg(msg);
    return 1;
}


PWP_UINT32
CaeUnsPluginRecomb2D::streamFace(const PWGM_FACESTREAM_DATA &data)
{
    char msg[512];
    sprintf(msg, "  STREAM FACE: %lu %lu",
        (unsigned long)data.elemData.index[0],
        (unsigned long)data.elemData.index[1]);
    sendInfoMsg(msg);
    return 1;
}


PWP_UINT32
CaeUnsPluginRecomb2D::streamEnd(const PWGM_ENDSTREAM_DATA &data)
{
    char msg[512];
    sprintf(msg, "STREAM END: %s", (data.ok ? "true" : "false"));
    sendInfoMsg(msg);
    return 1;
}


bool
CaeUnsPluginRecomb2D::create(CAEP_RTITEM &rti)
{
    (void)rti.BCCnt; // silence unused arg warning
    return
        caeuPublishValueDefinition(attrMaxIncludedAngle, PWP_VALTYPE_REAL, "100", "RW",
            "Maximum included angle allowed for a recombined quad",
            "90 180 95 175") &&
        caeuPublishValueDefinition(attrDebugDump, PWP_VALTYPE_BOOL, "no", "RW",
            "Generate a debug dump file?", "no|yes");
}


void
CaeUnsPluginRecomb2D::destroy(CAEP_RTITEM &rti)
{
    (void)rti.BCCnt; // silence unused arg warning
}
