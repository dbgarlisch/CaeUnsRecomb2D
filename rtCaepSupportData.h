/****************************************************************************
 *
 * Pointwise Plugin utility functions
 *
 * Proprietary software product of Pointwise, Inc.
 * Copyright (c) 1995-2013 Pointwise, Inc.
 * All rights reserved.
 *
 ***************************************************************************/

#ifndef _RTCAEPSUPPORTDATA_H_
#define _RTCAEPSUPPORTDATA_H_

#include "condtypes.h"

/*! \cond */

/*------------------------------------*/
/* CaeUnsRecomb2D format item setup data */
/*------------------------------------*/
CAEP_BCINFO CaeUnsRecomb2DBCInfo[] = {
    { "inlet",      BcIdInlet },
    { "outlet",     BcIdOutlet },
    { "wall",       BcIdWall },
};
/*------------------------------------*/
CAEP_VCINFO CaeUnsRecomb2DVCInfo[] = {
    { "fluid",  VcIdFluid },
    { "plasma", VcIdPlasma },
    { "solid",  VcIdSolid },
};
/*------------------------------------*/
const char *CaeUnsRecomb2DFileExt[] = {
    "msh"
};

/*! \endcond */

#endif /* _RTCAEPSUPPORTDATA_H_ */
