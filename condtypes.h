/****************************************************************************
 *
 * Pointwise Plugin utility functions
 *
 * Proprietary software product of Pointwise, Inc.
 * Copyright (c) 1995-2013 Pointwise, Inc.
 * All rights reserved.
 *
 ***************************************************************************/

#ifndef _CONDTYPES_H_
#define _CONDTYPES_H_

/*! \cond */

/*------------------------------------*/
/* CaeUnsRecomb2D format item setup data */
/*------------------------------------*/
enum {
    BcIdFirst_ = 100,

    // BC ids. bookkeeping value above
    BcIdInlet = BcIdFirst_,
    BcIdOutlet,
    BcIdWall,

    // add new ids above here. bookkeeping values below
    BcIdEnd_,
    BcIdLast_ = BcIdEnd_ - 1,
    BcIdCount_ = BcIdEnd_ - BcIdFirst_
};


enum {
    VcIdFirst_ = 200,

    // VC ids. bookkeeping value above
    VcIdFluid = VcIdFirst_,
    VcIdPlasma,
    VcIdSolid,

    // add new ids above here. bookkeeping values below
    VcIdEnd_,
    VcIdLast_ = VcIdEnd_ - 1,
    VcIdCount_ = VcIdEnd_ - VcIdFirst_
};

/*! \endcond */

#endif /* _CONDTYPES_H_ */
