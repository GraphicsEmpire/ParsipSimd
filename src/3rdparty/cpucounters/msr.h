/* 
  Copyright 2009-2011 Intel Corporation All Rights Reserved.
 
  The source code, information and material ("Material") contained herein is 
  owned by Intel Corporation or its suppliers or licensors, and title to such 
  Material remains with Intel Corporation or its suppliers or licensors. The 
  Material contains proprietary information of Intel or its suppliers and 
  licensors. The Material is protected by worldwide copyright laws and treaty 
  provisions. No part of the Material may be used, copied, reproduced, 
  modified, published, uploaded, posted, transmitted, distributed or 
  disclosed in any way without Intel's prior express written permission. 
  No license under any patent, copyright or other intellectual property 
  rights in the Material is granted to or conferred upon you, either expressly, 
  by implication, inducement, estoppel or otherwise. Any license under such 
  intellectual property rights must be express and approved by Intel in 
  writing.
  Unless otherwise agreed by Intel in writing, you may not remove or alter 
  this notice or any other notice embedded in Materials by Intel or Intel's 
  suppliers or licensors in any way.
  Use of this material must comply with the 
  rights and restrictions set forth in the accompnied license terms set
  forth in file "license.txt"
*/
// written by Roman Dementiev
//

#ifndef CPUCounters_MSR_H
#define CPUCounters_MSR_H

/*!     \file msr.h
        \brief Low level interface to access hardware model specific registers

        Implemented and tested for Linux and 64-bit Windows 7
*/

#include <cpucounters/types.h>

#ifdef _MSC_VER
#include "windows.h"
#endif


class MsrHandle
{
#ifdef _MSC_VER
    HANDLE hDriver;
#else
    int32 fd;
#endif
    uint32 cpu_id;

    MsrHandle();            // forbidden
    MsrHandle(MsrHandle &); // forbidden

public:
    MsrHandle(uint32 cpu);
    int32 read(uint64 msr_number, uint64 * value);
    int32 write(uint64 msr_number, uint64 value);
    uint32 getCoreId() { return cpu_id; }
    virtual ~MsrHandle();
};


#endif
