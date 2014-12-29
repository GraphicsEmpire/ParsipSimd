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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <cpucounters/types.h>
#include <cpucounters/msr.h>
#include <assert.h>

#ifdef _MSC_VER

#include <windows.h>
#include "Winmsrdriver\win7\msrstruct.h"

// here comes an implementatation for Windows
MsrHandle::MsrHandle(uint32 cpu) : cpu_id(cpu)
{
    hDriver = CreateFile(L"\\\\.\\RDMSR", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hDriver == INVALID_HANDLE_VALUE)
        throw std::exception();
}

MsrHandle::~MsrHandle()
{
    CloseHandle(hDriver);
}

int32 MsrHandle::write(uint64 msr_number, uint64 value)
{
    MSR_Request req;
    ULONG64 result;
    DWORD reslength = 0;
    req.core_id = cpu_id;
    req.msr_address = msr_number;
    req.write_value = value;
    BOOL status = DeviceIoControl(hDriver, IO_CTL_MSR_WRITE, &req, sizeof(MSR_Request), &result, sizeof(uint64), &reslength, NULL);
    assert(status && "Error in DeviceIoControl");
    return reslength;
}

int32 MsrHandle::read(uint64 msr_number, uint64 * value)
{
    MSR_Request req;
    // ULONG64 result;
    DWORD reslength = 0;
    req.core_id = cpu_id;
    req.msr_address = msr_number;
    BOOL status = DeviceIoControl(hDriver, IO_CTL_MSR_READ, &req, sizeof(MSR_Request), value, sizeof(uint64), &reslength, NULL);
    assert(status && "Error in DeviceIoControl");
    return reslength;
}


#else
// here comes a Linux version
MsrHandle::MsrHandle(uint32 cpu) : fd(-1), cpu_id(cpu)
{
    char * path = new char[200];
    sprintf(path, "/dev/cpu/%d/msr", cpu);
    int handle = ::open(path, O_RDWR);
    delete[] path;
    if (handle < 0) throw std::exception();
    fd = handle;
}

MsrHandle::~MsrHandle()
{
    if (fd >= 0) ::close(fd);
}

int32 MsrHandle::write(uint64 msr_number, uint64 value)
{
    int32 result = ::lseek(fd, msr_number, SEEK_SET);
    if (result < 0) return result;

    return ::write(fd, (const void *)&value, sizeof(uint64));
}

int32 MsrHandle::read(uint64 msr_number, uint64 * value)
{
    int32 result = ::lseek(fd, msr_number, SEEK_SET);
    if (result < 0) return result;

    return ::read(fd, (void *)value, sizeof(uint64));
}

#endif
