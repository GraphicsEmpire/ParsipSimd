//
// asynchonous CPU conters
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
//
// contact: Thomas Willhalm

#ifndef CPUASYNCHCOUNTER_HEADER
#define CPUASYNCHCOUNTER_HEADER


/*!     \file cpuasynchcounter.h
        \brief Implementation of a POSIX thread that periodically saves the current state of counters and exposes them to other threads
*/

#include <cpucounters/cpucounters.h>
#include <pthread.h>
#include <stdlib.h>

#define DELAY 1 // in seconds

using namespace std;


void * UpdateCounters(void *);

class AsynchronCounterState {
    PCM * m;

    CoreCounterState * cstates1, * cstates2;
    SocketCounterState * skstates1, * skstates2;
    SystemCounterState sstate1, sstate2;

    pthread_t UpdateThread;
    pthread_mutex_t CounterMutex;

    friend void * UpdateCounters(void *);

//	AsynchronCounterState(const& AsynchronCounterState); //unimplemeted
//	const& AsynchronCounterState operator=(const& AsynchronCounterState); //unimplemented

public:
    AsynchronCounterState()
    {
        m = PCM::getInstance();

        if (!m->good())
        {
            cout << "Can not access CPU counters" << endl;
            exit(-1);
        }
        m->program();


        cstates1 = new  CoreCounterState[m->getNumCores()];
        cstates2 = new  CoreCounterState[m->getNumCores()];
        skstates1 = new SocketCounterState[m->getNumSockets()];
        skstates2 = new SocketCounterState[m->getNumSockets()];

        for (int i = 0; i < m->getNumCores(); ++i) {
            cstates1[i] = getCoreCounterState(i);
            cstates2[i] = getCoreCounterState(i);
        }

        for (int i = 0; i < m->getNumSockets(); ++i) {
            skstates1[i] = getSocketCounterState(i);
            skstates2[i] = getSocketCounterState(i);
        }

        pthread_mutex_init(&CounterMutex, NULL);
        pthread_create(&UpdateThread, NULL, UpdateCounters, this);
    }
    ~AsynchronCounterState()
    {
        pthread_cancel(UpdateThread);
        pthread_mutex_destroy(&CounterMutex);
        delete[] cstates1;
        delete[] cstates2;
        delete[] skstates1;
        delete[] skstates2;
    }

    uint32 getNumCores()
    { return m->getNumCores(); }

    uint32 getNumSockets()
    { return m->getNumSockets(); }

    uint32 getQPILinksPerSocket()
    {
        return m->getQPILinksPerSocket();
    }

    uint32 getSocketId(uint32 c)
    {
        return m->getSocketId(c);
    }

    template <typename T, T func(CoreCounterState const &, CoreCounterState const &)>
    T get(uint32 core)
    {
        pthread_mutex_lock(&CounterMutex);
        T value = func(cstates1[core], cstates2[core]);
        pthread_mutex_unlock(&CounterMutex);
        return value;
    }

    template <typename T, T func(SocketCounterState const &, SocketCounterState const &)>
    T getSocket(uint32 socket)
    {
        pthread_mutex_lock(&CounterMutex);
        T value = func(skstates1[socket], skstates2[socket]);
        pthread_mutex_unlock(&CounterMutex);
        return value;
    }

    template <typename T, T func(uint32, uint32, SystemCounterState const &, SystemCounterState const &)>
    T getSocket(uint32 socket, uint32 param)
    {
        pthread_mutex_lock(&CounterMutex);
        T value = func(socket, param, sstate1, sstate2);
        pthread_mutex_unlock(&CounterMutex);
        return value;
    }

    template <typename T, T func(SystemCounterState const &, SystemCounterState const &)>
    T getSystem()
    {
        pthread_mutex_lock(&CounterMutex);
        T value = func(sstate1, sstate2);
        pthread_mutex_unlock(&CounterMutex);
        return value;
    }
};

void * UpdateCounters(void * state)
{
    AsynchronCounterState * s = (AsynchronCounterState *)state;

    while (true) {
        pthread_mutex_lock(&(s->CounterMutex));
        for (int core = 0; core < s->m->getNumCores(); ++core) {
            s->cstates1[core] = s->cstates2[core];
            s->cstates2[core] = s->m->getCoreCounterState(core);
        }

        for (int socket = 0; socket < s->m->getNumSockets(); ++socket) {
            s->skstates1[socket] = s->skstates2[socket];
            s->skstates2[socket] = s->m->getSocketCounterState(socket);
        }

        s->sstate1 = s->sstate2;
        s->sstate2 = s->m->getSystemCounterState();

        pthread_mutex_unlock(&(s->CounterMutex));
        sleep(1);
    }
}

#endif
