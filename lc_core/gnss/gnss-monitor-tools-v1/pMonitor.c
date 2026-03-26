
#include "./include/monitor.h"
#include "./include/licenseLib.h"
#include <stdio.h>
#include <stdlib.h>
#if _WIN32
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

#ifndef WIN_DLL

extern char uid[20];
extern bool isValid;

int main()
{

    mInfo moniInfo1;
    gtime_t ts, te;
    double es[6] = {2025, 6, 30, 7, 30, 37}, ee[6] = {2025, 6, 30, 7, 50, 37};
    ts = epoch2time(es);
    te = epoch2time(ee);
    gtime_t tn = te;
    char tsstr[40], testr[40];
    time2str(ts, tsstr, 0);
    time2str(tn, testr, 0);
    sprintf(moniInfo1.configStr, "3@2@%s@%s@@0@PSYJ01@PSYJ02@/home/gct/LC-CurveModel/data/@.obs@/home/gct/LC-CurveModel/data/brdc.rnx@45@/home/gct/LC-CurveModel/data/test.pos@0@0@0.5@@@", tsstr, testr);

    startMonitor(&moniInfo1);

    printf("solBuf: %s\n", moniInfo1.solBuf);

    return 1;
}

#endif
