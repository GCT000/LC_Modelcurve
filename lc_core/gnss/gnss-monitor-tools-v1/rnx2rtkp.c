/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*           2014/01/27  1.7 fix bug on default output time format
*           2015/05/15  1.8 -r or -l options for fixed or ppp-fixed mode
*           2015/06/12  1.9 output patch level in header
*           2016/09/07  1.10 add option -sys
*-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "./include/rtklib.h"
#include "./include/monitor.h"
#include "./include/licenseLib.h"
#define VERSION     "2.0.6.3"
#define PROGNAME    "mingguNav"          /* program name */
#define MAXFILE     100                  /* max number of input files */
#define MAXFIELD    30                  /* max number of input configure*/
// #define ZCFORMER    1
// #define XQHFORMER     1

/* global variables for license function -------------------------------------*/
char uid[20];
char file_name[200];
bool isValid;
int workMode = -2;

/* help text -----------------------------------------------------------------*/
static const char* help[] = {
"Library Name : libDMonitor",
"Version      : 2.0.6.3",
"Data Storage Format : root directory/YYYY/DOY/HH/name.YYYYDOYbinRTCM3",
"Available Functions : ",
"\tvoid* startMonitor(mInfo* arg)",
"\t\tclass mInfo{",
"\t\t\tchar configStr[600];",
"\t\t\tchar errMsg[1000];",
"\t\t\tchar solBuf[400];",
"\t\t};",
"\t\tconfigStr defination format:",
"\t\t\tcalculateMode@fileMode@startTime@endTime@sample@VRSMode@roverName@baseName@rootDirectory@suffix@broadcastFile@outputFile@storageMode@outMode",
"\t\t\tcalculateMode  : 0:spp; 2:kinematic; 3:static",
"\t\t\tfileMode       : 1:rtcm3; 2:rinex",
"\t\t\tstartTime      : YYYY/MM/DD hh:mm:ss(GPST)",
"\t\t\tendTime        : YYYY/MM/DD hh:mm:ss(GPST)",
"\t\t\tsample         : optional",
"\t\t\tVRSMode        : 0-false; 1:true",
"\t\t\troverName",
"\t\t\tbaseName",
"\t\t\trootDirectory",
"\t\t\tsuffix",
"\t\t\tbroadcastFile  : multi-ephemeris files divided with signal &",
"\t\t\tNAVSYS          :1:GPS; 2:SBAS; 4:Glonass; 8:Galileo; 16:QZSS; 32:BDS; 64:IRN",
"\t\t\toutputFile",
"\t\t\tstorageMode    : 0:single file; 1:hour file",
"\t\t\toutMode        : 0:all; 1:single",
"\t\t\tfixRate threshold: 0~1",
"\t\tsolBuf defination format:",
"\t\t\ttimeStart timeEnd dposMax dposAvg dposSTD epochRate(u) epochRate(r) fixRate E N U stat solutioType nSat nRover nBase obsFlag navFlag elapsedTime",
"\t\t\ttimeStart     :YYYY/MM/DD hh:mm:ss",
"\t\t\ttimeEnd       :YYYY/MM/DD hh:mm:ss",
"\t\t\tdposMax       :maximum displacement at adjacent time",
"\t\t\tdposAvg       :average displacement at adjacent time",
"\t\t\tdposSTD       :std of displacement at adjacent time",
"\t\t\tepochRate(u)  :epoch rate of user(0~1)",
"\t\t\tepochRate(r)  :epoch rate of reference(0~1)",
"\t\t\tfixRate       :if it is greater than 75%, the fixed solution will be output; otherwise, no result will be output",
"\t\t\tE",
"\t\t\tN",
"\t\t\tU",
"\t\t\tstat          :state of the solution result(Fixed;Float;None)",
"\t\t\tsolutioType   :1~4, 1 is available, 4 is inavailable",
"\t\t\tnSat          :average number of satellites available",
"\t\t\ttint_u        :sample(user)",
"\t\t\ttint_r        :sample(reference)",
"\t\t\tnRover        :number of epoch of rover",
"\t\t\tnBase         :number of epoch of base",
"\t\t\tobsFlag       :observation flag,0 is normal",
"\t\t\tnavFlag       :ephemeris flag,0 is normal",
"\t\t\telapsedTime   :calculation time(s)",
"\tchar* getVersion()",
"\tchar* getDeadline()",
"",
"Library Notes",
"",
"Note that this library is only for scientific research. For commercial inquiries, please contact <wxz_gnss@foxmail.com>",
"",
"In addition to this standard mode, we also have a full mode and a higher-precision post-processing mode. You can also contact <wxz_gnss@foxmail.com>",
""
};
/* show message --------------------------------------------------------------*/
extern int showmsg(const char *format, ...)
{
    va_list arg;
    va_start(arg,format); vfprintf(stderr,format,arg); va_end(arg);
    fprintf(stderr,"\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}

/* print help ----------------------------------------------------------------*/
#ifdef _WIN32
extern __declspec(dllexport) void printhelp(void)
#else
extern void printhelp(void)
#endif
{
    int i;
    for (i=0;i<(int)(sizeof(help)/sizeof(*help));i++) fprintf(stderr,"%s\n",help[i]);
    exit(0);
}

#ifdef _WIN32
extern __declspec(dllexport) char* getVersion()
#else
extern char* getVersion()
#endif
{
    printf("%s\n", VERSION);
    return VERSION;
}

#ifdef _WIN32
extern __declspec(dllexport) char* getDeadline()
#else
extern char* getDeadline()
#endif
{
    char t_str[64] = "2025/12/25 00:00:00";
    printf("%s\n", t_str);
    return t_str;
}

/* setLicense() */
#ifdef _WIN32
extern __declspec(dllexport) bool setLicense(char* _filename, int _workmode)
#else
extern bool setLicense(char* _filename, int _workmode)
#endif
{
    /*bool res;
    workMode=_workmode;
    if(workMode==-1){
        res=true;
    }else if(workMode==0){
        res=getAccessOn(_filename,uid);
        strcpy(file_name,_filename);
    }else if (workMode==1)
    {
        res=getAccessOff(_filename);
        isValid=res;
        strcpy(file_name,_filename);
    }else if (workMode==2)
    {
#ifndef _WIN32
        res = getAccessFrontend(_filename);
        isValid = res;
#endif
    }else{
        res=false;
        printf("work mode is not valid\n");
    }
    return res;*/
}
/// @brief set valid
#ifdef _WIN32
extern __declspec(dllexport) void setValue(char* _uid, bool _isValid)
#else
extern void setValue(char* _uid, bool _isValid)
#endif
{
    if (strcmp(_uid, uid) == 0) {
        if (_isValid) {
            isValid = true;
        }
        else {
            // printf("Date or user is invalid, please contact the developer.\n");
            isValid = false;
        }
    }
    else {
        // printf("Date or user is invalid, please contact the developer.\n");
        isValid = false;
    }
}

/* rtMode(0-2)@fileMode(rtcm rinex)@timeStart@timeEnd@filterPeriod@ProcessIntervel@sample@static(1:static 2:kinematic)@roverName@baseName@dir@suffix@brdm*/
void startMonitor(mInfo* arg)
{
    mInfo* info = arg;
    
    int i, ret, n = 0;
    char* infile[MAXFILE], outfile[200] = "";
    char* p, * q, * val[MAXFIELD], buff[600], roverName[50] = { 0 }, baseName[50] = { 0 }, brdcPath[100] = "";
    int navsys, access;
    monitor_t* monitor = NULL;

    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = { 0 };
    int ti = 0;/* process arc length */
    double ep_s[6], ep_e[6];
    int year, doy, hr;
    int tick_s, tick_e;

    int filetype = -1, solstatic = 1;
    int fileMode = 1, calMode = 1;/*real time and rtcm3*/
    int vrsMode = 0;
    double es[6] = { 0 }, ee[6] = { 0 }, eep[6] = { 0 };
    gtime_t timeStart = { 0 }, timeEnd = { 0 };
    gtime_t timePre = utc2gpst(timeget());
    char dataDir[256] = "";
    char suffix[20] = "", outSuffix[20]="", tmpts[40], tmpte[40];
    int en = 0;
    double fixThresh = -1;

    char* os = getenv("OS");
    char* sep;

    if (os && strcmp(os, "Windows_NT") == 0) {
        sep = "\\";
    }
    else {
        sep = "/";
    }

    char t_str[64] = "";
    /* rtMode@fileMode@timeStart@timeEnd@filterPeriod@ProcessIntervel@static@roverName@baseName@dir@configFile*/
    strcpy(buff, info->configStr);

    for (p = buff; *p && n < MAXFIELD; p = q + 1) {
        if (q = strchr(p, '@')) {
            val[n++] = p; *q = '\0';
        }
        else
        {
            val[n++] = p;
            break;
        }
    }

    for (i = 0; i < n; i++) {
        if (i == 0) {
            calMode = atoi(val[i]);
        }
        else if (i == 1)
        {
            fileMode = atoi(val[i]);
            if (fileMode != 1 && fileMode != 2)
            {
                en += sprintf(info->errMsg + en, "%s:%d\n", "File mode error", fileMode); 
                return 0;
            }
        }
        else if (i == 2) {
            sscanf(val[i], "%lf/%lf/%lf %lf:%lf:%lf", es, es + 1, es + 2, es + 3, es + 4, es + 5);
            timeStart = epoch2time(es);
            time2str(timeStart,tmpts,0);
        }
        else if (i == 3) {
            sscanf(val[i], "%lf/%lf/%lf %lf:%lf:%lf", ee, ee + 1, ee + 2, ee + 3, ee + 4, ee + 5);
            timeEnd = epoch2time(ee);
            time2str(timeEnd,tmpte,0);
        }
        else if (i == 4) ti = atoi(val[i]);
        else if (i == 5) {
            vrsMode = atoi(val[i]);
        }
        else if (i == 6) {
            if (strlen(val[i]) > 50)
            {
                en += sprintf(info->errMsg + en, "%s\n", "Rover name overs range");
                return 0;
            }
            strcpy(roverName, val[i]);
        }
        else if (i == 7) {
            if (strlen(val[i]) > 50)
            {
                en += sprintf(info->errMsg + en, "%s\n", "Base name overs range");
                return 0;
            }
            strcpy(baseName, val[i]);
        }
        else if (i == 8) {
            strcpy(dataDir, val[i]);
        }
        else if (i == 9) {
            strcpy(outSuffix, val[i]);
        }
        else if (i == 10) {
            strcpy(brdcPath, val[i]);
        }
        else if (i == 11) {
            navsys=atoi(val[i]);
        }
        else if (i == 12) {
            strcpy(outfile, val[i]);
        }
        else if (i == 13) {
            filetype = atoi(val[i]);
        }
        else if (i == 14) {
            solstatic = atoi(val[i]);
        }
        else if (i == 15) {
            fixThresh = atof(val[i]);
        }
        else if (i == 16) {
            prcopt.rb[0] = atof(val[i]);
        }
        else if (i == 17) {
            prcopt.rb[1] = atof(val[i]);
        }
        else if (i == 18) {
            prcopt.rb[2] = atof(val[i]);
        }
        else if (i == 19) {
            strcpy(filopt.satantp, val[i]);
            strcpy(filopt.rcvantp, val[i]);
        }

    }
    prcopt.mode = calMode;
    if(prcopt.mode==2){
        prcopt.modear=1;
    }
    info->solBuf[0] = '\0';
    info->errMsg[0] = '\0';
    if (timediff(timeEnd,timeStart) < 0) {
        printf("the end time is earlier than the start time");
    }
    for (i = 0; i < MAXFILE && i < (timediff(timeEnd, timeStart) / 3600 + 2) * 2 + 1; i++)
    {
        if (!(infile[i] = (char*)malloc(200)))
        {
            en += sprintf(info->errMsg + en, "%s\n", "Input file error");
            free(infile[i]); infile[i] = NULL;
            return 0;
        }
    }
    n = 0;
    if (filetype == 0) {
        time2epoch(timeStart, ep_s);
        year = (int)ep_s[0];
        doy = (int)time2doy(timeStart);
        if (outSuffix[0] == '\0') {
            sprintf(suffix, ".%4d%03dbinRTCM3", year, doy);
        }
        else
        {
            sprintf(suffix, outSuffix);
        }
        sprintf(infile[n++], "%s%s%s", dataDir, roverName, suffix);
        sprintf(infile[n++], "%s%s%s", dataDir, baseName, suffix);
    }
    else if (filetype == 1) {
        time2epoch(timeStart, ep_s); time2epoch(timeEnd, ep_e);

        ep_s[4] = ep_s[5] = 0.0; 
        
        gtime_t ts = epoch2time(ep_s), te;
        if (ep_e[4] != 0.0 || ep_e[5] != 0) {
            ep_e[4] = ep_e[5] = 0.0;
            te = epoch2time(ep_e);
            te = timeadd(te, 3600);
        }
        else
        {
            ep_e[4] = ep_e[5] = 0.0;
            te = epoch2time(ep_e);
        }
        for (ts = epoch2time(ep_s); timediff(ts, te) < -1; ts = timeadd(ts, 3600))
        {
            time2epoch(ts, ep_s);
            year = (int)ep_s[0]; hr = (int)ep_s[3];
            doy = (int)time2doy(ts);
            if (outSuffix[0] == '\0') {
                sprintf(suffix, ".%4d%03dbinRTCM3", year, doy);
            }
            else
            {
                sprintf(suffix, outSuffix);
            }
            sprintf(infile[n++], "%s%4d%s%03d%s%02d%s%s%s", dataDir, year, sep, doy, sep, hr, sep, roverName, suffix);
            sprintf(infile[n++], "%s%4d%s%03d%s%02d%s%s%s", dataDir, year, sep, doy, sep, hr, sep, baseName, suffix);
        }
    }
    else if (filetype == 2) {
        time2epoch(timeStart, ep_s); time2epoch(timeEnd, ep_e);

        ep_s[3] = ep_s[4] = ep_s[5] = 0.0;

        gtime_t ts = epoch2time(ep_s), te;
        if (ep_e[3] != 0.0 || ep_e[4] != 0.0 || ep_e[5] != 0) {
            ep_e[3] = ep_e[4] = ep_e[5] = 0.0;
            te = epoch2time(ep_e);
            te = timeadd(te, 24 * 3600);
        }
        else
        {
            ep_e[3] = ep_e[4] = ep_e[5] = 0.0;
            te = epoch2time(ep_e);
        }
        for (ts = epoch2time(ep_s); timediff(ts, te) < -1; ts = timeadd(ts, 24 * 3600))
        {
            time2epoch(ts, ep_s);
            year = (int)ep_s[0]; doy = (int)time2doy(ts);
            if (outSuffix[0] == '\0') {
                sprintf(suffix, ".%4d%03dbinRTCM3", year, doy);
            }
            else
            {
                sprintf(suffix, outSuffix);
            }
            sprintf(infile[n++], "%s%4d%s%03d%s%s%s", dataDir, year, sep, doy, sep, roverName, suffix);
            sprintf(infile[n++], "%s%4d%s%03d%s%s%s", dataDir, year, sep, doy, sep, baseName, suffix);
        }
    }

    if (brdcPath[0] != '\0'){
        for (p = brdcPath; *p; p = q + 1) {
            if (q = strchr(p, '&')) {
                strcpy(infile[n++],p);
                *q = '\0';
            }
            else
            {
                strcpy(infile[n++],p);
                break;
            }
        }
    }
    solopt.trace = 3;
    solopt.solstatic = solstatic;
    if(solstatic==0) solopt.posf=SOLF_ENU;
#ifdef XQHFORMER
    solopt.posf = SOLF_XYZ;
#endif
    prcopt.debug = 0;
    prcopt.exsats[satid2no("C01") - 1] = 1;
    prcopt.exsats[satid2no("C02") - 1] = 1;
    prcopt.exsats[satid2no("C03") - 1] = 1;
    prcopt.exsats[satid2no("C04") - 1] = 1;
    prcopt.exsats[satid2no("C05") - 1] = 1;

    prcopt.exsats[satid2no("C18") - 1] = 1;
    prcopt.exsats[satid2no("C59") - 1] = 1;
    prcopt.exsats[satid2no("C60") - 1] = 1;
    prcopt.exsats[satid2no("C61") - 1] = 1;
    if (fileMode == 1) prcopt.format = 1; /* rtcm3 */
    else prcopt.format = 2; /* obs */

    /* The ability to cull the system needs to be added */
    if (!(monitor = (monitor_t*)malloc(sizeof(monitor_t))))
    {
        en += sprintf(info->errMsg + en, "%s\n", "Initial error");
        return 0;
    }
    memset(monitor, 0, sizeof(monitor_t));
    if (fixThresh > 0) {
        monitor->fixThresh = fixThresh;
    }
    else
    {
        monitor->fixThresh = 0.75;
    }
    if(navsys>0){
        prcopt.navsys=navsys;
    }else{
        prcopt.navsys=45;
    }
    monitor->ts = timeStart; monitor->te = timeEnd;
    if(ti>0) {monitor->ti_r=ti; monitor->ti_u=ti;}
    monitor->ts = timeStart; monitor->te = timeEnd;
    tick_s = tickget();
    char tmptime[40];
    time2str(timeStart, tmptime, 0);
    strcat(info->solBuf, " ");
    strcat(info->solBuf, tmptime);
    time2str(timeEnd, tmptime, 0);
    strcat(info->solBuf, " ");
    strcat(info->solBuf, tmptime);
    ret = postpos(timeStart, timeEnd, ti > 0 ? ti : 0.0, 0.0, &prcopt, &solopt, &filopt, infile, n, outfile, "", "", info, monitor);
    tick_e = tickget();
    for (i = 0; i < n; i++)
    {
        free(infile[i]); infile[i] = NULL;
    }

    info->blh[0] = monitor->blh[0];
    info->blh[1] = monitor->blh[1];
    info->blh[2] = monitor->blh[2];
    info->enu[0] = monitor->enu[0];
    info->enu[1] = monitor->enu[1];
    info->enu[2] = monitor->enu[2];

    /* out result */
    if (strlen(info->solBuf) < 41 || monitor->obsflag > 0 || ((monitor->navflag & 4) && (monitor->navflag & 8))) {
        //time2str(monitor->t_p,t_str, 0);
        printf("roverNamef=%s, baseName=%s\n", roverName, baseName);
        sprintf(info->solBuf + strlen(info->solBuf), " %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %s 4 %d", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NONE", 0);
    }
    if(prcopt.mode != PMODE_SINGLE && monitor->validFlag == 0) {
        // if (monitor->obsflag != NULL && monitor->obsflag == 0) {
        if (monitor->obsflag == 0 && strstr(info->solBuf,"NONE") ==NULL) {
            sprintf(info->solBuf + strlen(info->solBuf), " %6.4f %6.4f %6.4f %s 4 %d", 0.0, 0.0, 0.0, "NONE", 0);
        }
    }
    sprintf(info->solBuf + strlen(info->solBuf), " %d %d %d %d %2d %2d %.1fs", monitor->ti_u, monitor->ti_r, monitor->nepoch_u, monitor->nepoch_r, monitor->obsflag, (vrsMode == 1) ? 0 : monitor->navflag, (tick_e - (double)tick_s) / 1000.0);

    if (fabs(monitor->enu[0] > 99999.999) || fabs(monitor->enu[1] > 99999.999) || fabs(monitor->enu[2] > 99999.999))
    {
        printf("roverNamef=%s, baseName=%s\n", roverName, baseName);
        printf("len solBuf=%d, obsflag=%d, navflag=%d, mode=%d, validFlag=%d\n", (int)strlen(info->solBuf), monitor->obsflag, monitor->navflag, prcopt.mode, monitor->validFlag);
        printf("f(startMonitor), abnormal value detected!!\n,monitor->enu[0]=%lf monitor->enu[1]=%lf monitor->enu[2]=%lf", monitor->enu[0], monitor->enu[1], monitor->enu[2]);
    }
    if (monitor) {
        free(monitor); monitor = NULL;
    }


}
