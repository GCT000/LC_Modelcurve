/*------------------------------------------------------------------------------
* postpos.c : post-processing positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/05/08  1.0  new
*           2008/06/16  1.1  support binary inputs
*           2009/01/02  1.2  support new rtk positioing api
*           2009/09/03  1.3  fix bug on combined mode of moving-baseline
*           2009/12/04  1.4  fix bug on obs data buffer overflow
*           2010/07/26  1.5  support ppp-kinematic and ppp-static
*                            support multiple sessions
*                            support sbas positioning
*                            changed api:
*                                postpos()
*                            deleted api:
*                                postposopt()
*           2010/08/16  1.6  fix bug sbas message synchronization (2.4.0_p4)
*           2010/12/09  1.7  support qzss lex and ssr corrections
*           2011/02/07  1.8  fix bug on sbas navigation data conflict
*           2011/03/22  1.9  add function reading g_tec file
*           2011/08/20  1.10 fix bug on freez if solstatic=single and combined
*           2011/09/15  1.11 add function reading stec file
*           2012/02/01  1.12 support keyword expansion of rtcm ssr corrections
*           2013/03/11  1.13 add function reading otl and erp data
*           2014/06/29  1.14 fix problem on overflow of # of satellites
*           2015/03/23  1.15 fix bug on ant type replacement by rinex header
*                            fix bug on combined filter for moving-base mode
*           2015/04/29  1.16 fix bug on reading rtcm ssr corrections
*                            add function to read satellite fcb
*                            add function to read stec and troposphere file
*                            add keyword replacement in dcb, erp and ionos file
*           2015/11/13  1.17 add support of L5 antenna phase center parameters
*                            add *.stec and *.trp file for ppp correction
*           2015/11/26  1.18 support opt->freqopt(disable L2)
*           2016/01/12  1.19 add carrier-phase bias correction by ssr
*           2016/07/31  1.20 fix error message problem in rnx2rtkp
*           2016/08/29  1.21 suppress warnings
*           2016/10/10  1.22 fix bug on identification of file fopt->blq
*           2017/06/13  1.23 add smoother of velocity solution
*           2020/11/30  1.24 use API sat2freq() to get carrier frequency
*                            fix bug on select best solution in static mode
*                            delete function to use L2 instead of L5 PCV
*                            writing solution file in binary mode
*-----------------------------------------------------------------------------*/
#include "./include/rtklib.h"

#define MIN(x,y)    ((x)<(y)?(x):(y))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

#define MAXPRCDAYS  100          /* max days of continuous processing */
#define MAXINFILE   1000         /* max number of input files */
#define NINCOBS     262144              /* incremental number of obs data */
#define MAXINVALIDTM 100         /* max number of invalid time marks */


static int findMode(int *arr, int n) {
    // Initialize variables to keep track of the mode and its frequency
    int mode = arr[0];
    int maxCount = 1;

    // Loop through the array and count the frequency of each element
    for (int i = 0; i < n; i++) {
        int count = 0;
        for (int j = 0; j < n; j++) {
            if (arr[j] == arr[i]) {
                count++;
            }
        }
        // If the frequency of the current element is greater than the current mode, update the mode and its frequency
        if (count > maxCount) {
            mode = arr[i];
            maxCount = count;
        }
    }

    // Return the mode
    return mode;
}

/* show message and check break ----------------------------------------------*/
static int checkbrk(const char *format, ...)
{
    va_list arg;
    char buff[1024],*p=buff;
    if (!*format) return showmsg("");
    va_start(arg,format);
    p+=vsprintf(p,format,arg);
    va_end(arg);
    return showmsg(buff);
}
/* output reference position -------------------------------------------------*/
static void outrpos(FILE *fp, const double *r, const solopt_t *opt)
{
    double pos[3],dms1[3],dms2[3];
    const char *sep=opt->sep;
    
    /*trace(3,"outrpos :\n");*/
    
    if (opt->posf==SOLF_LLH||opt->posf==SOLF_ENU) {
        ecef2pos(r,pos);
        if (opt->degf) {
            deg2dms(pos[0]*R2D,dms1,5);
            deg2dms(pos[1]*R2D,dms2,5);
            fprintf(fp,"%3.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f%s%10.4f",
                    dms1[0],sep,dms1[1],sep,dms1[2],sep,dms2[0],sep,dms2[1],
                    sep,dms2[2],sep,pos[2]);
        }
        else {
            fprintf(fp,"%13.9f%s%14.9f%s%10.4f",pos[0]*R2D,sep,pos[1]*R2D,
                    sep,pos[2]);
        }
    }
    else if (opt->posf==SOLF_XYZ) {
        fprintf(fp,"%14.4f%s%14.4f%s%14.4f",r[0],sep,r[1],sep,r[2]);
    }
}
/* output header -------------------------------------------------------------*/
static void outheader(FILE *fp, char **file, int n, const prcopt_t *popt,
                      const solopt_t *sopt, monitor_t* monitor)
{
    const char *s1[]={"GPST","UTC","JST"};
    gtime_t ts,te;
    double t1,t2;
    int i,j,w1,w2;
    char s2[32],s3[32];
    
    /*trace(3,"outheader: n=%d\n",n);*/
    
    if (sopt->posf==SOLF_NMEA||sopt->posf==SOLF_STAT) {
        return;
    }
    if (sopt->outhead) {
        fprintf(fp,"%s program   : mingguNav %s\n",COMMENTH,VER);
        for (i=0;i<n;i++) {
            fprintf(fp,"%s inp file  : %s\n",COMMENTH,file[i]);
        }
        for (i=0;i< monitor->obss.n;i++)    if (monitor->obss.data[i].rcv==1) break;
        for (j= monitor->obss.n-1;j>=0;j--) if (monitor->obss.data[j].rcv==1) break;
        if (j<i) {fprintf(fp,"\n%s no rover obs data\n",COMMENTH); return;}
        ts= monitor->obss.data[i].time;
        te= monitor->obss.data[j].time;
        t1=time2gpst(ts,&w1);
        t2=time2gpst(te,&w2);
        if (sopt->times>=1) ts=gpst2utc(ts);
        if (sopt->times>=1) te=gpst2utc(te);
        if (sopt->times==2) ts=timeadd(ts,9*3600.0);
        if (sopt->times==2) te=timeadd(te,9*3600.0);
        time2str(ts,s2,1);
        time2str(te,s3,1);
        fprintf(fp,"%s obs start : %s %s (week%04d %8.1fs)\n",COMMENTH,s2,s1[sopt->times],w1,t1);
        fprintf(fp,"%s obs end   : %s %s (week%04d %8.1fs)\n",COMMENTH,s3,s1[sopt->times],w2,t2);
    }
    if (sopt->outopt) {
        outprcopt(fp,popt);
    }
    if (PMODE_DGPS<=popt->mode&&popt->mode<=PMODE_FIXED&&popt->mode!=PMODE_MOVEB) {
        fprintf(fp,"%s base position   :",COMMENTH);
        outrpos(fp,popt->rb,sopt);
        fprintf(fp,"\n");
    }
    if (sopt->outhead||sopt->outopt) fprintf(fp,"%s\n",COMMENTH);
    
    outsolhead(fp,sopt);
}
/* search next observation data index ----------------------------------------*/
static int nextobsf(const obs_t *obs, int *i, int rcv)
{
    double tt;
    int n;
    
    for (;*i<obs->n;(*i)++) if (obs->data[*i].rcv==rcv) break;
    for (n=0;*i+n<obs->n;n++) {
        tt=timediff(obs->data[*i+n].time,obs->data[*i].time);
        if (obs->data[*i+n].rcv!=rcv||tt>DTTOL) break;
    }
    return n;
}
static int nextobsb(const obs_t *obs, int *i, int rcv)
{
    double tt;
    int n;
    
    for (;*i>=0;(*i)--) if (obs->data[*i].rcv==rcv) break;
    for (n=0;*i-n>=0;n++) {
        tt=timediff(obs->data[*i-n].time,obs->data[*i].time);
        if (obs->data[*i-n].rcv!=rcv||tt<-DTTOL) break;
    }
    return n;
}
/* update rtcm ssr correction ------------------------------------------------*/
static void update_rtcm_ssr(gtime_t time, monitor_t* monitor)
{
    char path[1024];
    int i;
    
    /* open or swap rtcm file */
    reppath(monitor->rtcm_file,path,time,"","");
    
    if (strcmp(path, monitor->rtcm_path)) {
        strcpy(monitor->rtcm_path,path);
        
        if (monitor->fp_rtcm) fclose(monitor->fp_rtcm);
        monitor->fp_rtcm=fopen(path,"rb");
        if (monitor->fp_rtcm) {
            monitor->rtcm.time=time;
            input_rtcm3f(&monitor->rtcm, monitor->fp_rtcm);
            /*trace(2,"rtcm file open: %s\n",path);*/
        }
    }
    if (!monitor->fp_rtcm) return;
    
    /* read rtcm file until current time */
    while (timediff(monitor->rtcm.time,time)<1E-3) {
        if (input_rtcm3f(&monitor->rtcm, monitor->fp_rtcm)<-1) break;
        
        /* update ssr corrections */
        for (i=0;i<MAXSAT;i++) {
            if (!monitor->rtcm.ssr[i].update||
                monitor->rtcm.ssr[i].iod[0]!= monitor->rtcm.ssr[i].iod[1]||
                timediff(time, monitor->rtcm.ssr[i].t0[0])<-1E-3) continue;
            monitor->navs.ssr[i]= monitor->rtcm.ssr[i];
            monitor->rtcm.ssr[i].update=0;
        }
    }
}
/* input obs data, navigation messages and sbas correction -------------------*/
static int inputobs(obsd_t *obs, int solq, const prcopt_t *popt, monitor_t* monitor)
{
    gtime_t time={0};
    int i,nu,nr,n=0;
    
    /*trace(3,"\ninfunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n",revs,iobsu,iobsr,isbs);*/
    
    if (0<= monitor->iobsu&& monitor->iobsu< monitor->obss.n) {
        settime((time= monitor->obss.data[monitor->iobsu].time));
    }
    if (!monitor->revs) { /* input forward data */
        if ((nu=nextobsf(&monitor->obss,&monitor->iobsu,1))<=0) return -1;
        if (popt->intpref) {
            for (;(nr=nextobsf(&monitor->obss,&monitor->iobsr,2))>0; monitor->iobsr+=nr)
                if (timediff(monitor->obss.data[monitor->iobsr].time, monitor->obss.data[monitor->iobsu].time)>-DTTOL) break;
        }
        else {
            for (i= monitor->iobsr;(nr=nextobsf(&monitor->obss,&i,2))>0; monitor->iobsr=i,i+=nr)
                if (timediff(monitor->obss.data[i].time, monitor->obss.data[monitor->iobsu].time)>DTTOL) break;
        }
        nr=nextobsf(&monitor->obss,&monitor->iobsr,2);
        if (nr<=0) {
            nr=nextobsf(&monitor->obss,&monitor->iobsr,2);
        }
        for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]= monitor->obss.data[monitor->iobsu+i];
        for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]= monitor->obss.data[monitor->iobsr+i];
        monitor->iobsu+=nu;
        
        /* update sbas corrections */
        while (monitor->isbs< monitor->sbss.n) {
            time=gpst2time(monitor->sbss.msgs[monitor->isbs].week, monitor->sbss.msgs[monitor->isbs].tow);
            
            if (getbitu(monitor->sbss.msgs[monitor->isbs].msg,8,6)!=9) { /* except for geo nav */
                sbsupdatecorr(monitor->sbss.msgs+ monitor->isbs,&monitor->navs);
            }
            if (timediff(time,obs[0].time)>-1.0-DTTOL) break;
            monitor->isbs++;
        }
        /* update rtcm ssr corrections */
        if (*monitor->rtcm_file) {
            update_rtcm_ssr(obs[0].time, monitor);
        }
    }
    else { /* input backward data */
        if ((nu=nextobsb(&monitor->obss,&monitor->iobsu,1))<=0) return -1;
        if (popt->intpref) {
            for (;(nr=nextobsb(&monitor->obss,&monitor->iobsr,2))>0; monitor->iobsr-=nr)
                if (timediff(monitor->obss.data[monitor->iobsr].time, monitor->obss.data[monitor->iobsu].time)<DTTOL) break;
        }
        else {
            for (i= monitor->iobsr;(nr=nextobsb(&monitor->obss,&i,2))>0; monitor->iobsr=i,i-=nr)
                if (timediff(monitor->obss.data[i].time, monitor->obss.data[monitor->iobsu].time)<-DTTOL) break;
        }
        nr=nextobsb(&monitor->obss,&monitor->iobsr,2);
        for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]= monitor->obss.data[monitor->iobsu-nu+1+i];
        for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]= monitor->obss.data[monitor->iobsr-nr+1+i];
        monitor->iobsu-=nu;
        
        /* update sbas corrections */
        while (monitor->isbs>=0) {
            time=gpst2time(monitor->sbss.msgs[monitor->isbs].week, monitor->sbss.msgs[monitor->isbs].tow);
            
            if (getbitu(monitor->sbss.msgs[monitor->isbs].msg,8,6)!=9) { /* except for geo nav */
                sbsupdatecorr(monitor->sbss.msgs+ monitor->isbs,&monitor->navs);
            }
            if (timediff(time,obs[0].time)<1.0+DTTOL) break;
            monitor->isbs--;
        }
    }
    return n;
}
/* output to file message of invalid time mark -------------------------------*/
static void outinvalidtm(FILE *fptm, const solopt_t *opt, const gtime_t tm)
{
    gtime_t time = tm;
    double gpst;
    const double secondsInAWeek = 604800;
    int week,timeu;
    char s[100];

    timeu=opt->timeu<0?0:(opt->timeu>20?20:opt->timeu);

    if (opt->times>=TIMES_UTC) time=gpst2utc(time);
    if (opt->times==TIMES_JST) time=timeadd(time,9*3600.0);

    if (opt->timef) time2str(time,s,timeu);
    else {
        gpst=time2gpst(time,&week);
        if (secondsInAWeek-gpst < 0.5/pow(10.0,timeu)) {
            week++;
            gpst=0.0;
        }
        sprintf(s,"%4d   %*.*f",week,6+(timeu<=0?0:timeu+1),timeu,gpst);
    }
    strcat(s, "   Q=0, Time mark is not valid\n");

    fwrite(s,strlen(s),1,fptm);
}
/* fill structure sol_t for time mark ----------------------------------------*/
static sol_t fillsoltm(const sol_t solold, const sol_t solnew, const gtime_t tm)
{
    gtime_t t1={0},t2={0};
    sol_t sol=solold;
    int i=0;

    if (solold.stat == 0 || solnew.stat == 0) {
        sol.stat = 0;
    } else {
        sol.stat = (solold.stat > solnew.stat) ? solold.stat : solnew.stat;
    }
    sol.ns = (solold.ns < solnew.ns) ? solold.ns : solnew.ns;
    sol.ratio = (solold.ratio < solnew.ratio) ? solold.ratio : solnew.ratio;

    /* interpolation position and speed of time mark */
    t1 = solold.time;
    t2 = solnew.time;
    sol.time = tm;

    for (i=0;i<6;i++)
    {
        sol.rr[i] = solold.rr[i] + timediff(tm,t1) / timediff(t2,t1) * (solnew.rr[i] - solold.rr[i]);
    }

    return sol;
}

/* carrier-phase bias correction by ssr --------------------------------------*/
static void corr_phase_bias_ssr(obsd_t *obs, int n, const nav_t *nav)
{
    double freq;
    uint8_t code;
    int i,j;
    
    for (i=0;i<n;i++) for (j=0;j<NFREQ;j++) {
        code=obs[i].code[j];
        
        if ((freq=sat2freq(obs[i].sat,code,nav))==0.0) continue;
        
        /* correct phase bias (cyc) */
        obs[i].L[j]-=nav->ssr[obs[i].sat-1].pbias[code-1]*freq/CLIGHT;
    }
}

int compare(const void *a, const void *b) {
    const double *arr1 = *(const double **)a;
    const double *arr2 = *(const double **)b;

    // 按第一个元素排序
    return (arr1[0] > arr2[0]) - (arr1[0] < arr2[0]);
}

void findMidSpp(sol_t* sol, int num, double mid[]) {
    // 创建一个二维数组，用于存储 rr 的值
    double **rr_values = (double **)malloc(num * sizeof(double *));
    for (int i = 0; i < num; i++) {
        rr_values[i] = sol[i].rr;
    }


    // 对 rr_values 数组进行排序
    qsort(rr_values, num, sizeof(double *), compare);
    
  
    // 将中位数赋值给 mid
    for (int i = 0; i < 3; i++) {
        if (num % 2 == 1) {
            mid[i] = rr_values[num / 2][i];
        } else {
            mid[i] = (rr_values[num / 2 - 1][i] + rr_values[num / 2][i]) / 2.0;
        }
    }

    // 释放内存
    free(rr_values);
}

static double coorlen(double* x1, double* x2){
    double rr[3];
    for(int i=0;i<3;i++) rr[i]=x1[i]-x2[i];
    return norm(rr,3);
}

static int cal_distance(sol_t* solarr,int num){
    int i,j;
    double x1[3],x2[3];
    for(i=0;i<num;i++){
        if(solarr[i].stat==1){
            matcpy(x1,solarr[i].rr,1,3);
            break;
        }
    }
    for(j=num-1;j>-1;j--){
        if(solarr[j].stat==1){
            matcpy(x2,solarr[j].rr,1,3);
            break;
        }
    }
    if(i!=j&&coorlen(x1,x2)>0.2) return 1;
    else return 0;
}


/* process positioning -------------------------------------------------------*/
static void procpos(FILE* fp, FILE* fp_trace, FILE* fp_table, const prcopt_t *popt, const solopt_t *sopt,
                    rtk_t *rtk, int mode, mInfo* minfo, monitor_t* monitor)
{
    gtime_t time={0};
    sol_t sol = { {0} };
    obsd_t* last_obs = (obsd_t*)malloc(sizeof(obsd_t) * MAXOBS * 2);/* rover and base for last epoch */
    double rb[3]={0}, rr[3] = { 0 }, pos[3];
    int i, nobs, lastn, solstatic, num = 0, pri[] = { 6,1,2,3,4,5,1,6 };
    int slp = 0;  /* tdcpi:tdcp index; slp:slip indicator */int nSat = 0;
    double fixRate = 0;
    
    /*trace(3,"procpos : mode=%d\n",mode);*/
    
    solstatic=(sopt->solstatic&&
              (popt->mode==PMODE_STATIC||popt->mode==PMODE_STATIC_START||popt->mode==PMODE_PPP_STATIC)) || popt->mode == PMODE_SINGLE;
    
    /* initialize unless running backwards on a combined run with phase reset disabled */
    if (mode==0 || !monitor->revs || popt->soltype==2)
        rtkinit(rtk,popt);
    
    monitor->rtcm_path[0]='\0';
    monitor->epochNum = 0; monitor->fixNum = 0;
    while ((nobs=inputobs(last_obs,rtk->sol.stat,popt, monitor))>=0) {
        /* exclude satellites */
        double eptmp[6] = {0};
        time2epoch(last_obs->time,eptmp);
        for (i=lastn=0;i<nobs;i++) {
            if ((satsys(last_obs[i].sat,NULL)&popt->navsys)&&
                popt->exsats[last_obs[i].sat-1]!=1) last_obs[lastn++]= last_obs[i];
        }
        if (lastn<=0) continue;
        /* carrier-phase bias correction */
        if (!strstr(popt->pppopt,"-ENA_FCB")) {
            corr_phase_bias_ssr(last_obs,lastn,&monitor->navs);
        }
        if (!rtkpos(fp_trace, fp_table, rtk, last_obs, lastn, &monitor->navs)) {
            if (rtk->sol.eventime.time != 0) {
                if (mode == 0) {
                    // outinvalidtm(fptm, sopt, rtk->sol.eventime);
                } else if (!monitor->revs&& monitor->nitm<MAXINVALIDTM) {
                    monitor->invalidtm[monitor->nitm++] = rtk->sol.eventime;
                }
            }
            continue;
        }

        /* add in 2023/03/22 10:19 by Guo Zihuai*/
        for (i = 0; i < 3; i++) monitor->dpos[monitor->epochNum * 3 + i] = rtk->dpos[0][0][i];
        monitor->epochNum++;
        if (mode==0) { /* forward/backward */
            if (!solstatic && fp) {
                gtime_t tmptime = { 0 };
                //trace(NULL,2,"rtk->rr=%14.4f,%14.4f,%14.4f,rtk->rb=%14.4f,%14.4f,%14.4f\n",rtk->sol.rr[0],rtk->sol.rr[1],rtk->sol.rr[2],rtk->rb[0],rtk->rb[1],rtk->rb[2]);
                outsol(fp,&rtk->sol,rtk->rb,sopt,&tmptime);
                if(rtk->sol.stat==1) sol = rtk->sol;
                for (i = 0; i < 3; i++) rb[i] = rtk->rb[i];
                time = rtk->sol.time;
            }
            else if (time.time==0||pri[rtk->sol.stat]<=pri[sol.stat]) {
                sol=rtk->sol;
                for (i=0;i<3;i++) rb[i]=rtk->rb[i];
                if (time.time==0||timediff(rtk->sol.time,time)<0.0) {
                    time=rtk->sol.time;
                }
            }
            if (rtk->sol.stat == 1) {
                monitor->solf[monitor->fixOrFloatNum] = rtk->sol;
                sol = rtk->sol;
                monitor->fixNum++;
                monitor->fixOrFloatNum++;
            }
            else if (rtk->sol.stat == 2) {
                monitor->solf[monitor->fixOrFloatNum] = rtk->sol;
                monitor->fixOrFloatNum++;
                sol = rtk->sol;
            }
            if(rtk->opt.mode==0){
                monitor->solf[monitor->fixOrFloatNum++] = rtk->sol;
            }
            nSat += rtk->sol.ns;
        }
        else if (!monitor->revs) { /* combined-forward */
            if (monitor->isolf >= monitor->nepoch) {
                free(last_obs);
                return;
            }
            monitor->solf[monitor->isolf]=rtk->sol;
            for (i=0;i<3;i++) monitor->rbf[i+ monitor->isolf*3]=rtk->rb[i];
            monitor->isolf++;
        }
        else { /* combined-backward */
            if (monitor->isolb>= monitor->nepoch) {
                free(last_obs);
                return;
            }
            monitor->solb[monitor->isolb]=rtk->sol;
            for (i=0;i<3;i++) monitor->rbb[i+ monitor->isolb*3]=rtk->rb[i];
            monitor->isolb++;
        }
    }
    if((rtk->opt.mode==0||rtk->opt.mode==2)&&monitor->fixOrFloatNum>0){
        // 取中位数
        findMidSpp(monitor->solf, monitor->fixOrFloatNum, sol.rr);
    }
    /* add in 2023/03/22 10:23 by Guo Zihuai */
    double dposmax = 0, dposavg = 0, dposstd = 0, dpos = 0;
    for (i = 0; i < monitor->epochNum; i++) {
        trace(NULL, 5, "dpos="); tracemat(fp_trace, 5, monitor->dpos + i * 3, 1, 3, 14, 8);
        dpos = norm(monitor->dpos + i * 3, 3);
        if (dposmax < dpos) dposmax = dpos;
        dposavg += dpos;
    }
    dposavg /= monitor->epochNum;
    for (i = 0; i < monitor->epochNum; i++) {
        dpos = norm(monitor->dpos + i * 3, 3);
        dposstd += pow(dpos - dposavg, 2);
    }
    dposstd = sqrt(dposstd / monitor->epochNum);
    trace(NULL, 5, "DPOS index : max = %.4f average = %.4f std = %.4f\n", dposmax, dposavg, dposstd);

    if (mode == 0 && time.time != 0.0) {

        if (minfo->solBuf) {
            sprintf(minfo->solBuf + strlen(minfo->solBuf), " %6.4f %6.4f %6.4f", dposmax, dposavg, dposstd);
        }
        /* add a fixed rate judgement */
        fixRate = (double)(monitor->fixNum) / (double)(monitor->epochNum);
        sprintf(minfo->solBuf + strlen(minfo->solBuf), " %6.4f %6.4f %6.4f", fixRate, monitor->epochRate_u, monitor->epochRate_r);
        if (monitor->fixOrFloatNum > 0) {
            monitor->nSatUsed = ((double)nSat) / monitor->epochNum;
            ecef2pos(sol.rr, monitor->blh);
            for (i = 0; i < 3; i++) rr[i] = sol.rr[i] - rb[i];
            ecef2pos(rb, pos);
            ecef2enu(pos, rr, monitor->enu);
            if (sol.stat == SOLQ_FIX) {
                monitor->validFlag = SOLQ_FIX;
            }
            else if (sol.stat == SOLQ_FLOAT)
                monitor->validFlag = SOLQ_FLOAT;
            else
                monitor->validFlag = 0;
        }
        if(sol.stat==0){
            free(last_obs); return;
        }
        if (rtk->opt.mode == PMODE_SINGLE) {
            ecef2pos(rtk->sol.rr, pos);
            sprintf(minfo->solBuf + strlen(minfo->solBuf), " %.8f %.8f %.4f %s", pos[0] * R2D, pos[1] * R2D, pos[2], "Single");
        }
        else
        {
            /*sol_t outSol = sortSol(monitor->solf, monitor->fixNum, rb);*/
            outsolMonitor(minfo->solBuf + strlen(minfo->solBuf), &sol, rb, sopt);
            if (fp && solstatic) {
                outsol(fp, &sol, rb, sopt, &time);
            }
            if (fixRate <= monitor->fixThresh) {
                sprintf(minfo->errMsg + strlen(minfo->errMsg), "fix-rate less %.2f\n",monitor->fixThresh);
            }
        }
        if(fixRate>monitor->fixThresh){
            if(monitor->epochRate_r>0.6 && monitor->epochRate_u>0.6) sprintf(minfo->solBuf + strlen(minfo->solBuf), " 1");
            else  sprintf(minfo->solBuf + strlen(minfo->solBuf), " 2");
        }else{
            if(monitor->epochRate_r>0.6 && monitor->epochRate_u>0.6) sprintf(minfo->solBuf + strlen(minfo->solBuf), " 3");
            else  sprintf(minfo->solBuf + strlen(minfo->solBuf), " 4");
        }
        sprintf(minfo->solBuf + strlen(minfo->solBuf), " %d", (int)floor(monitor->nSatUsed));
        int isMoved=0;
        // int isMoved=cal_distance(monitor->solf,monitor->fixNum);
        sprintf(minfo->solBuf + strlen(minfo->solBuf), " %d", isMoved);
    }
    free(last_obs); /* moved from stack to heap to kill a stack overflow warning */
}
/* validation of combined solutions ------------------------------------------*/
static int valcomb(const sol_t *solf, const sol_t *solb)
{
    double dr[3],var[3];
    int i;
    char tstr[32];
    
    /*trace(3,"valcomb :\n");*/
    
    /* compare forward and backward solution */
    for (i=0;i<3;i++) {
        dr[i]=solf->rr[i]-solb->rr[i];
        var[i]=(double)solf->qr[i] + (double)solb->qr[i];
    }
    for (i=0;i<3;i++) {
        if (dr[i]*dr[i]<=16.0*var[i]) continue; /* ok if in 4-sigma */
        
        time2str(solf->time,tstr,2);
        /*trace(2,"degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
              tstr+11,dr[0],dr[1],dr[2],SQRT(var[0]),SQRT(var[1]),SQRT(var[2]));*/
        return 0;
    }
    return 1;
}
/* combine forward/backward solutions and output results ---------------------*/
static void combres(const prcopt_t *popt, const solopt_t *sopt,gtime_t *outtime, monitor_t* monitor)
{
    gtime_t time={0};
    sol_t sols={{0}},sol={{0}},oldsol={{0}},newsol={{0}};
    double tt,Qf[9],Qb[9],Qs[9],rbs[3]={0},rb[3]={0},rr_f[3],rr_b[3],rr_s[3];
    int i,j,k,solstatic,num=0,pri[]={0,1,2,3,4,5,1,6};
    int nfixf = 0, nfixb = 0;
    
    /*trace(3,"combres : isolf=%d isolb=%d\n",isolf,isolb);*/
    
    solstatic=sopt->solstatic&&
              (popt->mode==PMODE_STATIC||popt->mode==PMODE_STATIC_START||popt->mode==PMODE_PPP_STATIC);
    
    for (i=0,j= monitor->isolb-1;i< monitor->isolf&&j>=0;i++,j--) {
        if (monitor->solf[i].stat == 1) nfixf++;
        if (monitor->solb[j].stat == 1) nfixb++;
        if ((tt=timediff(monitor->solf[i].time, monitor->solb[j].time))<-DTTOL) {
            sols= monitor->solf[i];
            for (k=0;k<3;k++) rbs[k]= monitor->rbf[k+i*3];
            j++;
        }
        else if (tt>DTTOL) {
            sols= monitor->solb[j];
            for (k=0;k<3;k++) rbs[k]= monitor->rbb[k+j*3];
            i--;
        }
        else if (monitor->solf[i].stat< monitor->solb[j].stat) {
            sols= monitor->solf[i];
            for (k=0;k<3;k++) rbs[k]= monitor->rbf[k+i*3];
        }
        else if (monitor->solf[i].stat> monitor->solb[j].stat) {
            sols= monitor->solb[j];
            for (k=0;k<3;k++) rbs[k]= monitor->rbb[k+j*3];
        }
        else {
            sols= monitor->solf[i];
            sols.time=timeadd(sols.time,-tt/2.0);
            
            if ((popt->mode==PMODE_KINEMA||popt->mode==PMODE_MOVEB)&&
                sols.stat==SOLQ_FIX) {
                
                /* degrade fix to float if validation failed */
                if (!valcomb(monitor->solf + i, monitor->solb + j)) {
                    char tmptime[128];
                    time2str(sols.time, tmptime, 2);
                    /*trace(5, "%sǰ�����˲��޷����\n", tmptime);*/
                    sols.stat = SOLQ_FLOAT;
                }
            }
            for (k=0;k<3;k++) {
                Qf[k+k*3]= monitor->solf[i].qr[k];
                Qb[k+k*3]= monitor->solb[j].qr[k];
            }
            Qf[1]=Qf[3]= monitor->solf[i].qr[3];
            Qf[5]=Qf[7]= monitor->solf[i].qr[4];
            Qf[2]=Qf[6]= monitor->solf[i].qr[5];
            Qb[1]=Qb[3]= monitor->solb[j].qr[3];
            Qb[5]=Qb[7]= monitor->solb[j].qr[4];
            Qb[2]=Qb[6]= monitor->solb[j].qr[5];
            
            if (popt->mode==PMODE_MOVEB) {
                for (k=0;k<3;k++) rr_f[k]= monitor->solf[i].rr[k]- monitor->rbf[k+i*3];
                for (k=0;k<3;k++) rr_b[k]= monitor->solb[j].rr[k]- monitor->rbb[k+j*3];
                if (smoother(rr_f,Qf,rr_b,Qb,3,rr_s,Qs)) continue;
                for (k=0;k<3;k++) sols.rr[k]=rbs[k]+rr_s[k];
            }
            else {
                if (smoother(monitor->solf[i].rr,Qf, monitor->solb[j].rr,Qb,3,sols.rr,Qs)) continue;
            }
            sols.qr[0]=(float)Qs[0];
            sols.qr[1]=(float)Qs[4];
            sols.qr[2]=(float)Qs[8];
            sols.qr[3]=(float)Qs[1];
            sols.qr[4]=(float)Qs[5];
            sols.qr[5]=(float)Qs[2];
            
            /* smoother for velocity solution */
            if (popt->dynamics) {
                for (k=0;k<3;k++) {
                    Qf[k+k*3]= monitor->solf[i].qv[k];
                    Qb[k+k*3]= monitor->solb[j].qv[k];
                }
                Qf[1]=Qf[3]= monitor->solf[i].qv[3];
                Qf[5]=Qf[7]= monitor->solf[i].qv[4];
                Qf[2]=Qf[6]= monitor->solf[i].qv[5];
                Qb[1]=Qb[3]= monitor->solb[j].qv[3];
                Qb[5]=Qb[7]= monitor->solb[j].qv[4];
                Qb[2]=Qb[6]= monitor->solb[j].qv[5];
                if (smoother(monitor->solf[i].rr+3,Qf, monitor->solb[j].rr+3,Qb,3,sols.rr+3,Qs)) continue;
                sols.qv[0]=(float)Qs[0];
                sols.qv[1]=(float)Qs[4];
                sols.qv[2]=(float)Qs[8];
                sols.qv[3]=(float)Qs[1];
                sols.qv[4]=(float)Qs[5];
                sols.qv[5]=(float)Qs[2];
            }
        }
        if (!solstatic) {
            gtime_t tmptime = { 0 };
            //outsol(fp, &sols, rbs, sopt, solstatic ? outtime : &tmptime);
        }
        else if (time.time==0||pri[sols.stat]<=pri[sol.stat]) {
            sol=sols;
            for (k=0;k<3;k++) rb[k]=rbs[k];
            if (time.time==0||timediff(sols.time,time)<0.0) {
                time=sols.time;
            }
        }
        if (monitor->iitm < monitor->nitm && timediff(monitor->invalidtm[monitor->iitm],sols.time)<0.0)
        {
            //outinvalidtm(fptm,sopt,invalidtm[iitm]);
            monitor->iitm++;
        }
        if (sols.eventime.time != 0)
        {
            newsol = fillsoltm(oldsol,sols,sols.eventime);
            num++;
            if (!solstatic) {
                //outsol(fptm,&newsol,rb,sopt,outtime);
            }
        }
        oldsol = sols;
    }
    /*trace(5, "˫���˲���ǰ���˲��̶��ʣ�%10.2f\n", nfixf / monitor->isolf);
    trace(5, "˫���˲��к����˲��̶��ʣ�%10.2f\n", nfixb / monitor->isolb);*/
    if (solstatic) {
        //outsol(fp,&sol,rb,sopt,outtime);
    }
}
/* read prec ephemeris, sbas data, tec grid and open rtcm --------------------*/
static void readpreceph(char **infile, int n, const prcopt_t *prcopt,
                        nav_t *nav, sbs_t *sbs, monitor_t* monitor)
{
    seph_t seph0={0};
    int i;
    char *ext;
    
    /*trace(2,"readpreceph: n=%d\n",n);*/
    
    nav->ne=nav->nemax=0;
    nav->nc=nav->ncmax=0;
    sbs->n =sbs->nmax =0;
    
    /* read precise ephemeris files */
    for (i=0;i<n;i++) {
        if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
        readsp3(infile[i],nav,0);
    }
    /* read precise clock files */
    for (i=0;i<n;i++) {
        if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
        readrnxc(infile[i],nav);
    }
    /* read sbas message files */
    for (i=0;i<n;i++) {
        if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
        sbsreadmsg(infile[i],prcopt->sbassatsel,sbs);
    }
    /* allocate sbas ephemeris */
    nav->ns=nav->nsmax=NSATSBS*2;
    if (!(nav->seph=(seph_t *)malloc(sizeof(seph_t)*nav->ns))) {
         showmsg("error : sbas ephem memory allocation");
         /*trace(1,"error : sbas ephem memory allocation");*/
         return;
    }
    for (i=0;i<nav->ns;i++) nav->seph[i]=seph0;
    
    /* set rtcm file and initialize rtcm struct */
    monitor->rtcm_file[0]= monitor->rtcm_path[0]='\0'; monitor->fp_rtcm=NULL;
    
    for (i=0;i<n;i++) {
        if ((ext=strrchr(infile[i],'.'))&&
            (!strcmp(ext,".rtcm3")||!strcmp(ext,".RTCM3"))) {
            strcpy(monitor->rtcm_file,infile[i]);
            init_rtcm(&monitor->rtcm);
            break;
        }
    }
}
/* free prec ephemeris and sbas data -----------------------------------------*/
static void freepreceph(nav_t *nav, sbs_t *sbs, monitor_t* monitor)
{
    int i;
    
    /*trace(3,"freepreceph:\n");*/
    
    free(nav->peph); nav->peph=NULL; nav->ne=nav->nemax=0;
    free(nav->pclk); nav->pclk=NULL; nav->nc=nav->ncmax=0;
    free(nav->seph); nav->seph=NULL; nav->ns=nav->nsmax=0;
    free(sbs->msgs); sbs->msgs=NULL; sbs->n =sbs->nmax =0;
    for (i=0;i<nav->nt;i++) {
        free(nav->tec[i].data);
        free(nav->tec[i].rms );
    }
    free(nav->tec ); nav->tec =NULL; nav->nt=nav->ntmax=0;
    
    if (monitor->fp_rtcm) fclose(monitor->fp_rtcm);
    free_rtcm(&monitor->rtcm);
}
/* read obs and nav data -----------------------------------------------------*/
static int readobsnav(gtime_t ts, gtime_t te, double ti, char **infile,
                      const int *index, int n, const prcopt_t *prcopt,
                      obs_t *obs, nav_t *nav, sta_t *sta, monitor_t* monitor)
{
    int i,j,ind=0,nobs=0,rcv=1;
    
    /*trace(3,"readobsnav: ts=%s n=%d\n",time_str(ts,0),n);*/
    
    //obs->data=NULL; obs->n =obs->nmax =0;
    //nav->eph =NULL; nav->n =nav->nmax =0;
    //nav->geph=NULL; nav->ng=nav->ngmax=0;
    ///* free(nav->seph); */ /* is this needed to avoid memory leak??? */
    //nav->seph=NULL; nav->ns=nav->nsmax=0;
    monitor->nepoch=0;
    
    for (i=0;i<n;i++) {
        if (checkbrk("")) return 0;
        
        if (index[i]!=ind) {
            if (obs->n>nobs) rcv++;
            ind=index[i]; nobs=obs->n; 
        }
        /* read rinex obs and nav file */
        if (readrnxt(infile[i],rcv,ts,te,ti,prcopt->rnxopt[rcv<=1?0:1],obs,nav,
                     rcv<=2?sta+rcv-1:NULL)<0) {
            checkbrk("error : insufficient memory");
            /*trace(1,"insufficient memory\n");*/
            return 0;
        }
    }
    if (obs->n<=0) {
        checkbrk("error : no obs data");
        /*trace(1,"\n");*/
        return 0;
    }
    if (nav->n<=0&&nav->ng<=0&&nav->ns<=0) {
        checkbrk("error : no nav data");
        /*trace(1,"\n");*/
        return 0;
    }
    /* sort observation data */
    monitor->nepoch=sortobs(obs);
    
    /* delete duplicated ephemeris */
    uniqnav(nav);

    /* set time span for progress display */
    if (ts.time==0||te.time==0) {
        for (i=0;   i<obs->n;i++) if (obs->data[i].rcv==1) break;
        for (j=obs->n-1;j>=0;j--) if (obs->data[j].rcv==1) break;
        if (i<j) {
            if (ts.time==0) ts=obs->data[i].time;
            if (te.time==0) te=obs->data[j].time;
            settspan(ts,te);
        }
    }
    return 1;
}

/* add observation data ------------------------------------------------------*/
static int addobsdata(obs_t* obs, const obsd_t* data)
{
    obsd_t* obs_data;

    if (obs->nmax <= obs->n) {
        if (obs->nmax <= 0) obs->nmax = NINCOBS; else obs->nmax *= 2;
        if (!(obs_data = (obsd_t*)realloc(obs->data, sizeof(obsd_t) * obs->nmax))) {
            /*trace(1, "addobsdata: malloc error n=%dx%d\n", sizeof(obsd_t), obs->nmax);*/
            free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
            return -1;
        }
        obs->data = obs_data;
    }
    obs->data[obs->n++] = *data;
    return 1;
}
/* add ephemeris to navigation data ------------------------------------------*/
static int add_eph(nav_t* nav, const eph_t* eph)
{
    eph_t* nav_eph;

    if (nav->nmax <= nav->n) {
        nav->nmax += 1024;
        if (!(nav_eph = (eph_t*)realloc(nav->eph, sizeof(eph_t) * nav->nmax))) {
            /*trace(1, "decode_eph malloc error: n=%d\n", nav->nmax);*/
            free(nav->eph); nav->eph = NULL; nav->n = nav->nmax = 0;
            return 0;
        }
        nav->eph = nav_eph;
    }
    nav->eph[nav->n++] = *eph;
    return 1;
}
static int add_geph(nav_t* nav, const geph_t* geph)
{
    geph_t* nav_geph;

    if (nav->ngmax <= nav->ng) {
        nav->ngmax += 1024;
        if (!(nav_geph = (geph_t*)realloc(nav->geph, sizeof(geph_t) * nav->ngmax))) {
            /*trace(1, "decode_geph malloc error: n=%d\n", nav->ngmax);*/
            free(nav->geph); nav->geph = NULL; nav->ng = nav->ngmax = 0;
            return 0;
        }
        nav->geph = nav_geph;
    }
    nav->geph[nav->ng++] = *geph;
    return 1;
}

/* update observation data ---------------------------------------------------*/
static void update_obs(rtcm_t* rtcm, obs_t* obs, int rcv)
{
    int i, n = 0, stat;
    char id[10], t[33];
    for (i = 0; i < rtcm->obs.n; i++) {
        /* save obs data */
        rtcm->obs.data[i].rcv = rcv;
        /*satno2id(rtcm->obs.data[i].sat, id);
        time2str(rtcm->obs.data[i].time, t, 0);
        if (rtcm->obs.data[i].LLI[0] > 0)
            printf("%s  %s\n",t,id);*/
        if ((stat = addobsdata(obs, rtcm->obs.data + i)) < 0) break;

    }

    /*sortobs(obs);*/
}
/* update ephemeris ----------------------------------------------------------*/
static void update_eph(rtcm_t* rtcm, nav_t* nav, int ephsat, int ephset)
{
    int i, sys, prn;

    /* add ephemeris to navigation data */
    sys = satsys(ephsat, &prn);
    if (prn > 0)
    {
        if (sys == SYS_GLO)
        {
            add_geph(nav, rtcm->nav.geph + prn - 1);
        }
        else
        {
            if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_CMP)
            {
                add_eph(nav, rtcm->nav.eph + ephsat + ephset * MAXSAT - 1);
            }
            else
            {
                ;
            }
        }
    }
}

/* read obs and nav data from monitor->rtcm -----------------------------------------------------*/
static int readobsnavrtcm(gtime_t ts, gtime_t te, double* ti, char** infile,
    const int* index, int n, const prcopt_t* prcopt,
    obs_t* obs, nav_t* nav, sta_t* sta, monitor_t* monitor)
{
    int i, j, ind = 0, nobs = 0, rcv = 1, ret = -1;
    char* ext;
    gtime_t st;
    st.time = 0; st.sec = 0;
    /* set monitor->rtcm file and initialize monitor->rtcm struct */
    monitor->rtcm_path[0] = '\0'; monitor->fp_rtcm = NULL;

    obs->data = NULL; obs->n = obs->nmax = 0;
    nav->eph = NULL; nav->n = nav->nmax = 0;
    nav->geph = NULL; nav->ng = nav->ngmax = 0;
    /* free(nav->seph); */ /* is this needed to avoid memory leak??? */
    nav->seph = NULL; nav->ns = nav->nsmax = 0;
    monitor->nepoch = 0;
    rtcm_t* rtcm = NULL;

    rtcm = (rtcm_t*)malloc(sizeof(rtcm_t) * 2);

    init_rtcm(rtcm);
    init_rtcm(rtcm + 1);

    for (i = 0; i < n; i++) {
        int tiarr[30], m = 0;
        /**ti = 0;*/
        /*if (checkbrk("")) return 0;*/
        rcv = i % 2 + 1;
        nobs = obs->n;
        strcpy(monitor->rtcm_path, infile[i]);
        if (strlen(monitor->rtcm_path) <= 0) continue;

        if (!(monitor->fp_rtcm = fopen(monitor->rtcm_path, "rb")))
        {
            if (monitor->fp_rtcm) {
                fclose(monitor->fp_rtcm);
                monitor->fp_rtcm = NULL;
            }
            continue;
        }

        /* init_rtcm(&monitor->rtcm);
         monitor->rtcm.time=te;*/
        rtcm[rcv - 1].time = te;

        /*trace(5, "decode file %d of %d : %s\n", i + 1, n, infile[i]);*/
        /* read monitor->rtcm file until current time */
        while (1) {/*until obs te, regardless of nav*/
            if ((ret = input_rtcm3f(rtcm + rcv - 1, monitor->fp_rtcm)) < -1) break;
            // if ((timediff(rtcm[rcv - 1].obs.data[0].time, ts) < 1E-3 && ret == 1) || ret == 0)
            if (((timediff(rtcm[rcv - 1].obs.data[0].time, ts) < 1E-3 || timediff(rtcm[rcv - 1].obs.data[0].time, te) > 1E-3) && ret == 1) || ret == 0)
                continue;
            if (ret == 1)
            {
                if (m > 0 && m <= 30) {
                    tiarr[m - 1] = (int)timediff(rtcm[rcv - 1].obs.data[0].time, st);
                }
                m++;
                st = rtcm[rcv - 1].obs.data[0].time;
                /* screen data by time */
                if (rtcm[rcv - 1].obs.n > 0 && !screent(rtcm[rcv - 1].obs.data[0].time, ts, te, *ti)) continue;
                update_obs(rtcm + rcv - 1, obs, rcv);
                if (rcv == 1) {
                    if (timediff(rtcm[rcv - 1].obs.data[0].time, monitor->t_u) == 0.0)continue;
                    if (timediff(rtcm[rcv - 1].obs.data[0].time, monitor->ts) < 0.0 || timediff(rtcm[rcv - 1].obs.data[0].time, monitor->te) > 0.0) continue;
                    monitor->nepoch_u++;
                    monitor->t_u = rtcm[rcv - 1].obs.data[0].time;
                }
                if (rcv == 2) {
                    if (timediff(rtcm[rcv - 1].obs.data[0].time, monitor->t_r) == 0.0)continue;
                    if (timediff(rtcm[rcv - 1].obs.data[0].time, monitor->ts) < 0.0 || timediff(rtcm[rcv - 1].obs.data[0].time, monitor->te) > 0.0) continue;
                    monitor->nepoch_r++;
                    monitor->t_r = rtcm[rcv - 1].obs.data[0].time;
                }
                if (timediff(rtcm[rcv - 1].obs.data[0].time, te) >= -1E-3)
                    break;
            }
            else if (ret == 2)
            {
                if (satsys(rtcm[rcv - 1].ephsat, NULL) == SYS_GAL)
                    setseleph(SYS_GAL, rtcm[rcv - 1].ephset);
                update_eph(rtcm + rcv - 1, nav, rtcm[rcv - 1].ephsat, rtcm[rcv - 1].ephset);
                if (rcv == 1) {
                    monitor->nnav_u++;
                }
                if (rcv == 2) {
                    monitor->nnav_r++;
                }
            }

        }
        if (m > 30) {
            /*if (abs(*ti - 0) < 1E-3 || *ti < findMode(tiarr, 60)) {
                *ti = findMode(tiarr, 60);
            }*/
            if (rcv == 1 && abs(monitor->ti_u - 0) < 1E-3) {
                monitor->ti_u = findMode(tiarr, 30);
            }
            if (rcv == 2 && abs(monitor->ti_r - 0) < 1E-3) {
                monitor->ti_r = findMode(tiarr, 30);
            }
        }
        fclose(monitor->fp_rtcm);
        monitor->fp_rtcm = NULL;
    }
    free_rtcm(rtcm + 1);
    free_rtcm(rtcm);
    free(rtcm);
    rtcm = NULL;


    if (obs->n <= 0 || nav->n <= 0) {
        if (obs->n <= 0) {
            monitor->obsflag = 63;
        }
        if (nav->n <= 0) {
            monitor->navflag = 7;
        }
    }

    if (obs->n <= 0)
    {
        return 0;
    }
    /**ti = findMode(tiarr, 60);*/

    /* sort observation data */
    monitor->nepoch = sortobs(obs);
    if (obs->data != NULL)
    {
        if (timediff(monitor->t_u, monitor->t_p) <= 1E-3)
        {
            monitor->obsflag |= 1;
        }
        if (timediff(monitor->t_r, monitor->t_p) <= 1E-3 && timediff(monitor->t_u, monitor->t_r) > prcopt->maxtdiff)
        {
            monitor->obsflag |= 2;
        }
        if (monitor->nepoch_u == 0)
        {
            monitor->obsflag |= 16;
        }
        if (monitor->nepoch_r == 0)
        {
            monitor->obsflag |= 32;
        }

        if (monitor->nnav_u == 0)
        {
            monitor->navflag |= 1;
        }
        if (monitor->nnav_r == 0)
        {
            monitor->navflag |= 2;
        }
        if (monitor->obsflag > 0)
            return 0;
    }

    /* delete duplicated ephemeris */
    uniqnav(nav);

    if (nav->n == 0)
    {
        monitor->navflag |= 4;
    }

    /* set time span for progress display */
    if (ts.time == 0 || te.time == 0) {
        for (i = 0; i < obs->n; i++) if (obs->data[i].rcv == 1) break;
        for (j = obs->n - 1; j >= 0; j--) if (obs->data[j].rcv == 1) break;
        if (i < j) {
            if (ts.time == 0) ts = obs->data[i].time;
            if (te.time == 0) te = obs->data[j].time;
            settspan(ts, te);
        }
    }
    return 1;
}

/* free obs and nav data -----------------------------------------------------*/
static void freeobsnav(obs_t *obs, nav_t *nav)
{
    /*trace(3,"freeobsnav:\n");*/
    
    free(obs->data); obs->data=NULL; obs->n =obs->nmax =0;
    free(nav->eph ); nav->eph =NULL; nav->n =nav->nmax =0;
    free(nav->geph); nav->geph=NULL; nav->ng=nav->ngmax=0;
    free(nav->seph); nav->seph=NULL; nav->ns=nav->nsmax=0;
}
/* average of single position ------------------------------------------------*/
static int avepos(double *ra, int rcv, const obs_t *obs, const nav_t *nav,
                  const prcopt_t *opt)
{
    obsd_t data[MAXOBS];
    gtime_t ts={0};
    sol_t sol={{0}};
    int i,j,n=0,m,iobs;
    char msg[128];
    double deltax[3];
    double pre_rb[3] = { 0 };

    /*trace(3,"avepos: rcv=%d obs.n=%d\n",rcv,obs->n);*/
    
    for (i=0;i<3;i++) ra[i]=0.0;
    
    for (iobs=0;(m=nextobsf(obs,&iobs,rcv))>0;iobs+=m) {
        
        for (i=j=0;i<m&&i<MAXOBS;i++) {
            data[j]=obs->data[iobs+i];
            if ((satsys(data[j].sat,NULL)&opt->navsys)&&
                opt->exsats[data[j].sat-1]!=1) j++;
        }
        if (j<=0||!screent(data[0].time,ts,ts,1.0)) continue; /* only 1 hz */
        
        if (!pntpos(NULL, data,j,nav,opt,NULL,&sol,NULL,NULL,msg)) continue;
        if (norm(ra, 3) < 1E-3) {
            for (i = 1; i < 3; i++) ra[i] = sol.rr[i];
        }
        if (norm(pre_rb, 3) < 1E-2) {
            for (i = 0; i < 3; i++) pre_rb[i] = sol.rr[i];
            continue;
        }
        if (fabs(pre_rb[0] - sol.rr[0]) < 5 && fabs(pre_rb[1] - sol.rr[1]) < 5 && fabs(pre_rb[2] - sol.rr[2]) < 5) {
            for (i = 0; i < 3; i++) ra[i] = sol.rr[i];
            break;
        }
        else
        {
            for (i = 0; i < 3; i++) pre_rb[i] = sol.rr[i];
        }
    }
    return 1;
}
/* station position from file ------------------------------------------------*/
static int getstapos(const char *file, char *name, double *r)
{
    FILE *fp;
    char buff[256],sname[256],*p,*q;
    double pos[3];
    
    /*trace(3,"getstapos: file=%s name=%s\n",file,name);*/
    
    if (!(fp=fopen(file,"r"))) {
        /*trace(1,"station position file open error: %s\n",file);*/
        return 0;
    }
    while (fgets(buff,sizeof(buff),fp)) {
        if ((p=strchr(buff,'%'))) *p='\0';
        
        if (sscanf(buff,"%lf %lf %lf %s",pos,pos+1,pos+2,sname)<4) continue;
        
        for (p=sname,q=name;*p&&*q;p++,q++) {
            if (toupper((int)*p)!=toupper((int)*q)) break;
        }
        if (!*p) {
            pos[0]*=D2R;
            pos[1]*=D2R;
            pos2ecef(pos,r);
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
    /*trace(1,"no station position: %s %s\n",name,file);*/
    return 0;
}
/* antenna phase center position ---------------------------------------------*/
static int antpos(prcopt_t *opt, int rcvno, const obs_t *obs, const nav_t *nav,
                  const sta_t *sta, const char *posfile, monitor_t* monitor)
{
    double *rr=rcvno==1?opt->ru:opt->rb,del[3],pos[3],dr[3]={0};
    int i,postype=rcvno==1?opt->rovpos:opt->refpos;
    char *name;
    
    /*trace(3,"antpos  : rcvno=%d\n",rcvno);*/
    
    if (postype==POSOPT_SINGLE) { /* average of single position */
        if (!avepos(rr,rcvno,obs,nav,opt)) {
            showmsg("error : station pos computation");
            return 0;
        }
    }
    else if (postype==POSOPT_FILE) { /* read from position file */
        name= monitor->stas[rcvno==1?0:1].name;
        if (!getstapos(posfile,name,rr)) {
            showmsg("error : no position of %s in %s",name,posfile);
            return 0;
        }
    }
    else if (postype==POSOPT_RINEX) { /* get from rinex header */
        if (norm(monitor->stas[rcvno==1?0:1].pos,3)<=0.0) {
            showmsg("error : no position in rinex header");
            /*trace(1,"no position in rinex header\n");*/
            return 0;
        }
        /* add antenna delta unless already done in antpcv() */
        if (!strcmp(opt->anttype[rcvno],"*")) {
            if (monitor->stas[rcvno==1?0:1].deltype==0) { /* enu */
                for (i=0;i<3;i++) del[i]= monitor->stas[rcvno==1?0:1].del[i];
                del[2]+= monitor->stas[rcvno==1?0:1].hgt;
                ecef2pos(monitor->stas[rcvno==1?0:1].pos,pos);
                enu2ecef(pos,del,dr);
            }  else { /* xyz */
                for (i=0;i<3;i++) dr[i]= monitor->stas[rcvno==1?0:1].del[i];
            }
        }
        for (i=0;i<3;i++) rr[i]= monitor->stas[rcvno==1?0:1].pos[i]+dr[i];
    }
    return 1;
}
/* open procssing session ----------------------------------------------------*/
static int openses(const prcopt_t *popt, const solopt_t *sopt,
                   const filopt_t *fopt, nav_t *nav, pcvs_t *pcvs, pcvs_t *pcvr)
{
    /*trace(3,"openses :\n");*/
    
    /* read satellite antenna parameters */
    if (*fopt->satantp&&!(readpcv(fopt->satantp,pcvs))) {
        showmsg("error : no sat ant pcv in %s",fopt->satantp);
        /*trace(1,"sat antenna pcv read error: %s\n",fopt->satantp);*/
        return 0;
    }
    /* read receiver antenna parameters */
    if (*fopt->rcvantp&&!(readpcv(fopt->rcvantp,pcvr))) {
        showmsg("error : no rec ant pcv in %s",fopt->rcvantp);
        /*trace(1,"rec antenna pcv read error: %s\n",fopt->rcvantp);*/
        return 0;
    }
    /* open geoid data */
    if (sopt->geoid>0&&*fopt->geoid) {
        if (!opengeoid(sopt->geoid,fopt->geoid)) {
            showmsg("error : no geoid data %s",fopt->geoid);
            /*trace(2,"no geoid data %s\n",fopt->geoid);*/
        }
    }
    return 1;
}
/* close procssing session ---------------------------------------------------*/
static void closeses(nav_t *nav, pcvs_t *pcvs, pcvs_t *pcvr)
{
    /*trace(3,"closeses:\n");*/
    
    /* free antenna parameters */
    free(pcvs->pcv); pcvs->pcv=NULL; pcvs->n=pcvs->nmax=0;
    free(pcvr->pcv); pcvr->pcv=NULL; pcvr->n=pcvr->nmax=0;
    
    /* close geoid data */
    closegeoid();
    
    /* free erp data */
    free(nav->erp.data); nav->erp.data=NULL; nav->erp.n=nav->erp.nmax=0;
    
    /* close solution statistics and debug trace */
    rtkclosestat();
}
/* set antenna parameters ----------------------------------------------------*/
static void setpcv(gtime_t time, prcopt_t *popt, nav_t *nav, const pcvs_t *pcvs,
                   const pcvs_t *pcvr, const sta_t *sta, monitor_t* monitor)
{
    pcv_t *pcv,pcv0={0};
    double pos[3],del[3];
    int i,j,mode=PMODE_DGPS<=popt->mode&&popt->mode<=PMODE_FIXED;
    char id[64];
    
    /* set satellite antenna parameters */
    for (i=0;i<MAXSAT;i++) {
        nav->pcvs[i]=pcv0;
        if (!(satsys(i+1,NULL)&popt->navsys)) continue;
        if (!(pcv=searchpcv(i+1,"",time,pcvs))) {
            satno2id(i+1,id);
            /*trace(4,"no satellite antenna pcv: %s\n",id);*/
            continue;
        }
        nav->pcvs[i]=*pcv;
    }
    for (i=0;i<(mode?2:1);i++) {
        popt->pcvr[i]=pcv0;
        if (!strcmp(popt->anttype[i],"*")) { /* set by station parameters */
            strcpy(popt->anttype[i],sta[i].antdes);
            if (sta[i].deltype==1) { /* xyz */
                if (norm(sta[i].pos,3)>0.0) {
                    ecef2pos(sta[i].pos,pos);
                    ecef2enu(pos,sta[i].del,del);
                    for (j=0;j<3;j++) popt->antdel[i][j]=del[j];
                }
            }
            else { /* enu */
                for (j=0;j<3;j++) popt->antdel[i][j]= monitor->stas[i].del[j];
            }
        }
        if (!(pcv=searchpcv(0,popt->anttype[i],time,pcvr))) {
            /*trace(2,"no receiver antenna pcv: %s\n",popt->anttype[i]);*/
            *popt->anttype[i]='\0';
            continue;
        }
        strcpy(popt->anttype[i],pcv->type);
        popt->pcvr[i]=*pcv;
    }
}
/* read ocean tide loading parameters ----------------------------------------*/
static void readotl(prcopt_t *popt, const char *file, const sta_t *sta)
{
    int i,mode=PMODE_DGPS<=popt->mode&&popt->mode<=PMODE_FIXED;
    
    for (i=0;i<(mode?2:1);i++) {
        readblq(file,sta[i].name,popt->odisp[i]);
    }
}
/* write header to output file -----------------------------------------------*/
static int outhead(const char *outfile, char **infile, int n,
                   const prcopt_t *popt, const solopt_t *sopt, monitor_t* monitor)
{
    FILE* fp = NULL;
    
    /*trace(3,"outhead: outfile=%s n=%d\n",outfile,n);*/
    
    if (*outfile) {
        //createdir(outfile);
        
        if (!(fp=fopen(outfile,"wb"))) {
            showmsg("error : open output file %s",outfile);
            return 0;
        }
        /* output header */
        outheader(fp, infile, n, popt, sopt, monitor);
        fclose(fp);
    }

    
    return 1;
}
/* open output file for append -----------------------------------------------*/
static FILE *openfile(const char *outfile)
{
    /*trace(3,"openfile: outfile=%s\n",outfile);*/
    
    return !*outfile?stdout:fopen(outfile,"ab");
}
/* Name time marks file ------------------------------------------------------*/
static void namefiletm(char *outfiletm, const char *outfile)
{
    int i;

    for (i=(int)strlen(outfile);i>0;i--) {
        if (outfile[i] == '.') {
            break;
        }
    }
    /* if no file extension, then name time marks file as name of outfile + _events.pos */
    if (i == 0) {
        i = (int)strlen(outfile);
    }
    strncpy(outfiletm, outfile, i);
    strcat(outfiletm, "_events.pos");
}
/* execute processing session ------------------------------------------------*/
static int execses(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                   const solopt_t *sopt, const filopt_t *fopt, int flag,
                   char **infile, const int *index, int n, char* outfile, mInfo* minfo, monitor_t* monitor)
{
    FILE* fp_table = NULL;
    FILE* fp; FILE* fp_trace = NULL;
    rtk_t *rtk_ptr = (rtk_t *)malloc(sizeof(rtk_t)); /* moved from stack to heap to avoid stack overflow warning */
    prcopt_t popt_=*popt;
    solopt_t tmsopt = *sopt;
    char tracefile[1024],tablefile[1024],statfile[1024],path[1024],*ext,outfiletm[1024]={0};
    int i,j,k;
    int glo_fcnzero[] = {
    1,-4,5,6,1,-4,5,6,-2,-7,
    0,-1,-2,-7,0,-1,4,-3,3,2,
    4,-3,3,2
    };

    // /* open debug trace */
    // if (flag && sopt->trace > 0 && *outfile) {
    //     strcpy(tracefile, outfile);
    //     strcat(tracefile, ".res");
    //     strcpy(tablefile, outfile);
    //     strcat(tablefile, ".table");
    //     if (!(fp_trace = fopen(tracefile, "w"))) return 0;
    //     if (!(fp_table = fopen(tablefile, "w"))) return 0;
    //     traceopen();
    //     tableopen();
    // }
    /* read obs and nav data */
    if (popt_.format == 2)
    {
        if (!readobsnav(ts, te, ti, infile, index, n, &popt_, &monitor->obss, &monitor->navs, monitor->stas, monitor)) {
            /* free obs and nav data */
            freeobsnav(&monitor->obss, &monitor->navs);
            sprintf(minfo->errMsg + strlen(minfo->errMsg), "read obs error\n");
            free(rtk_ptr);
            traceclose(fp_trace);
            tableclose(fp_table);
            return 0;
        }
    }
    else
    {
        if (!readobsnavrtcm(ts, te, &ti, infile, index, n, &popt_, &monitor->obss, &monitor->navs, monitor->stas, monitor))
        {
            freeobsnav(&monitor->obss, &monitor->navs);
            sprintf(minfo->errMsg + strlen(minfo->errMsg), "read rtcm error\n");
            free(rtk_ptr);
            traceclose(fp_trace);
            tableclose(fp_table);
            return 0;
        }
        ext = strrchr(infile[n - 1], '.');
        if (ext && (strstr(ext, ".sp3") || strstr(ext, ".SP3") || strstr(ext, ".eph") || strstr(ext, ".EPH"))) {
            readpreceph(infile + n - 1, 1, &popt_, &monitor->navs, &monitor->sbss, monitor);
            if (monitor->navs.ne > 10) popt_.sateph = EPHOPT_PREC;
        }     
        else if ((strstr(ext, ".rnx") || strstr(ext, ".nav"))) {
            readobsnav(ts, te, ti, infile + n - 1, index, 1, &popt_, &monitor->obss, &monitor->navs, monitor->stas, monitor);
        }
        if (abs((int)n % 2) < 1E-3) {
            ext = strrchr(infile[n - 2], '.');
            if (ext && (strstr(ext, ".sp3") || strstr(ext, ".SP3") || strstr(ext, ".eph") || strstr(ext, ".EPH"))) {
                readpreceph(infile + n - 1, 1, &popt_, &monitor->navs, &monitor->sbss, monitor);
                if (monitor->navs.ne > 10) popt_.sateph = EPHOPT_PREC;
            }
            else if ((strstr(ext, ".rnx") || strstr(ext, ".nav"))) {
                readobsnav(ts, te, ti, infile + n - 1, index, 1, &popt_, &monitor->obss, &monitor->navs, monitor->stas, monitor);
            }
        }
        for (int i = 0; i < sizeof(glo_fcnzero); i++)
        {
            monitor->navs.glo_fcn[i] = glo_fcnzero[i] + 8;
        }
        if (popt_.sateph == EPHOPT_PREC) {
            monitor->navs.n = 0;
            monitor->navs.nmax = 0;
            monitor->navs.ng = 0;
            monitor->navs.ngmax = 0;
            monitor->navs.ns = 0;
            monitor->navs.nsmax = 0;
            memset(&monitor->navs.eph, 0, sizeof(&monitor->navs.eph));
            memset(&monitor->navs.eph, 0, sizeof(&monitor->navs.geph));
            memset(&monitor->navs.eph, 0, sizeof(&monitor->navs.seph));
        }
    }
    double expEpoch_u = timediff(te, ts) / monitor->ti_u;
    double expEpoch_r = timediff(te, ts) / monitor->ti_r;
    trace(NULL, 2, "expEpoch=%10.4f,%10.4f,nepoch_r=%d,nepoch_u=%d\n", expEpoch_u, expEpoch_r, monitor->nepoch_r, monitor->nepoch_u);
    if ((monitor->epochRate_r = (monitor->nepoch_r / expEpoch_r) > 1 ? 1 : monitor->nepoch_r / expEpoch_r) < 0.6) {
        sprintf(minfo->errMsg + strlen(minfo->errMsg), "expEpoch=%10.4f,nepoch_r=%d,nepoch_u=%d\n", expEpoch_r, monitor->nepoch_r, monitor->nepoch_u);
        sprintf(minfo->errMsg + strlen(minfo->errMsg), "reference station epoch rate less than 60%\n");
    }
    if ((monitor->epochRate_u = (monitor->nepoch_u / expEpoch_u) > 1 ? 1 : monitor->nepoch_u / expEpoch_u) < 0.6) {
        sprintf(minfo->errMsg + strlen(minfo->errMsg), "expEpoch=%10.4f,nepoch_r=%d,nepoch_u=%d\n", expEpoch_u, monitor->nepoch_r, monitor->nepoch_u);
        sprintf(minfo->errMsg + strlen(minfo->errMsg), "monitoring station epoch rate less than 60%\n");
    }
    /* read dcb parameters */
    for (i=0;i<3;i++) {
        for (j=0;j<MAXSAT;j++) monitor->navs.cbias[j][i]=0;
        for (j=0;j<MAXRCV;j++) for (k=0;k<2;k++) monitor->navs.rbias[j][k][i]=0;
    }
    /* set antenna parameters */
    if (popt_.mode!=PMODE_SINGLE) {
        setpcv(monitor->obss.n>0? monitor->obss.data[0].time:timeget(),&popt_,&monitor->navs,&monitor->pcvss,&monitor->pcvsr,
            monitor->stas, monitor);
    }
    if (norm(popt_.rb,3)<1&&PMODE_DGPS<=popt_.mode&&popt_.mode<=PMODE_STATIC_START) {
        if (!antpos(&popt_,2,&monitor->obss,&monitor->navs, monitor->stas,fopt->stapos, monitor)) {
            sprintf(minfo->errMsg + strlen(minfo->errMsg), "base average spp error\n");
            freeobsnav(&monitor->obss,&monitor->navs);
            free(rtk_ptr);
            traceclose(fp_trace);
            tableclose(fp_table);
            return 0;
        }
    }
    trace(NULL, 2, "base average spp = %.4f\t%.4f\t%.4f\n", popt_.rb[0], popt_.rb[1], popt_.rb[2]);
    /* write header to output file */
    if (flag && !outhead(outfile, infile, n, &popt_, sopt,monitor)) {
        freeobsnav(&monitor->obss, &monitor->navs);
        free(rtk_ptr);
        traceclose(fp_trace);
        tableclose(fp_table);
        return 0;
    }
    monitor->iobsu= monitor->iobsr= monitor->isbs= monitor->revs= monitor->aborts=0;

    /* add in 2023/03/22 10:21 by Guo Zihuai */
    monitor->dpos = (double*)malloc(sizeof(double) * monitor->nepoch * 3);
    if (popt_.mode==PMODE_SINGLE||popt_.soltype==0) {
        monitor->solf = (sol_t*)malloc(sizeof(sol_t) * monitor->nepoch);
        if (*outfile && (fp = openfile(outfile))) {
            procpos(fp, fp_trace, fp_table, &popt_, sopt, rtk_ptr, 0, minfo, monitor); /* forward */
            fclose(fp);
        }
        else
        {
            procpos(NULL, NULL, NULL, &popt_, sopt, rtk_ptr, 0, minfo, monitor); /* forward */
        }
        free(monitor->solf); monitor->solf = NULL;
    }
    else if (popt_.soltype==1) {
        monitor->revs=1; monitor->iobsu= monitor->iobsr= monitor->obss.n-1; monitor->isbs= monitor->sbss.n-1;
        if (*outfile && (fp = openfile(outfile))) {
            procpos(fp, fp_trace, fp_table, &popt_, sopt, rtk_ptr, 0, minfo, monitor); /* forward */
            fclose(fp);
        }
        else
        {
            procpos(NULL, NULL, NULL, &popt_, sopt, rtk_ptr, 0, minfo, monitor); /* forward */
        }
    }
    else { /* combined */
        monitor->solf=(sol_t *)malloc(sizeof(sol_t)* monitor->nepoch);
        monitor->solb=(sol_t *)malloc(sizeof(sol_t)* monitor->nepoch);
        monitor->rbf=(double *)malloc(sizeof(double)* monitor->nepoch*3);
        monitor->rbb=(double *)malloc(sizeof(double)* monitor->nepoch*3);
        
        if (monitor->solf&& monitor->solb) {
            monitor->isolf= monitor->isolb=0;
            procpos(NULL, NULL, NULL, &popt_, sopt, rtk_ptr, 1, minfo, monitor); /* forward */
            monitor->revs=1; monitor->iobsu= monitor->iobsr= monitor->obss.n-1; monitor->isbs= monitor->sbss.n-1;
            procpos(NULL, NULL, NULL, &popt_,sopt,rtk_ptr,1, minfo, monitor); /* backward */
            
            combres(&popt_,sopt,&te, monitor);
        }
        else showmsg("error : memory allocation");
        free(monitor->solf); monitor->solf = NULL;
        free(monitor->solb); monitor->solb = NULL;
        free(monitor->rbf); monitor->rbf = NULL;
        free(monitor->rbb); monitor->rbb = NULL;
    }
    trace(NULL, 2, "%s\n", minfo->solBuf);
    trace(NULL, 2, "%s\n", minfo->errMsg);
    tracemat(fp_trace, 5, rtk_ptr->hasL, NFREQ, MAXSAT, 4, 0);
    tracemat(fp_trace, 5, rtk_ptr->hasP, NFREQ, MAXSAT, 4, 0);
    /* free rtk, obs and nav data */
    rtkfree(rtk_ptr);
    free(rtk_ptr);
    free(monitor->dpos); monitor->dpos = NULL;
    freeobsnav(&monitor->obss,&monitor->navs);
    traceclose(fp_trace);
    tableclose(fp_table);
    return monitor->aborts?1:0;
}
/* execute processing session for each rover ---------------------------------*/
static int execses_r(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                     const solopt_t *sopt, const filopt_t *fopt, int flag,
                     char **infile, const int *index, int n, char* outfile,
                     const char *rov, mInfo* minfo, monitor_t* monitor)
{
    gtime_t t0={0};
    int i,stat=0;
    char *ifile[MAXINFILE],*rov_,*p,*q,s[64]="";
    
    /*trace(3,"execses_r: n=%d outfile=%s\n",n,outfile);*/
    
    for (i=0;i<n;i++) if (strstr(infile[i],"%r")) break;
    stat = execses(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile, minfo, monitor);
    return stat;
}
/* execute processing session for each base station --------------------------*/
static int execses_b(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                     const solopt_t *sopt, const filopt_t *fopt, int flag,
                     char **infile, const int *index, int n, char* outfile,
                     const char *rov, const char *base, mInfo* minfo, monitor_t* monitor)
{
    gtime_t t0={0};
    int i,stat=0;
    char *ifile[MAXINFILE],*base_,*p,*q,s[64];
    
    for (i=0;i<n;i++) if (strstr(infile[i],"%b")) break;
    
    stat=execses_r(ts,te,ti,popt,sopt,fopt,flag,infile,index,n,outfile,rov, minfo, monitor);
    /* free prec ephemeris and sbas data */
    freepreceph(&monitor->navs,&monitor->sbss, monitor);
    
    return stat;
}
/* post-processing positioning -------------------------------------------------
* post-processing positioning
* args   : gtime_t ts       I   processing start time (ts.time==0: no limit)
*        : gtime_t te       I   processing end time   (te.time==0: no limit)
*          double ti        I   processing interval  (s) (0:all)
*          double tu        I   processing unit time (s) (0:all)
*          prcopt_t *popt   I   processing options
*          solopt_t *sopt   I   solution options
*          filopt_t *fopt   I   file options
*          char   **infile  I   input files (see below)
*          int    n         I   number of input files
*          char   *outfile  I   output file ("":stdout, see below)
*          char   *rov      I   rover id list        (separated by " ")
*          char   *base     I   base station id list (separated by " ")
* return : status (0:ok,0>:error,1:aborted)
* notes  : input files should contain observation data, navigation data, precise 
*          ephemeris/clock (optional), sbas log file (optional), ssr message
*          log file (optional) and tec grid file (optional). only the first 
*          observation data file in the input files is recognized as the rover
*          data.
*
*          the type of an input file is recognized by the file extension as ]
*          follows:
*              .sp3,.SP3,.eph*,.EPH*: precise ephemeris (sp3c)
*              .sbs,.SBS,.ems,.EMS  : sbas message log files (rtklib or ems)
*              .rtcm3,.RTCM3        : ssr message log files (rtcm3)
*              .*i,.*I              : tec grid files (ionex)
*              others               : rinex obs, nav, gnav, hnav, qnav or clock
*
*          inputs files can include wild-cards (*). if an file includes
*          wild-cards, the wild-card expanded multiple files are used.
*
*          inputs files can include keywords. if an file includes keywords,
*          the keywords are replaced by date, time, rover id and base station
*          id and multiple session analyses run. refer reppath() for the
*          keywords.
*
*          the output file can also include keywords. if the output file does
*          not include keywords. the results of all multiple session analyses
*          are output to a single output file.
*
*          ssr corrections are valid only for forward estimation.
*-----------------------------------------------------------------------------*/
extern int postpos(gtime_t ts, gtime_t te, double ti, double tu,
                   const prcopt_t *popt, const solopt_t *sopt,
                   const filopt_t *fopt, char **infile, int n, char* outfile,
                   const char *rov, const char *base, mInfo* minfo, monitor_t* monitor)
{
    gtime_t tts,tte,ttte;
    double tunit,tss;
    int i,j,k,nf,stat=0,week,flag=1,index[MAXINFILE]={0};
    char *ifile[MAXINFILE],*ext;
    
    /*trace(3,"postpos : ti=%.0f tu=%.0f n=%d outfile=%s\n",ti,tu,n,outfile);*/
    
    /* open processing session */

    if (!openses(popt, sopt, fopt, &monitor->navs, &monitor->pcvss, &monitor->pcvsr)) return -1;
    for (i = 0; i < n; i++) index[i] = i;
    /* execute processing session */
    stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, infile, index, n, outfile, rov,
        base, minfo, monitor);
    /* close processing session */
    closeses(&monitor->navs,&monitor->pcvss,&monitor->pcvsr);
    
    return stat;
}
