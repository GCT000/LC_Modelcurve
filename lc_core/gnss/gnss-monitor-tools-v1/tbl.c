#include "./include/rtklib.h"

#define SQR(x)      ((x)*(x))
#define THRES       0.05 
/*Find the max value in a[b],return the index*/
static int findMax(double* a, int b)
{
	int i, imax = -1;
	double vmax = 0.0;
	if (b <= 0)return -9999;
	for (i = 0; i < b; i++)
	{
		if (a[i] == 0)
			continue;
		if (vmax == 0.0 || a[i] > vmax)
		{
			vmax = a[i];
			imax = i;
		}
	}
	return imax;
}

/*Find the min value in a[b],return the index*/
static int findMin(double* a, int b)
{
	int i, imin = -1;
	double vmin = 0.0;
	if (b <= 0)return -9999;
	for (i = 0; i < b; i++)
	{
		if (a[i] == 0)
			continue;
		if (vmin == 0.0 || a[i] < vmin)
		{
			vmin = a[i];
			imin = i;
		}
	}
	return imin;
}

static void isZupt(rtk_t* rtk)
{
	int i, j;
	double a[NSYS][3], pos[3], e[3], de[NSYS], dn[NSYS], du[NSYS], dpos[3];
	char t_str[64] = "";
	time2str(rtk->sol.time, t_str, 2);
	for (i = 0; i < NSYS; i++)
	{
		de[i] = dn[i] = du[i] = 9999.0;
		if (rtk->dposFlag[0][i][0] == 1 && rtk->dposFlag[1][i][0] == 1)
		{
			for (j = 0; j < 3; j++)
			{
				a[i][j] = rtk->dpos[0][i][j] - rtk->dpos[1][i][j];
			}
			ecef2pos(rtk->sol.rr, pos);
			ecef2enu(pos, a[i], e);
			/*matcpy(a[i], e, 3, 1);*/
			/*trace(3, "BaseLine 1-2: %2d %s %9.4f %9.4f %9.4f\n", (int)pow(2, i < 1 ? i : (i < 3 ? i + 1 : i + 2)), t_str, a[i][0], a[i][1], a[i][2]);*/
			de[i] = fabs(a[i][0]); dn[i] = fabs(a[i][1]); du[i] = fabs(a[i][2]);
		}
	}
	j = findMin(de, NSYS);
	dpos[0] = de[j];
	j = findMin(dn, NSYS);
	dpos[1] = dn[j];
	j = findMin(du, NSYS);
	dpos[2] = du[j];

	if (fabs(dpos[0] - 9999.0) < 1E-3 || fabs(dpos[0] - 9999.0) < 1E-3 || fabs(dpos[0] - 9999.0) < 1E-3)
	{
		for (i = 0; i < 3; i++)
		{
			rtk->dposFlag[0][4][i] = rtk->dposFlag[1][4][i] = 0;
		}
	}
	else
	{
		rtk->dpos[0][4][0] = dpos[0];
		rtk->dpos[0][4][1] = dpos[1];
		rtk->dpos[0][4][2] = dpos[2];
		/*trace(3, "BaseLine final: %2d %s %9.4f %9.4f %9.4f %9.4f\n", (int)pow(2, i < 1 ? i : (i < 3 ? i + 1 : i + 2)), t_str, dpos[0], dpos[1], dpos[2], norm(dpos, 3));*/
		for (i = 0; i < 3; i++)
		{
			if (dpos[i] < 0.015)
			{
				rtk->dposFlag[0][4][i] = 1;
			}
			else if (dpos[i] < 0.02)
			{
				rtk->dposFlag[0][4][i] = 2;
			}
			else if (dpos[i] < 0.03)
			{
				rtk->dposFlag[0][4][i] = 3;
			}
			else if (dpos[i] < 0.04)
			{
				rtk->dposFlag[0][4][i] = 4;
			}
			else if (dpos[i] < 0.05)
			{
				rtk->dposFlag[0][4][i] = 5;
			}
			else if (dpos[i] < 0.1)
			{
				rtk->dposFlag[0][4][i] = 6;
			}
			else
			{
				rtk->dposFlag[0][4][i] = 7;
			}

		}


	}


}

static int vPhaseRes(rtk_t* rtk, int nsat0, double* H, double* v, double* var, int* sats)
{
	int i, j, nv = 0, nx = 4;

	for (i = 0; i < nsat0; i++)
	{
		if (!rtk->ssat[sats[i] - 1].vsphase[0])
		{

			for (j = 0; j < NFREQ; j++)
			{
				rtk->ssat[sats[i] - 1].slip[j] |= 1;
			}
			continue;
		}
		for (j = 0; j < nx; j++)
		{
			H[nv * nx + j] = j < 3 ? H[i * nx + j] : 1.0;
		}
		sats[nv] = sats[i];
		v[nv] = v[i];
		var[nv] = var[i];
		nv++;
	}
	return nv;
}
#define MAX(x,y)    ((x)>=(y)?(x):(y))

/*
* args   : obsd_t * obs      I   observation data
*		   int      n        I   number of observation data
*		   nav_t*   nav      I   navigation data
*		   prcopt_t* opt     I   processing options
*/
extern int getTimeBaseLine(FILE *fp_trace, const obsd_t* obs, int n, int sys0, const nav_t* nav, prcopt_t* opt, rtk_t* rtk, sol_t* sol, double* rs, double* dts, int* svh)
{
	int i, j, k, niter = 0, sat, sys, hasL = 0, imin, imax, nv = 0, nx = 4, nsat, nsat0, sats[MAXOBS] = { 0 }, syss[MAXOBS] = { 0 }, rcv = obs->rcv - 1;
	double previousLMeas, currentLMeas, previousSatPos[3], previousSatVel[3], previousRcvPos[3], currentSatPos[3], currentSatVel[3], currentRcvPos[3];
	double dt, freq[NFREQ];
	double sig, * v, * v_t, * H, * var;
	double r1, r2, e1[3], e2[3], e2s2, e1s1, e2r1, e1r1;
	double x[4] = { 0.0 }, dx[4], Q[16] = { 0.0 }, Qv[16], Qx[6] = { 0.0 };
	int ind = 0;
	char t_str[64] = "";
	time2str(obs->time, t_str, 2);
	if (sys0 == SYS_GPS) ind = 0;
	if (sys0 == SYS_GLO) ind = 1;
	if (sys0 == SYS_GAL) ind = 2;
	if (sys0 == SYS_CMP) ind = 3;
	v = zeros(n, 1);
	v_t = zeros(n, 1);
	H = zeros(n, nx); /*3 receiver velocity, GNSS sys receiver clock velocity*/
	var = zeros(n, 1);
	for (i = 0; i < n; i++)
	{
		sat = obs[i].sat;
		sys = satsys(sat, NULL);
		if (sys != sys0) continue;
		if (!rtk->ssat[sat - 1].vs) continue;
		hasL = 0;
		for (j = 0; j < NFREQ && j < opt->nf; j++)
		{
			rtk->ssat[sat - 1].vsphase[j] = 1;
			freq[j] = sat2freq(sat, obs[i].code[j], nav);
			if (obs[i].L[j] < 1E-3 || freq[j] < 1E-3 || svh[i] < 0) continue;
			if (timediff(obs->time, rtk->ssat[sat - 1].tpt[rcv][j]) == 0) continue;
			/*if (obs[i].SNR[j]*SNR_UNIT<40) continue;
			if(sin(rtk->ssat[sat-1].azel[1])==0.0) continue;*/

			if (rtk->ssat[sat - 1].tph[rcv][j] == 0.0 || obs[i].LLI[j]/*||rtk->ssat[sat-1].slip[j]*/) {
				rtk->ssat[sat - 1].tph[rcv][j] = obs[i].L[j] + rtk->ssat[sat - 1].php[rcv][j];
				rtk->ssat[sat - 1].tpt[rcv][j] = obs->time;
				matcpy(rtk->ssat[sat - 1].prs[rcv], rs + 6 * i, 3, 1);
				matcpy(rtk->ssat[sat - 1].pvs[rcv], rs + 6 * i + 3, 3, 1);
				rtk->ssat[sat - 1].pts[rcv] = dts[i * 2];
			}
			else
			{
				if ((dt = timediff(obs->time, rtk->ssat[sat - 1].tpt[rcv][j])) > 20.0 - DTTOL)
				{
					rtk->ssat[sat - 1].tph[rcv][j] = obs[i].L[j] + rtk->ssat[sat - 1].php[rcv][j];
					rtk->ssat[sat - 1].tpt[rcv][j] = obs->time;
					matcpy(rtk->ssat[sat - 1].prs[rcv], rs + 6 * i, 3, 1);
					matcpy(rtk->ssat[sat - 1].pvs[rcv], rs + 6 * i + 3, 3, 1);
					rtk->ssat[sat - 1].pts[rcv] = dts[i * 2];
					continue;
				}
				previousLMeas = rtk->ssat[sat - 1].tph[rcv][j] * CLIGHT / freq[j];
				currentLMeas = (obs[i].L[j] + rtk->ssat[sat - 1].php[rcv][j]) * CLIGHT / freq[j];
				matcpy(previousSatPos, rtk->ssat[sat - 1].prs[rcv], 3, 1);
				matcpy(previousSatVel, rtk->ssat[sat - 1].pvs[rcv], 3, 1);
				matcpy(previousRcvPos, sol->prr, 3, 1);

				matcpy(currentSatPos, rs + 6 * i, 3, 1);
				matcpy(currentSatVel, rs + 6 * i + 3, 3, 1);
				matcpy(currentRcvPos, sol->rr, 3, 1);

				hasL = 1;
				break;/*only one available frequency with respect to j*/
			}
		}
		if (hasL == 0) continue;
		if ((r1 = geodist(previousSatPos, previousRcvPos, e1)) <= 0 || (r2 = geodist(currentSatPos, currentRcvPos, e2)) <= 0) continue;

		e2s2 = dot(e2, currentSatPos, 3);
		e1s1 = dot(e1, previousSatPos, 3);
		e2r1 = dot(e2, previousRcvPos, 3);
		e1r1 = dot(e1, previousRcvPos, 3);

		v[nv] = currentLMeas - previousLMeas;
		v[nv] += CLIGHT * (dts[i * 2] - rtk->ssat[sat - 1].pts[rcv]);
		v[nv] -= (e2s2 - e1s1) - (e2r1 - e1r1);
		sats[nv] = sat;
		syss[nv] = sys;

		for (k = 0; k < 4; k++)
		{
			H[nv * nx + k] = k < 3 ? -e2[k] : 1.0;
		}
		var[nv++] = SQR(0.001) * pow(10, 0.1 * MAX(52 - obs[i].SNR[j] * 0.001, 0));
		for (j = 0; j < NFREQ && j < opt->nf; j++)
		{
			rtk->ssat[sat - 1].tph[rcv][j] = obs[i].L[j] + rtk->ssat[sat - 1].php[rcv][j];
			rtk->ssat[sat - 1].tpt[rcv][j] = obs->time;
		}
		matcpy(rtk->ssat[sat - 1].prs[rcv], rs + 6 * i, 3, 1);
		matcpy(rtk->ssat[sat - 1].pvs[rcv], rs + 6 * i + 3, 3, 1);
		rtk->ssat[sat - 1].pts[rcv] = dts[i * 2];
	}
	nsat = nv;

	if (nv < 6)
	{
		/*trace(5, "Baseline: lack of valid sats ns=%d\n", nsat);*/
	}

	else
	{
		/* not lower the weight, remove the satellite instead*/
		nsat0 = nsat;
		while (1)
		{
			if (niter > 0)
			{
				nv = vPhaseRes(rtk, nsat0, H, v, var, sats);
			}
			nsat0 = nv;
			memset(x, 0, nx * sizeof(double));
			matcpy(v_t, v, n, 1);
			// if(strstr(t_str, "00:30:15") && ind == 3){
			// 	trace(NULL,2,"&&TDCP TEST:\n");
			// 	trace(NULL,2,"H=\n"); tracemat(fp_trace,2,H,nx,nv,10,6);
			// 	trace(NULL,2,"v=\n"); tracemat(fp_trace,2,v,1,nv,10,6);
			// 	trace(NULL,2,"var=\n"); tracemat(fp_trace,2,var,1,nv,14,6);
			// }
			for (j = 0; j < nv; j++)
			{
				sig = sqrt(var[j]);
				v_t[j] /= sig;
				for (k = 0; k < nx; k++) H[k + j * nx] /= sig;
			}
			if (nv < 6 || lsq(H, v_t, nx, nv, dx, Q) || sqrt(Q[0] + Q[5] + Q[10])>20 * opt->err[1])
			{
				rtk->dposFlag[rcv][ind][0] = 0;
				break;
			}
			for (j = 0; j < nv; j++)
			{
				sig = sqrt(var[j]);
				v_t[j] *= sig;
				for (k = 0; k < nx; k++) H[k + j * nx] *= sig;
			}
			for (j = 0; j < nx; j++) x[j] += dx[j];
			matmul("TN", nv, 1, nx, -1.0, H, x, 1.0, v_t);

			imax = findMax(v_t, nsat);
			imin = findMin(v_t, nsat);

			if (fabs(v_t[imax]) < THRES && fabs(v_t[imin]) < THRES)
			{
				/*calculate the VDOP*/
				matmul("NT", nx, nx, nv, 1.0, H, H, 0.0, Qv);
				matinv(Qv, nx);
				rtk->dposFlag[rcv][ind][0] = 1;
				break;
			}

			if (fabs(v_t[imax]) >= THRES)
			{
				for (j = 0; j < NFREQ && j < opt->nf; j++)
				{
					rtk->ssat[sats[imax] - 1].vsphase[j] = 0;
				}
				nsat--;
			}
			if (fabs(v_t[imin]) >= THRES)
			{
				for (j = 0; j < NFREQ && j < opt->nf; j++)
				{
					rtk->ssat[sats[imin] - 1].vsphase[j] = 0;
				}
				nsat--;
			}
			niter++;
		}
	}

	if (rtk->dposFlag[rcv][ind][0] == 1) {
		for (i = 0; i < 3; i++)
		{
			Qx[i] = Q[i + i * nx];
		}
		Qx[3] = Q[1];      /*cov xy*/
		Qx[4] = Q[2 + nx];   /*cov yz*/
		Qx[5] = Q[2];      /*cov zx*/

		matcpy(rtk->dpos[rcv][ind], x, 3, 1);
		matcpy(rtk->qdpos[rcv][ind], Qx, 6, 1);
	}
	double enu[3];
	double pos[3];
	ecef2pos(sol->rr, pos);
	ecef2enu(pos, x, enu);
	if (norm(rtk->dpos[rcv][ind], 3) > 1E-5) {
		trace(NULL, 2, "&TDCP,%d,%s,%9.4f,%9.4f,%9.4f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f\n", sys0, t_str, rtk->dpos[rcv][ind][0], rtk->dpos[rcv][ind][1], rtk->dpos[rcv][ind][2], rtk->qdpos[rcv][ind][0], rtk->qdpos[rcv][ind][1], rtk->qdpos[rcv][ind][2], rtk->qdpos[rcv][ind][3], rtk->qdpos[rcv][ind][4], rtk->qdpos[rcv][ind][5]);
	}
	/*trace(3,"BaseLine   %d: %s %9.4f %9.4f %9.4f\n",rcv+1,time_str(obs->time,2),enu[0],enu[1],enu[2]);
	matcpy(rtk->dpos[rcv],enu,3,1);*/


	if (rcv == 0)
	{
		double delta_x[3];
		for (i = 0; i < 3; i++)delta_x[i] = rtk->rb[i] - sol->rr[i];
		ecef2enu(pos, delta_x, enu);
	}

	if (rcv == 0 && sys0 == SYS_CMP && norm(enu, 3) < 100)
	{
		isZupt(rtk);
	}

	free(v); free(v_t); free(H); free(var);
	return rtk->dposFlag[rcv][ind][0];
}


extern int getRefTimeBaseLine(const obsd_t* obs, int n, const nav_t* nav, const prcopt_t* opt, rtk_t* rtk, sol_t* sol)
{
	prcopt_t opt_ = *opt;
	double* rs, * dts, * var;
	int i, j, svh[MAXOBS], rcv = obs->rcv - 1, sys0;

	sol->stat = SOLQ_NONE;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < NSYS + 1; j++) {
			rtk->dpos[rcv][j][i] = 0.0;
			rtk->dposFlag[rcv][j][i] = 0;
		}
	}

	if (n <= 0) {
		return 0;
	}
	sol->time = obs[0].time;

	rs = mat(6, n); dts = mat(2, n); var = mat(1, n);

	opt_.sateph = EPHOPT_BRDC;
	/* satellite positons, velocities and clocks */
	satposs(sol->time, obs, n, nav, opt_.sateph, rs, dts, var, svh);

	for (j = 0; j < NSYS; j++)
	{
		if (j == 0)sys0 = SYS_GPS;
		else if (j == 1)sys0 = SYS_GLO;
		else if (j == 2)sys0 = SYS_GAL;
		else if (j == 3)sys0 = SYS_CMP;
		rtk->dposFlag[rcv][j][0] = getTimeBaseLine(NULL, obs, n, sys0, nav, &opt_, rtk, sol, rs, dts, svh);
	}
	matcpy(sol->prr, sol->rr, 3, 1);
	free(rs); free(dts); free(var);
	return 0;
}

extern void getMultipath(const obsd_t* obs, int n, const nav_t* nav, const prcopt_t* opt, rtk_t* rtk, ssat_t* ssat)
{
	int i, j, f1 = 0, f2 = 1, sat, sys, count = 0;
	double freq1 = 0.0, freq2 = 0.0, c, a, b, mp1, mp2;
	char id[4];

	for (i = 0; i < n; i++)
	{
		sat = obs[i].sat;
		sys = satsys(sat, NULL);
		satno2id(sat, id);
		mp1 = mp2 = 0.0;
		if (sys == SYS_CMP)
		{
			f1 = 0; f2 = 3;
		}
		if (obs[i].P[f1] == 0.0 || obs[i].P[f2] == 0.0 || obs[i].L[f1] == 0.0 || obs[i].L[f2] == 0.0) continue;
		if (sys == SYS_GLO) continue;
		if (sys == SYS_GPS)
		{
			freq1 = FREQL1;
			freq2 = FREQL2;
		}
		else if (sys == SYS_CMP)
		{
			freq1 = FREQ1_CMP;
			freq2 = FREQ3_CMP;
		}
		else
		{
			freq1 = FREQL1;
			freq2 = FREQL2;
		}

		c = SQR(freq1 / freq2);
		a = 2 / (c - 1);
		b = 2 * c / (c - 1);
		mp1 = obs[i].P[f1] - (1 + a) * obs[i].L[f1] * CLIGHT / freq1 + a * obs[i].L[f2] * CLIGHT / freq2;
		mp2 = obs[i].P[f2] - (b)*obs[i].L[f1] * CLIGHT / freq1 + (b - 1) * obs[i].L[f2] * CLIGHT / freq2;
		if (rtk->ssat[sat - 1].nmp[f1] == 0) {
			rtk->ssat[sat - 1].mp[f1] = 0;
		}
		else
		{
			rtk->ssat[sat - 1].mp[f1] = (rtk->ssat[sat - 1].mp[f1] * (rtk->ssat[sat - 1].nmp[f1] + 1) + (mp1 - rtk->ssat[sat - 1].crtmp[f1]) * rtk->ssat[sat - 1].nmp[f1]) / (rtk->ssat[sat - 1].nmp[f1] + 1);
		}
		rtk->ssat[sat - 1].nmp[f1] ++;
		if (rtk->ssat[sat - 1].nmp[f2] == 0) {
			rtk->ssat[sat - 1].mp[f2] = 0;
		}
		else
		{
			rtk->ssat[sat - 1].mp[f2] = (rtk->ssat[sat - 1].mp[f2] * (rtk->ssat[sat - 1].nmp[f2] + 1) + (mp2 - rtk->ssat[sat - 1].crtmp[f2]) * rtk->ssat[sat - 1].nmp[f2]) / (rtk->ssat[sat - 1].nmp[f2] + 1);
		}
		rtk->ssat[sat - 1].nmp[f2] ++;
		rtk->ssat[sat - 1].crtmp[f1] = mp1;
		rtk->ssat[sat - 1].crtmp[f2] = mp2;

		/*trace(5, "Multipath: %4s f(1): %12.4f %12.4f f(2): %12.4f %12.4f ele=%6.2f snr=%5.1f\n", id, mp1, rtk->ssat[sat - 1].mp[f1], mp2, rtk->ssat[sat - 1].mp[f2], ssat[sat - 1].azel[1] * R2D, obs[i].SNR[f1] * SNR_UNIT);*/
		/*if(fabs(rtk->ssat[sat-1].mp[f1])>5.0)
		{
			ssat[sat-1].vs=0;
		}*/
	}
}