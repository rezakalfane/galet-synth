#include "touch.h"
#include "config.h"     // TOUCH_THRESHOLD, MIN_FINGER_SEP, N_CH, PRESSURE_MAX_REF, MAX_POS_JUMP

TrackedFinger tracked[MAX_FINGERS] = {};

int32_t centroid_window(int32_t* d,int s,int e,int32_t* pk_out,int* pkch_out)
{
    int32_t sw=0,swx=0,pk=0;int pkc=s;
    for(int i=s;i<=e;i++){
        int32_t w=d[i]>0?d[i]:0;
        sw+=w;swx+=w*i;
        if(w>pk){pk=w;pkc=i;}
    }
    if(pk_out)*pk_out=pk;if(pkch_out)*pkch_out=pkc;
    if(sw==0)return -1;
    int32_t p=(swx*1000)/(sw*(N_CH-1));
    if(p<0)p=0;if(p>1000)p=1000;
    return p;
}

int32_t pressure_pct(int32_t pk, int ch){
    int32_t range = PRESSURE_MAX_REF[ch] - TOUCH_THRESHOLD;
    int32_t raw   = pk - TOUCH_THRESHOLD;
    if(raw <= 0)     return 0;
    if(raw >= range) return 100;
    // Clamp raw strictly before sqrt to prevent Newton overshoot
    if(raw > range) raw = range;
    int32_t scaled = raw * 10000 / range; // 0..10000
    if(scaled > 10000) scaled = 10000;
    // Integer sqrt via Newton
    int32_t s = scaled / 2 + 1;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    // Hard clamp output — Newton can land at 101 due to rounding
    if(s < 0)   s = 0;
    if(s > 100) s = 100;
    return s;
}

// Find up to `maxf` fingers: repeatedly take the strongest remaining peak, and
// if it's far enough from the peaks already found, record its centroid; then
// zero a ±2 window around it and look again. The first two peaks come out
// identical to the old 2-finger detector, so the mono path is unchanged.
int detect_raw(int32_t* delta, Finger* out, int maxf)
{
    for(int i=0;i<maxf;i++) out[i].active=false;
    int32_t work[N_CH];
    for(int i=0;i<N_CH;i++) work[i]=delta[i];

    int peaks[MAX_FINGERS];
    int found=0;
    for(int attempt=0; attempt<maxf && found<maxf; attempt++){
        int pk=0; int32_t mx=0;
        for(int i=0;i<N_CH;i++) if(work[i]>mx){ mx=work[i]; pk=i; }
        if(mx<TOUCH_THRESHOLD) break;                 // nothing left above threshold

        bool sep_ok=true;
        for(int j=0;j<found;j++){ int s=pk-peaks[j]; if(s<0)s=-s; if(s<MIN_FINGER_SEP){ sep_ok=false; break; } }
        if(sep_ok){
            int ws=pk-2; if(ws<0)ws=0; int we=pk+2; if(we>=N_CH)we=N_CH-1;
            int32_t pkv; int pkc;
            int32_t pos=centroid_window(work,ws,we,&pkv,&pkc);
            out[found]={true,pos,pressure_pct(pkv,pkc),pkc,pkv};
            peaks[found]=pk;
            found++;
        }
        // Zero this peak's window so the next pass finds a different finger.
        for(int i=pk-2;i<=pk+2;i++) if(i>=0 && i<N_CH) work[i]=0;
    }
    return found;
}

void update_tracked(Finger* raw,int n)
{
    bool ra[MAX_FINGERS]={};
    for(int slot=0;slot<MAX_FINGERS;slot++){
        if(!tracked[slot].alive)continue;
        int32_t best=MAX_POS_JUMP+1;int br=-1;
        for(int r=0;r<n;r++){
            if(ra[r])continue;
            int32_t c=raw[r].pos-tracked[slot].pos;if(c<0)c=-c;
            if(c<best){best=c;br=r;}
        }
        if(br>=0){
            tracked[slot].pos=raw[br].pos;tracked[slot].pressure=raw[br].pressure;
            tracked[slot].peak_ch=raw[br].peak_ch;tracked[slot].peak_delta=raw[br].peak_delta;
            ra[br]=true;
        } else tracked[slot].alive=false;
    }
    for(int r=0;r<n;r++){
        if(ra[r])continue;
        for(int slot=0;slot<MAX_FINGERS;slot++){
            if(!tracked[slot].alive){
                tracked[slot]={true,raw[r].pos,raw[r].pressure,raw[r].peak_ch,raw[r].peak_delta};
                ra[r]=true;break;
            }
        }
    }
}
