#ifndef MONITOR_H
#define MONITOR_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "rtklib.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <pthread.h>
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#elif defined(WIN32)
#define EXPORT
#else
#define EXPORT
#endif

/*typedef struct monitorInfo {
    char configStr[200];
    char errMsg[1000];
    char solBuf[200];
    uint8_t processFlag;
    uint8_t status;
    double progress;
}mInfo;*/

/* common define -------------------------------------------------------------*/
#define BASE64_ENCODE_OUT_SIZE(s) ((unsigned int)((((s) + 2) / 3) * 4 + 1))
#define BASE64_DECODE_OUT_SIZE(s) ((unsigned int)(((s) / 4) * 3))

unsigned int base64_encode(const unsigned char* in, unsigned int inlen, char* out);
unsigned int base64_decode(const char* in, unsigned int inlen, unsigned char* out);

char* GetMacAddress();
/*-----------------------------------------------------------------------END  */
void startMonitor(mInfo* arg);
EXPORT void sleepms(int ms);
EXPORT char* getDeadline();
EXPORT char* getVersion();
EXPORT void setValue(char* _uid, bool _isValid);
extern bool setLicense(char* _filename, int _workmode);

#ifdef __cplusplus
}
#endif
