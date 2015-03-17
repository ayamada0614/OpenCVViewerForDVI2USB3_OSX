/*=========================================================================

  Program:   RobinMedical Open IGT Link Server Program
  Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: 03162015$
  Version:   $Revision: 0.1$

  Copyright (c) Atsushi Yamada, PhD, Biomedical Innovation Center, 
    Shiga University of Medical Science, Japan All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <stdio.h>
#include <stdlib.h>


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <iostream>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include <pthread.h>
#include <semaphore.h>
#include "frmgrab.h"
//#include "v2u_defs.h"

#ifndef V2U_COUNT
#  define V2U_COUNT(array) (sizeof(array)/sizeof((array)[0]))
#endif /* V2U_COUNT */


//static const char * pname = "v2u";
static const char * pname = "cvViewer_c";
static const char * opt_help = "-h";
static const char * opt_help2 = "--help";
static const char * opt_version = "-v";
static const char * opt_version2 = "--version";
static const char * opt_getparam = "-p";
static const char * opt_getparam2 = "--params";
static const char * opt_setparam = "-a";
static const char * opt_setparam2 = "--adjust";
static const char * opt_getsn = "-s";
static const char * opt_getsn2 = "--serial";
static const char * opt_model = "-m";
static const char * opt_model2 = "--model";
static const char * opt_usesn = "-u";
static const char * opt_usesn2 = "--use";
static const char * opt_addr = "-A";
static const char * opt_addr2 = "--address";
static const char * opt_list = "-l";
static const char * opt_list2 = "--list";
static const char * opt_crop = "-c";
static const char * opt_crop2 = "--crop";
static const char * opt_resize = "-r";
static const char * opt_resize2 = "--resize";
static const char * opt_rotate = "-R";
static const char * opt_rotate2 = "--rotate";
static const char * opt_palette = "-f";
static const char * opt_palette2 = "--format";
static const char * opt_count = "-n";
static const char * opt_count2 = "--frame-count";
static const char * opt_streaming = "-S";
static const char * opt_streaming2 = "--no-streaming";
static const char * opt_vga_mode = "-V";
static const char * opt_vga_mode2 = "--vga-mode";
static const char * opt_vga_list = "--vga-list";
static const char * opt_vga_get = "--vga-get";
static const char * opt_vga_set = "--vga-set";
static const char * opt_get_user_data = "--get-user-data";
static const char * opt_set_user_data = "--set-user-data";
static const char * opt_set_digitalmodedetection = "--set-dvi-mode";

static const char * opt_scaler_nearest      = "nearest";
static const char * opt_scaler_average      = "average";
static const char * opt_scaler_fastbilinear = "fastbilinear";
static const char * opt_scaler_bilinear     = "bilinear";
static const char * opt_scaler_bicubic      = "bicubic";
static const char * opt_scaler_experimental = "experimental";
static const char * opt_scaler_point        = "point";
static const char * opt_scaler_area         = "area";
static const char * opt_scaler_bicublin     = "bicublin";
static const char * opt_scaler_sinc         = "sinc";
static const char * opt_scaler_lanczos      = "lanczos";
static const char * opt_scaler_spline       = "spline";
static const char * opt_scaler_hardware     = "hardware";


int count1 = 1;
void* thread1(void* pParam);
pthread_mutex_t mutex;
IplImage* img4FrmGrabber;


typedef struct _V2UCaptureFormatInfo {
    V2U_UINT32 format;
    const char* opt;
    const char* fourcc;
    V2U_BOOL flip;
} V2UCaptureFormatInfo;


static const V2UCaptureFormatInfo v2uCaptureFormatInfo[] = {
      /* format */                /* opt */ /* fourcc */  /* flip */
    { V2U_GRABFRAME_FORMAT_RGB4   ,"rgb4"   ,NULL        ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_RGB8   ,"rgb8"   ,"\0\0\0\0"  ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_RGB16  ,"rgb16"  ,"\0\0\0\0"  ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_BGR16  ,"bgr16"  ,NULL        ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_RGB24  ,"rgb24"  ,NULL        ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_BGR24  ,"bgr24"  ,"\0\0\0\0"  ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_ARGB32 ,"argb32" ,NULL        ,V2U_TRUE  },
    { V2U_GRABFRAME_FORMAT_CRGB24 ,"crgb24" ,"V2UV"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_CBGR24 ,"cbgr24" ,"V2UV"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_CYUY2  ,"cyuy2"  ,"V2UV"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_Y8     ,"y8"     ,NULL        ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_YUY2   ,"yuy2"   ,"YUY2"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_2VUY   ,"uyvy"   ,"UYVY"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_YV12   ,"yv12"   ,"YV12"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_I420   ,"i420"   ,"IYUV"      ,V2U_FALSE },
    { V2U_GRABFRAME_FORMAT_NV12   ,"nv12"   ,NULL      ,V2U_FALSE }
};

/* exit codes */
#define STATUS_OK       0  /* successful completion */
#define STATUS_NODEV    1  /* VGA2USB device not found */
#define STATUS_VMERR    2  /* Video mode detection failure */
#define STATUS_NOSIGNAL 3  /* No signal detected */
#define STATUS_GRABERR  4  /* Capture error */
#define STATUS_IOERR    5  /* File save error */
#define STATUS_CMDLINE  6  /* Command line syntax error */


/* command line parsing context */
typedef struct _v2u_cmdline_context {
    V2URect cropRect;           /* Crop rectangle */
    V2U_UINT32 captureFlags;    /* Capture format and flags */
    V2U_UINT32 frameCount;      /* Frame count */
    V2U_BOOL noStreaming;       /* Streaming flag */
    V2U_BOOL actionPerformed;   /* Something meaningful has been done */
} v2u_cmdline_context;


/**
 * Prints a formatted string to the specified stream, adding v2u: prefix and
 * terminating the line.
 */
static void v2u_vprintln(FILE* out, const char* format, va_list va)
{
    size_t len;
    fprintf(out, "%s: ", pname);
    vfprintf(out, format, va);
    len = strlen(format);
    if (!len || format[len-1] != '\n') {
        fputs("\n", out);
    }
}

/**
 * Prints a formatted string to standard output, adding v2u: prefix and
 * terminating the line.
 */
static void v2u_println(const char* format, ...)
{
    va_list va;
    va_start(va, format);
    v2u_vprintln(stdout, format, va);
    va_end(va);
}

/**
 * Prints a formatted string to standard error, adding v2u: prefix and
 * terminating the line.
 */
static void v2u_error(const char* format, ...)
{
    va_list va;
    va_start(va, format);
    v2u_vprintln(stderr, format, va);
    va_end(va);
}

/**
 * Finds V2UCaptureFormatInfo descriptor that matches the specified
 * V2U_GRABFRAME_FORMAT flag
 */
static const V2UCaptureFormatInfo* v2u_get_format_info(V2U_UINT32 format)
{
    int i;
    format &= V2U_GRABFRAME_FORMAT_MASK;
    for (i=0; i<V2U_COUNT(v2uCaptureFormatInfo); i++) {
        if (v2uCaptureFormatInfo[i].format == format) {
            return v2uCaptureFormatInfo + i;
        }
    }
    return NULL;
}

/**
 * Prints video mode flags in human-readable form
 */
static void v2u_dump_vga_mode_flags(V2U_UINT32 flags, V2U_UINT32 mask)
{
    static struct v2u_vgamode_flags {
        V2U_UINT32 flag;
        const char* on;
        const char* off;
    } vgamode_flags [] = {
        {VIDEOMODE_TYPE_VALID, NULL, "INVALID"},
        {VIDEOMODE_TYPE_ENABLED, "ENABLED", "DISABLED"},
        {VIDEOMODE_TYPE_SUPPORTED, "SUPPORTED", NULL},
        {VIDEOMODE_TYPE_DIGITAL, "DIGITAL", NULL},
        {VIDEOMODE_TYPE_DUALLINK, "DUALLINK", NULL},
        {VIDEOMODE_TYPE_INTERLACED, "INTERLACED", NULL},
        {VIDEOMODE_TYPE_HSYNCPOSITIVE, "HSYNCPOSITIVE", NULL},
        {VIDEOMODE_TYPE_VSYNCPOSITIVE, "VSYNCPOSITIVE", NULL}
    };

    int k, flags_printed = 0;
    for (k=0; k<V2U_COUNT(vgamode_flags); k++) {
        if (vgamode_flags[k].flag & mask) {
            const char* name = (flags & vgamode_flags[k].flag) ?
                vgamode_flags[k].on : vgamode_flags[k].off;
            if (name) {
                printf(flags_printed ? " + %s" : "%s", name);
                flags_printed |= vgamode_flags[k].flag;
            }
        }
        flags &= ~vgamode_flags[k].flag;
    }

    if (flags) {
        printf(flags_printed ? " + 0x%02X" : "0x%02X", flags);
    }
}

/**
 * Dumps VGA modes
 */
static void v2u_dump_vga_modes(const V2UVideoModeDescr* modes, int n, int i0)
{
    int i;
    printf("idx\tWxH-VF\t\t\tFlags\n");
    for (i=0; i<n; i++) {
        if (modes[i].Type & VIDEOMODE_TYPE_VALID) {
            printf("%d\t%dx%d-%d\t\t0x%02X (", i+i0, modes[i].HorAddrTime,
                modes[i].VerAddrTime, modes[i].VerFrequency, modes[i].Type);
            v2u_dump_vga_mode_flags(modes[i].Type, -1);
            printf(")\n");
        }
    }
}


/**
 * Detects and prints video mode
 */
static int v2u_print_video_mode(FrmGrabber* fg, V2U_VideoMode* vm,
    V2U_BOOL details)
{
    if (FrmGrab_DetectVideoMode(fg, vm)) {
        if (vm->width || vm->height) {
            v2u_println("detected!! %dx%d (%d.%d Hz)",
                        vm->width, vm->height, 
                        (vm->vfreq+50)/1000,((vm->vfreq+50)%1000)/100);

            /* Print video mode details */
            if (details) {
                V2U_Property p;
                p.key = V2UKey_CurrentVGAMode;
                if (FrmGrab_GetProperty(fg, &p)) {
                    const V2UVideoModeDescr* mode = &p.value.vesa_mode;
                    printf("  VerFreq:         %u\n", mode->VerFrequency);
                    printf("  HorAddrTime:     %hu\n", mode->HorAddrTime);
                    printf("  HorFrontPorch:   %hu\n", mode->HorFrontPorch);
                    printf("  HorSyncTime:     %hu\n", mode->HorSyncTime);
                    printf("  HorBackPorch:    %hu\n", mode->HorBackPorch);
                    printf("  VerAddrTime:     %hu\n", mode->VerAddrTime);
                    printf("  VerFrontPorch:   %hu\n", mode->VerFrontPorch);
                    printf("  VerSyncTime:     %hu\n", mode->VerSyncTime);
                    printf("  VerBackPorch:    %hu\n", mode->VerBackPorch);
                    printf("  Flags:           0x%02x", mode->Type);
                    if (mode->Type) {
                        printf(" (");
                        /* Some versions of the driver don't set VALID and
                        * ENABLED flags for digital modes - ignore them */
                        v2u_dump_vga_mode_flags(mode->Type,
                            ~(VIDEOMODE_TYPE_VALID | VIDEOMODE_TYPE_ENABLED));
                        printf(")");
                    }
                    printf("\n");
                } else {
                    v2u_error("failed to get video mode details");
                }
            }

            return STATUS_OK;
        } else {
            v2u_println("no signal detected");
            return STATUS_NOSIGNAL;
        }
    } else {
        v2u_error("failed to detect video mode");
        return STATUS_VMERR;
    }
}


void* thread1(void* pParam)
{

  int result = STATUS_OK;

  //int result = STATUS_OK;
  FrmGrabber* fg = NULL;

  // Initialize frmgrab library
  FrmGrabNet_Init();
  fg = FrmGrabLocal_Open();

  // after checking the serial number and open the device : ayamada 3/10/2015
    if (fg != NULL) {

        int i,j;
        V2U_VideoMode vm;
        V2U_BOOL video_mode_detected = V2U_FALSE;
        v2u_cmdline_context cmdline;
        memset(&cmdline, 0, sizeof(cmdline));
        cmdline.captureFlags = V2U_GRABFRAME_FORMAT_BGR24; // 8Ux3 = 24 ayamada


        // detect video mode when we get here first time 
        if (!video_mode_detected) {
            v2u_println("detected the video mode\n");

            result = v2u_print_video_mode(fg, &vm, V2U_FALSE);
            if (result == STATUS_OK) {
                video_mode_detected = V2U_TRUE;
                cmdline.actionPerformed = V2U_TRUE;
                if (!cmdline.cropRect.width || !cmdline.cropRect.height) {
                    // will grab full frame 
                    cmdline.cropRect.x = 0;
                    cmdline.cropRect.y = 0;
                    cmdline.cropRect.width = vm.width;
                    cmdline.cropRect.height = vm.height;
                    v2u_println("detected the video mode2\n");
                }
            }
        } 

        if (video_mode_detected) {

            //printf("l.361\n\n");
            // grab a single frame 
            V2U_GrabFrame2* f = FrmGrab_Frame(fg,cmdline.captureFlags, &cmdline.cropRect);

            // opencv code
            //WaitForSingleObject(hMutex,INFINITE);
            pthread_mutex_lock(&mutex);
            img4FrmGrabber->imageData = (char *)f->pixbuf;
            pthread_mutex_unlock(&mutex);
            //ReleaseMutex(hMutex);
            FrmGrab_Release(fg, f);
        
            while(count1)
            {
                f = FrmGrab_Frame(fg,cmdline.captureFlags, &cmdline.cropRect);
      
                //WaitForSingleObject(hMutex,INFINITE);
                pthread_mutex_lock(&mutex);
                img4FrmGrabber->imageData = (char *)f->pixbuf;
                pthread_mutex_unlock(&mutex);
                //ReleaseMutex(hMutex);
                FrmGrab_Release(fg, f);
            }

     }



    }

    FrmGrab_Close(fg);
    printf("stop capturing thread\n");

}

int main(int argc, char * argv[])
{

  int result = STATUS_OK;
  int keyCode = 0;
  IplImage* destination = cvCreateImage(cvSize(960,540), IPL_DEPTH_8U,3);

  // OpenCV part
  img4FrmGrabber = cvCreateImage(cvSize(1920,1080), IPL_DEPTH_8U,3);
  cvNamedWindow("cvViewer4FrmGrabber",CV_WINDOW_AUTOSIZE);

  pthread_t tid1;

  pthread_mutex_init(&mutex, NULL);

  // generate thread
  pthread_create(&tid1, NULL, thread1, NULL);
  
  while(count1)
  {
    //WaitForSingleObject(hMutex,INFINITE);
    pthread_mutex_lock(&mutex);
    cvResize(img4FrmGrabber, destination, CV_INTER_LINEAR);
    pthread_mutex_unlock(&mutex);
    //ReleaseMutex(hMutex);
    cvShowImage("cvViewer4FrmGrabber", destination);
    keyCode = cvWaitKey(33);
    if(keyCode == 27) count1 = 0;
  }  

  pthread_join(tid1,NULL);
  pthread_mutex_destroy(&mutex);

  cvDestroyWindow("cvViewer4FrmGrabber");
  cvReleaseImage(&img4FrmGrabber);

  return 0;                                                                                                                          

}

