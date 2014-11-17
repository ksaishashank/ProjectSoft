#ifndef _FORCE_RECORDER_RENDER_TYPE
#define _FORCE_RECORDER_RENDER_TYPE

typedef unsigned int ForceRecorderRenderType;

#define	FRRT_Contact 0x00000001
#define	FRRT_Elastic 0x00000010
#define	FRRT_Muscle  0x00000100
#define	FRRT_Gravity 0x00001000
#define	FRRT_Total   0x00010000

#endif