/* =======================================================
 * pcdDecode - a Photo-CD image decoder and converter
 * =======================================================
 *
 * Project Info:  http://sourceforge.net/projects/pcdtojpeg/
 * Project Lead:  Sandy McGuffog (sandy.cornerfix@gmail.com);
 *
 * (C) Copyright 2009-2011, by Sandy McGuffog and Contributors.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this
 * library; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * ---------------
 * pcdDecode is written to provide a PCD decoder library that is:
 * 1. Easy to use, with a clear API, and human readable messages
 * 2. Suited to modern multi-core processors; so multithreaded where applicable
 * 3. Multi-platform - OS X, Windows, *nix, little- or big-endian
 * 4. Minimum dependencies - pcdDecode will use pthreads if available, but is 
 *    otherwise standalone (other than for normal C++ libraries)
 * 5. Correctly converts all forms of PCD files, including all classes of huffman 
 *    encoding (classes 1-4)
 * 6. Converts all PCD resolutions, including 64Base images
 * 7. Correct and predictable color space handling (no blown highlights, no color casts);
 *    RGB data can be returned in a choice of well specified color spaces.
 * 8. Full support for "Kodak standard" bilinear interpolation of chroma
 * 9. Extracts PCD file metadata information
 * 10. Transparently recovers from many data errors
 * 11. GPL licensed
 *
 * pcdDecode is also available is non-GPL version that adds AHD style interpolation; 
 * this offers lower noise and better edge performance.
 *
 * ---------------
 * Compiling: pcdDecode should compile without modification on most systems. It has been
 * tested on OS X (Xcode), MS Windows (Visual C++) and linux (GCC), on both big- and 
 * little-endian CPUs
 * The only thing to do is to define "mNoPThreads" if you don't have (or don't want)
 * pThreads. 
 *
 * ---------------
 * pcdDecode.cpp
 * ---------------
 * (C) Copyright 2009-2011, by Sandy McGuffog and Contributors.
 *
 * Original Author:  Sandy McGuffog;
 * Contributor(s):   -;
 *
 * Acknowlegements:   Hadmut Danisch (danisch@ira.uka.de), who authored hpcdtoppm in the early
 *                    90's. Although this software itself is unrelated to hpcdtoppm, the pcdDecoder 
 *                    package would not have been possible without Hadmut's original reverse
 *                    engineering of the PCD file format.
 *                    
 *                    Ted Felix (http://tedfelix.com/ ), who provided sample PCD images and assorted 
 *                    PCD documentation, as well as testing early versions of this software
 *
 * Changes
 * -------
 *
 * V1.0.2 - 1 July 2009 - Added YCC color space to the decoder library
 *                        Improved sRGB shadow detail and color handling
 *
 * V1.0.3 - 1 Sept 2009 - More descriptive error messages
 *
 * V1.0.4 - 19 Sept 2009 - Multithreading under Windows
 *
 * V1.0.5 - 21 Sept 2009 - More efficient memory management
 *
 * V1.0.6 - 12 Jan 2010 - Fix for compile problems under Ubuntu
 *
 * V1.0.7 - 12 Feb 2010 - Fix bug in advanced iterpolation routine
 *                        Note this has no impact on the GPL version
 *
 * V1.0.8 - 20 Mar 2010 - Fix bug in the conversion of 64base images that
 *                        have a non-native aspect ratio 
 *
 * V1.0.9 - 23 Sept 2010 - Fix bug in the conversion of 64base images that
 *                        have long sequence lengths; modifications to allow 
 *                        easy compilation as a DLL
 *
 * V1.0.10 - 28 March 2011 - Work around for a bug in GCC that incorrectly handles 
 *                           nested structures on the stack
 */

// You can define this manuallly here, to disable threading, or do so at the command line
// Note that this is defined later for MS compilers, as MS doesn't support pthreads
// #define mNoPThreads

#include "pcdDecode.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

//////////////////////////////////////////////////////////////
//
// Coding
//
//////////////////////////////////////////////////////////////
// Broadly, this code follows this as a standard:
//	clock_t represents the system times in clock ticks.
//	dev_t is used for device numbers.
//	off_t is used for file sizes and offsets.
//	ptrdiff_t is the signed integral type for the result of subtracting two pointers.
//	size_t reflects the size, in bytes, of objects in memory.
//	ssize_t is used by functions that return a count of bytes or an error indication.
//	time_t counts time in seconds.

#define KSectorSize 0x800
#define UseFourPixels 1

#ifdef __debug
	#define mInformPrintf 1
#endif

//////////////////////////////////////////////////////////////
//
// Performance Analysis
//
//////////////////////////////////////////////////////////////
// Define this to enable performance analysis on the Mac
#define ___PerformanceAnalysis 1
#ifdef __debug
	#define ___PerformanceAnalysis 1
#endif


//////////////////////////////////////////////////////////////
//
// Threading controls
//
//////////////////////////////////////////////////////////////
// Undefine this to disable threading

// Typically you want about threads = cores
#ifdef mNoPThreads
#define kNumThreads 1
#define pcdThreadFunction static void *
#else
#define kNumThreads 8
#ifdef _MSC_VER
// This defines a set of macros that make the WIN32 multithread
// API look somewhat like pthreads - at least enough for our
// simple purposes
// It would be more elegant to use e.g., pthreads-win32, but
// this way works, and again keeps dependencies to a minimum
#include <windows.h>
#include <process.h>
#define PTHREAD_STACK_MIN 65536
#define pcdThreadDescriptor HANDLE
#define pcdThreadFunction static unsigned __stdcall
// Under Win32 we use pthread_attr_t just to hold the stack size
#define pthread_attr_t unsigned
#define pthread_attr_init(threadAttr) {(*threadAttr)=PTHREAD_STACK_MIN<<1;}
#define pthread_attr_setstacksize(threadAttr, stackSize) {(*threadAttr)=stackSize;}
#define pthread_attr_setdetachstate(threadAttr, theAttribute) {}
#define pthread_attr_destroy(threadAttr) {}
#define pcdStartThread(theThread, theThreadAttr, theFunction, theData) ((theThread = (HANDLE)_beginthreadex(NULL, theThreadAttr, theFunction, theData, 0, NULL)) == NULL ? -1 : 0)
#define pcdThreadJoin(theThread, result) ((WaitForSingleObject(theThread,INFINITE) != WAIT_OBJECT_0) || !CloseHandle(theThread))
#else
#include <pthread.h>
#include <limits.h>
#define pcdThreadDescriptor pthread_t
#define pcdThreadFunction static void *
#define pcdStartThread(theThread, theThreadAttr, theFunction, theData) pthread_create(&theThread, &theThreadAttr, theFunction, theData)
#define pcdThreadJoin(theThread, result) pthread_join(theThread,result)
#endif
#endif

#ifdef mUseNonGPLCode
	pcdThreadFunction upResLumaInterpolatePassI(void *t);
	pcdThreadFunction upResLumaInterpolatePassII(void *t);
#endif

//////////////////////////////////////////////////////////////
//
// Overall PCD file structure
// (in sector measures)
//
//////////////////////////////////////////////////////////////
//
// Name				Start			Length
// IP-ICA			0				1
// IPA				1				2
// Base/16 ICA		3				1
// Base/16 ICD		4				18
// Base/4 ICA		22				1
// Base/4 ICD		23				72
// Base ICA			95				1
// Base ICD			96				288
// 4Base ICA		384				1
// 4Base LPT-MRS	385				2
// 4Base LPT		387				1
// 4Base HCT		388				1
// 4Base ICD		389				variable
// 16Base ICA		var				1				// Start defined by the base4Stop field in the 
													// ImageComponentAttributes
// 16Base LPT-MRS	var				9
// 16Base LPT		var				2
// 16Base HCT		var				2
// 16Base ICD		var				variable
//
// ICA - Image Component Attributes
// IPA - Image Pack Attributes
// ICI - Image Component Information
// ICD - Image Component Data (luma and interleaved chroma)
// LPT - Line Pointer Table
// HCT - Huffman Code Table


//////////////////////////////////////////////////////////////
//
// Definition Tables - these control most aspects of decoder
// behaviour
//
//////////////////////////////////////////////////////////////

#define kSceneSectorSize KSectorSize

static unsigned int PCDLumaWidth[kMaxScenes]			= {192, 192<<1, 192<<2, 192<<3, 192<<4, 192<<5};
static unsigned int PCDLumaHeight[kMaxScenes]			= {128, 128<<1, 128<<2, 128<<3, 128<<4, 128<<5};
static unsigned int PCDChromaWidth[kMaxScenes]			= {96, 96<<1, 96<<2, 96<<2, 96<<4, 96<<5};
static unsigned int PCDChromaHeight[kMaxScenes]			= {64, 64<<1, 64<<2, 64<<2, 64<<4, 64<<5};
static unsigned int PCDChromaResFactor[kMaxScenes]		= {1, 1, 1, 1, 1, 1};
static uint32_t RowShift[kMaxScenes]					= {0, 0, 0, 9, 9, 6};
static uint32_t RowMask[kMaxScenes]						= {0, 0, 0, 0x1fff, 0x1fff, 0x3fff};
static uint32_t RowSubSample[kMaxScenes]				= {1, 1, 1, 1, 1, 2};
static uint32_t SequenceShift[kMaxScenes]				= {0, 0, 0, 0, 0, 1};
static uint32_t SequenceMask[kMaxScenes]				= {0, 0, 0, 0, 0x0, 0xf};
static uint32_t PlaneShift[kMaxScenes]					= {0, 0, 0, 22, 22, 19};
static uint32_t PlaneMask[kMaxScenes]					= {0, 0, 0, 0x3, 0x3, 0x6};
static uint32_t HuffmanHeaderSize[kMaxScenes]			= {0, 0, 0, 3, 3, 4};

enum PCDOutputDataSize {
	pcdByteSize = 0,
	pcdInt16Size,
	pcdFloatSize
};


//////////////////////////////////////////////////////////////
//
// Private IPI structure definitions
//
//////////////////////////////////////////////////////////////
struct IPIHeader
{
	char ipiSignature[7];							// "PCD_IPI"
	uint8_t specificationVersion[2];				// Binary coded MSB major, LSB minor
	uint8_t authoringSoftwareRelease[2];			// Binary coded MSB major, LSB minor
	uint8_t imageMagnificationDecriptor[2];			// BCD coded MSB major, LSB minor
	uint8_t imageScanningTime[4];					// Binary coded seconds since 1970-01-01 GMT
	uint8_t imageModificationTime[4];				// Binary coded seconds since 1970-01-01 GMT
	uint8_t imageMedium;							// 0 - color negative
													// 1 - color reversal
													// 2 - color hard copy
													// 3 - thermal hard copy
													// 4 - black and white negative
													// 5 - black and white reversal
													// 6 - black and white hard copy
													// 7 - internegative
													// 8 - synthetic image
	char productType[20];							// Product code, ISO 646, etc blank padded
	char scannerVendorIdentity[20];					// Vendor ID, ISO 646, etc blank padded
	char scannerProductIdentity[16];				// Scanner ID, ISO 646, etc blank padded
	char scannerFirmwareRevision[4];				// ISO 646, etc blank padded
	char scannerFirmwareDate[8];					// ISO 646, etc blank padded
	char scannerSerialNumber[20];					// ISO 646, etc blank padded
	uint8_t scannerPixelSize[2];					// BCD coded in microns, LSB is after the decimal
	char piwEquipmentManufacturer[20];				// Imaging workstation manufacturer, ISO 646, etc blank padded
	uint8_t photoFinisherCharSet;					// 1 - 38 characters ISO 646
													// 2 - 65 characters ISO 646
													// 3 - 95 characters ISO 646
													// 4 - 191 characters ISO 8859-1
													// 5 - ISO 2022
													// 6 - includes characters not ISO 2375 registered
	char photoFinisherEscapeSequence[32];			// Used for char set 5 and 6, conforms to ISO 2022 excluding esc
													// unused chars 00
	char photoFinisherName[60];						// per previous two fields
	char sbaSignature[3];							// "SBA" - rest of the sba fields exist only if this is defined
	uint8_t sbaRevision[2];							// Binary coded MSB major, LSB minor
	uint8_t sbaCommand;								// 0 - neutral SBA on, color SBA on
													// 1 - neutral SBA off, color SBA off
													// 2 - neutral SBA on, color SBA off
													// 3 - neutral SBA off, color SBA on	
	uint8_t sbaData[94];							// Proprietary Kodak data
	uint8_t sbaFTN[2];								// FTN code
	uint8_t sbaData2[4];							// Proprietary Kodak data
	uint8_t copyrightStatus;						// 1 - restrictions apply
													// 0xff - restrictions not specified
													// If restrictions apply, see next field
	char copyrightFile[12];							// Filename of the rights file in the "RIGHTS" directory on the CD
	uint8_t reservedBytes[1192];					// Used fill to 1536
};


//////////////////////////////////////////////////////////////
//
// File header structures - IP-ICA and IPA areas
// This is the structure of the file header section of the PCD
// file. In as much as we are interested/understand it anyway
//
//////////////////////////////////////////////////////////////

// In the IPA areas
struct ImageComponentAttributes {
	uint8_t	reserved[2];
	uint8_t	attributes;								// Bit 7 unused
													// Bit 6-5 : Huffman code class
													// Bit 4 : IPE present (should always be 0)
													// Bit 3-2 : 00 - Base, 01 - 4Base, 10 - 16Base
													// Bit 0-1 : Rotation 00 - 0deg, 01 - 90CCW, 10 - 180CCW, 11 - 270CCW
	uint8_t	sectorStop4Base[2];						// Sector end of the 4 base data
	uint8_t	sectorStop16Base[2];					// Sector end of the 16 base data
	uint8_t	sectorStopIPE[2];						// Should be 0 as IPE is unimplemented
	uint8_t	interleaveRatio;						// Should be 1 for photo CD images only
													// if anything else, we have audio data interleaved.
	uint8_t	ADPCMResolution;
	uint8_t	ADPCMMagnificationPanning[2];
	uint8_t	ADPCMMagnifacationFactor;
	uint8_t	ADPCMDisplayOffset[2];
	uint8_t	ADPCMTransitionDescriptor;
	uint8_t	reserved2[495];							// Fill to 512 bytes
};

// IP-ICA area
struct PCDFileHeader {
	char	signature[7];							// PCD_OPA is overview file
	uint8_t	reserved[2041];							// Fill to sector size
};

// The ICI for each image (scene) is identical to the base16 image, IF the scene exists.
struct PCDFile {
	struct PCDFileHeader					header;		// Signature only
	struct IPIHeader						ipiHeader;	// Basic Image info
	struct ImageComponentAttributes			iciBase16;	// Image attributes for the base16 image
	struct ImageComponentAttributes			iciBase4;	// Image attributes for the base4 image
	struct ImageComponentAttributes			iciBase;	// Image attributes for the base image
	struct ImageComponentAttributes			ici4Base;	// Image attributes for the 4base image
	struct ImageComponentAttributes			ici16Base;	// Image attributes for the 16base image
};


//////////////////////////////////////////////////////////////
//
// Support code for performance analysis - Mac only
//
//////////////////////////////////////////////////////////////
#ifdef ___PerformanceAnalysis
#ifdef qMacOS
static float HowLong(AbsoluteTime endTime,
					 AbsoluteTime bgnTime)
{
    AbsoluteTime absTime;
    Nanoseconds  nanosec;
	
    absTime = SubAbsoluteFromAbsolute(endTime, bgnTime);
    nanosec = AbsoluteToNanoseconds(absTime);
    return (float) UnsignedWideToUInt64( nanosec ) / 1000.0;
}
#endif
#endif

//////////////////////////////////////////////////////////////
//
// Lookups for metadata, film code definitions, etc
//
//////////////////////////////////////////////////////////////
enum MetaLookUpLengths {
	kMaxPCDFilms		= 219,
	kMaxPCDmediums		= 10,
	kMaxSBATypes		= 4,
	kMaxHuffmanClasses	= 4
};

static const char *PCDMediumTypes[kMaxPCDmediums] = {
"color negative",
"color reversal",
"color hard copy",
"thermal hard copy",
"black and white negative",
"black and white reversal",
"black and white hard copy",
"internegative",
"synthetic image",
"chromogenic"
};

static const char *PCDSBATypes[kMaxSBATypes] = {
"neutral SBA on, color SBA on",
"neutral SBA off, color SBA off",
"neutral SBA on, color SBA off",
"neutral SBA off, color SBA on"
};

static const char *PCDHuffmanClasses[kMaxHuffmanClasses] = {
"class 1 - 35mm film; pictoral hard copy",
"class 2 - large format film",
"class 3 - text and graphics, high resolution",
"class 4 - text and graphics, high dynamic range"
};

// -1 for GC indicates not specified
static const short PCDFTN_PC_GC_Medium[kMaxPCDFilms][4] = {
	1, 18, 7, 0,     // 3M ScotchColor AT 100
	2, 18, 9, 0,     // 3M ScotchColor AT 200
	3, 18, 8, 0,     // 3M ScotchColor HR2 400
	7, 18, 3, 0,     // 3M Scotch HR 200 Gen 2
	9, 18, 5, 0,     // 3M Scotch HR 400 Gen 2
	16, 113, -1, 0,  // AGFA AGFACOLOR XRS 400 Gen 1
	17, 17, 7, 0,    // AGFA AGFACOLOR XRG/XRS 400
	18, 17, 4, 0,    // AGFA AGFACOLOR XRG/XRS 200
	19, 17, 10, 0,   // AGFA AGFACOLOR XRS 1000 Gen 2
	20, 49, 7, 0,    // AGFA AGFACOLOR XRS 400 Gen 2
	21, 17, 1, 0,    // AGFA AGFACOLOR XRS/XRC 100
	26, 10, 6, 0,    // FUJI Reala 100 (JAPAN)
	27, 10, 12, 0,   // FUJI Reala 100 Gen 1
	28, 10, 14, 0,   // FUJI Reala 100 Gen 2
	29, 10, 2, 0,    // FUJI SHR 400 Gen 2
	30, 10, 5, 0,    // FUJI Super HG 100
	31, 10, 8, 0,    // FUJI Super HG 1600 Gen 1
	32, 10, 11, 0,   // FUJI Super HG 200
	33, 10, 10, 0,   // FUJI Super HG 400
	34, 10, 13, 0,   // FUJI Super HG 100 Gen 2
	35, 8, 4, 0,     // FUJI Super HR 100 Gen 1
	36, 10, 4, 0,    // FUJI Super HR 100 Gen 2
	37, 8, -1, 0,    // FUJI Super HR 1600 Gen 2
	38, 8, 3, 0,     // FUJI Super HR 200 Gen 1
	39, 10, 3, 0,    // FUJI Super HR 200 Gen 2
	40, 8, 2, 0,     // FUJI Super HR 400 Gen 1
	43, 8, 6, 0,     // FUJI NSP 160S (PRO)
	45, 82, 2, 0,    // KODAK KODACOLOR VR 100 Gen 2
	47, 82, 3, 0,    // KODAK GOLD 400 Gen 3
	55, 81, 9, 0,    // KODAK EKTAR 100 Gen 1
	56, 81, 3, 0,    // KODAK EKTAR 1000 Gen 1
	57, 81, 2, 0,    // KODAK EKTAR 125 Gen 1
	58, 81, 1, 0,    // KODAK ROYAL GOLD 25 RZ
	60, 80, 9, 0,    // KODAK GOLD 1600 Gen 1
	61, 80, 12, 0,   // KODAK GOLD 200 Gen 2
	62, 81, 7, 0,    // KODAK GOLD 400 Gen 2
	65, 80, 4, 0,    // KODAK KODACOLOR VR 100 Gen 1
	66, 80, 5, 0,    // KODAK KODACOLOR VR 1000 Gen 2
	67, 80, 14, 0,   // KODAK KODACOLOR VR 1000 Gen 1
	68, 80, 3, 0,    // KODAK KODACOLOR VR 200 Gen 1
	69, 80, 2, 0,    // KODAK KODACOLOR VR 400 Gen 1
	70, 82, 1, 0,    // KODAK KODACOLOR VR 200 Gen 2
	71, 80, 6, 0,    // KODAK KODACOLOR VRG 100 Gen 1
	72, 80, 11, 0,   // KODAK GOLD 100 Gen 2
	73, 80, 8, 0,    // KODAK KODACOLOR VRG 200 Gen 1
	74, 80, 7, 0,    // KODAK GOLD 400 Gen 1
	87, 112, 4, 0,   // KODAK EKTACOLOR GOLD 160
	88, 81, 6, 0,    // KODAK EKTAPRESS 1600 Gen 1 PPC
	89, 81, 4, 0,    // KODAK EKTAPRESS GOLD 100 Gen 1 PPA
	90, 81, 10, 0,   // KODAK EKTAPRESS GOLD 400 PPB-3
	92, 81, 8, 0,    // KODAK EKTAR 25 Professional PHR
	97, 67, 1, 4,    // KODAK T-MAX 100 Professional
	98, 67, 3, 4,    // KODAK T-MAX 3200 Professional
	99, 67, 2, 4,    // KODAK T-MAX 400 Professional
	101, 112, 3, 0,  // KODAK VERICOLOR 400 Prof VPH
	102, 112, 1, 0,  // KODAK VERICOLOR III Pro
	121, 2, 11, 0,   // KONICA KONICA COLOR SR-G 3200
	122, 40, -1, 0,  // KONICA KONICA COLOR SUPER SR100
	123, 40, 6, 0,   // KONICA KONICA COLOR SUPER SR 400
	138, 80, -1, 0,  // KODAK GOLD UNKNOWN
	139, -1, -1, 0,  // KODAK UNKNOWN NEG A- Normal SBA
	143, 81, 11, 0,  // KODAK EKTAR 100 Gen 2
	147, 129, 1, 0,  // KODAK KODACOLOR CII
	148, 129, 2, 0,  // KODAK KODACOLOR II
	149, 82, 7, 0,   // KODAK GOLD Plus 200 Gen 3
	150, 130, 1, 7,  // KODAK Internegative +10% Contrast
	151, 17, 3, 0,   // AGFA AGFACOLOR Ultra 50
	152, 10, 9, 0,   // FUJI NHG 400
	153, 17, 2, 0,   // AGFA AGFACOLOR XRG 100
	154, 82, 6, 0,   // KODAK GOLD Plus 100 Gen 3
	155, 40, 13, 0,  // KONICA KONICA COLOR SUPER SR200 GEN 1
	156, 40, 4, 0,   // KONICA KONICA COLOR SR-G 160
	157, 17, 2, 0,   // AGFA AGFACOLOR OPTIMA 125
	158, 17, 2, 0,   // AGFA AGFACOLOR PORTRAIT 160
	162, 80, 7, 0,   // KODAK KODACOLOR VRG 400 Gen 1
	163, 80, 8, 0,   // KODAK GOLD 200 Gen 1
	164, 80, 11, 0,  // KODAK KODACOLOR VRG 100 Gen 2
	174, 130, 2, 7,  // KODAK Internegative +20% Contrast
	175, 130, 3, 7,  // KODAK Internegative +30% Contrast
	176, 130, 4, 7,  // KODAK Internegative +40% Contrast
	184, 67, 20, 4,  // KODAK TMAX-100 D-76 CI = .40
	185, 67, 21, 4,  // KODAK TMAX-100 D-76 CI = .50
	186, 67, 22, 4,  // KODAK TMAX-100 D-76 CI = .55
	187, 67, 23, 4,  // KODAK TMAX-100 D-76 CI = .70
	188, 67, 24, 4,  // KODAK TMAX-100 D-76 CI = .80
	189, 67, 25, 4,  // KODAK TMAX-100 TMAX CI = .40
	190, 67, 26, 4,  // KODAK TMAX-100 TMAX CI = .50
	191, 67, 27, 4,  // KODAK TMAX-100 TMAX CI = .55
	192, 67, 28, 4,  // KODAK TMAX-100 TMAX CI = .70
	193, 67, 29, 4,  // KODAK TMAX-100 TMAX CI = .80
	195, 67, 31, 4,  // KODAK TMAX-400 D-76 CI = .40
	196, 67, 32, 4,  // KODAK TMAX-400 D-76 CI = .50
	197, 67, 33, 4,  // KODAK TMAX-400 D-76 CI = .55
	198, 67, 34, 4,  // KODAK TMAX-400 D-76 CI = .70
	214, 67, 35, 4,  // KODAK TMAX-400 D-76 CI = .80
	215, 67, 36, 4,  // KODAK TMAX-400 TMAX CI = .40
	216, 67, 37, 4,  // KODAK TMAX-400 TMAX CI = .50
	217, 67, 38, 4,  // KODAK TMAX-400 TMAX CI = .55
	218, 67, 39, 4,  // KODAK TMAX-400 TMAX CI = .70
	219, 67, 40, 4,  // KODAK TMAX-400 TMAX CI = .80
	224, 66, 10, 0,  // 3M ScotchColor ATG 400/EXL 400
	266, 17, 5, 0,   // AGFA AGFACOLOR OPTIMA 200
	267, 40, 3, 0,   // KONICA IMPRESSA 50
	268, 18, 9, 0,   // POLAROID POLAROID CP 200
	269, 40, 11, 0,  // KONICA KONICA COLOR SUPER SR200 GEN 2
	270, 110, 3, 9,  // ILFORD XP2 400
	271, 40, -1, 0,  // POLAROID POLAROID COLOR HD2 100
	272, 40, 6, 0,   // POLAROID POLAROID COLOR HD2 400
	273, 40, 11, 0,  // POLAROID POLAROID COLOR HD2 200
	282, 66, 5, 0,   // 3M ScotchColor ATG-1 200
	284, 40, 7, 0,   // KONICA XG 400
	307, 67, 99, 1,  // KODAK UNIVERSAL REVERSAL B / W
	308, 20, 64, 1,  // KODAK RPC COPY FILM Gen 1
	312, 52, 55, 1,  // KODAK UNIVERSAL E6
	324, 82, 10, 0,  // KODAK GOLD Ultra 400 Gen 4
	328, 12, 12, 0,  // FUJI Super G 100
	329, 12, 3, 0,   // FUJI Super G 200
	330, 12, 10, 0,  // FUJI Super G 400 Gen 2
	333, 116, 22, 1, // KODAK UNIVERSAL K14
	334, 12, 2, 0,   // FUJI Super G 400 Gen 1
	366, 150, 1, 0,  // KODAK VERICOLOR HC 6329 VHC
	367, 150, 2, 0,  // KODAK VERICOLOR HC 4329 VHC
	368, 150, 3, 0,  // KODAK VERICOLOR L 6013 VPL
	369, 150, 4, 0,  // KODAK VERICOLOR L 4013 VPL
	418, 82, 10, 0,  // KODAK EKTACOLOR Gold II 400 Prof
	430, 83, 2, 0,   // KODAK ROYAL GOLD 1000
	431, 82, 13, 0,  // KODAK KODACOLOR VR 200 / 5093
	432, 83, 4, 0,   // KODAK GOLD Plus 100 Gen 4
	443, 83, 8, 0,   // KODAK ROYAL GOLD 100
	444, 83, 10, 0,  // KODAK ROYAL GOLD 400
	445, 52, 70, 1,  // KODAK UNIVERSAL E6 auto-balance
	446, 52, 71, 1,  // KODAK UNIVERSAL E6 illum. corr.
	447, 116, 70, 1, // KODAK UNIVERSAL K14 auto-balance
	448, 116, 71, 1, // KODAK UNIVERSAL K14 illum. corr.
	449, 83, 8, 0,   // KODAK EKTAR 100 Gen 3 SY
	456, 81, 1, 0,   // KODAK EKTAR 25
	457, 83, 8, 0,   // KODAK EKTAR 100 Gen 3 CX
	458, 83, 8, 0,   // KODAK EKTAPRESS PLUS 100 Prof PJA-1
	459, 83, 8, 0,   // KODAK EKTAPRESS GOLD II 100 Prof
	460, 83, 8, 0,   // KODAK Pro 100 PRN
	461, 83, 8, 0,   // KODAK VERICOLOR HC 100 Prof VHC-2
	462, 83, 8, 0,   // KODAK Prof Color Neg 100
	463, 83, 2, 0,   // KODAK EKTAR 1000 Gen 2
	464, 83, 2, 0,   // KODAK EKTAPRESS PLUS 1600 Pro PJC-1
	465, 83, 2, 0,   // KODAK EKTAPRESS GOLD II 1600 Prof
	466, 83, 2, 0,   // KODAK SUPER GOLD 1600 GF Gen 2
	467, 83, 4, 0,   // KODAK KODACOLOR 100 Print Gen 4
	468, 83, 4, 0,   // KODAK SUPER GOLD 100 Gen 4
	469, 83, 4, 0,   // KODAK GOLD 100 Gen 4
	470, 83, 4, 0,   // KODAK GOLD III 100 Gen 4
	471, 83, 9, 0,   // KODAK FUNTIME 100 FA
	472, 82, 13, 0,  // KODAK FUNTIME 200 FB
	473, 82, 13, 0,  // KODAK KODACOLOR VR 200 Gen 4
	474, 83, 5, 0,   // KODAK GOLD Super 200 Gen 4
	475, 83, 5, 0,   // KODAK KODACOLOR 200 Print Gen 4
	476, 83, 5, 0,   // KODAK SUPER GOLD 200 Gen 4
	477, 83, 5, 0,   // KODAK GOLD 200 Gen 4
	478, 83, 5, 0,   // KODAK GOLD III 200 Gen 4
	479, 83, 6, 0,   // KODAK GOLD Ultra 400 Gen 5
	480, 83, 6, 0,   // KODAK SUPER GOLD 400 Gen 5
	481, 83, 6, 0,   // KODAK GOLD 400 Gen 5
	482, 83, 6, 0,   // KODAK GOLD III 400 Gen 5
	483, 83, 6, 0,   // KODAK KODACOLOR 400 Print Gen 5
	484, 83, 6, 0,   // KODAK EKTAPRESS PLUS 400 Prof PJB-2
	485, 83, 6, 0,   // KODAK EKTAPRESS GOLD II 400 Prof G5
	486, 83, 6, 0,   // KODAK Pro 400 PPF-2
	487, 83, 6, 0,   // KODAK EKTACOLOR GOLD II 400 EGP-4
	488, 83, 6, 0,   // KODAK EKTACOLOR GOLD 400 Prof EGP-4
	489, 83, 3, 0,   // KODAK EKTAPRESS GOLD II Multspd PJM
	490, 112, 11, 0, // KODAK Pro 400 MC PMC
	491, 112, 11, 0, // KODAK VERICOLOR 400 Prof VPH-2
	492, 112, 11, 0, // KODAK VERICOLOR 400 PLUS Prof VPH-2
	493, 83, -1, 0,  // KODAK UNKNOWN NEG Product Code 83
	505, 112, 12, 0, // KODAK EKTACOLOR PRO GOLD 160 GPX
	508, 83, 11, 0,  // KODAK ROYAL GOLD 200
	517, 52, 72, 1,  // KODAK 4050000000
	519, 83, 12, 0,  // KODAK GOLD Plus 100 Gen 5
	520, 83, 14, 0,  // KODAK GOLD 800 Gen 1
	521, 83, 13, 0,  // KODAK GOLD Super 200 Gen 5
	522, 91, 10, 0,  // KODAK EKTAPRESS PLUS 200 Prof
	523, 52, 73, 1,  // KODAK 4050 E6 auto-balance
	524, 52, 74, 1,  // KODAK 4050 E6 ilum. corr.
	525, 116, 72, 1, // KODAK 4050 K14
	526, 116, 73, 1, // KODAK 4050 K14 auto-balance
	527, 116, 74, 1, // KODAK 4050 K14 ilum. corr.
	528, 67, 72, 1,  // KODAK 4050 REVERSAL B&W
	532, 91, 2, 0,   // KODAK ADVANTIX 200
	533, 91, 3, 0,   // KODAK ADVANTIX 400
	534, 91, 1, 0,   // KODAK ADVANTIX 100
	535, 78, 8, 0,   // KODAK EKTAPRESS Multspd Prof PJM-2
	536, 79, 2, 0,   // KODAK KODACOLOR VR 200 Gen 5
	537, 79, 2, 0,   // KODAK FUNTIME 200 FB Gen 2
	538, 79, 2, 0,   // KODAK Commercial 200
	539, 132, 1, 0,  // KODAK Royal Gold 25 Copystand
	540, 78, 1, 0,   // KODAK KODACOLOR DA 100 Gen 5
	545, 79, 4, 0,   // KODAK KODACOLOR VR 400 Gen 2
	546, 78, 1, 0,   // KODAK GOLD 100 Gen 6
	547, 78, 2, 0,   // KODAK GOLD 200 Gen 6
	548, 78, 3, 0,   // KODAK GOLD 400 Gen 6
	549, 78, 4, 0,   // KODAK ROYAL GOLD 100 Gen 2
	550, 78, 5, 0,   // KODAK ROYAL GOLD 200 Gen 2
	551, 78, 6, 0,   // KODAK ROYAL GOLD 400 Gen 2
	552, 78, 7, 0,   // KODAK GOLD MAX 800 GEN 2
	554, 52, 75, 1,  // KODAK 4050 E6 high contrast
	555, 52, 76, 1,  // KODAK 4050 E6 low saturation high contrast
	556, 52, 77, 1,  // KODAK 4050 E6 low saturation
	557, 52, 78, 1,  // KODAK Universal E-6 Low Saturation
	558, 78, -1, 9,  // KODAK T-MAX T400 CN
	563, 78, 4, 0,   // KODAK EKTAPRESS PJ100
	564, 78, 6, 0,   // KODAK EKTAPRESS PJ400
	565, 78, 7, 0,   // KODAK EKTAPRESS PJ800
	567, 79, 11, 0,  // KODAK PORTRA 160NC
	568, 79, 11, 0,  // KODAK PORTRA 160VC
	569, 79, 13, 0,  // KODAK PORTRA 400NC
	570, 79, 13, 0,  // KODAK PORTRA 400VC
	575, 91, 5, 0,   // KODAK ADVANTIX 100-2
	576, 91, 6, 0,   // KODAK ADVANTIX 200-2
	577, 94, 1, 9,   // KODAK ADVANTIX Black & White + 400
	578, 78, 15, 0,  // KODAK EKTAPRESS PJ800-2
};

static const char *PCDMediumNames[kMaxPCDFilms] = {
	"3M ScotchColor AT 100",
	"3M ScotchColor AT 200",
	"3M ScotchColor HR2 400",
	"3M Scotch HR 200 Gen 2",
	"3M Scotch HR 400 Gen 2",
	"AGFA AGFACOLOR XRS 400 Gen 1",
	"AGFA AGFACOLOR XRG/XRS 400",
	"AGFA AGFACOLOR XRG/XRS 200",
	"AGFA AGFACOLOR XRS 1000 Gen 2",
	"AGFA AGFACOLOR XRS 400 Gen 2",
	"AGFA AGFACOLOR XRS/XRC 100",
	"FUJI Reala 100 (JAPAN)",
	"FUJI Reala 100 Gen 1",
	"FUJI Reala 100 Gen 2",
	"FUJI SHR 400 Gen 2",
	"FUJI Super HG 100",
	"FUJI Super HG 1600 Gen 1",
	"FUJI Super HG 200",
	"FUJI Super HG 400",
	"FUJI Super HG 100 Gen 2",
	"FUJI Super HR 100 Gen 1",
	"FUJI Super HR 100 Gen 2",
	"FUJI Super HR 1600 Gen 2",
	"FUJI Super HR 200 Gen 1",
	"FUJI Super HR 200 Gen 2",
	"FUJI Super HR 400 Gen 1",
	"FUJI NSP 160S (PRO)",
	"KODAK KODACOLOR VR 100 Gen 2",
	"KODAK GOLD 400 Gen 3",
	"KODAK EKTAR 100 Gen 1",
	"KODAK EKTAR 1000 Gen 1",
	"KODAK EKTAR 125 Gen 1",
	"KODAK ROYAL GOLD 25 RZ",
	"KODAK GOLD 1600 Gen 1",
	"KODAK GOLD 200 Gen 2",
	"KODAK GOLD 400 Gen 2",
	"KODAK KODACOLOR VR 100 Gen 1",
	"KODAK KODACOLOR VR 1000 Gen 2",
	"KODAK KODACOLOR VR 1000 Gen 1",
	"KODAK KODACOLOR VR 200 Gen 1",
	"KODAK KODACOLOR VR 400 Gen 1",
	"KODAK KODACOLOR VR 200 Gen 2",
	"KODAK KODACOLOR VRG 100 Gen 1",
	"KODAK GOLD 100 Gen 2",
	"KODAK KODACOLOR VRG 200 Gen 1",
	"KODAK GOLD 400 Gen 1",
	"KODAK EKTACOLOR GOLD 160",
	"KODAK EKTAPRESS 1600 Gen 1 PPC",
	"KODAK EKTAPRESS GOLD 100 Gen 1 PPA",
	"KODAK EKTAPRESS GOLD 400 PPB-3",
	"KODAK EKTAR 25 Professional PHR",
	"KODAK T-MAX 100 Professional",
	"KODAK T-MAX 3200 Professional",
	"KODAK T-MAX 400 Professional",
	"KODAK VERICOLOR 400 Prof VPH",
	"KODAK VERICOLOR III Pro",
	"KONICA KONICA COLOR SR-G 3200",
	"KONICA KONICA COLOR SUPER SR100",
	"KONICA KONICA COLOR SUPER SR 400",
	"KODAK GOLD UNKNOWN",
	"KODAK UNKNOWN NEG A-",
	"KODAK EKTAR 100 Gen 2",
	"KODAK KODACOLOR CII",
	"KODAK KODACOLOR II",
	"KODAK GOLD Plus 200 Gen 3",
	"KODAK Internegative +10% Contrast",
	"AGFA AGFACOLOR Ultra 50",
	"FUJI NHG 400",
	"AGFA AGFACOLOR XRG 100",
	"KODAK GOLD Plus 100 Gen 3",
	"KONICA KONICA COLOR SUPER SR200 GEN 1",
	"KONICA KONICA COLOR SR-G 160",
	"AGFA AGFACOLOR OPTIMA 125",
	"AGFA AGFACOLOR PORTRAIT 160",
	"KODAK KODACOLOR VRG 400 Gen 1",
	"KODAK GOLD 200 Gen 1",
	"KODAK KODACOLOR VRG 100 Gen 2",
	"KODAK Internegative +20% Contrast",
	"KODAK Internegative +30% Contrast",
	"KODAK Internegative +40% Contrast",
	"KODAK TMAX-100 D-76 CI = .40",
	"KODAK TMAX-100 D-76 CI = .50",
	"KODAK TMAX-100 D-76 CI = .55",
	"KODAK TMAX-100 D-76 CI = .70",
	"KODAK TMAX-100 D-76 CI = .80",
	"KODAK TMAX-100 TMAX CI = .40",
	"KODAK TMAX-100 TMAX CI = .50",
	"KODAK TMAX-100 TMAX CI = .55",
	"KODAK TMAX-100 TMAX CI = .70",
	"KODAK TMAX-100 TMAX CI = .80",
	"KODAK TMAX-400 D-76 CI = .40",
	"KODAK TMAX-400 D-76 CI = .50",
	"KODAK TMAX-400 D-76 CI = .55",
	"KODAK TMAX-400 D-76 CI = .70",
	"KODAK TMAX-400 D-76 CI = .80",
	"KODAK TMAX-400 TMAX CI = .40",
	"KODAK TMAX-400 TMAX CI = .50",
	"KODAK TMAX-400 TMAX CI = .55",
	"KODAK TMAX-400 TMAX CI = .70",
	"KODAK TMAX-400 TMAX CI = .80",
	"3M ScotchColor ATG 400/EXL 400",
	"AGFA AGFACOLOR OPTIMA 200",
	"KONICA IMPRESSA 50",
	"POLAROID POLAROID CP 200",
	"KONICA KONICA COLOR SUPER SR200 GEN 2",
	"ILFORD XP2 400",
	"POLAROID POLAROID COLOR HD2 100",
	"POLAROID POLAROID COLOR HD2 400",
	"POLAROID POLAROID COLOR HD2 200",
	"3M ScotchColor ATG-1 200",
	"KONICA XG 400",
	"KODAK UNIVERSAL REVERSAL B / W",
	"KODAK RPC COPY FILM Gen 1",
	"KODAK UNIVERSAL E6",
	"KODAK GOLD Ultra 400 Gen 4",
	"FUJI Super G 100",
	"FUJI Super G 200",
	"FUJI Super G 400 Gen 2",
	"KODAK UNIVERSAL K14",
	"FUJI Super G 400 Gen 1",
	"KODAK VERICOLOR HC 6329 VHC",
	"KODAK VERICOLOR HC 4329 VHC",
	"KODAK VERICOLOR L 6013 VPL",
	"KODAK VERICOLOR L 4013 VPL",
	"KODAK EKTACOLOR Gold II 400 Prof",
	"KODAK ROYAL GOLD 1000",
	"KODAK KODACOLOR VR 200 / 5093",
	"KODAK GOLD Plus 100 Gen 4",
	"KODAK ROYAL GOLD 100",
	"KODAK ROYAL GOLD 400",
	"KODAK UNIVERSAL E6 auto-balance",
	"KODAK UNIVERSAL E6 illum. corr.",
	"KODAK UNIVERSAL K14 auto-balance",
	"KODAK UNIVERSAL K14 illum. corr.",
	"KODAK EKTAR 100 Gen 3 SY",
	"KODAK EKTAR 25",
	"KODAK EKTAR 100 Gen 3 CX",
	"KODAK EKTAPRESS PLUS 100 Prof PJA-1",
	"KODAK EKTAPRESS GOLD II 100 Prof",
	"KODAK Pro 100 PRN",
	"KODAK VERICOLOR HC 100 Prof VHC-2",
	"KODAK Prof Color Neg 100",
	"KODAK EKTAR 1000 Gen 2",
	"KODAK EKTAPRESS PLUS 1600 Pro PJC-1",
	"KODAK EKTAPRESS GOLD II 1600 Prof",
	"KODAK SUPER GOLD 1600 GF Gen 2",
	"KODAK KODACOLOR 100 Print Gen 4",
	"KODAK SUPER GOLD 100 Gen 4",
	"KODAK GOLD 100 Gen 4",
	"KODAK GOLD III 100 Gen 4",
	"KODAK FUNTIME 100 FA",
	"KODAK FUNTIME 200 FB",
	"KODAK KODACOLOR VR 200 Gen 4",
	"KODAK GOLD Super 200 Gen 4",
	"KODAK KODACOLOR 200 Print Gen 4",
	"KODAK SUPER GOLD 200 Gen 4",
	"KODAK GOLD 200 Gen 4",
	"KODAK GOLD III 200 Gen 4",
	"KODAK GOLD Ultra 400 Gen 5",
	"KODAK SUPER GOLD 400 Gen 5",
	"KODAK GOLD 400 Gen 5",
	"KODAK GOLD III 400 Gen 5",
	"KODAK KODACOLOR 400 Print Gen 5",
	"KODAK EKTAPRESS PLUS 400 Prof PJB-2",
	"KODAK EKTAPRESS GOLD II 400 Prof G5",
	"KODAK Pro 400 PPF-2",
	"KODAK EKTACOLOR GOLD II 400 EGP-4",
	"KODAK EKTACOLOR GOLD 400 Prof EGP-4",
	"KODAK EKTAPRESS GOLD II Multspd PJM",
	"KODAK Pro 400 MC PMC",
	"KODAK VERICOLOR 400 Prof VPH-2",
	"KODAK VERICOLOR 400 PLUS Prof VPH-2",
	"KODAK UNKNOWN NEG Product Code 83",
	"KODAK EKTACOLOR PRO GOLD 160 GPX",
	"KODAK ROYAL GOLD 200",
	"KODAK 4050000000",
	"KODAK GOLD Plus 100 Gen 5",
	"KODAK GOLD 800 Gen 1",
	"KODAK GOLD Super 200 Gen 5",
	"KODAK EKTAPRESS PLUS 200 Prof",
	"KODAK 4050 E6 auto-balance",
	"KODAK 4050 E6 ilum. corr.",
	"KODAK 4050 K14",
	"KODAK 4050 K14 auto-balance",
	"KODAK 4050 K14 ilum. corr.",
	"KODAK 4050 REVERSAL B&W",
	"KODAK ADVANTIX 200",
	"KODAK ADVANTIX 400",
	"KODAK ADVANTIX 100",
	"KODAK EKTAPRESS Multspd Prof PJM-2",
	"KODAK KODACOLOR VR 200 Gen 5",
	"KODAK FUNTIME 200 FB Gen 2",
	"KODAK Commercial 200",
	"KODAK Royal Gold 25 Copystand",
	"KODAK KODACOLOR DA 100 Gen 5",
	"KODAK KODACOLOR VR 400 Gen 2",
	"KODAK GOLD 100 Gen 6",
	"KODAK GOLD 200 Gen 6",
	"KODAK GOLD 400 Gen 6",
	"KODAK ROYAL GOLD 100 Gen 2",
	"KODAK ROYAL GOLD 200 Gen 2",
	"KODAK ROYAL GOLD 400 Gen 2",
	"KODAK GOLD MAX 800 GEN 2",
	"KODAK 4050 E6 high contrast",
	"KODAK 4050 E6 low saturation high contrast",
	"KODAK 4050 E6 low saturation",
	"KODAK Universal E-6 Low Saturation",
	"KODAK T-MAX T400 CN",
	"KODAK EKTAPRESS PJ100",
	"KODAK EKTAPRESS PJ400",
	"KODAK EKTAPRESS PJ800",
	"KODAK PORTRA 160NC",
	"KODAK PORTRA 160VC",
	"KODAK PORTRA 400NC",
	"KODAK PORTRA 400VC",
	"KODAK ADVANTIX 100-2",
	"KODAK ADVANTIX 200-2",
	"KODAK ADVANTIX Black & White + 400",
	"KODAK EKTAPRESS PJ800-2",
};

static const char *PCDMetadataDescriptions[kMaxPCDMetadata] = {
	"PCD specification version",	
	"Authoring software Release number",		
	"Scanning time",	
	"Last modification time",
	"Image medium",
	"Product type",							
	"Scanner vendor identity",					
	"Scanner product identity",				
	"Scanner firmware revision",				
	"Scanner firmware date",					
	"Scanner serial number",					
	"Scanner pixel size (microns)",						
	"Image workstation equipment manufacturer",				
	"Photo finisher name",	
	"Scene balance algorithm revision",	
	"Scene balance algorithm command",
	"Scene balance algorithm film identification",		
	"Copyright status",
	"Copyright file name",
	"Compression",
};

//////////////////////////////////////////////////////////////
//
// Look-up tables for the color space conversions 
//
//////////////////////////////////////////////////////////////
//

enum LUTValues {
	numLUTItems = 1389,
};

// Used if we are generating the tables
//static uint16_t toLinearLight[numLUTItems];
//static  uint16_t CCIR709tosRGB[numLUTItems];
//static  uint8_t	uint8Output[numLUTItems];
//static  uint16_t	uint16Output[numLUTItems];
//static  float	floatOutput[numLUTItems];

static const uint16_t toLinearLight[numLUTItems] = {
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 
	0x0001, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003, 0x0003, 0x0003, 0x0003, 
	0x0003, 0x0004, 0x0004, 0x0004, 0x0004, 0x0004, 0x0004, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 
	0x0005, 0x0005, 0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x0007, 0x0007, 0x0007, 0x0007, 
	0x0007, 0x0007, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0009, 0x0009, 0x0009, 0x0009, 
	0x0009, 0x0009, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000b, 0x000b, 0x000b, 
	0x000b, 0x000b, 0x000b, 0x000c, 0x000c, 0x000c, 0x000c, 0x000c, 0x000c, 0x000d, 0x000d, 0x000d, 
	0x000d, 0x000d, 0x000d, 0x000e, 0x000e, 0x000e, 0x000e, 0x000e, 0x000e, 0x000f, 0x000f, 0x000f, 
	0x000f, 0x000f, 0x000f, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0011, 0x0011, 0x0011, 
	0x0011, 0x0011, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 
	0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0016, 0x0016, 
	0x0016, 0x0016, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0018, 0x0018, 0x0018, 0x0018, 0x0019, 
	0x0019, 0x0019, 0x0019, 0x0019, 0x001a, 0x001a, 0x001a, 0x001a, 0x001b, 0x001b, 0x001b, 0x001b, 
	0x001c, 0x001c, 0x001c, 0x001c, 0x001d, 0x001d, 0x001d, 0x001d, 0x001e, 0x001e, 0x001e, 0x001e, 
	0x001f, 0x001f, 0x001f, 0x001f, 0x0020, 0x0020, 0x0020, 0x0020, 0x0021, 0x0021, 0x0021, 0x0021, 
	0x0022, 0x0022, 0x0022, 0x0023, 0x0023, 0x0023, 0x0023, 0x0024, 0x0024, 0x0024, 0x0024, 0x0025, 
	0x0025, 0x0025, 0x0026, 0x0026, 0x0026, 0x0026, 0x0027, 0x0027, 0x0027, 0x0028, 0x0028, 0x0028, 
	0x0029, 0x0029, 0x0029, 0x0029, 0x002a, 0x002a, 0x002a, 0x002b, 0x002b, 0x002b, 0x002c, 0x002c, 
	0x002c, 0x002d, 0x002d, 0x002d, 0x002d, 0x002e, 0x002e, 0x002e, 0x002f, 0x002f, 0x002f, 0x0030, 
	0x0030, 0x0030, 0x0031, 0x0031, 0x0031, 0x0032, 0x0032, 0x0032, 0x0033, 0x0033, 0x0033, 0x0034, 
	0x0034, 0x0034, 0x0035, 0x0035, 0x0035, 0x0036, 0x0036, 0x0036, 0x0037, 0x0037, 0x0038, 0x0038, 
	0x0038, 0x0039, 0x0039, 0x0039, 0x003a, 0x003a, 0x003a, 0x003b, 0x003b, 0x003c, 0x003c, 0x003c, 
	0x003d, 0x003d, 0x003d, 0x003e, 0x003e, 0x003f, 0x003f, 0x003f, 0x0040, 0x0040, 0x0040, 0x0041, 
	0x0041, 0x0042, 0x0042, 0x0042, 0x0043, 0x0043, 0x0044, 0x0044, 0x0044, 0x0045, 0x0045, 0x0046, 
	0x0046, 0x0046, 0x0047, 0x0047, 0x0048, 0x0048, 0x0048, 0x0049, 0x0049, 0x004a, 0x004a, 0x004a, 
	0x004b, 0x004b, 0x004c, 0x004c, 0x004d, 0x004d, 0x004d, 0x004e, 0x004e, 0x004f, 0x004f, 0x004f, 
	0x0050, 0x0050, 0x0051, 0x0051, 0x0052, 0x0052, 0x0053, 0x0053, 0x0053, 0x0054, 0x0054, 0x0055, 
	0x0055, 0x0056, 0x0056, 0x0057, 0x0057, 0x0057, 0x0058, 0x0058, 0x0059, 0x0059, 0x005a, 0x005a, 
	0x005b, 0x005b, 0x005c, 0x005c, 0x005d, 0x005d, 0x005d, 0x005e, 0x005e, 0x005f, 0x005f, 0x0060, 
	0x0060, 0x0061, 0x0061, 0x0062, 0x0062, 0x0063, 0x0063, 0x0064, 0x0064, 0x0065, 0x0065, 0x0066, 
	0x0066, 0x0067, 0x0067, 0x0068, 0x0068, 0x0069, 0x0069, 0x006a, 0x006a, 0x006b, 0x006b, 0x006c, 
	0x006c, 0x006d, 0x006d, 0x006e, 0x006e, 0x006f, 0x006f, 0x0070, 0x0070, 0x0071, 0x0071, 0x0072, 
	0x0072, 0x0073, 0x0073, 0x0074, 0x0075, 0x0075, 0x0076, 0x0076, 0x0077, 0x0077, 0x0078, 0x0078, 
	0x0079, 0x0079, 0x007a, 0x007a, 0x007b, 0x007c, 0x007c, 0x007d, 0x007d, 0x007e, 0x007e, 0x007f, 
	0x007f, 0x0080, 0x0081, 0x0081, 0x0082, 0x0082, 0x0083, 0x0083, 0x0084, 0x0084, 0x0085, 0x0086, 
	0x0086, 0x0087, 0x0087, 0x0088, 0x0089, 0x0089, 0x008a, 0x008a, 0x008b, 0x008b, 0x008c, 0x008d, 
	0x008d, 0x008e, 0x008e, 0x008f, 0x0090, 0x0090, 0x0091, 0x0091, 0x0092, 0x0093, 0x0093, 0x0094, 
	0x0094, 0x0095, 0x0096, 0x0096, 0x0097, 0x0097, 0x0098, 0x0099, 0x0099, 0x009a, 0x009b, 0x009b, 
	0x009c, 0x009c, 0x009d, 0x009e, 0x009e, 0x009f, 0x00a0, 0x00a0, 0x00a1, 0x00a1, 0x00a2, 0x00a3, 
	0x00a3, 0x00a4, 0x00a5, 0x00a5, 0x00a6, 0x00a7, 0x00a7, 0x00a8, 0x00a8, 0x00a9, 0x00aa, 0x00aa, 
	0x00ab, 0x00ac, 0x00ac, 0x00ad, 0x00ae, 0x00ae, 0x00af, 0x00b0, 0x00b0, 0x00b1, 0x00b2, 0x00b2, 
	0x00b3, 0x00b4, 0x00b4, 0x00b5, 0x00b6, 0x00b6, 0x00b7, 0x00b8, 0x00b8, 0x00b9, 0x00ba, 0x00bb, 
	0x00bb, 0x00bc, 0x00bd, 0x00bd, 0x00be, 0x00bf, 0x00bf, 0x00c0, 0x00c1, 0x00c1, 0x00c2, 0x00c3, 
	0x00c4, 0x00c4, 0x00c5, 0x00c6, 0x00c6, 0x00c7, 0x00c8, 0x00c9, 0x00c9, 0x00ca, 0x00cb, 0x00cb, 
	0x00cc, 0x00cd, 0x00ce, 0x00ce, 0x00cf, 0x00d0, 0x00d1, 0x00d1, 0x00d2, 0x00d3, 0x00d3, 0x00d4, 
	0x00d5, 0x00d6, 0x00d6, 0x00d7, 0x00d8, 0x00d9, 0x00d9, 0x00da, 0x00db, 0x00dc, 0x00dc, 0x00dd, 
	0x00de, 0x00df, 0x00df, 0x00e0, 0x00e1, 0x00e2, 0x00e2, 0x00e3, 0x00e4, 0x00e5, 0x00e6, 0x00e6, 
	0x00e7, 0x00e8, 0x00e9, 0x00e9, 0x00ea, 0x00eb, 0x00ec, 0x00ed, 0x00ed, 0x00ee, 0x00ef, 0x00f0, 
	0x00f0, 0x00f1, 0x00f2, 0x00f3, 0x00f4, 0x00f4, 0x00f5, 0x00f6, 0x00f7, 0x00f8, 0x00f8, 0x00f9, 
	0x00fa, 0x00fb, 0x00fc, 0x00fd, 0x00fd, 0x00fe, 0x00ff, 0x0100, 0x0101, 0x0101, 0x0102, 0x0103, 
	0x0104, 0x0105, 0x0106, 0x0106, 0x0107, 0x0108, 0x0109, 0x010a, 0x010b, 0x010b, 0x010c, 0x010d, 
	0x010e, 0x010f, 0x0110, 0x0110, 0x0111, 0x0112, 0x0113, 0x0114, 0x0115, 0x0116, 0x0116, 0x0117, 
	0x0118, 0x0119, 0x011a, 0x011b, 0x011c, 0x011c, 0x011d, 0x011e, 0x011f, 0x0120, 0x0121, 0x0122, 
	0x0123, 0x0123, 0x0124, 0x0125, 0x0126, 0x0127, 0x0128, 0x0129, 0x012a, 0x012a, 0x012b, 0x012c, 
	0x012d, 0x012e, 0x012f, 0x0130, 0x0131, 0x0132, 0x0133, 0x0133, 0x0134, 0x0135, 0x0136, 0x0137, 
	0x0138, 0x0139, 0x013a, 0x013b, 0x013c, 0x013d, 0x013d, 0x013e, 0x013f, 0x0140, 0x0141, 0x0142, 
	0x0143, 0x0144, 0x0145, 0x0146, 0x0147, 0x0148, 0x0149, 0x014a, 0x014b, 0x014b, 0x014c, 0x014d, 
	0x014e, 0x014f, 0x0150, 0x0151, 0x0152, 0x0153, 0x0154, 0x0155, 0x0156, 0x0157, 0x0158, 0x0159, 
	0x015a, 0x015b, 0x015c, 0x015d, 0x015e, 0x015f, 0x0160, 0x0161, 0x0162, 0x0163, 0x0163, 0x0164, 
	0x0165, 0x0166, 0x0167, 0x0168, 0x0169, 0x016a, 0x016b, 0x016c, 0x016d, 0x016e, 0x016f, 0x0170, 
	0x0171, 0x0172, 0x0173, 0x0174, 0x0175, 0x0176, 0x0177, 0x0178, 0x0179, 0x017a, 0x017b, 0x017c, 
	0x017d, 0x017e, 0x0180, 0x0181, 0x0182, 0x0183, 0x0184, 0x0185, 0x0186, 0x0187, 0x0188, 0x0189, 
	0x018a, 0x018b, 0x018c, 0x018d, 0x018e, 0x018f, 0x0190, 0x0191, 0x0192, 0x0193, 0x0194, 0x0195, 
	0x0196, 0x0197, 0x0198, 0x019a, 0x019b, 0x019c, 0x019d, 0x019e, 0x019f, 0x01a0, 0x01a1, 0x01a2, 
	0x01a3, 0x01a4, 0x01a5, 0x01a6, 0x01a7, 0x01a8, 0x01aa, 0x01ab, 0x01ac, 0x01ad, 0x01ae, 0x01af, 
	0x01b0, 0x01b1, 0x01b2, 0x01b3, 0x01b4, 0x01b6, 0x01b7, 0x01b8, 0x01b9, 0x01ba, 0x01bb, 0x01bc, 
	0x01bd, 0x01be, 0x01bf, 0x01c1, 0x01c2, 0x01c3, 0x01c4, 0x01c5, 0x01c6, 0x01c7, 0x01c8, 0x01ca, 
	0x01cb, 0x01cc, 0x01cd, 0x01ce, 0x01cf, 0x01d0, 0x01d1, 0x01d3, 0x01d4, 0x01d5, 0x01d6, 0x01d7, 
	0x01d8, 0x01d9, 0x01db, 0x01dc, 0x01dd, 0x01de, 0x01df, 0x01e0, 0x01e2, 0x01e3, 0x01e4, 0x01e5, 
	0x01e6, 0x01e7, 0x01e9, 0x01ea, 0x01eb, 0x01ec, 0x01ed, 0x01ee, 0x01f0, 0x01f1, 0x01f2, 0x01f3, 
	0x01f4, 0x01f5, 0x01f7, 0x01f8, 0x01f9, 0x01fa, 0x01fb, 0x01fd, 0x01fe, 0x01ff, 0x0200, 0x0201, 
	0x0203, 0x0204, 0x0205, 0x0206, 0x0207, 0x0209, 0x020a, 0x020b, 0x020c, 0x020d, 0x020f, 0x0210, 
	0x0211, 0x0212, 0x0214, 0x0215, 0x0216, 0x0217, 0x0218, 0x021a, 0x021b, 0x021c, 0x021d, 0x021f, 
	0x0220, 0x0221, 0x0222, 0x0224, 0x0225, 0x0226, 0x0227, 0x0229, 0x022a, 0x022b, 0x022c, 0x022e, 
	0x022f, 0x0230, 0x0231, 0x0233, 0x0234, 0x0235, 0x0236, 0x0238, 0x0239, 0x023a, 0x023b, 0x023d, 
	0x023e, 0x023f, 0x0241, 0x0242, 0x0243, 0x0244, 0x0246, 0x0247, 0x0248, 0x0249, 0x024b, 0x024c, 
	0x024d, 0x024f, 0x0250, 0x0251, 0x0253, 0x0254, 0x0255, 0x0256, 0x0258, 0x0259, 0x025a, 0x025c, 
	0x025d, 0x025e, 0x0260, 0x0261, 0x0262, 0x0264, 0x0265, 0x0266, 0x0268, 0x0269, 0x026a, 0x026c, 
	0x026d, 0x026e, 0x0270, 0x0271, 0x0272, 0x0274, 0x0275, 0x0276, 0x0278, 0x0279, 0x027a, 0x027c, 
	0x027d, 0x027e, 0x0280, 0x0281, 0x0282, 0x0284, 0x0285, 0x0286, 0x0288, 0x0289, 0x028b, 0x028c, 
	0x028d, 0x028f, 0x0290, 0x0291, 0x0293, 0x0294, 0x0295, 0x0297, 0x0298, 0x029a, 0x029b, 0x029c, 
	0x029e, 0x029f, 0x02a1, 0x02a2, 0x02a3, 0x02a5, 0x02a6, 0x02a8, 0x02a9, 0x02aa, 0x02ac, 0x02ad, 
	0x02af, 0x02b0, 0x02b1, 0x02b3, 0x02b4, 0x02b6, 0x02b7, 0x02b8, 0x02ba, 0x02bb, 0x02bd, 0x02be, 
	0x02c0, 0x02c1, 0x02c2, 0x02c4, 0x02c5, 0x02c7, 0x02c8, 0x02ca, 0x02cb, 0x02cc, 0x02ce, 0x02cf, 
	0x02d1, 0x02d2, 0x02d4, 0x02d5, 0x02d7, 0x02d8, 0x02d9, 0x02db, 0x02dc, 0x02de, 0x02df, 0x02e1, 
	0x02e2, 0x02e4, 0x02e5, 0x02e7, 0x02e8, 0x02ea, 0x02eb, 0x02ed, 0x02ee, 0x02ef, 0x02f1, 0x02f2, 
	0x02f4, 0x02f5, 0x02f7, 0x02f8, 0x02fa, 0x02fb, 0x02fd, 0x02fe, 0x0300, 0x0301, 0x0303, 0x0304, 
	0x0306, 0x0307, 0x0309, 0x030a, 0x030c, 0x030d, 0x030f, 0x0310, 0x0312, 0x0313, 0x0315, 0x0316, 
	0x0318, 0x0319, 0x031b, 0x031d, 0x031e, 0x0320, 0x0321, 0x0323, 0x0324, 0x0326, 0x0327, 0x0329, 
	0x032a, 0x032c, 0x032d, 0x032f, 0x0331, 0x0332, 0x0334, 0x0335, 0x0337, 0x0338, 0x033a, 0x033b, 
	0x033d, 0x033f, 0x0340, 0x0342, 0x0343, 0x0345, 0x0346, 0x0348, 0x0349, 0x034b, 0x034d, 0x034e, 
	0x0350, 0x0351, 0x0353, 0x0355, 0x0356, 0x0358, 0x0359, 0x035b, 0x035c, 0x035e, 0x0360, 0x0361, 
	0x0363, 0x0364, 0x0366, 0x0368, 0x0369, 0x036b, 0x036c, 0x036e, 0x0370, 0x0371, 0x0373, 0x0375, 
	0x0376, 0x0378, 0x0379, 0x037b, 0x037d, 0x037e, 0x0380, 0x0382, 0x0383, 0x0385, 0x0386, 0x0388, 
	0x038a, 0x038b, 0x038d, 0x038f, 0x0390, 0x0392, 0x0394, 0x0395, 0x0397, 0x0399, 0x039a, 0x039c, 
	0x039d, 0x039f, 0x03a1, 0x03a2, 0x03a4, 0x03a6, 0x03a7, 0x03a9, 0x03ab, 0x03ac, 0x03ae, 0x03b0, 
	0x03b1, 0x03b3, 0x03b5, 0x03b7, 0x03b8, 0x03ba, 0x03bc, 0x03bd, 0x03bf, 0x03c1, 0x03c2, 0x03c4, 
	0x03c6, 0x03c7, 0x03c9, 0x03cb, 0x03cd, 0x03ce, 0x03d0, 0x03d2, 0x03d3, 0x03d5, 0x03d7, 0x03d9, 
	0x03da, 0x03dc, 0x03de, 0x03df, 0x03e1, 0x03e3, 0x03e5, 0x03e6, 0x03e8, 0x03ea, 0x03eb, 0x03ed, 
	0x03ef, 0x03f1, 0x03f2, 0x03f4, 0x03f6, 0x03f8, 0x03f9, 0x03fb, 0x03fd, 0x03ff, 0x0400, 0x0402, 
	0x0404, 0x0406, 0x0407, 0x0409, 0x040b, 0x040d, 0x040e, 0x0410, 0x0412, 0x0414, 0x0416, 0x0417, 
	0x0419, 0x041b, 0x041d, 0x041e, 0x0420, 0x0422, 0x0424, 0x0426, 0x0427, 0x0429, 0x042b, 0x042d, 
	0x042f, 0x0430, 0x0432, 0x0434, 0x0436, 0x0438, 0x0439, 0x043b, 0x043d, 0x043f, 0x0441, 0x0442, 
	0x0444, 0x0446, 0x0448, 0x044a, 0x044b, 0x044d, 0x044f, 0x0451, 0x0453, 0x0455, 0x0456, 0x0458, 
	0x045a, 0x045c, 0x045e, 0x0460, 0x0461, 0x0463, 0x0465, 0x0467, 0x0469, 0x046b, 0x046d, 0x046e, 
	0x0470, 0x0472, 0x0474, 0x0476, 0x0478, 0x047a, 0x047b, 0x047d, 0x047f, 0x0481, 0x0483, 0x0485, 
	0x0487, 0x0488, 0x048a, 0x048c, 0x048e, 0x0490, 0x0492, 0x0494, 0x0496, 0x0498, 0x0499, 0x049b, 
	0x049d, 0x049f, 0x04a1, 0x04a3, 0x04a5, 0x04a7, 0x04a9, 0x04ab, 0x04ac, 0x04ae, 0x04b0, 0x04b2, 
	0x04b4, 0x04b6, 0x04b8, 0x04ba, 0x04bc, 0x04be, 0x04c0, 0x04c2, 0x04c3, 0x04c5, 0x04c7, 0x04c9, 
	0x04cb, 0x04cd, 0x04cf, 0x04d1, 0x04d3, 0x04d5, 0x04d7, 0x04d9, 0x04db, 0x04dd, 0x04df, 0x04e1, 
	0x04e3, 0x04e5, 0x04e6, 0x04e8, 0x04ea, 0x04ec, 0x04ee, 0x04f0, 0x04f2, 0x04f4, 0x04f6, 0x04f8, 
	0x04fa, 0x04fc, 0x04fe, 0x0500, 0x0502, 0x0504, 0x0506, 0x0508, 0x050a, 0x050c, 0x050e, 0x0510, 
	0x0512, 0x0514, 0x0516, 0x0518, 0x051a, 0x051c, 0x051e, 0x0520, 0x0522, 0x0524, 0x0526, 0x0528, 
	0x052a, 0x052c, 0x052e, 0x0530, 0x0532, 0x0534, 0x0536, 0x0538, 0x053a, 0x053c, 0x053e, 0x0540, 
	0x0542, 0x0544, 0x0546, 0x0549, 0x054b, 0x054d, 0x054f, 0x0551, 0x0553, 0x0555, 0x0557, 0x0559, 
	0x055b, 0x055d, 0x055f, 0x0561, 0x0563, 0x0565, 0x0567, 0x0569, 0x056b
};
static const uint16_t CCIR709tosRGB[numLUTItems] = {
	0x0000, 0x000c, 0x0019, 0x0026, 0x0033, 0x0040, 0x004b, 0x0055, 0x005e, 0x0067, 0x006f, 0x0076, 
	0x007d, 0x0084, 0x008b, 0x0091, 0x0097, 0x009d, 0x00a3, 0x00a8, 0x00ad, 0x00b3, 0x00b8, 0x00bc, 
	0x00c1, 0x00c6, 0x00ca, 0x00cf, 0x00d3, 0x00d7, 0x00db, 0x00e0, 0x00e4, 0x00e7, 0x00eb, 0x00ef, 
	0x00f3, 0x00f7, 0x00fa, 0x00fe, 0x0101, 0x0105, 0x0108, 0x010b, 0x010f, 0x0112, 0x0115, 0x0118, 
	0x011c, 0x011f, 0x0122, 0x0125, 0x0128, 0x012b, 0x012e, 0x0131, 0x0134, 0x0136, 0x0139, 0x013c, 
	0x013f, 0x0141, 0x0144, 0x0147, 0x0149, 0x014c, 0x014f, 0x0151, 0x0154, 0x0156, 0x0159, 0x015b, 
	0x015e, 0x0160, 0x0163, 0x0165, 0x0168, 0x016a, 0x016c, 0x016f, 0x0171, 0x0173, 0x0176, 0x0178, 
	0x017a, 0x017c, 0x017f, 0x0181, 0x0183, 0x0185, 0x0188, 0x018a, 0x018c, 0x018e, 0x0190, 0x0192, 
	0x0194, 0x0196, 0x0198, 0x019a, 0x019d, 0x019f, 0x01a1, 0x01a3, 0x01a5, 0x01a7, 0x01a9, 0x01ab, 
	0x01ac, 0x01ae, 0x01b0, 0x01b2, 0x01b4, 0x01b6, 0x01b8, 0x01ba, 0x01bc, 0x01be, 0x01bf, 0x01c1, 
	0x01c3, 0x01c5, 0x01c7, 0x01c9, 0x01ca, 0x01cc, 0x01ce, 0x01d0, 0x01d2, 0x01d3, 0x01d5, 0x01d7, 
	0x01d9, 0x01da, 0x01dc, 0x01de, 0x01df, 0x01e1, 0x01e3, 0x01e5, 0x01e6, 0x01e8, 0x01ea, 0x01eb, 
	0x01ed, 0x01ee, 0x01f0, 0x01f2, 0x01f3, 0x01f5, 0x01f7, 0x01f8, 0x01fa, 0x01fb, 0x01fd, 0x01ff, 
	0x0200, 0x0202, 0x0203, 0x0205, 0x0206, 0x0208, 0x0209, 0x020b, 0x020d, 0x020e, 0x0210, 0x0211, 
	0x0213, 0x0214, 0x0216, 0x0217, 0x0219, 0x021a, 0x021c, 0x021d, 0x021f, 0x0220, 0x0221, 0x0223, 
	0x0224, 0x0226, 0x0227, 0x0229, 0x022a, 0x022c, 0x022d, 0x022e, 0x0230, 0x0231, 0x0233, 0x0234, 
	0x0235, 0x0237, 0x0238, 0x023a, 0x023b, 0x023c, 0x023e, 0x023f, 0x0240, 0x0242, 0x0243, 0x0244, 
	0x0246, 0x0247, 0x0248, 0x024a, 0x024b, 0x024c, 0x024e, 0x024f, 0x0250, 0x0252, 0x0253, 0x0254, 
	0x0256, 0x0257, 0x0258, 0x025a, 0x025b, 0x025c, 0x025d, 0x025f, 0x0260, 0x0261, 0x0263, 0x0264, 
	0x0265, 0x0266, 0x0268, 0x0269, 0x026a, 0x026b, 0x026d, 0x026e, 0x026f, 0x0270, 0x0272, 0x0273, 
	0x0274, 0x0275, 0x0276, 0x0278, 0x0279, 0x027a, 0x027b, 0x027c, 0x027e, 0x027f, 0x0280, 0x0281, 
	0x0282, 0x0284, 0x0285, 0x0286, 0x0287, 0x0288, 0x028a, 0x028b, 0x028c, 0x028d, 0x028e, 0x028f, 
	0x0291, 0x0292, 0x0293, 0x0294, 0x0295, 0x0296, 0x0297, 0x0299, 0x029a, 0x029b, 0x029c, 0x029d, 
	0x029e, 0x029f, 0x02a0, 0x02a2, 0x02a3, 0x02a4, 0x02a5, 0x02a6, 0x02a7, 0x02a8, 0x02a9, 0x02aa, 
	0x02ac, 0x02ad, 0x02ae, 0x02af, 0x02b0, 0x02b1, 0x02b2, 0x02b3, 0x02b4, 0x02b5, 0x02b6, 0x02b8, 
	0x02b9, 0x02ba, 0x02bb, 0x02bc, 0x02bd, 0x02be, 0x02bf, 0x02c0, 0x02c1, 0x02c2, 0x02c3, 0x02c4, 
	0x02c5, 0x02c6, 0x02c7, 0x02c9, 0x02ca, 0x02cb, 0x02cc, 0x02cd, 0x02ce, 0x02cf, 0x02d0, 0x02d1, 
	0x02d2, 0x02d3, 0x02d4, 0x02d5, 0x02d6, 0x02d7, 0x02d8, 0x02d9, 0x02da, 0x02db, 0x02dc, 0x02dd, 
	0x02de, 0x02df, 0x02e0, 0x02e1, 0x02e2, 0x02e3, 0x02e4, 0x02e5, 0x02e6, 0x02e7, 0x02e8, 0x02e9, 
	0x02ea, 0x02eb, 0x02ec, 0x02ed, 0x02ee, 0x02ef, 0x02f0, 0x02f1, 0x02f2, 0x02f3, 0x02f4, 0x02f5, 
	0x02f6, 0x02f7, 0x02f8, 0x02f9, 0x02fa, 0x02fa, 0x02fb, 0x02fc, 0x02fd, 0x02fe, 0x02ff, 0x0300, 
	0x0301, 0x0302, 0x0303, 0x0304, 0x0305, 0x0306, 0x0307, 0x0308, 0x0309, 0x030a, 0x030b, 0x030b, 
	0x030c, 0x030d, 0x030e, 0x030f, 0x0310, 0x0311, 0x0312, 0x0313, 0x0314, 0x0315, 0x0316, 0x0317, 
	0x0317, 0x0318, 0x0319, 0x031a, 0x031b, 0x031c, 0x031d, 0x031e, 0x031f, 0x0320, 0x0321, 0x0321, 
	0x0322, 0x0323, 0x0324, 0x0325, 0x0326, 0x0327, 0x0328, 0x0329, 0x032a, 0x032a, 0x032b, 0x032c, 
	0x032d, 0x032e, 0x032f, 0x0330, 0x0331, 0x0331, 0x0332, 0x0333, 0x0334, 0x0335, 0x0336, 0x0337, 
	0x0338, 0x0338, 0x0339, 0x033a, 0x033b, 0x033c, 0x033d, 0x033e, 0x033e, 0x033f, 0x0340, 0x0341, 
	0x0342, 0x0343, 0x0344, 0x0344, 0x0345, 0x0346, 0x0347, 0x0348, 0x0349, 0x034a, 0x034a, 0x034b, 
	0x034c, 0x034d, 0x034e, 0x034f, 0x034f, 0x0350, 0x0351, 0x0352, 0x0353, 0x0354, 0x0354, 0x0355, 
	0x0356, 0x0357, 0x0358, 0x0359, 0x0359, 0x035a, 0x035b, 0x035c, 0x035d, 0x035e, 0x035e, 0x035f, 
	0x0360, 0x0361, 0x0362, 0x0362, 0x0363, 0x0364, 0x0365, 0x0366, 0x0366, 0x0367, 0x0368, 0x0369, 
	0x036a, 0x036a, 0x036b, 0x036c, 0x036d, 0x036e, 0x036f, 0x036f, 0x0370, 0x0371, 0x0372, 0x0372, 
	0x0373, 0x0374, 0x0375, 0x0376, 0x0376, 0x0377, 0x0378, 0x0379, 0x037a, 0x037a, 0x037b, 0x037c, 
	0x037d, 0x037e, 0x037e, 0x037f, 0x0380, 0x0381, 0x0381, 0x0382, 0x0383, 0x0384, 0x0385, 0x0385, 
	0x0386, 0x0387, 0x0388, 0x0388, 0x0389, 0x038a, 0x038b, 0x038b, 0x038c, 0x038d, 0x038e, 0x038f, 
	0x038f, 0x0390, 0x0391, 0x0392, 0x0392, 0x0393, 0x0394, 0x0395, 0x0395, 0x0396, 0x0397, 0x0398, 
	0x0398, 0x0399, 0x039a, 0x039b, 0x039b, 0x039c, 0x039d, 0x039e, 0x039e, 0x039f, 0x03a0, 0x03a1, 
	0x03a1, 0x03a2, 0x03a3, 0x03a4, 0x03a4, 0x03a5, 0x03a6, 0x03a7, 0x03a7, 0x03a8, 0x03a9, 0x03a9, 
	0x03aa, 0x03ab, 0x03ac, 0x03ac, 0x03ad, 0x03ae, 0x03af, 0x03af, 0x03b0, 0x03b1, 0x03b2, 0x03b2, 
	0x03b3, 0x03b4, 0x03b4, 0x03b5, 0x03b6, 0x03b7, 0x03b7, 0x03b8, 0x03b9, 0x03b9, 0x03ba, 0x03bb, 
	0x03bc, 0x03bc, 0x03bd, 0x03be, 0x03be, 0x03bf, 0x03c0, 0x03c1, 0x03c1, 0x03c2, 0x03c3, 0x03c3, 
	0x03c4, 0x03c5, 0x03c6, 0x03c6, 0x03c7, 0x03c8, 0x03c8, 0x03c9, 0x03ca, 0x03cb, 0x03cb, 0x03cc, 
	0x03cd, 0x03cd, 0x03ce, 0x03cf, 0x03cf, 0x03d0, 0x03d1, 0x03d2, 0x03d2, 0x03d3, 0x03d4, 0x03d4, 
	0x03d5, 0x03d6, 0x03d6, 0x03d7, 0x03d8, 0x03d8, 0x03d9, 0x03da, 0x03db, 0x03db, 0x03dc, 0x03dd, 
	0x03dd, 0x03de, 0x03df, 0x03df, 0x03e0, 0x03e1, 0x03e1, 0x03e2, 0x03e3, 0x03e3, 0x03e4, 0x03e5, 
	0x03e5, 0x03e6, 0x03e7, 0x03e7, 0x03e8, 0x03e9, 0x03ea, 0x03ea, 0x03eb, 0x03ec, 0x03ec, 0x03ed, 
	0x03ee, 0x03ee, 0x03ef, 0x03f0, 0x03f0, 0x03f1, 0x03f2, 0x03f2, 0x03f3, 0x03f4, 0x03f4, 0x03f5, 
	0x03f6, 0x03f6, 0x03f7, 0x03f8, 0x03f8, 0x03f9, 0x03fa, 0x03fa, 0x03fb, 0x03fc, 0x03fc, 0x03fd, 
	0x03fd, 0x03fe, 0x03ff, 0x03ff, 0x0400, 0x0401, 0x0401, 0x0402, 0x0403, 0x0403, 0x0404, 0x0405, 
	0x0405, 0x0406, 0x0407, 0x0407, 0x0408, 0x0409, 0x0409, 0x040a, 0x040b, 0x040b, 0x040c, 0x040c, 
	0x040d, 0x040e, 0x040e, 0x040f, 0x0410, 0x0410, 0x0411, 0x0412, 0x0412, 0x0413, 0x0414, 0x0414, 
	0x0415, 0x0415, 0x0416, 0x0417, 0x0417, 0x0418, 0x0419, 0x0419, 0x041a, 0x041b, 0x041b, 0x041c, 
	0x041c, 0x041d, 0x041e, 0x041e, 0x041f, 0x0420, 0x0420, 0x0421, 0x0421, 0x0422, 0x0423, 0x0423, 
	0x0424, 0x0425, 0x0425, 0x0426, 0x0426, 0x0427, 0x0428, 0x0428, 0x0429, 0x042a, 0x042a, 0x042b, 
	0x042b, 0x042c, 0x042d, 0x042d, 0x042e, 0x042f, 0x042f, 0x0430, 0x0430, 0x0431, 0x0432, 0x0432, 
	0x0433, 0x0434, 0x0434, 0x0435, 0x0435, 0x0436, 0x0437, 0x0437, 0x0438, 0x0438, 0x0439, 0x043a, 
	0x043a, 0x043b, 0x043b, 0x043c, 0x043d, 0x043d, 0x043e, 0x043f, 0x043f, 0x0440, 0x0440, 0x0441, 
	0x0442, 0x0442, 0x0443, 0x0443, 0x0444, 0x0445, 0x0445, 0x0446, 0x0446, 0x0447, 0x0448, 0x0448, 
	0x0449, 0x0449, 0x044a, 0x044b, 0x044b, 0x044c, 0x044c, 0x044d, 0x044e, 0x044e, 0x044f, 0x044f, 
	0x0450, 0x0451, 0x0451, 0x0452, 0x0452, 0x0453, 0x0453, 0x0454, 0x0455, 0x0455, 0x0456, 0x0456, 
	0x0457, 0x0458, 0x0458, 0x0459, 0x0459, 0x045a, 0x045b, 0x045b, 0x045c, 0x045c, 0x045d, 0x045d, 
	0x045e, 0x045f, 0x045f, 0x0460, 0x0460, 0x0461, 0x0462, 0x0462, 0x0463, 0x0463, 0x0464, 0x0464, 
	0x0465, 0x0466, 0x0466, 0x0467, 0x0467, 0x0468, 0x0469, 0x0469, 0x046a, 0x046a, 0x046b, 0x046b, 
	0x046c, 0x046d, 0x046d, 0x046e, 0x046e, 0x046f, 0x046f, 0x0470, 0x0471, 0x0471, 0x0472, 0x0472, 
	0x0473, 0x0473, 0x0474, 0x0475, 0x0475, 0x0476, 0x0476, 0x0477, 0x0477, 0x0478, 0x0479, 0x0479, 
	0x047a, 0x047a, 0x047b, 0x047b, 0x047c, 0x047c, 0x047d, 0x047e, 0x047e, 0x047f, 0x047f, 0x0480, 
	0x0480, 0x0481, 0x0482, 0x0482, 0x0483, 0x0483, 0x0484, 0x0484, 0x0485, 0x0485, 0x0486, 0x0487, 
	0x0487, 0x0488, 0x0488, 0x0489, 0x0489, 0x048a, 0x048a, 0x048b, 0x048c, 0x048c, 0x048d, 0x048d, 
	0x048e, 0x048e, 0x048f, 0x048f, 0x0490, 0x0491, 0x0491, 0x0492, 0x0492, 0x0493, 0x0493, 0x0494, 
	0x0494, 0x0495, 0x0496, 0x0496, 0x0497, 0x0497, 0x0498, 0x0498, 0x0499, 0x0499, 0x049a, 0x049a, 
	0x049b, 0x049c, 0x049c, 0x049d, 0x049d, 0x049e, 0x049e, 0x049f, 0x049f, 0x04a0, 0x04a0, 0x04a1, 
	0x04a1, 0x04a2, 0x04a3, 0x04a3, 0x04a4, 0x04a4, 0x04a5, 0x04a5, 0x04a6, 0x04a6, 0x04a7, 0x04a7, 
	0x04a8, 0x04a9, 0x04a9, 0x04aa, 0x04aa, 0x04ab, 0x04ab, 0x04ac, 0x04ac, 0x04ad, 0x04ad, 0x04ae, 
	0x04ae, 0x04af, 0x04af, 0x04b0, 0x04b1, 0x04b1, 0x04b2, 0x04b2, 0x04b3, 0x04b3, 0x04b4, 0x04b4, 
	0x04b5, 0x04b5, 0x04b6, 0x04b6, 0x04b7, 0x04b7, 0x04b8, 0x04b8, 0x04b9, 0x04ba, 0x04ba, 0x04bb, 
	0x04bb, 0x04bc, 0x04bc, 0x04bd, 0x04bd, 0x04be, 0x04be, 0x04bf, 0x04bf, 0x04c0, 0x04c0, 0x04c1, 
	0x04c1, 0x04c2, 0x04c2, 0x04c3, 0x04c3, 0x04c4, 0x04c5, 0x04c5, 0x04c6, 0x04c6, 0x04c7, 0x04c7, 
	0x04c8, 0x04c8, 0x04c9, 0x04c9, 0x04ca, 0x04ca, 0x04cb, 0x04cb, 0x04cc, 0x04cc, 0x04cd, 0x04cd, 
	0x04ce, 0x04ce, 0x04cf, 0x04cf, 0x04d0, 0x04d0, 0x04d1, 0x04d1, 0x04d2, 0x04d2, 0x04d3, 0x04d4, 
	0x04d4, 0x04d5, 0x04d5, 0x04d6, 0x04d6, 0x04d7, 0x04d7, 0x04d8, 0x04d8, 0x04d9, 0x04d9, 0x04da, 
	0x04da, 0x04db, 0x04db, 0x04dc, 0x04dc, 0x04dd, 0x04dd, 0x04de, 0x04de, 0x04df, 0x04df, 0x04e0, 
	0x04e0, 0x04e1, 0x04e1, 0x04e2, 0x04e2, 0x04e3, 0x04e3, 0x04e4, 0x04e4, 0x04e5, 0x04e5, 0x04e6, 
	0x04e6, 0x04e7, 0x04e7, 0x04e8, 0x04e8, 0x04e9, 0x04e9, 0x04ea, 0x04ea, 0x04eb, 0x04eb, 0x04ec, 
	0x04ec, 0x04ed, 0x04ed, 0x04ee, 0x04ee, 0x04ef, 0x04ef, 0x04f0, 0x04f0, 0x04f1, 0x04f1, 0x04f2, 
	0x04f2, 0x04f3, 0x04f3, 0x04f4, 0x04f4, 0x04f5, 0x04f5, 0x04f6, 0x04f6, 0x04f7, 0x04f7, 0x04f8, 
	0x04f8, 0x04f9, 0x04f9, 0x04fa, 0x04fa, 0x04fb, 0x04fb, 0x04fc, 0x04fc, 0x04fd, 0x04fd, 0x04fe, 
	0x04fe, 0x04ff, 0x04ff, 0x0500, 0x0500, 0x0501, 0x0501, 0x0502, 0x0502, 0x0502, 0x0503, 0x0503, 
	0x0504, 0x0504, 0x0505, 0x0505, 0x0506, 0x0506, 0x0507, 0x0507, 0x0508, 0x0508, 0x0509, 0x0509, 
	0x050a, 0x050a, 0x050b, 0x050b, 0x050c, 0x050c, 0x050d, 0x050d, 0x050e, 0x050e, 0x050f, 0x050f, 
	0x0510, 0x0510, 0x0511, 0x0511, 0x0512, 0x0512, 0x0512, 0x0513, 0x0513, 0x0514, 0x0514, 0x0515, 
	0x0515, 0x0516, 0x0516, 0x0517, 0x0517, 0x0518, 0x0518, 0x0519, 0x0519, 0x051a, 0x051a, 0x051b, 
	0x051b, 0x051c, 0x051c, 0x051c, 0x051d, 0x051d, 0x051e, 0x051e, 0x051f, 0x051f, 0x0520, 0x0520, 
	0x0521, 0x0521, 0x0522, 0x0522, 0x0523, 0x0523, 0x0524, 0x0524, 0x0525, 0x0525, 0x0525, 0x0526, 
	0x0526, 0x0527, 0x0527, 0x0528, 0x0528, 0x0529, 0x0529, 0x052a, 0x052a, 0x052b, 0x052b, 0x052c, 
	0x052c, 0x052d, 0x052d, 0x052d, 0x052e, 0x052e, 0x052f, 0x052f, 0x0530, 0x0530, 0x0531, 0x0531, 
	0x0532, 0x0532, 0x0533, 0x0533, 0x0534, 0x0534, 0x0534, 0x0535, 0x0535, 0x0536, 0x0536, 0x0537, 
	0x0537, 0x0538, 0x0538, 0x0539, 0x0539, 0x053a, 0x053a, 0x053a, 0x053b, 0x053b, 0x053c, 0x053c, 
	0x053d, 0x053d, 0x053e, 0x053e, 0x053f, 0x053f, 0x053f, 0x0540, 0x0540, 0x0541, 0x0541, 0x0542, 
	0x0542, 0x0543, 0x0543, 0x0544, 0x0544, 0x0545, 0x0545, 0x0545, 0x0546, 0x0546, 0x0547, 0x0547, 
	0x0548, 0x0548, 0x0549, 0x0549, 0x054a, 0x054a, 0x054a, 0x054b, 0x054b, 0x054c, 0x054c, 0x054d, 
	0x054d, 0x054e, 0x054e, 0x054f, 0x054f, 0x054f, 0x0550, 0x0550, 0x0551, 0x0551, 0x0552, 0x0552, 
	0x0553, 0x0553, 0x0553, 0x0554, 0x0554, 0x0555, 0x0555, 0x0556, 0x0556, 0x0557, 0x0557, 0x0558, 
	0x0558, 0x0558, 0x0559, 0x0559, 0x055a, 0x055a, 0x055b, 0x055b, 0x055c, 0x055c, 0x055c, 0x055d, 
	0x055d, 0x055e, 0x055e, 0x055f, 0x055f, 0x0560, 0x0560, 0x0560, 0x0561, 0x0561, 0x0562, 0x0562, 
	0x0563, 0x0563, 0x0564, 0x0564, 0x0564, 0x0565, 0x0565, 0x0566, 0x0566, 0x0567, 0x0567, 0x0568, 
	0x0568, 0x0568, 0x0569, 0x0569, 0x056a, 0x056a, 0x056b, 0x056b, 0x056b,
};
static const uint8_t	uint8Output[numLUTItems] = {
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 
	 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 
	 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x05, 0x06, 0x06, 0x06, 
	 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x08, 
	 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 
	 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0d, 
	 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0f, 0x0f, 
	 0x0f, 0x0f, 0x0f, 0x0f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 
	 0x11, 0x11, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x13, 
	 0x13, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x15, 0x15, 0x15, 0x15, 0x15, 
	 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x17, 0x17, 0x17, 0x17, 0x17, 0x18, 
	 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x19, 0x19, 0x19, 0x1a, 0x1a, 
	 0x1a, 0x1a, 0x1a, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1c, 0x1c, 0x1c, 
	 0x1c, 0x1c, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x1e, 0x1e, 0x1e, 0x1e, 
	 0x1e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x20, 0x20, 0x20, 0x20, 0x20, 
	 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x22, 0x22, 0x22, 0x22, 0x22, 0x23, 
	 0x23, 0x23, 0x23, 0x23, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x25, 0x25, 
	 0x25, 0x25, 0x25, 0x26, 0x26, 0x26, 0x26, 0x26, 0x26, 0x27, 0x27, 0x27, 
	 0x27, 0x27, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x29, 0x29, 0x29, 0x29, 
	 0x29, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 
	 0x2c, 0x2c, 0x2c, 0x2c, 0x2c, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2e, 
	 0x2e, 0x2e, 0x2e, 0x2e, 0x2f, 0x2f, 0x2f, 0x2f, 0x2f, 0x2f, 0x30, 0x30, 
	 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x32, 0x32, 0x32, 
	 0x32, 0x32, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x34, 0x34, 0x34, 0x34, 
	 0x34, 0x35, 0x35, 0x35, 0x35, 0x35, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 
	 0x37, 0x37, 0x37, 0x37, 0x37, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x39, 
	 0x39, 0x39, 0x39, 0x39, 0x3a, 0x3a, 0x3a, 0x3a, 0x3a, 0x3a, 0x3b, 0x3b, 
	 0x3b, 0x3b, 0x3b, 0x3c, 0x3c, 0x3c, 0x3c, 0x3c, 0x3c, 0x3d, 0x3d, 0x3d, 
	 0x3d, 0x3d, 0x3e, 0x3e, 0x3e, 0x3e, 0x3e, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 
	 0x3f, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 
	 0x42, 0x42, 0x42, 0x42, 0x42, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x44, 
	 0x44, 0x44, 0x44, 0x44, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x46, 0x46, 
	 0x46, 0x46, 0x46, 0x47, 0x47, 0x47, 0x47, 0x47, 0x48, 0x48, 0x48, 0x48, 
	 0x48, 0x48, 0x49, 0x49, 0x49, 0x49, 0x49, 0x4a, 0x4a, 0x4a, 0x4a, 0x4a, 
	 0x4a, 0x4b, 0x4b, 0x4b, 0x4b, 0x4b, 0x4c, 0x4c, 0x4c, 0x4c, 0x4c, 0x4c, 
	 0x4d, 0x4d, 0x4d, 0x4d, 0x4d, 0x4e, 0x4e, 0x4e, 0x4e, 0x4e, 0x4e, 0x4f, 
	 0x4f, 0x4f, 0x4f, 0x4f, 0x50, 0x50, 0x50, 0x50, 0x50, 0x51, 0x51, 0x51, 
	 0x51, 0x51, 0x51, 0x52, 0x52, 0x52, 0x52, 0x52, 0x53, 0x53, 0x53, 0x53, 
	 0x53, 0x53, 0x54, 0x54, 0x54, 0x54, 0x54, 0x55, 0x55, 0x55, 0x55, 0x55, 
	 0x55, 0x56, 0x56, 0x56, 0x56, 0x56, 0x57, 0x57, 0x57, 0x57, 0x57, 0x58, 
	 0x58, 0x58, 0x58, 0x58, 0x58, 0x59, 0x59, 0x59, 0x59, 0x59, 0x5a, 0x5a, 
	 0x5a, 0x5a, 0x5a, 0x5a, 0x5b, 0x5b, 0x5b, 0x5b, 0x5b, 0x5c, 0x5c, 0x5c, 
	 0x5c, 0x5c, 0x5c, 0x5d, 0x5d, 0x5d, 0x5d, 0x5d, 0x5e, 0x5e, 0x5e, 0x5e, 
	 0x5e, 0x5e, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x60, 0x60, 0x60, 0x60, 0x60, 
	 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x62, 0x62, 0x62, 0x62, 0x62, 0x63, 
	 0x63, 0x63, 0x63, 0x63, 0x63, 0x64, 0x64, 0x64, 0x64, 0x64, 0x65, 0x65, 
	 0x65, 0x65, 0x65, 0x65, 0x66, 0x66, 0x66, 0x66, 0x66, 0x67, 0x67, 0x67, 
	 0x67, 0x67, 0x67, 0x68, 0x68, 0x68, 0x68, 0x68, 0x69, 0x69, 0x69, 0x69, 
	 0x69, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a, 0x6b, 0x6b, 0x6b, 0x6b, 0x6b, 
	 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6d, 0x6d, 0x6d, 0x6d, 0x6d, 0x6e, 
	 0x6e, 0x6e, 0x6e, 0x6e, 0x6e, 0x6f, 0x6f, 0x6f, 0x6f, 0x6f, 0x70, 0x70, 
	 0x70, 0x70, 0x70, 0x70, 0x71, 0x71, 0x71, 0x71, 0x71, 0x72, 0x72, 0x72, 
	 0x72, 0x72, 0x73, 0x73, 0x73, 0x73, 0x73, 0x73, 0x74, 0x74, 0x74, 0x74, 
	 0x74, 0x75, 0x75, 0x75, 0x75, 0x75, 0x75, 0x76, 0x76, 0x76, 0x76, 0x76, 
	 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x78, 0x78, 0x78, 0x78, 0x78, 0x79, 
	 0x79, 0x79, 0x79, 0x79, 0x79, 0x7a, 0x7a, 0x7a, 0x7a, 0x7a, 0x7b, 0x7b, 
	 0x7b, 0x7b, 0x7b, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7d, 0x7d, 0x7d, 
	 0x7d, 0x7d, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7f, 0x7f, 0x7f, 0x7f, 
	 0x7f, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x81, 0x81, 0x81, 0x81, 
	 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x83, 0x83, 0x83, 0x83, 0x83, 0x84, 
	 0x84, 0x84, 0x84, 0x84, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x86, 0x86, 
	 0x86, 0x86, 0x86, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x88, 0x88, 0x88, 
	 0x88, 0x88, 0x89, 0x89, 0x89, 0x89, 0x89, 0x89, 0x8a, 0x8a, 0x8a, 0x8a, 
	 0x8a, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0x8c, 0x8c, 0x8c, 0x8c, 0x8c, 
	 0x8d, 0x8d, 0x8d, 0x8d, 0x8d, 0x8e, 0x8e, 0x8e, 0x8e, 0x8e, 0x8e, 0x8f, 
	 0x8f, 0x8f, 0x8f, 0x8f, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x91, 0x91, 
	 0x91, 0x91, 0x91, 0x92, 0x92, 0x92, 0x92, 0x92, 0x92, 0x93, 0x93, 0x93, 
	 0x93, 0x93, 0x94, 0x94, 0x94, 0x94, 0x94, 0x94, 0x95, 0x95, 0x95, 0x95, 
	 0x95, 0x96, 0x96, 0x96, 0x96, 0x96, 0x97, 0x97, 0x97, 0x97, 0x97, 0x97, 
	 0x98, 0x98, 0x98, 0x98, 0x98, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x9a, 
	 0x9a, 0x9a, 0x9a, 0x9a, 0x9b, 0x9b, 0x9b, 0x9b, 0x9b, 0x9b, 0x9c, 0x9c, 
	 0x9c, 0x9c, 0x9c, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9e, 0x9e, 0x9e, 
	 0x9e, 0x9e, 0x9f, 0x9f, 0x9f, 0x9f, 0x9f, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 
	 0xa0, 0xa1, 0xa1, 0xa1, 0xa1, 0xa1, 0xa2, 0xa2, 0xa2, 0xa2, 0xa2, 0xa2, 
	 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa4, 0xa4, 0xa4, 0xa4, 0xa4, 0xa4, 0xa5, 
	 0xa5, 0xa5, 0xa5, 0xa5, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa7, 0xa7, 
	 0xa7, 0xa7, 0xa7, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa9, 0xa9, 0xa9, 0xa9, 
	 0xa9, 0xa9, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xab, 0xab, 0xab, 0xab, 0xab, 
	 0xab, 0xac, 0xac, 0xac, 0xac, 0xac, 0xad, 0xad, 0xad, 0xad, 0xad, 0xad, 
	 0xae, 0xae, 0xae, 0xae, 0xae, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf, 0xb0, 0xb0, 
	 0xb0, 0xb0, 0xb0, 0xb0, 0xb1, 0xb1, 0xb1, 0xb1, 0xb1, 0xb2, 0xb2, 0xb2, 
	 0xb2, 0xb2, 0xb2, 0xb3, 0xb3, 0xb3, 0xb3, 0xb3, 0xb4, 0xb4, 0xb4, 0xb4, 
	 0xb4, 0xb4, 0xb5, 0xb5, 0xb5, 0xb5, 0xb5, 0xb6, 0xb6, 0xb6, 0xb6, 0xb6, 
	 0xb6, 0xb7, 0xb7, 0xb7, 0xb7, 0xb7, 0xb8, 0xb8, 0xb8, 0xb8, 0xb8, 0xb9, 
	 0xb9, 0xb9, 0xb9, 0xb9, 0xb9, 0xba, 0xba, 0xba, 0xba, 0xba, 0xbb, 0xbb, 
	 0xbb, 0xbb, 0xbb, 0xbb, 0xbc, 0xbc, 0xbc, 0xbc, 0xbc, 0xbd, 0xbd, 0xbd, 
	 0xbd, 0xbd, 0xbd, 0xbe, 0xbe, 0xbe, 0xbe, 0xbe, 0xbf, 0xbf, 0xbf, 0xbf, 
	 0xbf, 0xbf, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc1, 0xc1, 0xc1, 0xc1, 0xc1, 
	 0xc2, 0xc2, 0xc2, 0xc2, 0xc2, 0xc2, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc4, 
	 0xc4, 0xc4, 0xc4, 0xc4, 0xc4, 0xc5, 0xc5, 0xc5, 0xc5, 0xc5, 0xc6, 0xc6, 
	 0xc6, 0xc6, 0xc6, 0xc6, 0xc7, 0xc7, 0xc7, 0xc7, 0xc7, 0xc8, 0xc8, 0xc8, 
	 0xc8, 0xc8, 0xc8, 0xc9, 0xc9, 0xc9, 0xc9, 0xc9, 0xca, 0xca, 0xca, 0xca, 
	 0xca, 0xcb, 0xcb, 0xcb, 0xcb, 0xcb, 0xcb, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 
	 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xcd, 0xce, 0xce, 0xce, 0xce, 0xce, 0xcf, 
	 0xcf, 0xcf, 0xcf, 0xcf, 0xcf, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xd1, 0xd1, 
	 0xd1, 0xd1, 0xd1, 0xd1, 0xd2, 0xd2, 0xd2, 0xd2, 0xd2, 0xd3, 0xd3, 0xd3, 
	 0xd3, 0xd3, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd4, 0xd5, 0xd5, 0xd5, 0xd5, 
	 0xd5, 0xd6, 0xd6, 0xd6, 0xd6, 0xd6, 0xd6, 0xd7, 0xd7, 0xd7, 0xd7, 0xd7, 
	 0xd8, 0xd8, 0xd8, 0xd8, 0xd8, 0xd8, 0xd9, 0xd9, 0xd9, 0xd9, 0xd9, 0xda, 
	 0xda, 0xda, 0xda, 0xda, 0xda, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0xdc, 0xdc, 
	 0xdc, 0xdc, 0xdc, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xde, 0xde, 0xde, 
	 0xde, 0xde, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xdf, 0xe0, 0xe0, 0xe0, 0xe0, 
	 0xe0, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe2, 0xe2, 0xe2, 0xe2, 0xe2, 
	 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe4, 0xe4, 0xe4, 0xe4, 0xe4, 0xe5, 
	 0xe5, 0xe5, 0xe5, 0xe5, 0xe6, 0xe6, 0xe6, 0xe6, 0xe6, 0xe6, 0xe7, 0xe7, 
	 0xe7, 0xe7, 0xe7, 0xe8, 0xe8, 0xe8, 0xe8, 0xe8, 0xe8, 0xe9, 0xe9, 0xe9, 
	 0xe9, 0xe9, 0xea, 0xea, 0xea, 0xea, 0xea, 0xea, 0xeb, 0xeb, 0xeb, 0xeb, 
	 0xeb, 0xec, 0xec, 0xec, 0xec, 0xec, 0xec, 0xed, 0xed, 0xed, 0xed, 0xed, 
	 0xee, 0xee, 0xee, 0xee, 0xee, 0xef, 0xef, 0xef, 0xef, 0xef, 0xef, 0xf0, 
	 0xf0, 0xf0, 0xf0, 0xf0, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf2, 0xf2, 
	 0xf2, 0xf2, 0xf2, 0xf3, 0xf3, 0xf3, 0xf3, 0xf3, 0xf3, 0xf4, 0xf4, 0xf4, 
	 0xf4, 0xf4, 0xf5, 0xf5, 0xf5, 0xf5, 0xf5, 0xf5, 0xf6, 0xf6, 0xf6, 0xf6, 
	 0xf6, 0xf7, 0xf7, 0xf7, 0xf7, 0xf7, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 
	 0xf9, 0xf9, 0xf9, 0xf9, 0xf9, 0xfa, 0xfa, 0xfa, 0xfa, 0xfa, 0xfa, 0xfb, 
	 0xfb, 0xfb, 0xfb, 0xfb, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfd, 0xfd, 
	 0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff 
};
static const uint16_t	uint16Output[numLUTItems] = {
	 0x0000, 0x002f, 0x005e, 0x008d, 0x00bc, 0x00ec, 0x011b, 0x014a, 0x0179, 0x01a8, 0x01d8, 0x0207, 
	 0x0236, 0x0265, 0x0295, 0x02c4, 0x02f3, 0x0322, 0x0351, 0x0381, 0x03b0, 0x03df, 0x040e, 0x043d, 
	 0x046d, 0x049c, 0x04cb, 0x04fa, 0x052a, 0x0559, 0x0588, 0x05b7, 0x05e6, 0x0616, 0x0645, 0x0674, 
	 0x06a3, 0x06d2, 0x0702, 0x0731, 0x0760, 0x078f, 0x07bf, 0x07ee, 0x081d, 0x084c, 0x087b, 0x08ab, 
	 0x08da, 0x0909, 0x0938, 0x0967, 0x0997, 0x09c6, 0x09f5, 0x0a24, 0x0a54, 0x0a83, 0x0ab2, 0x0ae1, 
	 0x0b10, 0x0b40, 0x0b6f, 0x0b9e, 0x0bcd, 0x0bfd, 0x0c2c, 0x0c5b, 0x0c8a, 0x0cb9, 0x0ce9, 0x0d18, 
	 0x0d47, 0x0d76, 0x0da5, 0x0dd5, 0x0e04, 0x0e33, 0x0e62, 0x0e92, 0x0ec1, 0x0ef0, 0x0f1f, 0x0f4e, 
	 0x0f7e, 0x0fad, 0x0fdc, 0x100b, 0x103a, 0x106a, 0x1099, 0x10c8, 0x10f7, 0x1127, 0x1156, 0x1185, 
	 0x11b4, 0x11e3, 0x1213, 0x1242, 0x1271, 0x12a0, 0x12cf, 0x12ff, 0x132e, 0x135d, 0x138c, 0x13bc, 
	 0x13eb, 0x141a, 0x1449, 0x1478, 0x14a8, 0x14d7, 0x1506, 0x1535, 0x1564, 0x1594, 0x15c3, 0x15f2, 
	 0x1621, 0x1651, 0x1680, 0x16af, 0x16de, 0x170d, 0x173d, 0x176c, 0x179b, 0x17ca, 0x17fa, 0x1829, 
	 0x1858, 0x1887, 0x18b6, 0x18e6, 0x1915, 0x1944, 0x1973, 0x19a2, 0x19d2, 0x1a01, 0x1a30, 0x1a5f, 
	 0x1a8f, 0x1abe, 0x1aed, 0x1b1c, 0x1b4b, 0x1b7b, 0x1baa, 0x1bd9, 0x1c08, 0x1c37, 0x1c67, 0x1c96, 
	 0x1cc5, 0x1cf4, 0x1d24, 0x1d53, 0x1d82, 0x1db1, 0x1de0, 0x1e10, 0x1e3f, 0x1e6e, 0x1e9d, 0x1ecc, 
	 0x1efc, 0x1f2b, 0x1f5a, 0x1f89, 0x1fb9, 0x1fe8, 0x2017, 0x2046, 0x2075, 0x20a5, 0x20d4, 0x2103, 
	 0x2132, 0x2161, 0x2191, 0x21c0, 0x21ef, 0x221e, 0x224e, 0x227d, 0x22ac, 0x22db, 0x230a, 0x233a, 
	 0x2369, 0x2398, 0x23c7, 0x23f7, 0x2426, 0x2455, 0x2484, 0x24b3, 0x24e3, 0x2512, 0x2541, 0x2570, 
	 0x259f, 0x25cf, 0x25fe, 0x262d, 0x265c, 0x268c, 0x26bb, 0x26ea, 0x2719, 0x2748, 0x2778, 0x27a7, 
	 0x27d6, 0x2805, 0x2834, 0x2864, 0x2893, 0x28c2, 0x28f1, 0x2921, 0x2950, 0x297f, 0x29ae, 0x29dd, 
	 0x2a0d, 0x2a3c, 0x2a6b, 0x2a9a, 0x2ac9, 0x2af9, 0x2b28, 0x2b57, 0x2b86, 0x2bb6, 0x2be5, 0x2c14, 
	 0x2c43, 0x2c72, 0x2ca2, 0x2cd1, 0x2d00, 0x2d2f, 0x2d5e, 0x2d8e, 0x2dbd, 0x2dec, 0x2e1b, 0x2e4b, 
	 0x2e7a, 0x2ea9, 0x2ed8, 0x2f07, 0x2f37, 0x2f66, 0x2f95, 0x2fc4, 0x2ff4, 0x3023, 0x3052, 0x3081, 
	 0x30b0, 0x30e0, 0x310f, 0x313e, 0x316d, 0x319c, 0x31cc, 0x31fb, 0x322a, 0x3259, 0x3289, 0x32b8, 
	 0x32e7, 0x3316, 0x3345, 0x3375, 0x33a4, 0x33d3, 0x3402, 0x3431, 0x3461, 0x3490, 0x34bf, 0x34ee, 
	 0x351e, 0x354d, 0x357c, 0x35ab, 0x35da, 0x360a, 0x3639, 0x3668, 0x3697, 0x36c6, 0x36f6, 0x3725, 
	 0x3754, 0x3783, 0x37b3, 0x37e2, 0x3811, 0x3840, 0x386f, 0x389f, 0x38ce, 0x38fd, 0x392c, 0x395b, 
	 0x398b, 0x39ba, 0x39e9, 0x3a18, 0x3a48, 0x3a77, 0x3aa6, 0x3ad5, 0x3b04, 0x3b34, 0x3b63, 0x3b92, 
	 0x3bc1, 0x3bf1, 0x3c20, 0x3c4f, 0x3c7e, 0x3cad, 0x3cdd, 0x3d0c, 0x3d3b, 0x3d6a, 0x3d99, 0x3dc9, 
	 0x3df8, 0x3e27, 0x3e56, 0x3e86, 0x3eb5, 0x3ee4, 0x3f13, 0x3f42, 0x3f72, 0x3fa1, 0x3fd0, 0x3fff, 
	 0x402e, 0x405e, 0x408d, 0x40bc, 0x40eb, 0x411b, 0x414a, 0x4179, 0x41a8, 0x41d7, 0x4207, 0x4236, 
	 0x4265, 0x4294, 0x42c3, 0x42f3, 0x4322, 0x4351, 0x4380, 0x43b0, 0x43df, 0x440e, 0x443d, 0x446c, 
	 0x449c, 0x44cb, 0x44fa, 0x4529, 0x4558, 0x4588, 0x45b7, 0x45e6, 0x4615, 0x4645, 0x4674, 0x46a3, 
	 0x46d2, 0x4701, 0x4731, 0x4760, 0x478f, 0x47be, 0x47ee, 0x481d, 0x484c, 0x487b, 0x48aa, 0x48da, 
	 0x4909, 0x4938, 0x4967, 0x4996, 0x49c6, 0x49f5, 0x4a24, 0x4a53, 0x4a83, 0x4ab2, 0x4ae1, 0x4b10, 
	 0x4b3f, 0x4b6f, 0x4b9e, 0x4bcd, 0x4bfc, 0x4c2b, 0x4c5b, 0x4c8a, 0x4cb9, 0x4ce8, 0x4d18, 0x4d47, 
	 0x4d76, 0x4da5, 0x4dd4, 0x4e04, 0x4e33, 0x4e62, 0x4e91, 0x4ec0, 0x4ef0, 0x4f1f, 0x4f4e, 0x4f7d, 
	 0x4fad, 0x4fdc, 0x500b, 0x503a, 0x5069, 0x5099, 0x50c8, 0x50f7, 0x5126, 0x5155, 0x5185, 0x51b4, 
	 0x51e3, 0x5212, 0x5242, 0x5271, 0x52a0, 0x52cf, 0x52fe, 0x532e, 0x535d, 0x538c, 0x53bb, 0x53eb, 
	 0x541a, 0x5449, 0x5478, 0x54a7, 0x54d7, 0x5506, 0x5535, 0x5564, 0x5593, 0x55c3, 0x55f2, 0x5621, 
	 0x5650, 0x5680, 0x56af, 0x56de, 0x570d, 0x573c, 0x576c, 0x579b, 0x57ca, 0x57f9, 0x5828, 0x5858, 
	 0x5887, 0x58b6, 0x58e5, 0x5915, 0x5944, 0x5973, 0x59a2, 0x59d1, 0x5a01, 0x5a30, 0x5a5f, 0x5a8e, 
	 0x5abd, 0x5aed, 0x5b1c, 0x5b4b, 0x5b7a, 0x5baa, 0x5bd9, 0x5c08, 0x5c37, 0x5c66, 0x5c96, 0x5cc5, 
	 0x5cf4, 0x5d23, 0x5d53, 0x5d82, 0x5db1, 0x5de0, 0x5e0f, 0x5e3f, 0x5e6e, 0x5e9d, 0x5ecc, 0x5efb, 
	 0x5f2b, 0x5f5a, 0x5f89, 0x5fb8, 0x5fe8, 0x6017, 0x6046, 0x6075, 0x60a4, 0x60d4, 0x6103, 0x6132, 
	 0x6161, 0x6190, 0x61c0, 0x61ef, 0x621e, 0x624d, 0x627d, 0x62ac, 0x62db, 0x630a, 0x6339, 0x6369, 
	 0x6398, 0x63c7, 0x63f6, 0x6425, 0x6455, 0x6484, 0x64b3, 0x64e2, 0x6512, 0x6541, 0x6570, 0x659f, 
	 0x65ce, 0x65fe, 0x662d, 0x665c, 0x668b, 0x66ba, 0x66ea, 0x6719, 0x6748, 0x6777, 0x67a7, 0x67d6, 
	 0x6805, 0x6834, 0x6863, 0x6893, 0x68c2, 0x68f1, 0x6920, 0x6950, 0x697f, 0x69ae, 0x69dd, 0x6a0c, 
	 0x6a3c, 0x6a6b, 0x6a9a, 0x6ac9, 0x6af8, 0x6b28, 0x6b57, 0x6b86, 0x6bb5, 0x6be5, 0x6c14, 0x6c43, 
	 0x6c72, 0x6ca1, 0x6cd1, 0x6d00, 0x6d2f, 0x6d5e, 0x6d8d, 0x6dbd, 0x6dec, 0x6e1b, 0x6e4a, 0x6e7a, 
	 0x6ea9, 0x6ed8, 0x6f07, 0x6f36, 0x6f66, 0x6f95, 0x6fc4, 0x6ff3, 0x7022, 0x7052, 0x7081, 0x70b0, 
	 0x70df, 0x710f, 0x713e, 0x716d, 0x719c, 0x71cb, 0x71fb, 0x722a, 0x7259, 0x7288, 0x72b7, 0x72e7, 
	 0x7316, 0x7345, 0x7374, 0x73a4, 0x73d3, 0x7402, 0x7431, 0x7460, 0x7490, 0x74bf, 0x74ee, 0x751d, 
	 0x754d, 0x757c, 0x75ab, 0x75da, 0x7609, 0x7639, 0x7668, 0x7697, 0x76c6, 0x76f5, 0x7725, 0x7754, 
	 0x7783, 0x77b2, 0x77e2, 0x7811, 0x7840, 0x786f, 0x789e, 0x78ce, 0x78fd, 0x792c, 0x795b, 0x798a, 
	 0x79ba, 0x79e9, 0x7a18, 0x7a47, 0x7a77, 0x7aa6, 0x7ad5, 0x7b04, 0x7b33, 0x7b63, 0x7b92, 0x7bc1, 
	 0x7bf0, 0x7c1f, 0x7c4f, 0x7c7e, 0x7cad, 0x7cdc, 0x7d0c, 0x7d3b, 0x7d6a, 0x7d99, 0x7dc8, 0x7df8, 
	 0x7e27, 0x7e56, 0x7e85, 0x7eb4, 0x7ee4, 0x7f13, 0x7f42, 0x7f71, 0x7fa1, 0x7fd0, 0x7fff, 0x802e, 
	 0x805d, 0x808d, 0x80bc, 0x80eb, 0x811a, 0x814a, 0x8179, 0x81a8, 0x81d7, 0x8206, 0x8236, 0x8265, 
	 0x8294, 0x82c3, 0x82f2, 0x8322, 0x8351, 0x8380, 0x83af, 0x83df, 0x840e, 0x843d, 0x846c, 0x849b, 
	 0x84cb, 0x84fa, 0x8529, 0x8558, 0x8587, 0x85b7, 0x85e6, 0x8615, 0x8644, 0x8674, 0x86a3, 0x86d2, 
	 0x8701, 0x8730, 0x8760, 0x878f, 0x87be, 0x87ed, 0x881c, 0x884c, 0x887b, 0x88aa, 0x88d9, 0x8909, 
	 0x8938, 0x8967, 0x8996, 0x89c5, 0x89f5, 0x8a24, 0x8a53, 0x8a82, 0x8ab1, 0x8ae1, 0x8b10, 0x8b3f, 
	 0x8b6e, 0x8b9e, 0x8bcd, 0x8bfc, 0x8c2b, 0x8c5a, 0x8c8a, 0x8cb9, 0x8ce8, 0x8d17, 0x8d47, 0x8d76, 
	 0x8da5, 0x8dd4, 0x8e03, 0x8e33, 0x8e62, 0x8e91, 0x8ec0, 0x8eef, 0x8f1f, 0x8f4e, 0x8f7d, 0x8fac, 
	 0x8fdc, 0x900b, 0x903a, 0x9069, 0x9098, 0x90c8, 0x90f7, 0x9126, 0x9155, 0x9184, 0x91b4, 0x91e3, 
	 0x9212, 0x9241, 0x9271, 0x92a0, 0x92cf, 0x92fe, 0x932d, 0x935d, 0x938c, 0x93bb, 0x93ea, 0x9419, 
	 0x9449, 0x9478, 0x94a7, 0x94d6, 0x9506, 0x9535, 0x9564, 0x9593, 0x95c2, 0x95f2, 0x9621, 0x9650, 
	 0x967f, 0x96ae, 0x96de, 0x970d, 0x973c, 0x976b, 0x979b, 0x97ca, 0x97f9, 0x9828, 0x9857, 0x9887, 
	 0x98b6, 0x98e5, 0x9914, 0x9944, 0x9973, 0x99a2, 0x99d1, 0x9a00, 0x9a30, 0x9a5f, 0x9a8e, 0x9abd, 
	 0x9aec, 0x9b1c, 0x9b4b, 0x9b7a, 0x9ba9, 0x9bd9, 0x9c08, 0x9c37, 0x9c66, 0x9c95, 0x9cc5, 0x9cf4, 
	 0x9d23, 0x9d52, 0x9d81, 0x9db1, 0x9de0, 0x9e0f, 0x9e3e, 0x9e6e, 0x9e9d, 0x9ecc, 0x9efb, 0x9f2a, 
	 0x9f5a, 0x9f89, 0x9fb8, 0x9fe7, 0xa016, 0xa046, 0xa075, 0xa0a4, 0xa0d3, 0xa103, 0xa132, 0xa161, 
	 0xa190, 0xa1bf, 0xa1ef, 0xa21e, 0xa24d, 0xa27c, 0xa2ab, 0xa2db, 0xa30a, 0xa339, 0xa368, 0xa398, 
	 0xa3c7, 0xa3f6, 0xa425, 0xa454, 0xa484, 0xa4b3, 0xa4e2, 0xa511, 0xa541, 0xa570, 0xa59f, 0xa5ce, 
	 0xa5fd, 0xa62d, 0xa65c, 0xa68b, 0xa6ba, 0xa6e9, 0xa719, 0xa748, 0xa777, 0xa7a6, 0xa7d6, 0xa805, 
	 0xa834, 0xa863, 0xa892, 0xa8c2, 0xa8f1, 0xa920, 0xa94f, 0xa97e, 0xa9ae, 0xa9dd, 0xaa0c, 0xaa3b, 
	 0xaa6b, 0xaa9a, 0xaac9, 0xaaf8, 0xab27, 0xab57, 0xab86, 0xabb5, 0xabe4, 0xac13, 0xac43, 0xac72, 
	 0xaca1, 0xacd0, 0xad00, 0xad2f, 0xad5e, 0xad8d, 0xadbc, 0xadec, 0xae1b, 0xae4a, 0xae79, 0xaea9, 
	 0xaed8, 0xaf07, 0xaf36, 0xaf65, 0xaf95, 0xafc4, 0xaff3, 0xb022, 0xb051, 0xb081, 0xb0b0, 0xb0df, 
	 0xb10e, 0xb13e, 0xb16d, 0xb19c, 0xb1cb, 0xb1fa, 0xb22a, 0xb259, 0xb288, 0xb2b7, 0xb2e6, 0xb316, 
	 0xb345, 0xb374, 0xb3a3, 0xb3d3, 0xb402, 0xb431, 0xb460, 0xb48f, 0xb4bf, 0xb4ee, 0xb51d, 0xb54c, 
	 0xb57b, 0xb5ab, 0xb5da, 0xb609, 0xb638, 0xb668, 0xb697, 0xb6c6, 0xb6f5, 0xb724, 0xb754, 0xb783, 
	 0xb7b2, 0xb7e1, 0xb810, 0xb840, 0xb86f, 0xb89e, 0xb8cd, 0xb8fd, 0xb92c, 0xb95b, 0xb98a, 0xb9b9, 
	 0xb9e9, 0xba18, 0xba47, 0xba76, 0xbaa6, 0xbad5, 0xbb04, 0xbb33, 0xbb62, 0xbb92, 0xbbc1, 0xbbf0, 
	 0xbc1f, 0xbc4e, 0xbc7e, 0xbcad, 0xbcdc, 0xbd0b, 0xbd3b, 0xbd6a, 0xbd99, 0xbdc8, 0xbdf7, 0xbe27, 
	 0xbe56, 0xbe85, 0xbeb4, 0xbee3, 0xbf13, 0xbf42, 0xbf71, 0xbfa0, 0xbfd0, 0xbfff, 0xc02e, 0xc05d, 
	 0xc08c, 0xc0bc, 0xc0eb, 0xc11a, 0xc149, 0xc178, 0xc1a8, 0xc1d7, 0xc206, 0xc235, 0xc265, 0xc294, 
	 0xc2c3, 0xc2f2, 0xc321, 0xc351, 0xc380, 0xc3af, 0xc3de, 0xc40d, 0xc43d, 0xc46c, 0xc49b, 0xc4ca, 
	 0xc4fa, 0xc529, 0xc558, 0xc587, 0xc5b6, 0xc5e6, 0xc615, 0xc644, 0xc673, 0xc6a3, 0xc6d2, 0xc701, 
	 0xc730, 0xc75f, 0xc78f, 0xc7be, 0xc7ed, 0xc81c, 0xc84b, 0xc87b, 0xc8aa, 0xc8d9, 0xc908, 0xc938, 
	 0xc967, 0xc996, 0xc9c5, 0xc9f4, 0xca24, 0xca53, 0xca82, 0xcab1, 0xcae0, 0xcb10, 0xcb3f, 0xcb6e, 
	 0xcb9d, 0xcbcd, 0xcbfc, 0xcc2b, 0xcc5a, 0xcc89, 0xccb9, 0xcce8, 0xcd17, 0xcd46, 0xcd75, 0xcda5, 
	 0xcdd4, 0xce03, 0xce32, 0xce62, 0xce91, 0xcec0, 0xceef, 0xcf1e, 0xcf4e, 0xcf7d, 0xcfac, 0xcfdb, 
	 0xd00a, 0xd03a, 0xd069, 0xd098, 0xd0c7, 0xd0f7, 0xd126, 0xd155, 0xd184, 0xd1b3, 0xd1e3, 0xd212, 
	 0xd241, 0xd270, 0xd2a0, 0xd2cf, 0xd2fe, 0xd32d, 0xd35c, 0xd38c, 0xd3bb, 0xd3ea, 0xd419, 0xd448, 
	 0xd478, 0xd4a7, 0xd4d6, 0xd505, 0xd535, 0xd564, 0xd593, 0xd5c2, 0xd5f1, 0xd621, 0xd650, 0xd67f, 
	 0xd6ae, 0xd6dd, 0xd70d, 0xd73c, 0xd76b, 0xd79a, 0xd7ca, 0xd7f9, 0xd828, 0xd857, 0xd886, 0xd8b6, 
	 0xd8e5, 0xd914, 0xd943, 0xd972, 0xd9a2, 0xd9d1, 0xda00, 0xda2f, 0xda5f, 0xda8e, 0xdabd, 0xdaec, 
	 0xdb1b, 0xdb4b, 0xdb7a, 0xdba9, 0xdbd8, 0xdc07, 0xdc37, 0xdc66, 0xdc95, 0xdcc4, 0xdcf4, 0xdd23, 
	 0xdd52, 0xdd81, 0xddb0, 0xdde0, 0xde0f, 0xde3e, 0xde6d, 0xde9d, 0xdecc, 0xdefb, 0xdf2a, 0xdf59, 
	 0xdf89, 0xdfb8, 0xdfe7, 0xe016, 0xe045, 0xe075, 0xe0a4, 0xe0d3, 0xe102, 0xe132, 0xe161, 0xe190, 
	 0xe1bf, 0xe1ee, 0xe21e, 0xe24d, 0xe27c, 0xe2ab, 0xe2da, 0xe30a, 0xe339, 0xe368, 0xe397, 0xe3c7, 
	 0xe3f6, 0xe425, 0xe454, 0xe483, 0xe4b3, 0xe4e2, 0xe511, 0xe540, 0xe56f, 0xe59f, 0xe5ce, 0xe5fd, 
	 0xe62c, 0xe65c, 0xe68b, 0xe6ba, 0xe6e9, 0xe718, 0xe748, 0xe777, 0xe7a6, 0xe7d5, 0xe804, 0xe834, 
	 0xe863, 0xe892, 0xe8c1, 0xe8f1, 0xe920, 0xe94f, 0xe97e, 0xe9ad, 0xe9dd, 0xea0c, 0xea3b, 0xea6a, 
	 0xea9a, 0xeac9, 0xeaf8, 0xeb27, 0xeb56, 0xeb86, 0xebb5, 0xebe4, 0xec13, 0xec42, 0xec72, 0xeca1, 
	 0xecd0, 0xecff, 0xed2f, 0xed5e, 0xed8d, 0xedbc, 0xedeb, 0xee1b, 0xee4a, 0xee79, 0xeea8, 0xeed7, 
	 0xef07, 0xef36, 0xef65, 0xef94, 0xefc4, 0xeff3, 0xf022, 0xf051, 0xf080, 0xf0b0, 0xf0df, 0xf10e, 
	 0xf13d, 0xf16c, 0xf19c, 0xf1cb, 0xf1fa, 0xf229, 0xf259, 0xf288, 0xf2b7, 0xf2e6, 0xf315, 0xf345, 
	 0xf374, 0xf3a3, 0xf3d2, 0xf401, 0xf431, 0xf460, 0xf48f, 0xf4be, 0xf4ee, 0xf51d, 0xf54c, 0xf57b, 
	 0xf5aa, 0xf5da, 0xf609, 0xf638, 0xf667, 0xf697, 0xf6c6, 0xf6f5, 0xf724, 0xf753, 0xf783, 0xf7b2, 
	 0xf7e1, 0xf810, 0xf83f, 0xf86f, 0xf89e, 0xf8cd, 0xf8fc, 0xf92c, 0xf95b, 0xf98a, 0xf9b9, 0xf9e8, 
	 0xfa18, 0xfa47, 0xfa76, 0xfaa5, 0xfad4, 0xfb04, 0xfb33, 0xfb62, 0xfb91, 0xfbc1, 0xfbf0, 0xfc1f, 
	 0xfc4e, 0xfc7d, 0xfcad, 0xfcdc, 0xfd0b, 0xfd3a, 0xfd69, 0xfd99, 0xfdc8, 0xfdf7, 0xfe26, 0xfe56, 
	 0xfe85, 0xfeb4, 0xfee3, 0xff12, 0xff42, 0xff71, 0xffa0, 0xffcf, 0xffff
};
static const float	floatOutput[numLUTItems] = {
	 0.000000f, 0.000720f, 0.001441f, 0.002161f, 0.002882f, 0.003602f, 0.004323f, 0.005043f, 0.005764f, 0.006484f, 0.007205f, 0.007925f, 
	 0.008646f, 0.009366f, 0.010086f, 0.010807f, 0.011527f, 0.012248f, 0.012968f, 0.013689f, 0.014409f, 0.015130f, 0.015850f, 0.016571f, 
	 0.017291f, 0.018012f, 0.018732f, 0.019452f, 0.020173f, 0.020893f, 0.021614f, 0.022334f, 0.023055f, 0.023775f, 0.024496f, 0.025216f, 
	 0.025937f, 0.026657f, 0.027378f, 0.028098f, 0.028818f, 0.029539f, 0.030259f, 0.030980f, 0.031700f, 0.032421f, 0.033141f, 0.033862f, 
	 0.034582f, 0.035303f, 0.036023f, 0.036744f, 0.037464f, 0.038184f, 0.038905f, 0.039625f, 0.040346f, 0.041066f, 0.041787f, 0.042507f, 
	 0.043228f, 0.043948f, 0.044669f, 0.045389f, 0.046110f, 0.046830f, 0.047550f, 0.048271f, 0.048991f, 0.049712f, 0.050432f, 0.051153f, 
	 0.051873f, 0.052594f, 0.053314f, 0.054035f, 0.054755f, 0.055476f, 0.056196f, 0.056916f, 0.057637f, 0.058357f, 0.059078f, 0.059798f, 
	 0.060519f, 0.061239f, 0.061960f, 0.062680f, 0.063401f, 0.064121f, 0.064842f, 0.065562f, 0.066282f, 0.067003f, 0.067723f, 0.068444f, 
	 0.069164f, 0.069885f, 0.070605f, 0.071326f, 0.072046f, 0.072767f, 0.073487f, 0.074207f, 0.074928f, 0.075648f, 0.076369f, 0.077089f, 
	 0.077810f, 0.078530f, 0.079251f, 0.079971f, 0.080692f, 0.081412f, 0.082133f, 0.082853f, 0.083573f, 0.084294f, 0.085014f, 0.085735f, 
	 0.086455f, 0.087176f, 0.087896f, 0.088617f, 0.089337f, 0.090058f, 0.090778f, 0.091499f, 0.092219f, 0.092939f, 0.093660f, 0.094380f, 
	 0.095101f, 0.095821f, 0.096542f, 0.097262f, 0.097983f, 0.098703f, 0.099424f, 0.100144f, 0.100865f, 0.101585f, 0.102305f, 0.103026f, 
	 0.103746f, 0.104467f, 0.105187f, 0.105908f, 0.106628f, 0.107349f, 0.108069f, 0.108790f, 0.109510f, 0.110231f, 0.110951f, 0.111671f, 
	 0.112392f, 0.113112f, 0.113833f, 0.114553f, 0.115274f, 0.115994f, 0.116715f, 0.117435f, 0.118156f, 0.118876f, 0.119597f, 0.120317f, 
	 0.121037f, 0.121758f, 0.122478f, 0.123199f, 0.123919f, 0.124640f, 0.125360f, 0.126081f, 0.126801f, 0.127522f, 0.128242f, 0.128963f, 
	 0.129683f, 0.130403f, 0.131124f, 0.131844f, 0.132565f, 0.133285f, 0.134006f, 0.134726f, 0.135447f, 0.136167f, 0.136888f, 0.137608f, 
	 0.138329f, 0.139049f, 0.139769f, 0.140490f, 0.141210f, 0.141931f, 0.142651f, 0.143372f, 0.144092f, 0.144813f, 0.145533f, 0.146254f, 
	 0.146974f, 0.147695f, 0.148415f, 0.149135f, 0.149856f, 0.150576f, 0.151297f, 0.152017f, 0.152738f, 0.153458f, 0.154179f, 0.154899f, 
	 0.155620f, 0.156340f, 0.157061f, 0.157781f, 0.158501f, 0.159222f, 0.159942f, 0.160663f, 0.161383f, 0.162104f, 0.162824f, 0.163545f, 
	 0.164265f, 0.164986f, 0.165706f, 0.166427f, 0.167147f, 0.167867f, 0.168588f, 0.169308f, 0.170029f, 0.170749f, 0.171470f, 0.172190f, 
	 0.172911f, 0.173631f, 0.174352f, 0.175072f, 0.175793f, 0.176513f, 0.177233f, 0.177954f, 0.178674f, 0.179395f, 0.180115f, 0.180836f, 
	 0.181556f, 0.182277f, 0.182997f, 0.183718f, 0.184438f, 0.185159f, 0.185879f, 0.186599f, 0.187320f, 0.188040f, 0.188761f, 0.189481f, 
	 0.190202f, 0.190922f, 0.191643f, 0.192363f, 0.193084f, 0.193804f, 0.194524f, 0.195245f, 0.195965f, 0.196686f, 0.197406f, 0.198127f, 
	 0.198847f, 0.199568f, 0.200288f, 0.201009f, 0.201729f, 0.202450f, 0.203170f, 0.203890f, 0.204611f, 0.205331f, 0.206052f, 0.206772f, 
	 0.207493f, 0.208213f, 0.208934f, 0.209654f, 0.210375f, 0.211095f, 0.211816f, 0.212536f, 0.213256f, 0.213977f, 0.214697f, 0.215418f, 
	 0.216138f, 0.216859f, 0.217579f, 0.218300f, 0.219020f, 0.219741f, 0.220461f, 0.221182f, 0.221902f, 0.222622f, 0.223343f, 0.224063f, 
	 0.224784f, 0.225504f, 0.226225f, 0.226945f, 0.227666f, 0.228386f, 0.229107f, 0.229827f, 0.230548f, 0.231268f, 0.231988f, 0.232709f, 
	 0.233429f, 0.234150f, 0.234870f, 0.235591f, 0.236311f, 0.237032f, 0.237752f, 0.238473f, 0.239193f, 0.239914f, 0.240634f, 0.241354f, 
	 0.242075f, 0.242795f, 0.243516f, 0.244236f, 0.244957f, 0.245677f, 0.246398f, 0.247118f, 0.247839f, 0.248559f, 0.249280f, 0.250000f, 
	 0.250720f, 0.251441f, 0.252161f, 0.252882f, 0.253602f, 0.254323f, 0.255043f, 0.255764f, 0.256484f, 0.257205f, 0.257925f, 0.258646f, 
	 0.259366f, 0.260086f, 0.260807f, 0.261527f, 0.262248f, 0.262968f, 0.263689f, 0.264409f, 0.265130f, 0.265850f, 0.266571f, 0.267291f, 
	 0.268012f, 0.268732f, 0.269452f, 0.270173f, 0.270893f, 0.271614f, 0.272334f, 0.273055f, 0.273775f, 0.274496f, 0.275216f, 0.275937f, 
	 0.276657f, 0.277378f, 0.278098f, 0.278818f, 0.279539f, 0.280259f, 0.280980f, 0.281700f, 0.282421f, 0.283141f, 0.283862f, 0.284582f, 
	 0.285303f, 0.286023f, 0.286744f, 0.287464f, 0.288184f, 0.288905f, 0.289625f, 0.290346f, 0.291066f, 0.291787f, 0.292507f, 0.293228f, 
	 0.293948f, 0.294669f, 0.295389f, 0.296109f, 0.296830f, 0.297550f, 0.298271f, 0.298991f, 0.299712f, 0.300432f, 0.301153f, 0.301873f, 
	 0.302594f, 0.303314f, 0.304035f, 0.304755f, 0.305476f, 0.306196f, 0.306916f, 0.307637f, 0.308357f, 0.309078f, 0.309798f, 0.310519f, 
	 0.311239f, 0.311960f, 0.312680f, 0.313401f, 0.314121f, 0.314842f, 0.315562f, 0.316282f, 0.317003f, 0.317723f, 0.318444f, 0.319164f, 
	 0.319885f, 0.320605f, 0.321326f, 0.322046f, 0.322767f, 0.323487f, 0.324207f, 0.324928f, 0.325648f, 0.326369f, 0.327089f, 0.327810f, 
	 0.328530f, 0.329251f, 0.329971f, 0.330692f, 0.331412f, 0.332133f, 0.332853f, 0.333573f, 0.334294f, 0.335014f, 0.335735f, 0.336455f, 
	 0.337176f, 0.337896f, 0.338617f, 0.339337f, 0.340058f, 0.340778f, 0.341499f, 0.342219f, 0.342939f, 0.343660f, 0.344380f, 0.345101f, 
	 0.345821f, 0.346542f, 0.347262f, 0.347983f, 0.348703f, 0.349424f, 0.350144f, 0.350865f, 0.351585f, 0.352305f, 0.353026f, 0.353746f, 
	 0.354467f, 0.355187f, 0.355908f, 0.356628f, 0.357349f, 0.358069f, 0.358790f, 0.359510f, 0.360231f, 0.360951f, 0.361671f, 0.362392f, 
	 0.363112f, 0.363833f, 0.364553f, 0.365274f, 0.365994f, 0.366715f, 0.367435f, 0.368156f, 0.368876f, 0.369597f, 0.370317f, 0.371037f, 
	 0.371758f, 0.372478f, 0.373199f, 0.373919f, 0.374640f, 0.375360f, 0.376081f, 0.376801f, 0.377522f, 0.378242f, 0.378963f, 0.379683f, 
	 0.380403f, 0.381124f, 0.381844f, 0.382565f, 0.383285f, 0.384006f, 0.384726f, 0.385447f, 0.386167f, 0.386888f, 0.387608f, 0.388329f, 
	 0.389049f, 0.389769f, 0.390490f, 0.391210f, 0.391931f, 0.392651f, 0.393372f, 0.394092f, 0.394813f, 0.395533f, 0.396254f, 0.396974f, 
	 0.397695f, 0.398415f, 0.399135f, 0.399856f, 0.400576f, 0.401297f, 0.402017f, 0.402738f, 0.403458f, 0.404179f, 0.404899f, 0.405620f, 
	 0.406340f, 0.407061f, 0.407781f, 0.408501f, 0.409222f, 0.409942f, 0.410663f, 0.411383f, 0.412104f, 0.412824f, 0.413545f, 0.414265f, 
	 0.414986f, 0.415706f, 0.416427f, 0.417147f, 0.417867f, 0.418588f, 0.419308f, 0.420029f, 0.420749f, 0.421470f, 0.422190f, 0.422911f, 
	 0.423631f, 0.424352f, 0.425072f, 0.425793f, 0.426513f, 0.427233f, 0.427954f, 0.428674f, 0.429395f, 0.430115f, 0.430836f, 0.431556f, 
	 0.432277f, 0.432997f, 0.433718f, 0.434438f, 0.435158f, 0.435879f, 0.436599f, 0.437320f, 0.438040f, 0.438761f, 0.439481f, 0.440202f, 
	 0.440922f, 0.441643f, 0.442363f, 0.443084f, 0.443804f, 0.444524f, 0.445245f, 0.445965f, 0.446686f, 0.447406f, 0.448127f, 0.448847f, 
	 0.449568f, 0.450288f, 0.451009f, 0.451729f, 0.452450f, 0.453170f, 0.453891f, 0.454611f, 0.455331f, 0.456052f, 0.456772f, 0.457493f, 
	 0.458213f, 0.458934f, 0.459654f, 0.460375f, 0.461095f, 0.461816f, 0.462536f, 0.463256f, 0.463977f, 0.464697f, 0.465418f, 0.466138f, 
	 0.466859f, 0.467579f, 0.468300f, 0.469020f, 0.469741f, 0.470461f, 0.471182f, 0.471902f, 0.472622f, 0.473343f, 0.474063f, 0.474784f, 
	 0.475504f, 0.476225f, 0.476945f, 0.477666f, 0.478386f, 0.479107f, 0.479827f, 0.480548f, 0.481268f, 0.481988f, 0.482709f, 0.483429f, 
	 0.484150f, 0.484870f, 0.485591f, 0.486311f, 0.487032f, 0.487752f, 0.488473f, 0.489193f, 0.489914f, 0.490634f, 0.491354f, 0.492075f, 
	 0.492795f, 0.493516f, 0.494236f, 0.494957f, 0.495677f, 0.496398f, 0.497118f, 0.497839f, 0.498559f, 0.499280f, 0.500000f, 0.500720f, 
	 0.501441f, 0.502161f, 0.502882f, 0.503602f, 0.504323f, 0.505043f, 0.505764f, 0.506484f, 0.507205f, 0.507925f, 0.508646f, 0.509366f, 
	 0.510086f, 0.510807f, 0.511527f, 0.512248f, 0.512968f, 0.513689f, 0.514409f, 0.515130f, 0.515850f, 0.516571f, 0.517291f, 0.518012f, 
	 0.518732f, 0.519452f, 0.520173f, 0.520893f, 0.521614f, 0.522334f, 0.523055f, 0.523775f, 0.524496f, 0.525216f, 0.525937f, 0.526657f, 
	 0.527378f, 0.528098f, 0.528818f, 0.529539f, 0.530259f, 0.530980f, 0.531700f, 0.532421f, 0.533141f, 0.533862f, 0.534582f, 0.535303f, 
	 0.536023f, 0.536744f, 0.537464f, 0.538184f, 0.538905f, 0.539625f, 0.540346f, 0.541066f, 0.541787f, 0.542507f, 0.543228f, 0.543948f, 
	 0.544669f, 0.545389f, 0.546109f, 0.546830f, 0.547550f, 0.548271f, 0.548991f, 0.549712f, 0.550432f, 0.551153f, 0.551873f, 0.552594f, 
	 0.553314f, 0.554035f, 0.554755f, 0.555476f, 0.556196f, 0.556916f, 0.557637f, 0.558357f, 0.559078f, 0.559798f, 0.560519f, 0.561239f, 
	 0.561960f, 0.562680f, 0.563401f, 0.564121f, 0.564842f, 0.565562f, 0.566282f, 0.567003f, 0.567723f, 0.568444f, 0.569164f, 0.569885f, 
	 0.570605f, 0.571326f, 0.572046f, 0.572767f, 0.573487f, 0.574207f, 0.574928f, 0.575648f, 0.576369f, 0.577089f, 0.577810f, 0.578530f, 
	 0.579251f, 0.579971f, 0.580692f, 0.581412f, 0.582133f, 0.582853f, 0.583573f, 0.584294f, 0.585014f, 0.585735f, 0.586455f, 0.587176f, 
	 0.587896f, 0.588617f, 0.589337f, 0.590058f, 0.590778f, 0.591499f, 0.592219f, 0.592939f, 0.593660f, 0.594380f, 0.595101f, 0.595821f, 
	 0.596542f, 0.597262f, 0.597983f, 0.598703f, 0.599424f, 0.600144f, 0.600865f, 0.601585f, 0.602305f, 0.603026f, 0.603746f, 0.604467f, 
	 0.605187f, 0.605908f, 0.606628f, 0.607349f, 0.608069f, 0.608790f, 0.609510f, 0.610231f, 0.610951f, 0.611671f, 0.612392f, 0.613112f, 
	 0.613833f, 0.614553f, 0.615274f, 0.615994f, 0.616715f, 0.617435f, 0.618156f, 0.618876f, 0.619597f, 0.620317f, 0.621037f, 0.621758f, 
	 0.622478f, 0.623199f, 0.623919f, 0.624640f, 0.625360f, 0.626081f, 0.626801f, 0.627522f, 0.628242f, 0.628963f, 0.629683f, 0.630403f, 
	 0.631124f, 0.631844f, 0.632565f, 0.633285f, 0.634006f, 0.634726f, 0.635447f, 0.636167f, 0.636888f, 0.637608f, 0.638329f, 0.639049f, 
	 0.639769f, 0.640490f, 0.641210f, 0.641931f, 0.642651f, 0.643372f, 0.644092f, 0.644813f, 0.645533f, 0.646254f, 0.646974f, 0.647695f, 
	 0.648415f, 0.649135f, 0.649856f, 0.650576f, 0.651297f, 0.652017f, 0.652738f, 0.653458f, 0.654179f, 0.654899f, 0.655620f, 0.656340f, 
	 0.657061f, 0.657781f, 0.658501f, 0.659222f, 0.659942f, 0.660663f, 0.661383f, 0.662104f, 0.662824f, 0.663545f, 0.664265f, 0.664986f, 
	 0.665706f, 0.666427f, 0.667147f, 0.667867f, 0.668588f, 0.669308f, 0.670029f, 0.670749f, 0.671470f, 0.672190f, 0.672911f, 0.673631f, 
	 0.674352f, 0.675072f, 0.675793f, 0.676513f, 0.677233f, 0.677954f, 0.678674f, 0.679395f, 0.680115f, 0.680836f, 0.681556f, 0.682277f, 
	 0.682997f, 0.683718f, 0.684438f, 0.685158f, 0.685879f, 0.686599f, 0.687320f, 0.688040f, 0.688761f, 0.689481f, 0.690202f, 0.690922f, 
	 0.691643f, 0.692363f, 0.693084f, 0.693804f, 0.694524f, 0.695245f, 0.695965f, 0.696686f, 0.697406f, 0.698127f, 0.698847f, 0.699568f, 
	 0.700288f, 0.701009f, 0.701729f, 0.702450f, 0.703170f, 0.703891f, 0.704611f, 0.705331f, 0.706052f, 0.706772f, 0.707493f, 0.708213f, 
	 0.708934f, 0.709654f, 0.710375f, 0.711095f, 0.711816f, 0.712536f, 0.713256f, 0.713977f, 0.714697f, 0.715418f, 0.716138f, 0.716859f, 
	 0.717579f, 0.718300f, 0.719020f, 0.719741f, 0.720461f, 0.721182f, 0.721902f, 0.722622f, 0.723343f, 0.724063f, 0.724784f, 0.725504f, 
	 0.726225f, 0.726945f, 0.727666f, 0.728386f, 0.729107f, 0.729827f, 0.730548f, 0.731268f, 0.731988f, 0.732709f, 0.733429f, 0.734150f, 
	 0.734870f, 0.735591f, 0.736311f, 0.737032f, 0.737752f, 0.738473f, 0.739193f, 0.739914f, 0.740634f, 0.741354f, 0.742075f, 0.742795f, 
	 0.743516f, 0.744236f, 0.744957f, 0.745677f, 0.746398f, 0.747118f, 0.747839f, 0.748559f, 0.749280f, 0.750000f, 0.750720f, 0.751441f, 
	 0.752161f, 0.752882f, 0.753602f, 0.754323f, 0.755043f, 0.755764f, 0.756484f, 0.757205f, 0.757925f, 0.758646f, 0.759366f, 0.760086f, 
	 0.760807f, 0.761527f, 0.762248f, 0.762968f, 0.763689f, 0.764409f, 0.765130f, 0.765850f, 0.766571f, 0.767291f, 0.768012f, 0.768732f, 
	 0.769452f, 0.770173f, 0.770893f, 0.771614f, 0.772334f, 0.773055f, 0.773775f, 0.774496f, 0.775216f, 0.775937f, 0.776657f, 0.777378f, 
	 0.778098f, 0.778818f, 0.779539f, 0.780259f, 0.780980f, 0.781700f, 0.782421f, 0.783141f, 0.783862f, 0.784582f, 0.785303f, 0.786023f, 
	 0.786744f, 0.787464f, 0.788184f, 0.788905f, 0.789625f, 0.790346f, 0.791066f, 0.791787f, 0.792507f, 0.793228f, 0.793948f, 0.794669f, 
	 0.795389f, 0.796109f, 0.796830f, 0.797550f, 0.798271f, 0.798991f, 0.799712f, 0.800432f, 0.801153f, 0.801873f, 0.802594f, 0.803314f, 
	 0.804035f, 0.804755f, 0.805476f, 0.806196f, 0.806916f, 0.807637f, 0.808357f, 0.809078f, 0.809798f, 0.810519f, 0.811239f, 0.811960f, 
	 0.812680f, 0.813401f, 0.814121f, 0.814842f, 0.815562f, 0.816282f, 0.817003f, 0.817723f, 0.818444f, 0.819164f, 0.819885f, 0.820605f, 
	 0.821326f, 0.822046f, 0.822767f, 0.823487f, 0.824207f, 0.824928f, 0.825648f, 0.826369f, 0.827089f, 0.827810f, 0.828530f, 0.829251f, 
	 0.829971f, 0.830692f, 0.831412f, 0.832133f, 0.832853f, 0.833573f, 0.834294f, 0.835014f, 0.835735f, 0.836455f, 0.837176f, 0.837896f, 
	 0.838617f, 0.839337f, 0.840058f, 0.840778f, 0.841499f, 0.842219f, 0.842939f, 0.843660f, 0.844380f, 0.845101f, 0.845821f, 0.846542f, 
	 0.847262f, 0.847983f, 0.848703f, 0.849424f, 0.850144f, 0.850865f, 0.851585f, 0.852305f, 0.853026f, 0.853746f, 0.854467f, 0.855187f, 
	 0.855908f, 0.856628f, 0.857349f, 0.858069f, 0.858790f, 0.859510f, 0.860231f, 0.860951f, 0.861671f, 0.862392f, 0.863112f, 0.863833f, 
	 0.864553f, 0.865274f, 0.865994f, 0.866715f, 0.867435f, 0.868156f, 0.868876f, 0.869597f, 0.870317f, 0.871037f, 0.871758f, 0.872478f, 
	 0.873199f, 0.873919f, 0.874640f, 0.875360f, 0.876081f, 0.876801f, 0.877522f, 0.878242f, 0.878963f, 0.879683f, 0.880403f, 0.881124f, 
	 0.881844f, 0.882565f, 0.883285f, 0.884006f, 0.884726f, 0.885447f, 0.886167f, 0.886888f, 0.887608f, 0.888329f, 0.889049f, 0.889769f, 
	 0.890490f, 0.891210f, 0.891931f, 0.892651f, 0.893372f, 0.894092f, 0.894813f, 0.895533f, 0.896254f, 0.896974f, 0.897695f, 0.898415f, 
	 0.899135f, 0.899856f, 0.900576f, 0.901297f, 0.902017f, 0.902738f, 0.903458f, 0.904179f, 0.904899f, 0.905620f, 0.906340f, 0.907061f, 
	 0.907781f, 0.908501f, 0.909222f, 0.909942f, 0.910663f, 0.911383f, 0.912104f, 0.912824f, 0.913545f, 0.914265f, 0.914986f, 0.915706f, 
	 0.916427f, 0.917147f, 0.917867f, 0.918588f, 0.919308f, 0.920029f, 0.920749f, 0.921470f, 0.922190f, 0.922911f, 0.923631f, 0.924352f, 
	 0.925072f, 0.925793f, 0.926513f, 0.927233f, 0.927954f, 0.928674f, 0.929395f, 0.930115f, 0.930836f, 0.931556f, 0.932277f, 0.932997f, 
	 0.933718f, 0.934438f, 0.935158f, 0.935879f, 0.936599f, 0.937320f, 0.938040f, 0.938761f, 0.939481f, 0.940202f, 0.940922f, 0.941643f, 
	 0.942363f, 0.943084f, 0.943804f, 0.944524f, 0.945245f, 0.945965f, 0.946686f, 0.947406f, 0.948127f, 0.948847f, 0.949568f, 0.950288f, 
	 0.951009f, 0.951729f, 0.952450f, 0.953170f, 0.953891f, 0.954611f, 0.955331f, 0.956052f, 0.956772f, 0.957493f, 0.958213f, 0.958934f, 
	 0.959654f, 0.960375f, 0.961095f, 0.961816f, 0.962536f, 0.963256f, 0.963977f, 0.964697f, 0.965418f, 0.966138f, 0.966859f, 0.967579f, 
	 0.968300f, 0.969020f, 0.969741f, 0.970461f, 0.971182f, 0.971902f, 0.972622f, 0.973343f, 0.974063f, 0.974784f, 0.975504f, 0.976225f, 
	 0.976945f, 0.977666f, 0.978386f, 0.979107f, 0.979827f, 0.980548f, 0.981268f, 0.981988f, 0.982709f, 0.983429f, 0.984150f, 0.984870f, 
	 0.985591f, 0.986311f, 0.987032f, 0.987752f, 0.988473f, 0.989193f, 0.989914f, 0.990634f, 0.991354f, 0.992075f, 0.992795f, 0.993516f, 
	 0.994236f, 0.994957f, 0.995677f, 0.996398f, 0.997118f, 0.997839f, 0.998559f, 0.999280f, 1.000000f
};

//////////////////////////////////////////////////////////////
//
// Endian neutral reads 
//
//////////////////////////////////////////////////////////////
// 
uint16_t getPCD16(uint8_t buffer[]) {
	return ((uint16_t) buffer[0])<<8 | (uint16_t) buffer[1];	
}

uint32_t getPCD32(uint8_t buffer[]) {
	return ((uint32_t) buffer[0])<<24 | ((uint32_t) buffer[1])<<16 | ((uint32_t) buffer[2])<<8 | (uint32_t) buffer[3];	
}

//////////////////////////////////////////////////////////////
//
// Utility File functions 
//
//////////////////////////////////////////////////////////////
// 
size_t readBytes(FILE *fp, const size_t length, uint8_t *data)
{
	int c;
	size_t count;
	if (length == 0) return(0);
	count=0;
	
	switch (length)
	{
		case 0:
			break;
		case 1:
		{
			c=getc(fp);
			if (c == EOF)
				break;
			*data++=(uint8_t) c;
			count++;
		}
		case 2:
		{
			c=getc(fp);
			if (c == EOF)
				break;
			*data++=(uint8_t) c;
			count++;
		}
		default:
		{
			count=(size_t) fread(data,1,length,fp);
			break;
		}
	}
	return(count);
}


//////////////////////////////////////////////////////////////
//
// Utility String functions 
//
//////////////////////////////////////////////////////////////
// 

int compareBytes(const char *buffer, const char *string)
{
	size_t length = strlen(string);
	return strncmp(buffer, string, length);
}

void copyWithoutPadding(char *dest, const char *src, int length)
{
	int i;
	dest[length] = 0x0;
	bool inPadding = true;
	for (i = length - 1; i >=0; i--) {
		if ((src[i] == ' ') && inPadding) {
			dest[i] = 0x0;
		}
		else {
			dest[i] = src[i];
			inPadding = false;
		}
	}	
}


//////////////////////////////////////////////////////////////
//
// Huffman decoder structure definitions
// These support classes 1 - 4 encoding
//
//////////////////////////////////////////////////////////////
struct ReadBuffer
{
	uint8_t sbuffer[KSectorSize];
	FILE *fp;
	unsigned long sum;
	unsigned long bits;
	uint8_t *p;
};

struct hctEntry 
{ 
	uint8_t length;										// (length - 1) in bits, so ranges from 0 - 15; max actual length is 16
	uint8_t codeWord[2];								// 16 bit codeword, left justified
	uint8_t key;										// the actual decoded value
};

struct hctTable  
{ 
	uint8_t entries;									// Number of hctEntries in this table
	struct hctEntry entry[256];							// NOTE: the actual length is 4*entries (+1 for the entire structure)
														// 256 is the maximum value
														// The EPT descriptor is after the last actual entry
														// Bits 2:0 define which tables exist
														// Then the actual EPT tables. But EPT never actually seems to have
														// been implemented...........													
};

struct huffTable
{
	uint8_t key[0x10000];
	uint8_t len[0x10000];	
};


struct huffTables {
	struct huffTable ht[3];
};

#define kHuffmanErrorLen 0x1f


//////////////////////////////////////////////////////////////
//
// Huffman decoder implementation
// This is a standard table-based Huffman decoder
//
//////////////////////////////////////////////////////////////

static void readHuffTable(struct hctTable *source, struct huffTable *destination, int *number)
{
	long i;
	struct hctEntry *sub;
	*number=(source->entries)+1;
	for(i=0;i<0x10000;i++) {
		destination->key[i] = 0x7f;
		destination->len[i] = kHuffmanErrorLen;
	}
#ifdef __debug
	fprintf(stderr, "Number of Huffman Tree entries: %d\n", *number);
#endif
	
	for(i=0;i<*number;i++)
	{
		sub = (struct hctEntry *)(((uint8_t *)source)+1+i*sizeof(*sub));
		unsigned int len = (unsigned int) (sub->length + 1);
		if (len > 16) {
				throw "Huffman code error!!";
		}
		
#ifdef __debug
//		fprintf(stderr, "Huffman item %d: %x len %d key %x\n", i, getPCD16(sub->codeWord), (unsigned int) (sub->length + 1), sub->key);
#endif
		unsigned int index = 0;
		for (index = 0; index < (0x1u << (16u-len)); index++) {
			uint16_t loc = getPCD16(sub->codeWord) | index;
			destination->key[loc] = sub->key;
			destination->len[loc] = len;
		}
	}
}

int readNextSector(ReadBuffer *buffer)
{
	size_t d;
	size_t n = KSectorSize;
	uint8_t *ptr = buffer->sbuffer;
	if(n == 0) return true;
	for(;;)
	{
		d=fread(ptr, 1, n, buffer->fp);
		if( d < 1 ) return false;
		n-=d;
		if ((n == 0) || (feof(buffer->fp) != 0)) break;
		ptr += d;
	}
	return true;
}

void PCDGetBits(ReadBuffer* b, int n) 
{  
	b->sum = (b->sum << n) & 0xffffffff; 
	b->bits -= n; 
	while (b->bits <= 24) 
	{ 
		if (b->p >= (b->sbuffer+KSectorSize)) 
		{ 
			if (!readNextSector(b)) {
				throw "Unexpected end of file in Huffman sequence";
			}
			b->p = b->sbuffer; 
		} 
		b->sum |= ((unsigned int) (*b->p) << (24-b->bits)); 
		b->bits+=8; 
		b->p++; 
	} 
}

static void initReadBuffer(ReadBuffer *buffer, FILE *file) 
{
	buffer->fp = file;
	buffer->p= buffer->sbuffer + sizeof(buffer->sbuffer);
	buffer->bits = 0;
	buffer->sum = 0;
	// Initialise the shift register
	PCDGetBits(buffer, 0);
}

void syncHuffman(ReadBuffer* b)
{
	while (!((b->sum & 0x00fff000) == 0x00fff000)) {
		PCDGetBits(b, 8);
	}
	while (!((b->sum & 0xffffff00) == 0xfffffe00)) {
		PCDGetBits(b, 1);
	}
#ifdef __debug
	//	fprintf(stderr, "Sync at : %d %d ftell:%d -> ", ftell(b->fp));
#endif
	
}

void PCDDecodeHuffman(ReadBuffer* b, struct huffTable *huf, uint8_t *dest, int length)
{
	int i;
	uint16_t code;
	uint8_t *ptr = dest;
	
	for (i = 0; i < length; i++) {
		code  = (b->sum >> 16) & 0xffff;
		if (huf->len[code] == kHuffmanErrorLen) {
#ifdef mInformPrintf
			fprintf(stderr, "*** Warning : Attempting to recover from Huffman sequence error......\n");
#endif
#ifdef __debug
			// If we're debugging, then this is almost certainly our own error
			throw "Huffman code sequence error";
#endif
			// Recovery procedure from error is to zero this sequence and just go on 
			// to the next - as these are deltas, we just lose one sequence of 
			// incremental information
			for (i = 0; i < length; i++) {
				*dest++ = 0x0;
			}
			syncHuffman(b);
			return;
		}
		else {
			*ptr++ = huf->key[code];
			PCDGetBits(b, huf->len[code]);
		}
	}	
}

void readAllHuffmanTables(FILE *fp, off_t offset, huffTables *tables, int numTables)
{
	int numBytes = kSceneSectorSize * (numTables == 1 ? 1 : 2) * sizeof(uint8_t);
	uint8_t *buffer = (uint8_t *) malloc(numBytes);
	
	if (buffer == NULL) {
		throw "memory allocation error";
		}
	
	fseek(fp, offset, SEEK_SET);
	fread(buffer, numBytes, 1, fp);

	int num = 0;
	int i;
	uint8_t *ptr = buffer;
	// Read in the Huffman decoder tables, and process into something we can use
	for (i = 0; i < numTables; i++) {
		
#ifdef __debug
		fprintf(stderr, "Processing Table number: %i\n", i);
#endif
		readHuffTable((struct hctTable *)ptr, &(tables->ht[i]), &num);
		// Move the pointer formward by the size of what we just read
		ptr+= sizeof(uint8_t)*(num*4 + 1);
		if ((num < 4) && (i > 0)) {
			// Assume the previous table applies(!)
			memcpy( &(tables->ht[i]), &(tables->ht[i-1]), sizeof(huffTable));
		}
	}
#ifdef __debug
	uint8_t eptDescriptor = *ptr++;
	fprintf(stderr, "EPT descriptor: %x\n", eptDescriptor);
#endif
	free(buffer);
}


//////////////////////////////////////////////////////////////
//
// Reader for the delta tables - this supports base, 16 base 
// and 64Base
//
//////////////////////////////////////////////////////////////

static bool readPCDDeltas(ReadBuffer *buf, struct huffTables *huf, int sceneSelect, int sequenceSize, int sequencesToProcess, uint8_t *data[3], off_t colOffset)
{		
	size_t count;
	unsigned long plane, row;
	unsigned int sequence;
	int planeTrack = ((data[0] != NULL) ? 0x1 : 0) | ((data[1] != NULL) ? 0x2 : 0) | ((data[2] != NULL) ? 0x4 : 0);
	
	if (sequencesToProcess == 0) {
		// for anything less than 64base, one sequence per row
		sequencesToProcess = (sceneSelect == k64Base) ? 1 : PCDLumaHeight[sceneSelect] + 2*PCDChromaHeight[sceneSelect];
	}
	
	plane = 0;
	row = 0;
	sequence = 0;
	while (((planeTrack != 0x0) || (row < PCDLumaHeight[sceneSelect])) && (sequencesToProcess > 0)) {
		// First check we're at the start of a sequence
		syncHuffman(buf);
		// Get the first 24 bits into the shift register - these have the plane, row and sequence numbers
		PCDGetBits(buf, 16);
		row = (buf->sum >> RowShift[sceneSelect]) & RowMask[sceneSelect];
		sequence = (buf->sum >> SequenceShift[sceneSelect]) & SequenceMask[sceneSelect];
		plane = (buf->sum >> PlaneShift[sceneSelect]) & PlaneMask[sceneSelect];
		row *= (plane == 0 ? 1 : RowSubSample[sceneSelect]);
		
#ifdef __debug
//		fprintf(stderr, "Row %d, Sequence %d, data:%x\n",  row, sequence, buf->sum);
#endif
		
		for (count = 0; count < HuffmanHeaderSize[sceneSelect]; count++) {
			// IPE headers have 32 bits of data
			PCDGetBits(buf, 8);
		}

		if (row < PCDLumaHeight[sceneSelect]) {
#ifdef __debug
//			fprintf(stderr, "Delta plane: %d row: %d\n", plane, row);
#endif
			switch (plane)
			{
				case 0:
				{
					PCDDecodeHuffman(buf, 
									 &(huf->ht[0]), 
									 data[0] + row*PCDLumaWidth[sceneSelect] + sequence*sequenceSize + colOffset, 
									 sequenceSize == 0 ? PCDLumaWidth[sceneSelect] : sequenceSize);
					planeTrack &= 0x6;
					break;
				}
				case 2:
				{
					if (data[1] != NULL) {
						PCDDecodeHuffman(buf, 
									 &(huf->ht[1]), 
									 data[1]+(row>>1)*PCDChromaWidth[sceneSelect] + sequence*sequenceSize + (colOffset>>1), 
									 sequenceSize == 0 ? PCDChromaWidth[sceneSelect] : sequenceSize);
					}
					planeTrack &= 0x5;
					break;
				}
				case 3:
				// Handle the strange IPE situation - plane numbers are different(!)
				case 4:
				{
					if (data[2] != NULL) {
						PCDDecodeHuffman(buf,
									 &(huf->ht[2]), 
									 data[2]+(row>>1)*PCDChromaWidth[sceneSelect] + sequence*sequenceSize + (colOffset>>1), 
									 sequenceSize == 0 ? PCDChromaWidth[sceneSelect] : sequenceSize);
					}
					planeTrack &= 0x3;
					break;
				}
				default:
				{
					throw "Corrupt Image";
				}
			}
		}
		else {
#ifdef __debug
			fprintf(stderr, "Delta plane invalid row: %ld row: %ld\n", plane, row);
#endif
		}
		sequencesToProcess--;
	}
	return(true);		
}

//////////////////////////////////////////////////////////////
//
// Interpolation routines 
//
//////////////////////////////////////////////////////////////
#define pcdMin(x,y) (((x) < (y)) ? x : y)
#define pcdMax(x,y) (((x) < (y)) ? y : x)
#define pcdPin(low, x, high) (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))

// Data structure to be passed to each thread - effectively a tile description
struct upResInterpolateData {
	uint8_t *base;
	uint8_t *dest;
	uint8_t *luma;
	unsigned int width;
	unsigned int height;
	bool hasDeltas;
	unsigned int startRow;
	unsigned int endRow;
};

//////////////////////////////////////////////////////////////
//
// basic "Kodak standard" bilinear upres interpolator
//
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//
// basic "Kodak standard" bilinear upres interpolator
//
//////////////////////////////////////////////////////////////
pcdThreadFunction upResInterpolate(void *t)
{
	struct upResInterpolateData *rd = (struct upResInterpolateData *) t;
	unsigned int row, column; 
	ptrdiff_t indexDelta;
	int sum;
	int8_t *deltaBase = (int8_t *) rd->dest;
	
	// This is as intended by Kodak - linear interpolation
	uint8_t *basePix, *basePix01, *basePix10, *basePix11;
	unsigned int rowPlus, columnPlus;
	for (row = rd->startRow>>1; row < rd->endRow>>1; row++) {
		for (column = 0; column < rd->width>>1; column++) {
			// When upresing, the factor is always two
			// Note here we're iterating in rd->base coordinates
			columnPlus = pcdMin(column + 1, (rd->width>>1)-1);
			rowPlus = pcdMin(row + 1, (rd->height>>1)-1);
			basePix = rd->base + column + row * (rd->width>>1);
			basePix01 = rd->base + columnPlus + row * (rd->width>>1);					
			basePix10 = rd->base + column + rowPlus * (rd->width>>1);		
			basePix11 = rd->base + columnPlus + rowPlus * (rd->width>>1);	
			
			// base Pixel
			indexDelta = (column<<1) + (row<<1) * rd->width;
			sum = (int) (*basePix);
			if (rd->hasDeltas) sum += (int) (*(deltaBase + indexDelta));
			sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
			*(rd->dest + indexDelta) = (uint8_t) sum;
			
			// 01 Pixel
			indexDelta = (column<<1) + 1 + (row<<1) * rd->width;
			sum = ((int) (*basePix) + (int) (*basePix01) + 1) >> 1;
			if (rd->hasDeltas) sum += (int) (*(deltaBase + indexDelta));
			sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
			*(rd->dest + indexDelta) = (uint8_t) sum;
			
			// 10 Pixel		
			indexDelta = (column<<1) + ((row<<1) + 1) * rd->width;
			sum = ((int) (*basePix) + (int) (*basePix10) + 1) >> 1;
			if (rd->hasDeltas) sum += (int) (*(deltaBase + indexDelta));
			sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
			*(rd->dest + indexDelta) = (uint8_t) sum;
			
			// 11 Pixel
			indexDelta = (column<<1) + 1 + ((row<<1) + 1) * rd->width;
#ifdef UseFourPixels
			sum = ((int) (*basePix) + (int) (*basePix01) + (int) (*basePix10) + (int) (*basePix11) + 2) >> 2;
#else
			sum = ((int) (*basePix) + (int) (*basePix11) + 1) >> 1;
#endif
			if (rd->hasDeltas) sum += (int) (*(deltaBase + indexDelta));
			sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
			*(rd->dest + indexDelta) = (uint8_t) sum;
			
		}
	}			
	return NULL;
}

#ifdef mUseNonGPLCode
#include "PCDLumaInterpolate.hpp"
#endif


void upResBuffer(uint8_t *base, uint8_t *dest, uint8_t *luma, unsigned int width, unsigned int height, int upResMethod, bool hasDeltas)
{
	unsigned int row, column;
	ptrdiff_t indexBase, indexDelta;
	int sum, thread;
	int previousRow = 0;
#ifndef mNoPThreads
	int rc;
	void *status;
	pcdThreadDescriptor threadDescriptors[kNumThreads];
	pthread_attr_t threadAttr;
	
	/* Initialize and set thread detached attribute */
	pthread_attr_init(&threadAttr);
	pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_JOINABLE);
	// Use minimum stacksize times two; we have only a few stack variables
	pthread_attr_setstacksize(&threadAttr, PTHREAD_STACK_MIN<<1);
#endif
#ifdef __PerformanceAnalysis
#ifdef qMacOS
	AbsoluteTime nowTime, bgnTime;
    bgnTime = UpTime();
#endif
#endif
	
	if (dest != NULL) {
#ifdef mUseNonGPLCode
		if ((upResMethod >= kUpResLumaIterpolate) && !hasDeltas && (luma != NULL)) {

			// This does a homogeniety minimisation routine.
			// We should only ever(!) use this for chroma interpolation
			struct upResInterpolateData rd[kNumThreads];
			for (thread = 0; thread < kNumThreads; thread++) {
				rd[thread].base = base;
				rd[thread].dest = dest;
				rd[thread].luma = luma;
				rd[thread].width = width;
				rd[thread].height = height;
				rd[thread].hasDeltas = hasDeltas;
				rd[thread].startRow = previousRow;
				rd[thread].endRow =height/kNumThreads*(thread+1);
				previousRow = rd[thread].endRow;
#ifndef mNoPThreads
				if (thread == (kNumThreads - 1)) {
					upResLumaInterpolatePassI(&(rd[thread]));
				}
				else {
					if (pcdStartThread(threadDescriptors[thread], threadAttr, upResLumaInterpolatePassI, (void *)&(rd[thread])) != 0) {
						// Too many threads already.....
						upResLumaInterpolatePassI(&(rd[thread]));
						// Don't try to join
						rd[thread].endRow = 0;
					}
				}
#else
				upResLumaInterpolatePassI(&(rd[thread]));
#endif
			}
#ifndef mNoPThreads
			pthread_attr_destroy(&threadAttr);
			for (thread = 0; thread < (kNumThreads-1); thread++) {
				if (rd[thread].endRow > 0) {
					rc = pcdThreadJoin(threadDescriptors[thread], &status);
				}
			}
#endif
			
			// For this algorithm, the easist thing is to prep the last rows and columns separately.....
			for (row = height-1; row < height; row++) {
				for (column = 0; column < width; column++) {
					*(dest + column + row * (width)) = *(base + (column>>1) + (row>>1) * (width>>1));
				}
			}
			for (row = 0; row < height; row++) {
				for (column = column-1; column < width; column++) {
					*(dest + column + row * (width)) = *(base + (column>>1) + (row>>1) * (width>>1));
				}
			}
			
			
			for (thread = 0; thread < kNumThreads; thread++) {
#ifndef mNoPThreads
				if (thread == (kNumThreads - 1)) {
					upResLumaInterpolatePassII(&(rd[thread]));
				}
				else {
					if (pcdStartThread(threadDescriptors[thread], threadAttr, upResLumaInterpolatePassII, (void *)&(rd[thread])) != 0) {
						// Too many threads already.....
						upResLumaInterpolatePassII(&(rd[thread]));
						// Don't try to join
						rd[thread].endRow = 0;
					}
				}
#else
				upResLumaInterpolatePassII(&(rd[thread]));
#endif
			}
#ifndef mNoPThreads
			pthread_attr_destroy(&threadAttr);
			for (thread = 0; thread < (kNumThreads-1); thread++) {
				if (rd[thread].endRow > 0) {
					rc = pcdThreadJoin(threadDescriptors[thread], &status);
				}
			}
#endif
#ifdef __PerformanceAnalysis
#ifdef qMacOS
			nowTime = UpTime();
			float uSec  = HowLong(nowTime, bgnTime);
			fprintf(stderr, " upResInterpolate: %.3f usec \n", uSec);
#endif
#endif
		}
		else
#endif
		if (upResMethod >= kUpResIterpolate) {
			struct upResInterpolateData rd[kNumThreads];
			for (thread = 0; thread < kNumThreads; thread++) {
				rd[thread].base = base;
				rd[thread].dest = dest;
				rd[thread].luma = luma;
				rd[thread].width = width;
				rd[thread].height = height;
				rd[thread].hasDeltas = hasDeltas;
				rd[thread].startRow = previousRow;
				rd[thread].endRow =height/kNumThreads*(thread+1);
				previousRow = rd[thread].endRow;
#ifndef mNoPThreads
				if (thread == (kNumThreads - 1)) {
					upResInterpolate(&(rd[thread]));
				}
				else {
					if (pcdStartThread(threadDescriptors[thread], threadAttr, upResInterpolate, (void *)&(rd[thread])) != 0) {
						// Too many threads already.....
						upResInterpolate(&(rd[thread]));
						// Don't try to join
						rd[thread].endRow = 0;
					}
				}
#else
				upResInterpolate(&(rd[thread]));
#endif
			}
#ifndef mNoPThreads
			pthread_attr_destroy(&threadAttr);
			status  = 0; // Avoid unreferenced local variable warning
			for (thread = 0; thread < (kNumThreads-1); thread++) {
				if (rd[thread].endRow > 0) {
					rc = pcdThreadJoin(threadDescriptors[thread], &status);
				}
			}
#endif
#ifdef __PerformanceAnalysis
#ifdef qMacOS
			nowTime = UpTime();
			float uSec  = HowLong(nowTime, bgnTime);
			fprintf(stderr, " upResInterpolate: %.3f usec \n", uSec);
#endif
#endif
		}
		else {
			// Here we do a very simple minded nearest neighbour look up; 
			// Shouldn't be used for any serious purpose.
			int8_t *deltaBase = (int8_t *) dest;
			for (row = 0; row < height; row++) {
				for (column = 0; column < width; column++) {
					// When upresing, the factor is always two
					indexBase = (column >> 1) + (row >> 1) * (width>>1);
					indexDelta = column + row * width;
					sum = ((int) *(base + indexBase));
					if (hasDeltas) {
						sum += ((int) *(deltaBase + indexDelta));
						sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
					}

					*(dest + indexDelta) = (uint8_t) sum;
				}
			}
		}
		// Now the new base is in the old dest....
	}
	
}

//////////////////////////////////////////////////////////////
//
// Test code
//
//////////////////////////////////////////////////////////////

#ifdef __debug
void dump8by8(uint8_t *b, int width)
{
	fprintf(stderr, "\nDump8x8x\n");
	int i, j;
	for (i=0; i<8; i++) {
		for (j=0; j<8; j++) {
			uint8_t val = *(b+j+i*width);
			fprintf(stderr, " %x", val);
		}
		fprintf(stderr, "\n");
	}
}

void dumpColumn(uint8_t *b, int col, int height, int width)
{
	fprintf(stderr, "\nDumpColumn\n");
	int i;
	for (i=0; i<height; i++) {
		uint8_t val = *(b+col+i*width);
		fprintf(stderr, " %x\n", val);
	}
}

void genTestBaseImage(int sceneNumber, uint8_t *luma, uint8_t *chroma1, uint8_t *chroma2)
{
	// Base image scene number......
	
	int row, column;
	
	for (row = 0; row < PCDLumaHeight[sceneNumber]; row++) {
		for (column = 0; column < PCDLumaWidth[sceneNumber]; column++) {
			bool block = ((row & 0x3) < 2) && ((column & 0x3) < 2);
			if (block) {
				*luma++ = 0xff;
			}
			else {
				*luma++ = 0x3f;
			}
			if (((row & 0x1) == 0x0) && ((column & 0x1) == 0x0)) {
				// write chroma
				// 156 and 137 are the "zero" values
				if (block) {
					*chroma1++ = 230;
					*chroma2++ = 230;
				}
				else {
					*chroma1++ = 156;
					*chroma2++ = 137;
				}
			}
		}
	}	
}

#endif

//////////////////////////////////////////////////////////////
//
// RGB conversion
//
//////////////////////////////////////////////////////////////

// Data structure to be passed to each thread - effectively a tile description
struct ConvertToRGBData {
	short outputSize;
	void *red;
	void *green;
	void *blue;
	void *alpha;
	ptrdiff_t d;
	size_t startRow;
	size_t endRow;
	size_t columns;
	size_t rows;
	uint8_t *lp;
	uint8_t *c1p;
	uint8_t *c2p;
	unsigned int resFactor;
	unsigned int imageRotate;
	size_t colorSpace;
	int whiteBalance;
};


//////////////////////////////////////////////////////////////
//
// The Micro CMM
//
//////////////////////////////////////////////////////////////
// 
// Normally, what we would do is to define a Photo CD color space, then
// hand that together with the data to a Color Management Module - e.g., 
// LittleCMS - and let it deal with all the nasty complex color space conversions.
// However, to keep this as standalone as possible, what is implemented here is 
// a "micro CMM", in the shape of a few LUTs and one matrix conversion. It does 
// everything that a full CMM would do.....admittedly, only for the limited 
// Photo CD to linear light to sRGB conversions that we need. And all in integer 
// math. And multi-threaded.
//
pcdThreadFunction convertToRGB(void *t)
{
	struct ConvertToRGBData *rd = (struct ConvertToRGBData *) t;
	size_t row = 0;
	size_t col = 0;
	int32_t Li = 0, C1i = 0, C2i = 0, ri = 0, gi = 0, bi = 0;
	int32_t rt = 0, gt = 0, bt = 0;
	ptrdiff_t chromaIndex = 0, lumaIndex = 0, destIndex = 0;
	
	for (row = rd->startRow; row != rd->endRow; row++) {
		for (col = 0; col != rd->columns; col++) {
			switch (rd->imageRotate) {
				case 0:
					destIndex = (col + row*rd->columns)*rd->d;
					break;
				case 1:
					destIndex = (row + (rd->columns - 1 - col)*rd->rows)*rd->d;
					break;
				case 2:					
					destIndex = (rd->columns - 1 - col + (rd->rows - 1 - row)*rd->columns)*rd->d;
					break;
				case 3:
					destIndex = (rd->rows - 1 - row + col*rd->rows)*rd->d;
					break;
				default:
					destIndex = (col + row*rd->columns)*rd->d;
					break;
			}
			lumaIndex = col + row * rd->columns;
			chromaIndex = (col>>rd->resFactor) + (row >> rd->resFactor) * (rd->columns >> rd->resFactor);
			
			if (rd->colorSpace == kPCDYCCColorSpace) {
				// Here we want the original YCC color space
				ri = pcdPin(0, (((int32_t) *(rd->lp + lumaIndex))<<10)/188, 1388);
				gi = pcdPin(0, (((int32_t) (rd->c1p)[chromaIndex])<<10)/188, 1388);
				bi = pcdPin(0, (((int32_t) (rd->c2p)[chromaIndex])<<10)/188, 1388);
			}
			else {
				// here one or the other of the RGB color spaces
				Li = *(rd->lp + lumaIndex) * 5573;								// Range 0 - 1,421,115
				if (rd->c1p != NULL) {
					C1i = (((int32_t) (rd->c1p)[chromaIndex]) - 156) * 9085;	// -1,417,260 to 899,415
				}
				if (rd->c2p != NULL) {
					C2i = (((int32_t) (rd->c2p)[chromaIndex]) - 137) * 7461;	// -1,022,157 to 880,398
				}
				ri = pcdPin(0, (Li + C2i) >> 10, 1388);							// 0 - 1388
				gi = pcdPin(0, (Li>>10) - C1i/5278 - C2i/2012, 1388);			// 0 - 1388
				bi = pcdPin(0, (Li + C1i) >> 10, 1388);							// 0 - 1388
			
				// Here we have RGB in the original photo CD color space. So we can either 
				// (a) pass that back raw, or
				// (b) convert to a CCIR709 linear light space, or
				// (c) convert to a sRGB space
				if ((rd->colorSpace == kPCDLinearCCIR709ColorSpace) || (rd->colorSpace == kPCDsRGBColorSpace)) {
					ri = toLinearLight[ri];
					gi = toLinearLight[gi];
					bi = toLinearLight[bi];
					// We only do whitebalance conversions for the processed spaces, not raw.....
					if (rd->whiteBalance == kPCDD50White) {
						// This implements the equivalent of:
						//	r = (0.9555f*r-0.0231f*g+0.0633f*b)/1.32;
						//	g = (-0.0283f*r+1.0100f*g+0.0211*b)/1.32;
						//	p = (0.0123f*r-0.0206f*g+1.3303f*b)/1.32;
						rt = ri;
						gt = gi;
						bt = bi;
						ri = (5930*rt - 143*gt + 393*bt)>>13;
						gi = (-176*rt + 6268*gt + 131*bt)>>13;
						bi = (76*rt - 128*gt + 8256*bt)>>13;
					}
				}
				if (rd->colorSpace == kPCDsRGBColorSpace) {
					// Recompress and pin
					ri = CCIR709tosRGB[pcdPin(0, ri, 1388)];
					gi = CCIR709tosRGB[pcdPin(0, gi, 1388)];
					bi = CCIR709tosRGB[pcdPin(0, bi, 1388)];
				}
				else {
					// just pin
					ri = pcdPin(0, ri, 1388);
					gi = pcdPin(0, gi, 1388);
					bi = pcdPin(0, bi, 1388);
				}
			}
			// Deliver back in the right format
			if (rd->outputSize == pcdFloatSize) {
				*(((float *) rd->red) + destIndex) = floatOutput[ri];
				*(((float *) rd->green) + destIndex) = floatOutput[gi];
				*(((float *) rd->blue) + destIndex) = floatOutput[bi];
				if (rd->alpha != NULL) *(((float *) rd->alpha) + destIndex) = 1.0f;
			}
			else if (rd->outputSize == pcdInt16Size) {
				*(((uint16_t *) rd->red) + destIndex) = uint16Output[ri];
				*(((uint16_t *) rd->green) + destIndex) = uint16Output[gi];
				*(((uint16_t *) rd->blue) + destIndex) = uint16Output[bi];
				if (rd->alpha != NULL) *(((uint16_t *) rd->alpha) + destIndex) = 0xffff;
			}
			else {
				*(((uint8_t *) rd->red) + destIndex) = uint8Output[ri];
				*(((uint8_t *) rd->green) + destIndex) = uint8Output[gi];
				*(((uint8_t *) rd->blue) + destIndex) = uint8Output[bi];
				if (rd->alpha != NULL) *(((uint8_t *) rd->alpha) + destIndex) = 0xff;					
			}
		}
	}
	return NULL;
}


//////////////////////////////////////////////////////////////
//
// Base (and lower) image reader 
//
//////////////////////////////////////////////////////////////


int readBaseImage(FILE *fp, int sceneNumber, int ICDOffset[kMaxScenes], uint8_t **luma, uint8_t **chroma1, uint8_t **chroma2)
{
	// Base image scene number......
	sceneNumber = (sceneNumber > kBase) ? kBase : sceneNumber;
	bool haveReadBase = false;
	
	while (!haveReadBase && (sceneNumber >= kBase16)) {
		try {
			size_t numBytes = PCDLumaWidth[sceneNumber]*PCDLumaHeight[sceneNumber]*sizeof(uint8_t)+1;
			*luma=(uint8_t *) malloc(numBytes);
			*chroma1=(uint8_t *) malloc(numBytes>>2);
			*chroma2=(uint8_t *) malloc(numBytes>>2);
			
			if ((*luma == NULL) || (*chroma1 == NULL) || (*chroma2 ==  NULL)) {
				throw "Memory allocation error";
			}
			
			// Read interleaved image.
			fseek(fp, kSceneSectorSize * ICDOffset[sceneNumber], SEEK_SET);
			long y;
			size_t count = 0;
			for (y=0; y < (long) (PCDChromaHeight[sceneNumber]); y++)
			{
				count += readBytes(fp, PCDLumaWidth[sceneNumber], *luma + y*2*PCDLumaWidth[sceneNumber]);
				count += readBytes(fp, PCDLumaWidth[sceneNumber], *luma + (y*2 + 1)*PCDLumaWidth[sceneNumber]);
				count += readBytes(fp, PCDChromaWidth[sceneNumber], *chroma1 + y*(PCDChromaWidth[sceneNumber]));
				count += readBytes(fp, PCDChromaWidth[sceneNumber], *chroma2 + y*(PCDChromaWidth[sceneNumber]));
			}
			if (count != ((PCDLumaWidth[sceneNumber]*2 + PCDChromaWidth[sceneNumber]*2)*PCDChromaHeight[sceneNumber])) {
				throw "File ended unexpectedly";
			}
			haveReadBase = true;
		}
		catch (...) {
			if ((*luma) != NULL) {
				free (*luma);
				*luma = NULL;
			}
			if ((*chroma1) != NULL) {
				free (*chroma1);
				*chroma1 = NULL;
			}
			if ((*chroma2) != NULL) {
				free (*chroma2);
				*chroma2 = NULL;
			}
			sceneNumber--;
		}
	}
	return sceneNumber;
}

//////////////////////////////////////////////////////////////
//
// Class initialiser and destructors 
//
//////////////////////////////////////////////////////////////
pcdDecode::pcdDecode(void)
{
	luma = NULL;
	chroma1 = NULL;
	chroma2 = NULL;
	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			deltas[i][j] = NULL;
		}
	}
	upResMethod = kUpResLumaIterpolate;
	pcdFileHeader = NULL;
	colorSpace = kPCDRawColorSpace;			// Default for PCD
	whiteBalance = kPCDD65White;			// Default for PCD
	monochrome = false;
	// Next line only used if we aren't using static LUTs
//	 populateLUTs();
}

pcdDecode::~pcdDecode()
{
	pcdFreeAll();
}

void pcdDecode::pcdFreeAll(void)
{
	if (luma != NULL) free(luma);
	luma = NULL;
	if (chroma1 != NULL) free(chroma1);
	chroma1 = NULL;
	if (chroma2 != NULL) free(chroma2);
	chroma2 = NULL;
	if (pcdFileHeader != NULL) free(pcdFileHeader);
	pcdFileHeader = NULL;
	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			if (deltas[i][j] != NULL) free(deltas[i][j]);
			deltas[i][j] = NULL;
		}
	}
}

//////////////////////////////////////////////////////////////
//
// Class accessors 
//
//////////////////////////////////////////////////////////////

size_t pcdDecode::getWidth()
{
	switch (imageRotate) {
		case 0:
			return PCDLumaWidth[sceneNumber];
			break;
		case 1:
			return PCDLumaHeight[sceneNumber];
			break;
		case 2:
			return PCDLumaWidth[sceneNumber];
			break;
		case 3:
			return PCDLumaHeight[sceneNumber];
			break;
		default:
			return PCDLumaWidth[sceneNumber];
			break;
	}
}

size_t pcdDecode::getHeight()
{
	switch (imageRotate) {
		case 0:
			return PCDLumaHeight[sceneNumber];
			break;
		case 1:
			return PCDLumaWidth[sceneNumber];
			break;
		case 2:
			return PCDLumaHeight[sceneNumber];
			break;
		case 3:
			return PCDLumaWidth[sceneNumber];
			break;
		default:
			return PCDLumaHeight[sceneNumber];
			break;
	}
}

void pcdDecode::setInterpolation(int value)
{
	upResMethod = value;
}

void pcdDecode::setColorSpace(int value)
{
	colorSpace = value;
}

int pcdDecode::getColorSpace()
{
	return colorSpace;
}

void pcdDecode::setWhiteBalance(int value)
{
	whiteBalance = value;
}

char *pcdDecode::getErrorString()
{
	return errorString;
}

bool pcdDecode::isMonochrome() 
{
	return monochrome;
}

void pcdDecode::setIsMonoChrome(bool val) {
	monochrome = monochrome | val;
}

long pcdDecode::digitisationTime() {
	if (pcdFileHeader == NULL) {
		return 0;
	}
	else {
		struct PCDFile *pcdFile = (struct PCDFile *) pcdFileHeader;
		return getPCD32(pcdFile->ipiHeader.imageScanningTime);
	}
}

void pcdDecode::getFilmTermData(int *FTN, int *PC, int *GC) {
	struct PCDFile *pcdFile = (struct PCDFile *) pcdFileHeader;
	if ((pcdFileHeader == NULL) || (compareBytes(pcdFile->ipiHeader.sbaSignature,"SBA")) != 0) {
		*FTN = 0;
		*PC = 0;
		*GC = 0;
	}
	else {
		int index = 0;
		int ftn = getPCD16(pcdFile->ipiHeader.sbaFTN);
		while ((index < kMaxPCDFilms) && (ftn != PCDFTN_PC_GC_Medium[index][0])) {
			index++;
		}
		if (index >=  kMaxPCDFilms) {
			*FTN = 0;
			*PC = 0;
			*GC = 0;
		}
		else {
			*FTN = PCDFTN_PC_GC_Medium[index][0];
			*PC = PCDFTN_PC_GC_Medium[index][1];
			*GC = PCDFTN_PC_GC_Medium[index][2];
		}
	}	
	return;
}

void pcdDecode::getMetadata(unsigned int select, char *description, char *value)
{
	struct PCDFile *pcdFile = (struct PCDFile *) pcdFileHeader;
	
	if ((select >= kMaxPCDMetadata) || (pcdFileHeader == NULL)) {
		if (description != NULL) strcpy(description, "Error");
		strcpy(value, "Error");
	}
	else {
		if (description != NULL) strcpy(description, PCDMetadataDescriptions[select]);
		if (compareBytes(pcdFile->ipiHeader.ipiSignature,"PCD_IPI") == 0) {
			time_t t;
			struct tm *brokentime;
			char *temp;
			switch (select) {
				case kspecificationVersion:
					if (getPCD32(pcdFile->ipiHeader.specificationVersion) == 0xffff) {
						strcpy(value, "-");
					}
					else {
						sprintf(value, "%d.%d", pcdFile->ipiHeader.specificationVersion[0], pcdFile->ipiHeader.specificationVersion[1]);
					}
					break;
				case kauthoringSoftwareRelease:		
					if (getPCD32(pcdFile->ipiHeader.authoringSoftwareRelease) == 0xffff) {
						strcpy(value, "-");
					}
					else {
						sprintf(value, "%d.%d", pcdFile->ipiHeader.authoringSoftwareRelease[0], pcdFile->ipiHeader.authoringSoftwareRelease[1]);
					}
					break;
				case kimageScanningTime:
					// Note we make the glorious assumption that the C library is Posix compliant in that
					// time_t is seconds since 1/1/1970.
					if (getPCD32(pcdFile->ipiHeader.imageScanningTime) == 0xffff) {
						strcpy(value, "-");
					}
					else {
						t = getPCD32(pcdFile->ipiHeader.imageScanningTime);
						brokentime = localtime(&t);
						temp = asctime (brokentime);
						// Get rid of the newline
						temp[strlen(temp)-1] = 0x0;
						strcpy(value, temp);
					}
					break;
				case kimageModificationTime:
					// Note we make the glorious assumption that the C library is Posix compliant in that
					// time_t is seconds since 1/1/1970
					if (getPCD32(pcdFile->ipiHeader.imageModificationTime) == 0xffff) {
							strcpy(value, "-");
						}
						else {
							t = getPCD32(pcdFile->ipiHeader.imageModificationTime);
							brokentime = localtime(&t);
							temp = asctime (brokentime);
							// Get rid of the newline
							temp[strlen(temp)-1] = 0x0;
							strcpy(value, temp);
						}
					break;
				case kimageMedium:
					if (pcdFile->ipiHeader.imageMedium < kMaxPCDmediums) {
						strcpy(value, PCDMediumTypes[pcdFile->ipiHeader.imageMedium]);
					}
					else {
						strcpy(value, "-");
					}
					break;
				case kproductType:
					copyWithoutPadding(value, pcdFile->ipiHeader.productType, sizeof(pcdFile->ipiHeader.productType));
					break;
				case kscannerVendorIdentity:					
					copyWithoutPadding(value, pcdFile->ipiHeader.scannerVendorIdentity, sizeof(pcdFile->ipiHeader.scannerVendorIdentity));
					break;
				case kscannerProductIdentity:				
					copyWithoutPadding(value, pcdFile->ipiHeader.scannerProductIdentity, sizeof(pcdFile->ipiHeader.scannerProductIdentity));
					break;
				case kscannerFirmwareRevision:				
					copyWithoutPadding(value, pcdFile->ipiHeader.scannerFirmwareRevision, sizeof(pcdFile->ipiHeader.scannerFirmwareRevision));
					break;
				case kscannerFirmwareDate:					
					copyWithoutPadding(value, pcdFile->ipiHeader.scannerFirmwareDate, sizeof(pcdFile->ipiHeader.scannerFirmwareDate));
					break;
				case kscannerSerialNumber:					
					copyWithoutPadding(value, pcdFile->ipiHeader.scannerSerialNumber, sizeof(pcdFile->ipiHeader.scannerSerialNumber));
					break;
				case kscannerPixelSize:	
					// BCD(!) coded
					sprintf(value, 
							"%d%d.%d%d", 
							(pcdFile->ipiHeader.scannerPixelSize[0]>>4) & 0xf, 
							(pcdFile->ipiHeader.scannerPixelSize[0]) & 0xf, 
							(pcdFile->ipiHeader.scannerPixelSize[1]>>4) & 0xf, 
							(pcdFile->ipiHeader.scannerPixelSize[1]) & 0xf
					);
					break;
				case kpiwEquipmentManufacturer:				
					copyWithoutPadding(value, pcdFile->ipiHeader.piwEquipmentManufacturer, sizeof(pcdFile->ipiHeader.piwEquipmentManufacturer));
					break;
				case kphotoFinisherName:
					// Don't return anything with a really exotic character set; the chances that
					// it will be displayed correctly are negligable.
					if (pcdFile->ipiHeader.photoFinisherCharSet < 5) {
						copyWithoutPadding(value, pcdFile->ipiHeader.photoFinisherName, sizeof(pcdFile->ipiHeader.piwEquipmentManufacturer));
					}
					else {
						strcpy(value, "-");
					}
					break;
				case ksbaRevision:	
					if ((compareBytes(pcdFile->ipiHeader.sbaSignature,"SBA") != 0) || (getPCD32(pcdFile->ipiHeader.specificationVersion) == 0xffff)) {
						strcpy(value, "-");
					}
					else {
						sprintf(value, "%d.%d", pcdFile->ipiHeader.specificationVersion[0], pcdFile->ipiHeader.specificationVersion[1]);
					}
					break;
				case ksbaCommand:
					if ((compareBytes(pcdFile->ipiHeader.sbaSignature,"SBA") != 0) || (pcdFile->ipiHeader.sbaCommand >= kMaxSBATypes)) {
						strcpy(value, "-");
					}
					else {
						strcpy(value, PCDSBATypes[pcdFile->ipiHeader.sbaCommand]);
					}
					break;
				case ksbaFilm:		
					if (compareBytes(pcdFile->ipiHeader.sbaSignature,"SBA") != 0) {
						strcpy(value, "-");
					}
					else {
						int index = 0;
						int ftn = getPCD16(pcdFile->ipiHeader.sbaFTN);
						while ((index < kMaxPCDFilms) && (ftn != PCDFTN_PC_GC_Medium[index][0])) {
						   index++;
						}
						if (index >=  kMaxPCDFilms) {
							strcpy(value, "Unknown film");
						}
						else {
							strcpy(value, PCDMediumNames[index]);
						}
					}
					break;
				case kcopyrightStatus:
					if (pcdFile->ipiHeader.copyrightStatus == 0x1) {
						strcpy(value, "Copyright restrictions apply - see copyright file on original CD-ROM for details");	
					}
					else {
						strcpy(value, "Copyright restrictions not specified");	
					}
					break;
				case kcopyrightFile:
					if (pcdFile->ipiHeader.copyrightStatus == 0x1) {
						copyWithoutPadding(value, pcdFile->ipiHeader.copyrightFile, sizeof(pcdFile->ipiHeader.copyrightFile));
					}
					else {
						strcpy(value, "-");						
					}
					break;
				case kcompressionClass:		
					strcpy(value, PCDHuffmanClasses[imageHuffmanClass]);
					break;
				default:
					strcpy(value, "-");
					break;
			}
		}
		else {
			strcpy(value, "-");
		}
	}
}

void pcdDecode::interpolateBuffers(uint8_t **c1UpRes, uint8_t **c2UpRes, int *resFactor)
{
	// This does an interpolate either by a factor of 2 or 4
	uint8_t *lp, *c1p, *c2p, *intermediate;
	lp = luma;
	c1p = chroma1;
	c2p = chroma2;
	intermediate = NULL;	
#ifdef __debug
	//	dumpColumn(lp, 356, PCDLumaHeight[sceneNumber], PCDLumaWidth[sceneNumber]);
	//	dump8by8(c1p, PCDChromaWidth[sceneNumber]);
#endif
	
	if (upResMethod >= kUpResIterpolate) {
		// Linear interpolation..........
		*c1UpRes = (uint8_t *) malloc(PCDLumaHeight[sceneNumber]*PCDLumaWidth[sceneNumber]*sizeof(uint8_t));
		*c2UpRes = (uint8_t *) malloc(PCDLumaHeight[sceneNumber]*PCDLumaWidth[sceneNumber]*sizeof(uint8_t));
		if (*c1UpRes == NULL || *c1UpRes == NULL) {
			throw "Memory Error!";
		}
		
		if (*resFactor == 2) {
			intermediate = (uint8_t *) malloc((PCDLumaHeight[sceneNumber]>>1)*(PCDLumaWidth[sceneNumber]>>1)*sizeof(uint8_t));
			if (intermediate == NULL) {
				throw "Memory Error!";
			}
			upResBuffer(c1p, intermediate, NULL, PCDLumaWidth[sceneNumber]>>1, PCDLumaHeight[sceneNumber]>>1, upResMethod, false);
			c1p = intermediate;
#ifdef __debug
			dump8by8(c1p, PCDLumaWidth[sceneNumber]>>1);
#endif
		}

		upResBuffer(c1p, *c1UpRes, lp, PCDLumaWidth[sceneNumber], PCDLumaHeight[sceneNumber], upResMethod, false);
		c1p = *c1UpRes;
		
		if (*resFactor == 2) {
			upResBuffer(c2p, intermediate, NULL, PCDLumaWidth[sceneNumber]>>1, PCDLumaHeight[sceneNumber]>>1, upResMethod, false);
			c2p = intermediate;
		}
		upResBuffer(c2p, *c2UpRes, lp, PCDLumaWidth[sceneNumber], PCDLumaHeight[sceneNumber], upResMethod, false);
		c2p = *c2UpRes;

		
		if (intermediate != NULL) {
			free(intermediate);
			intermediate = NULL;
		}
		*resFactor = 0;
	}
}

void pcdDecode::populateFloatBuffers(float *red, float *green, float *blue, float *alpha, int d)
{
	populateBuffers(red, green, blue, alpha, d, pcdFloatSize);
}

void pcdDecode::populateUInt16Buffers(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *alpha, int d)
{
	populateBuffers(red, green, blue, alpha, d, pcdInt16Size);
}

void pcdDecode::populateUInt8Buffers(uint8_t *red, uint8_t *green, uint8_t *blue, uint8_t *alpha, int d)
{
	populateBuffers(red, green, blue, alpha, d, pcdByteSize);
}

void pcdDecode::populateBuffers(void *red, void *green, void *blue, void *alpha, int d, int dataSize)
{

	uint8_t *lp, *c1p, *c2p, *intermediate, *c1UpRes, *c2UpRes;
	lp = luma;
	c1p = chroma1;
	c2p = chroma2;
	intermediate = NULL;
	c1UpRes = NULL;
	c2UpRes = NULL;
	int resFactor = PCDChromaResFactor[sceneNumber];
	
	if (pcdFileHeader == NULL) {
		// No file
		return;
	}
	
#ifdef __debug
//	dumpColumn(lp, 356, PCDLumaHeight[sceneNumber], PCDLumaWidth[sceneNumber]);
//	dump8by8(c1p, PCDChromaWidth[sceneNumber]);
#endif
	
	interpolateBuffers(&c1UpRes, &c2UpRes, &resFactor);
	if (c1UpRes != NULL) c1p = c1UpRes;
	if (c2UpRes != NULL) c2p = c2UpRes;

#ifdef __debug
//	dump8by8(c1p, PCDLumaWidth[sceneNumber]);
#endif	
	struct ConvertToRGBData rd[kNumThreads];
	size_t previousRow = 0;	
	int thread;
#ifndef mNoPThreads
	int rc;
	void *status;
	pcdThreadDescriptor threadDescriptors[kNumThreads];
	pthread_attr_t threadAttr;
	
	/* Initialize and set thread detached attribute */
	pthread_attr_init(&threadAttr);
	pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_JOINABLE);
	// Use minimum stacksize times two; we have only a few stack variables
	pthread_attr_setstacksize(&threadAttr, PTHREAD_STACK_MIN<<1);
#endif
#ifdef __PerformanceAnalysis
#ifdef qMacOS
	AbsoluteTime nowTime, bgnTime;
    bgnTime = UpTime();
#endif
#endif
	for (thread = 0; thread < kNumThreads; thread++) {
		rd[thread].outputSize = dataSize;
		rd[thread].red = red;
		rd[thread].green = green;
		rd[thread].blue = blue;
		rd[thread].alpha = alpha;
		rd[thread].d = d;
		rd[thread].startRow = previousRow;
		rd[thread].endRow = PCDLumaHeight[sceneNumber]/kNumThreads*(thread+1);
		rd[thread].columns = PCDLumaWidth[sceneNumber];
		rd[thread].rows = PCDLumaHeight[sceneNumber];
		rd[thread].lp = lp;
		rd[thread].c1p = monochrome ? NULL : c1p;
		rd[thread].c2p = monochrome ? NULL : c2p;
		rd[thread].resFactor = resFactor;
		rd[thread].imageRotate = imageRotate;
		rd[thread].colorSpace = colorSpace;		
		rd[thread].whiteBalance = whiteBalance;		
		
		previousRow = rd[thread].endRow;
#ifndef mNoPThreads
		if (thread == (kNumThreads - 1)) {
			convertToRGB(&(rd[thread]));
		}
		else {
			if (pcdStartThread(threadDescriptors[thread], threadAttr, convertToRGB, (void *)&(rd[thread])) != 0) {
				// Too many threads already.....
				convertToRGB(&(rd[thread]));
				// Don't try to join
				rd[thread].endRow = 0;
			}
		}
#else
		convertToRGB(&(rd[thread]));
#endif
	}
#ifdef __PerformanceAnalysis
#ifdef qMacOS
	nowTime = UpTime();
    float uSec  = HowLong(nowTime, bgnTime);
    fprintf(stderr, " ConvertRGB: %.3f usec \n", uSec);
#endif
#endif
#ifndef mNoPThreads
	pthread_attr_destroy(&threadAttr);
	status  = 0; // Avoid unreferenced local variable warning
	for (thread = 0; thread < (kNumThreads-1); thread++) {
		if (rd[thread].endRow > 0) {
			rc = pcdThreadJoin(threadDescriptors[thread], &status);
		}
	}
#endif
#ifdef __PerformanceAnalysis
#ifdef qMacOS
	nowTime = UpTime();
    uSec  = HowLong(nowTime, bgnTime);
    fprintf(stderr, " ConvertRGB: %.3f usec \n", uSec);
#endif
#endif
	if (c1UpRes != NULL) {
		free(c1UpRes);
		c1UpRes = NULL;
	}
	if (c2UpRes != NULL) {
		free(c2UpRes);
		c2UpRes = NULL;
	}
}


int pcdDecode::getOrientation()
{	
	return imageRotate;
}

void pcdDecode::postParse()
{
	int sceneNumber;
	bool haveDeltas;
	
	if (pcdFileHeader == NULL) {
		// No file
		return;
	}
	
	for (sceneNumber = k4Base; sceneNumber <= k64Base; sceneNumber++) {
		// Iterate the possible deltas that are avalable......
		if (deltas[sceneNumber-k4Base][0] != NULL) {
			// First the luma delta....
			upResBuffer(luma, deltas[sceneNumber-k4Base][0], NULL, PCDLumaWidth[sceneNumber], PCDLumaHeight[sceneNumber], pcdMin(kUpResIterpolate, upResMethod), true);
			if (deltas[sceneNumber-k4Base][0] != NULL) {
				free(luma);
				luma = deltas[sceneNumber-k4Base][0];
				deltas[sceneNumber-k4Base][0] = NULL;
			}
			// If there is a luma delta, we have to upres the chromas as well.....
			haveDeltas = (deltas[sceneNumber-k4Base][1] != NULL);
			if (!haveDeltas) {
				deltas[sceneNumber-k4Base][1] = (uint8_t *) malloc((PCDLumaWidth[sceneNumber]>>1) * (PCDLumaHeight[sceneNumber]>>1)*sizeof(uint8_t));
			}
			upResBuffer(chroma1, deltas[sceneNumber-k4Base][1], NULL, PCDLumaWidth[sceneNumber]>>1, PCDLumaHeight[sceneNumber]>>1, pcdMin(kUpResIterpolate, upResMethod), haveDeltas);
			if (deltas[sceneNumber-k4Base][1] != NULL) {
				free(chroma1);
				chroma1 = deltas[sceneNumber-k4Base][1];
				deltas[sceneNumber-k4Base][1] = NULL;
			}
			haveDeltas = (deltas[sceneNumber-k4Base][2] != NULL);
			if (!haveDeltas) {
				deltas[sceneNumber-k4Base][2] = (uint8_t *) malloc((PCDLumaWidth[sceneNumber]>>1) * (PCDLumaHeight[sceneNumber]>>1)*sizeof(uint8_t));
			}
			upResBuffer(chroma2, deltas[sceneNumber-k4Base][2], NULL, PCDLumaWidth[sceneNumber]>>1, PCDLumaHeight[sceneNumber]>>1, pcdMin(kUpResIterpolate, upResMethod), haveDeltas);
			if (deltas[sceneNumber-k4Base][2] != NULL) {
				free(chroma2);
				chroma2 = deltas[sceneNumber-k4Base][2];
				deltas[sceneNumber-k4Base][2] = NULL;
			}
		}
	}
}

//////////////////////////////////////////////////////////////
//
// Structures for the 64Base files 
//
//////////////////////////////////////////////////////////////

struct ic_header {
	char ic_name[0x28];
	uint8_t val1[2];
	uint8_t val2[2];
	uint8_t off_descr[4];
	uint8_t off_fnames[4];
	uint8_t off_pointers[4];
	uint8_t off_huffman[4];
};

struct ic_description {
	uint8_t len[2];
	uint8_t color;
	uint8_t fill;
	uint8_t width[2];
	uint8_t height[2];
	uint8_t offset[2];
	uint8_t length[4];
	uint8_t off_pointers[4];
	uint8_t off_huffman[4];
	uint8_t fill2[6];
};


struct ic_fname  {
	char fname[12];
	uint8_t size[4];
};

struct ic_entry {
	uint8_t fno[2];
	uint8_t offset[4];
};

bool pcdDecode::parseICFile (const pcdFilenameType *ipe_file)
{	
	FILE *ic = NULL;
	FILE *thisFile = NULL;
	struct ic_header *header;
	struct ic_description *description[3];
	struct ic_fname *names[10];
	bool retVal = true;
	pcdFilenameType processedFNames[10][13];		// 8.3 plus a terminating char......
	
	ReadBuffer hufBuffer;

	
	uint8_t *buffer = NULL;
	
	if (pcdMagicstrlen(ipe_file) < 10) {
		strncpy(errorString, "IPE filename too short to be valid", kPCDMaxStringLength*3-1);
		return false;
	}	
	// Check the E of 64BASE to determine whether we have a lower case environment
	bool usingLowerCase = (ipe_file[pcdMagicstrlen(ipe_file)-9] == 'e');
	
	ic=pcdMagicFOpen(ipe_file, pcdMagicFOpenMode);
	if (ic == NULL) {
		strncpy(errorString, "Could not open 64Base IPE file", kPCDMaxStringLength*3-1);
		return false;
	}
	
	// Find the total file size
	fseek(ic, 0, SEEK_END);
	size_t fileSize = (ftell(ic) / KSectorSize)+1;
	if (fileSize < 1) {
		strncpy(errorString, "Could not read 64Base IPE file", kPCDMaxStringLength*3-1);
		return false;
	}

	huffTables *hTables = (huffTables *) malloc(sizeof(huffTables));
	if (hTables == NULL) {
		strncpy(errorString, "Could not allocate huffman tables", kPCDMaxStringLength*3-1);
		return false;
	}
	
	try {
		// Read the whole file in......
		buffer = (uint8_t *)malloc(fileSize*KSectorSize*sizeof(uint8_t));
		if (buffer == NULL) {
			throw "Memory allocation error";
		}
		fseek(ic, 0, SEEK_SET);	
		if(fread(buffer, KSectorSize, fileSize, ic) < (fileSize - 1)) {
			throw "IC File too small";
		}		
		header = (ic_header *) buffer;
		ipeLayers = getPCD16(buffer + getPCD32(header->off_descr));
		
		if(!((ipeLayers==1) || (ipeLayers==3))) {
			throw "Invalid number of layers";
		}
		
		if (monochrome) {
			// Override
			ipeLayers = 1;
		}
		// Read the layer descriptions......
		description[0] = (ic_description *) (buffer + getPCD32(header->off_descr) + sizeof(uint16_t));
		description[1] = (ic_description *) (((uint8_t *) description[0]) + getPCD16(description[0]->len));
		description[2] = (ic_description *) (((uint8_t *) description[1]) + getPCD16(description[1]->len));	
		
		// Now read the filenames.....
		ipeFiles = getPCD16(buffer + getPCD32(header->off_fnames));
		
		if((ipeFiles<1) || (ipeFiles>10) || (ipeFiles < ipeLayers)) {
			throw "Invalid number of IPE files";
		}
		
		int i;
		for (i = 0; i < ipeFiles; i++) {
			names[i] = (ic_fname *) (buffer + getPCD32(header->off_fnames) + sizeof(ic_fname)*i + sizeof(uint16_t));
			int j;
			for (j = 0; j < 12; j++) {
				processedFNames[i][j] = (pcdFilenameType) (names[i]->fname[j]);
			}
//			pcdMagicstrncpy(processedFNames[i], names[i]->fname, 12);
			processedFNames[i][12] = (pcdFilenameType) 0x0;
			if (usingLowerCase) {
				unsigned int j;
				for (j = 0; j < pcdMagicstrlen(processedFNames[i]); j++) {
					// Using tolower here is ok; we know the encoding is straight ASCII
					processedFNames[i][j] = tolower(processedFNames[i][j]);
				}
			}
		}
		
		// Read the Huffman tables........
		readAllHuffmanTables(ic, getPCD32(header->off_huffman), hTables, ipeLayers);
		
		deltas[k64Base - k4Base][0] = (uint8_t *) malloc(PCDLumaWidth[k64Base]*PCDLumaHeight[k64Base]*sizeof(uint8_t));
		memset(deltas[k64Base - k4Base][0], 0x0, PCDLumaWidth[k64Base]*PCDLumaHeight[k64Base]*sizeof(uint8_t));
		if (ipeLayers == 3) {
			deltas[k64Base - k4Base][1] = (uint8_t *) malloc(PCDChromaWidth[k64Base]*PCDChromaHeight[k64Base]*sizeof(uint8_t));
			deltas[k64Base - k4Base][2] = (uint8_t *) malloc(PCDChromaWidth[k64Base]*PCDChromaHeight[k64Base]*sizeof(uint8_t));
			memset(deltas[k64Base - k4Base][1], 0x0, PCDChromaWidth[k64Base]*PCDChromaHeight[k64Base]*sizeof(uint8_t));
			memset(deltas[k64Base - k4Base][2], 0x0, PCDChromaWidth[k64Base]*PCDChromaHeight[k64Base]*sizeof(uint8_t));	
		}
			
		int layer;
		int currentFile = 0;
		for(layer = 0; layer< ipeLayers; layer++) {
#ifdef __debug 
			fprintf(stderr, "len: %d\n", getPCD16((uint8_t*) &description[layer]->len));
			fprintf(stderr, "color: %d\n", description[layer]->color);
			fprintf(stderr, "fill: %d\n", description[layer]->fill);
			fprintf(stderr, "width: %d\n", getPCD16((uint8_t*) &description[layer]->width));
			fprintf(stderr, "height: %d\n", getPCD16((uint8_t*) &description[layer]->height));
			fprintf(stderr, "offset: %d\n", getPCD16((uint8_t*) &description[layer]->offset));
			fprintf(stderr, "length: %d\n", getPCD32((uint8_t*) &description[layer]->length));
			fprintf(stderr, "off_pointers: %d\n", getPCD32((uint8_t*) &description[layer]->off_pointers));
			fprintf(stderr, "off_huffman: %d\n", getPCD32((uint8_t*) &description[layer]->off_huffman));
#endif		
			// Iterate through how ever many sectors there are 
			// we pass entire files to the Huffman decoder; all the row and sequence info comes 
			// out of the information encoded in the Huffman sequence headers
			int sequenceSize = getPCD32((uint8_t*) &description[layer]->length);
			int numSequences = getPCD16((uint8_t*) &description[layer]->width)*getPCD16((uint8_t*) &description[layer]->height)/sequenceSize;
			int sequence = 0;
			struct ic_entry *entry = (ic_entry *) (buffer + getPCD32((uint8_t*) &description[layer]->off_pointers));
			currentFile = getPCD16((uint8_t*) entry->fno);
			size_t startPoint = getPCD32((uint8_t*) entry->offset);
			while (numSequences-- > 0) {
#ifdef __debug
//				fprintf(stderr, "File No %d, offset %d\n",  getPCD16((uint8_t*) entry->fno), getPCD32((uint8_t*) entry->offset));
#endif
				sequence++;
				if ((currentFile != getPCD16((uint8_t*) entry->fno)) || (numSequences == 0)) {
					pcdFilenameType thisFilePath[512];
					pcdMagicstrcpy(thisFilePath, ipe_file);
					// Truncate the file name part, leaving the path separator
					thisFilePath[pcdMagicstrlen(ipe_file) - 7] = 0;
					pcdMagicstrcat(thisFilePath, processedFNames[currentFile]);
					thisFile = pcdMagicFOpen(thisFilePath, pcdMagicFOpenMode);
					if (thisFile == NULL) {
						throw "Could not open 64Base extension image";
					}
					fseek(thisFile, (long) startPoint, SEEK_SET);
					initReadBuffer(&hufBuffer, thisFile);
					readPCDDeltas(&hufBuffer, hTables, k64Base, sequenceSize, sequence-1, deltas[k64Base - k4Base], getPCD16((uint8_t*) &description[layer]->offset));
#ifdef __debug					
					uint8_t *test = deltas[k64Base - k4Base][1];
					test += ((PCDChromaWidth[k64Base]*PCDChromaHeight[k64Base]*sizeof(uint8_t)) >> 1) -32 -224;
#endif					
					fclose(thisFile);
					thisFile = NULL;
					currentFile = getPCD16((uint8_t*) entry->fno);
					startPoint = getPCD32((uint8_t*) entry->offset);
					sequence = 0;
				}
				entry++;
			}
		}		
	}
	catch (char *err) {
		if (errorString == NULL) {
			strncpy(errorString, err, kPCDMaxStringLength*3-1);
			strcat(errorString, " while processing 64Base image");
		}
		retVal = false;
	}
	catch (...) {
		if (errorString == NULL) strncpy(errorString, "Error while processing 64Base image", kPCDMaxStringLength*3-1);
		retVal = false;
	}
	if (!retVal) {
		int i;
		for(i = 0; i < 3; i++ ) {
			if (deltas[k64Base - k4Base][i] != NULL) {
				free(deltas[k64Base - k4Base][i]);
				deltas[k64Base - k4Base][i] = NULL;
			}
		}
	}
	
	if (hTables != NULL) {
		free(hTables);
	}
	
	if (ic != NULL) {
		fclose(ic);
		ic = NULL;
	}
	if (thisFile != NULL) {
		fclose(thisFile);
		thisFile = NULL;
	}
	if (buffer != NULL) {
		free(buffer);
		buffer = NULL;
	}
	return retVal;
}


bool pcdDecode::parseFile (const pcdFilenameType *in_file, const pcdFilenameType *ipe_file, unsigned int sNum)
{
	FILE *fp = NULL;
	size_t count = 0;
	bool overview;
	struct PCDFile *pcdFile;
	// Final elements of these tables calculated later
	int ICDOffset[kMaxScenes]	= {4, 23, 96, 389, 0, 0};
	int HCTOffset[kMaxScenes]	= {0, 0, 0, 388, 0, 0};
	
	// Free any memory from previous conversions
	pcdFreeAll();
	errorString[0] = 0x0;
	
	fp = pcdMagicFOpen (in_file, pcdMagicFOpenMode);
	if (fp == NULL) 
	{
		strncpy(errorString, "Could not open PCD file - may be a file permissions problem", kPCDMaxStringLength*3-1);
		return false;
	}
	
	// Check that this is a PCD file.
	pcdFileHeader = malloc(sizeof(PCDFile));
	if (pcdFileHeader == NULL) {
		return false;
	}
	pcdFile = (struct PCDFile *) pcdFileHeader;
	
	count = readBytes(fp, sizeof(PCDFile), (uint8_t *) pcdFile);
	if (count != sizeof(PCDFile)) {
		free(pcdFileHeader);
		pcdFileHeader = NULL;
		strncpy(errorString, "PCD file is too small to be valid", kPCDMaxStringLength*3-1);
		return false;
	}
	overview = compareBytes(pcdFile->header.signature,"PCD_OPA") == 0;

	if ((compareBytes(pcdFile->ipiHeader.ipiSignature,"PCD_IPI") != 0) && !overview)
	{
		free(pcdFileHeader);
		pcdFileHeader = NULL;
		strncpy(errorString, "That is not a valid PCD file", kPCDMaxStringLength*3-1);
		return false;
	}

	if (pcdFile->iciBase16.interleaveRatio != 1)
	{
		// We have interleaved audio......
		free(pcdFileHeader);
		pcdFileHeader = NULL;
		strncpy(errorString, "The file contains interleaved audio", kPCDMaxStringLength*3-1);
		return false;
	}	
	
	imageRotate = pcdFile->iciBase16.attributes & 0x03;
	imageResolution = ((pcdFile->iciBase16.attributes >> 2) & 0x03) + kBase;
	imageIPEAvailable = (pcdFile->iciBase16.attributes >> 4) & 0x01;
	imageHuffmanClass = (pcdFile->iciBase16.attributes >> 5) & 0x02;
	off_t base4Stop = getPCD16(pcdFile->iciBase16.sectorStop4Base);
	// Calculate the file locations that are based of variable sized data
	// See the file decription above for why the calculation values
	HCTOffset[k16Base] = base4Stop + 12;
	ICDOffset[k16Base] = base4Stop + 14;
	// unused in this implementation
//	size_t base16Stop = getPCD16(pcdFile->iciBase16.sectorStop16Base);
//	size_t ipeStop = getPCD16(pcdFile->iciBase16.sectorStopIPE);
	
	sceneNumber = sNum;
	// Limit the resolution to what we have available
	if (imageResolution < k16Base) {
		sceneNumber = pcdMin(sceneNumber, imageResolution);
	}

	// This reads in the base image - may be the right size, may be smaller
	// if smaller, we need to get delta images.........
	baseScene = readBaseImage(fp, sceneNumber, ICDOffset, &luma, &chroma1, &chroma2);
	
	// Test Image only
//	 genTestBaseImage(sceneNumber, luma, chroma1, chroma2);
	
	// Set for what we got now.......
	if (baseScene < kBase16) {
		// We couldn't find any image at all
		strncpy(errorString, "No valid base image could be found", kPCDMaxStringLength*3-1);
		return false;
	}
	else if (baseScene < kBase) {
		// The image was less than base resolution.....
		// so no deltas can be read
		sceneNumber = baseScene;
	}
	
	if (sceneNumber >= k4Base) {
		try {
			// Here we're reading in the 1536 by 1024 image's deltas - luma only
			// So we end up with an image with the chroma subsampled by a factor of 4
			ReadBuffer hufBuffer;
			huffTables *hTables = (huffTables *) malloc(sizeof(huffTables));
			if (hTables == NULL) {
				strncpy(errorString, "Could not allocate huffman tables", kPCDMaxStringLength*3-1);
			}
			else {
				readAllHuffmanTables(fp, kSceneSectorSize * HCTOffset[k4Base], hTables, 1);			
				// Now we need to get the actual data......
				fseek(fp, kSceneSectorSize * ICDOffset[k4Base], SEEK_SET);
				deltas[k4Base - k4Base][0] = (uint8_t *) malloc(PCDLumaWidth[k4Base]*PCDLumaHeight[k4Base]*sizeof(uint8_t));
				initReadBuffer(&hufBuffer, fp);
				readPCDDeltas(&hufBuffer, hTables, k4Base, 0, 0, deltas[k4Base - k4Base], 0);
				
				if (sceneNumber >= k16Base) {
					try {
						// Here we're reading in the 3072 by 2048 image's deltas - luma and chroma
						// Chroma is subsampled by a factor of two. Aka 16 times more data than
						// the 4 Base image			
						readAllHuffmanTables(fp, kSceneSectorSize * HCTOffset[k16Base], hTables, monochrome ? 1 : 3);	
						fseek(fp, kSceneSectorSize * ICDOffset[k16Base], SEEK_SET);
						deltas[k16Base - k4Base][0] = (uint8_t *) malloc(PCDLumaWidth[k16Base]*PCDLumaHeight[k16Base]*sizeof(uint8_t));	
						if (!monochrome) {
							deltas[k16Base - k4Base][1] = (uint8_t *) malloc(PCDChromaWidth[k16Base]*PCDChromaHeight[k16Base]*sizeof(uint8_t));
							deltas[k16Base - k4Base][2] = (uint8_t *) malloc(PCDChromaWidth[k16Base]*PCDChromaHeight[k16Base]*sizeof(uint8_t));
						}
						initReadBuffer(&hufBuffer, fp);
						readPCDDeltas(&hufBuffer, hTables, k16Base, 0, 0, deltas[k16Base - k4Base], 0);
						if (sceneNumber >= k64Base) {
							// the 6144 by 4096 image;
							// parseICFile has its own internal try/catch 
							if (!parseICFile(ipe_file)) {
								sceneNumber = k16Base;							
								if (errorString == NULL) {
									strncpy(errorString, "Error while processing 64Base image", kPCDMaxStringLength*3-1);
								}
							 }
						}
					}
					catch (char *err) {
						sceneNumber = k4Base;
						if (errorString == NULL) {
							strncpy(errorString, err, kPCDMaxStringLength*3-1);
							strcat(errorString, " while processing 16Base image");
						}
					}
					catch (...) {
						sceneNumber = k4Base;
						if (errorString == NULL) strncpy(errorString, "Could not find a valid 16Base image; falling back to 4Base", kPCDMaxStringLength*3-1);
					}
					if (sceneNumber == k4Base) {
						if (deltas[k16Base - k4Base][0] != NULL) {
							free (deltas[k16Base - k4Base][0]);
							deltas[k16Base - k4Base][0] = NULL;
						}			
						if (deltas[k16Base - k4Base][1] != NULL) {
							free (deltas[k16Base - k4Base][1]);
							deltas[k16Base - k4Base][1] = NULL;
						}	
						if (deltas[k16Base - k4Base][2] != NULL) {
							free (deltas[k16Base - k4Base][2]);
							deltas[k16Base - k4Base][2] = NULL;
						}
					}
				}
				free(hTables);
			}
		}
		catch (char *err) {
			sceneNumber = kBase;
			if (errorString == NULL) {
				strncpy(errorString, err, kPCDMaxStringLength*3-1);
				strcat(errorString, " while processing 4Base image");
			}
		}
		catch (...) {
			sceneNumber = kBase;
			if (errorString == NULL) strncpy(errorString, "Could not find a valid 4Base image; falling back to Base", kPCDMaxStringLength*3-1);
		}
		if (sceneNumber == kBase) {
			if (deltas[k4Base - k4Base][0] != NULL) {
				free (deltas[k4Base - k4Base][0]);
				deltas[k4Base - k4Base][0] = NULL;
			}
		}
	}

	fclose(fp);
	fp = NULL;
	return true;
}
