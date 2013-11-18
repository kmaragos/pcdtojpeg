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
 * pcdDecode.h
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
 *
 * V1.0.2 - 1 July 2009 - Added YCC color space to the decoder library
 *                        Improved sRGB shadow detail and color handling
 * V1.0.3 - 25 Aug 2009 - Fixed a potential string overrun problem
 *
 * V1.0.6 - 12 Jan 2010 - Fix for compile problems under Ubuntu
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
 *
 */


#include <ctype.h>
#ifndef _MSC_VER
// So, of course, MS doesn't support stdint.h(!)
#include <stdint.h>
//#define pcdMagicAPI
//#define pcdFilenameType char
//#define pcdMagicFOpen fopen
//#define pcdMagicFOpenMode "rb"
#else
// So we have to define our own MS specific stuff
typedef __int8            int8_t;
typedef __int16           int16_t;
typedef __int32           int32_t;
typedef __int64           int64_t;
typedef unsigned __int8   uint8_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int32  uint32_t;
typedef unsigned __int64  uint64_t;
#endif

#if defined (_USRDLL) && defined (_MSC_VER)
#define pcdMagicAPI __declspec(dllexport)
#define pcdFilenameType wchar_t
#define pcdMagicFOpen _wfopen
#define pcdMagicFOpenMode L"rb"
#define pcdMagicstrlen wcslen
#define pcdMagicstrcpy wcscpy
#define pcdMagicstrcat wcscat
#define pcdMagicstrncpy wcsncpy
#else
#define pcdMagicAPI
#define pcdFilenameType char
#define pcdMagicFOpen fopen
#define pcdMagicFOpenMode "rb"
#define pcdMagicstrlen strlen
#define pcdMagicstrcpy strcpy
#define pcdMagicstrcat strcat
#define pcdMagicstrncpy strncpy
#endif

#include <stddef.h>
#ifdef qMacOS
#include <CoreServices/CoreServices.h>
#endif

#ifndef __pcdDecodeIncluded
#define __pcdDecodeIncluded 1

enum PCDResolutions {
	kBase16=0,	//	128 × 192		0.025 Mpix	0.07 Mb		Preview (index print, thumbnail)
	kBase4,		//	256 × 384		0.098 Mpix	0.28 Mb		Web
	kBase,		//	512 × 768		0.393 Mpix	1.13 Mb		Computer screen, TV, Web
	k4Base,		//	1024 × 1536		1.573 Mpix	4.50 Mb		HDTV screen
	k16Base,	//	2048 × 3072		6.291 Mpix	18.00 Mb	Print-out up to ca. 20 x 30 cm
	k64Base,	//  4096 × 6144		25.166 Mpix	72.00 Mb	Professional print, pre-press, archiving (optional)
	kMaxScenes
};

enum PCDUpResMethodTags {
	kUpResNearest,
	kUpResIterpolate,
	kUpResLumaIterpolate,
};

enum PCDStringLength {
	kPCDMaxStringLength = 120,
};

enum PCDColorSpaces {
	kPCDRawColorSpace = 0,
	kPCDLinearCCIR709ColorSpace,
	kPCDsRGBColorSpace,
	kPCDYCCColorSpace,
};

enum PCDWhiteBalance {
	kPCDD65White = 0,		// 6500K
	kPCDD50White,			// 5000K
};

enum PCDMetaDataDictionary {
	kspecificationVersion = 0,	
	kauthoringSoftwareRelease,		
	kimageScanningTime,	
	kimageModificationTime,
	kimageMedium,
	kproductType,							
	kscannerVendorIdentity,					
	kscannerProductIdentity,				
	kscannerFirmwareRevision,				
	kscannerFirmwareDate,					
	kscannerSerialNumber,					
	kscannerPixelSize,						
	kpiwEquipmentManufacturer,				
	kphotoFinisherName,	
	ksbaRevision,	
	ksbaCommand,
	ksbaFilm,		
	kcopyrightStatus,
	kcopyrightFile,
	kcompressionClass,
	kMaxPCDMetadata
};

enum PCDMediums {
	kColorNegative = 0,
	kColorReversal,
	kColorHardcopy,
	kThermalHardcopy,
	kBlackandwhiteNegative,
	kBlackandwhiteReversal,
	kBlackandwhiteHardcopy,
	kinterNegative,
	kSyntheticImage,
	kChromogenic
};



class pcdDecode
	{
	public:
		//////////////////////////////////////////////////////////////
		//
		// Class initialiser
		//
		//////////////////////////////////////////////////////////////		
		pcdDecode ();
		
		virtual ~pcdDecode ();
		
		//////////////////////////////////////////////////////////////
		//
		// File parser
		//
		//////////////////////////////////////////////////////////////
		// in_file : Zero terminated PCD file location
		// ipe_file :  Zero terminated IPE (64Base) file location, NULL for none
		// sNum : Maximum resolution to decode; member of PCDResolutions
		//
		// return true if image data at any resolution could be read (see getErrorString)
		//
		// Note that the actual decoded resolution may be lower than requested if there
		// is a file error or the file doesn't have the requsted resolution
		// The file not having the requested resolution is NOT regarded as an error; the best 
		// available resolution is returned
		// When this function returns, metadata and image size is available, but no pixel data.
		virtual bool parseFile (const pcdFilenameType *in_file, const pcdFilenameType *ipe_file, unsigned int sNum);
		
		//////////////////////////////////////////////////////////////
		//
		// Post parser
		//
		//////////////////////////////////////////////////////////////
		// This assembles the various image deltas into a coherent YCC image
		// To get image data, you must call populateBuffers
		// Multithreaded on platforms that support threading
		virtual void postParse();
		
		//////////////////////////////////////////////////////////////
		//
		// get Width
		//
		//////////////////////////////////////////////////////////////
		// Returns the actual image width; note this is after the image has been rotated
		// to the normal
		virtual size_t getWidth();
		
		//////////////////////////////////////////////////////////////
		//
		// get Height
		//
		//////////////////////////////////////////////////////////////
		// Returns the actual image height; note this is after the image has been rotated
		// to the normal		
		virtual size_t getHeight();
		
		//////////////////////////////////////////////////////////////
		//
		// Is Mononchrome?
		//
		//////////////////////////////////////////////////////////////
		// Returns true if the image is monochrome
		// This can be either because setIsMonochrome was called, or because the image has only
		// Y data at the resolution requested. Note that color data may exist at a lower resolution.
		virtual bool isMonochrome();
		
		//////////////////////////////////////////////////////////////
		//
		// Set Mononchrome
		//
		//////////////////////////////////////////////////////////////
		// If this set to true, the images will be procssed as monochrome - i.e. the
		// chroma data will be ignored. Note that values returned from the populateBuffers
		// are still three component RGB, and that those three compoenets may not be equal.
		// The relationship between them depends on the white balance setting
		virtual void setIsMonoChrome(bool val);

		//////////////////////////////////////////////////////////////
		//
		// Get Orientation
		//
		//////////////////////////////////////////////////////////////
		// Returns the orientation of the original PCD image
		// Note that the RGB image returned by the decoder is rotated to the "correct"
		// orientation as indicated by this setting.  i.e. the returned RGB image is always 
		// orientation 0.
		// 0 - 0deg, 1 - 90CCW, 2 - 180CCW, 3 - 270CCW
		virtual int getOrientation();
		
		//////////////////////////////////////////////////////////////
		//
		// Digitisation Time
		//
		//////////////////////////////////////////////////////////////
		// Returns digitisation time in seconds since 1/1/1970
		virtual long digitisationTime();
		
		//////////////////////////////////////////////////////////////
		//
		// Set Interpolation
		//
		//////////////////////////////////////////////////////////////
		// Sets the interpolation to one of the values in PCDUpResMethodTags:
		// kUpResNearest - nearest neighbour
		// kUpResIterpolate - bilinear intepolation 
		// kUpResLumaIterpolate - AHD type adaptive intepolation; this gives lower noise
		// and substantially reduces edge artifacts
		// kUpResLumaIterpolate is only available in the restricted (non-GPL)
		// version of the decoder
		virtual void setInterpolation(int value);
		
		//////////////////////////////////////////////////////////////
		//
		// Set ColorSpace
		//
		//////////////////////////////////////////////////////////////
		// Sets the color space that RGB data will be returned in. It must be one of
		// the values in enum PCDColorSpaces
		// kPCDRawColorSpace - Raw PCD data; converted to RGB, but still compressed, etc
		// kPCDLinearCCIR709ColorSpace - a CCIR709 linear light (gamma 1) space
		// kPCDsRGBColorSpace - sRGB space (aka with sRBG primaries and sRGB gamma curve)
		virtual void setColorSpace(int value);
		
		//////////////////////////////////////////////////////////////
		//
		// Get ColorSpace
		//
		//////////////////////////////////////////////////////////////
		// Gets the color space as set by setColorSpace
		virtual int getColorSpace();
		
		//////////////////////////////////////////////////////////////
		//
		// Set WhiteBalance
		//
		//////////////////////////////////////////////////////////////
		// Sets the white balance for the CCIR709 and sRGB color spaces
		// Must be one of PCDWhiteBalance:
		// kPCDD65White - 6500K
		// kPCDD50White - 5000K
		// The default (and what PCD images should be scanned at!) is 6500K
		virtual void setWhiteBalance(int value);
		
		//////////////////////////////////////////////////////////////
		//
		// Get Error String
		//
		//////////////////////////////////////////////////////////////
		// Returns a zero terminated error string which contains an English language 
		// error description, or an empty string (0x0 as the first character), if no error
		// information is available. The error string is set by the parseFile function.
		// If parseFile return false, then the contents of the error string consitute 
		// an error message, and no image data is available. If parseFile returns true
		// then the contents of the error string consitute a warning.
		virtual char *getErrorString();
		
		//////////////////////////////////////////////////////////////
		//
		// Get Film Term Data
		//
		//////////////////////////////////////////////////////////////
		// Returns the Film Term Number, and the PC and GC values for the medium that
		// was scanned. See Kodak Document PCD067 for more information
		// http://www.kodak.com/global/en/professional/products/storage/pcd/techInfo/pcd067.jhtml
		// A GC value of -1 indicates that no GC value exists.
		// If no film term data is available, FTN, PC and GC are set to 0.
		virtual void getFilmTermData(int *FTN, int *PC, int *GC);
		
		//////////////////////////////////////////////////////////////
		//
		// Populate float buffers
		//
		//////////////////////////////////////////////////////////////
		// Populates the supplied buffers with float (usually 32-bit) RGB data.
		// Alpha is always set to 1.0; if your application does not require 
		// an alpha value, pass NULL for alpha. d is the pointer increment value
		// for the buffers, in units of <float>. This allows the use of either 
		// interleaved or separate RGB buffers.
		
		//////////////////////////////////////////////////////////////
		//
		// Populate float RGB buffers
		//
		//////////////////////////////////////////////////////////////
		// Populates the supplied buffers with float (usually 32-bit) RGB data.
		// Alpha is always set to 1.0; if your application does not require 
		// an alpha value, pass NULL for alpha. d is the pointer increment value
		// for the buffers, in units of <float>. This allows the use of either 
		// interleaved or separate RGB buffers.
		// This function can only be called if parseFile returned true, and 
		// postParse has been called.
		// Multithreaded on platforms that support threading
		virtual void populateFloatBuffers(float *red, float *green, float *blue, float *alpha, int d);
		
		//////////////////////////////////////////////////////////////
		//
		// Populate uint16 RGB buffers
		//
		//////////////////////////////////////////////////////////////
		// Populates the supplied buffers with 16-bit unsigned integer RGB data.
		// Alpha is always set to 0xffff; if your application does not require 
		// an alpha value, pass NULL for alpha. d is the pointer increment value
		// for the buffers, in units of <uint16>. This allows the use of either 
		// interleaved or separate RGB buffers.
		// This function can only be called if parseFile returned true, and 
		// postParse has been called.
		// Multithreaded on platforms that support threading
		virtual void populateUInt16Buffers(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *alpha, int d);	
		
		//////////////////////////////////////////////////////////////
		//
		// Populate uint8 RGB buffers
		//
		//////////////////////////////////////////////////////////////
		// Populates the supplied buffers with 8-bit unsigned integer RGB data.
		// Alpha is always set to 0xff; if your application does not require 
		// an alpha value, pass NULL for alpha. d is the pointer increment value
		// for the buffers, in units of <uint8>. This allows the use of either 
		// interleaved or separate RGB buffers.
		// This function can only be called if parseFile returned true, and 
		// postParse has been called.
		// Multithreaded on platforms that support threading
		virtual void populateUInt8Buffers(uint8_t *red, uint8_t *green, uint8_t *blue, uint8_t *alpha, int d);
		
		
		//////////////////////////////////////////////////////////////
		//
		// Get Metadata
		//
		//////////////////////////////////////////////////////////////
		// Returns image metadata in English language human readable form.
		// Index is a member of PCDMetaDataDictionary, and must be between
		// 0 and kMaxPCDMetadata
		// description and value must be char buffers of minimum length 
		// kPCDMaxStringLength
		virtual void getMetadata(unsigned int select, char *description, char *value);
		
	protected:
		
		int upResMethod;
		bool monochrome;
		uint8_t *luma;
		uint8_t *chroma1;
		uint8_t *chroma2;
		uint8_t *deltas[3][3];
		unsigned int imageRotate;
		unsigned int imageResolution;
		int colorSpace;
		int whiteBalance;
		size_t imageIPEAvailable;
		size_t imageHuffmanClass;
		unsigned int baseScene;
		unsigned int sceneNumber;
		uint16_t ipeLayers;
		uint16_t ipeFiles;
		void *pcdFileHeader;
		char errorString[kPCDMaxStringLength*3];
		
		void interpolateBuffers(uint8_t  **c1UpRes, uint8_t **c2UpRes, int *resFactor);
		virtual void populateBuffers(void *red, void *green, void *blue, void *alpha, int d, int dataSize);
		virtual bool parseICFile (const pcdFilenameType *ipe_file);
		void pcdFreeAll(void);
	};

#endif
