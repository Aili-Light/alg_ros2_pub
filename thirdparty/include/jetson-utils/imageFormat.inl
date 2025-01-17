/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __IMAGE_FORMAT_INLINE_H_
#define __IMAGE_FORMAT_INLINE_H_

#if defined(MSVC)
#include <string>
#define strcasecmp stricmp
#define strncasecmp strnicmp
#else
#include <strings.h>
#endif
#include <type_traits>
// #include "imageFormat.h"

// imageFormatToStr
inline const char* imageFormatToStr( imageFormat format )
{
	switch(format)
	{
		case IMAGE_RGB8:	 	return "rgb8";
		case IMAGE_RGBA8:	 	return "rgba8";
		case IMAGE_RGB32F:	 	return "rgb32f";
		case IMAGE_RGBA32F:	 	return "rgba32f";
		case IMAGE_BGR8:	 	return "bgr8";
		case IMAGE_BGRA8:	 	return "bgra8";
		case IMAGE_BGR32F:	 	return "bgr32f";
		case IMAGE_BGRA32F:	 	return "bgra32f";
		case IMAGE_I420:	 	return "i420";
		case IMAGE_YV12:	 	return "yv12";
		case IMAGE_NV12:	 	return "nv12";
		case IMAGE_UYVY:	 	return "uyvy";
		case IMAGE_VYUY:	 	return "vyuy";
		case IMAGE_YUYV:	 	return "yuyv";
		case IMAGE_YVYU:		return "yvyu";
		case IMAGE_UVY2:	 	return "uvy2";
		case IMAGE_VUY2:	 	return "vuy2";
		case IMAGE_Y2UV:	 	return "y2uv";
		case IMAGE_Y2VU:		return "y2vu";
		case IMAGE_BAYER_BGGR:	return "bayer-bggr";
		case IMAGE_BAYER_GBRG:	return "bayer-gbrg";
		case IMAGE_BAYER_GRBG:	return "bayer-grbg";
		case IMAGE_BAYER_RGGB:	return "bayer-rggb";
		case IMAGE_GRAY8:	 	return "gray8";
		case IMAGE_GRAY32F:  	return "gray32f";
		case IMAGE_UNKNOWN: 	return "unknown";
        default: return "unknown";
	};
}

// imageFormatIsRGB
inline bool imageFormatIsRGB( imageFormat format )
{
	//if( format == IMAGE_RGB8 || format == IMAGE_RGBA8 || format == IMAGE_RGB32F || format == IMAGE_RGBA32F )
	//	return true;
	if( format >= IMAGE_RGB8 && format <= IMAGE_RGBA32F )
		return true;
	
	return false;
}

// imageFormatIsBGR
inline bool imageFormatIsBGR( imageFormat format )
{
	if( format >= IMAGE_BGR8 && format <= IMAGE_BGRA32F )
		return true;
	
	return false;
}

// imageFormatIsYUV
inline bool imageFormatIsYUV( imageFormat format )
{
	if( format >= IMAGE_YUYV && format <= IMAGE_NV12 )
		return true;
		
	return false;
}

// imageFormatIsGray
inline bool imageFormatIsGray( imageFormat format )
{
	if( format == IMAGE_GRAY8 || format == IMAGE_GRAY32F )
		return true;
	
	return false;
}

// imageFormatIsBayer
inline bool imageFormatIsBayer( imageFormat format )
{
	//if( format == IMAGE_BAYER_BGGR || format == IMAGE_BAYER_GBRG || format == IMAGE_BAYER_GRBG || format == IMAGE_BAYER_RGGB )
	//	return true;
	if( format >= IMAGE_BAYER_BGGR && format <= IMAGE_BAYER_RGGB )
		return true;
		
	return false;
}

// imageFormatFromStr
inline imageFormat imageFormatFromStr( const char* str )
{
	if( !str )
		return IMAGE_UNKNOWN;

	for( uint32_t n=0; n < IMAGE_COUNT; n++ )
	{
		const imageFormat fmt = (imageFormat)n;

		if( strcasecmp(str, imageFormatToStr(fmt)) == 0 )
			return fmt;
	}

	if( strcasecmp(str, "yuy2") == 0 )
		return IMAGE_YUY2;
	else if( strcasecmp(str, "rgb32") == 0 )
		return IMAGE_RGB32F;
	else if( strcasecmp(str, "rgba32") == 0 )
		return IMAGE_RGBA32F;
	else if( strcasecmp(str, "grey8") == 0 )
		return IMAGE_GRAY8;
	else if( strcasecmp(str, "grey32f") == 0 )
		return IMAGE_GRAY32F;

	return IMAGE_UNKNOWN;
}


// imageFormatBaseType
inline imageBaseType imageFormatBaseType( imageFormat format )
{
	switch(format)
	{
		case IMAGE_GRAY32F:
		case IMAGE_RGB32F:
		case IMAGE_BGR32F:		
		case IMAGE_RGBA32F: 
		case IMAGE_BGRA32F:		return IMAGE_FLOAT;
        default: return IMAGE_UINT8;
	}
}


// imageFormatChannels
inline size_t imageFormatChannels( imageFormat format )
{
	switch(format)
	{
		case IMAGE_RGB8:
		case IMAGE_RGB32F:
		case IMAGE_BGR8:
		case IMAGE_BGR32F:		return 3;
		case IMAGE_RGBA8:
		case IMAGE_RGBA32F:
		case IMAGE_BGRA8:
		case IMAGE_BGRA32F: 	return 4;
		case IMAGE_GRAY8:
		case IMAGE_GRAY32F:		return 1;
		case IMAGE_I420:
		case IMAGE_YV12:
		case IMAGE_NV12:
		case IMAGE_UVY2:
		case IMAGE_VUY2:
		case IMAGE_Y2UV:		
		case IMAGE_Y2VU:
		case IMAGE_UYVY:
		case IMAGE_VYUY:
		case IMAGE_YUYV:		
		case IMAGE_YVYU:		return 3;
		case IMAGE_BAYER_BGGR:
		case IMAGE_BAYER_GBRG:
		case IMAGE_BAYER_GRBG:
		case IMAGE_BAYER_RGGB:	return 1;
        default: return 0;
	}
}


// imageFormatDepth
inline size_t imageFormatDepth( imageFormat format )
{
	switch(format)
	{
		case IMAGE_RGB8:		
		case IMAGE_BGR8:		
		case IMAGE_RGB8_WITH_AWB:return sizeof(uchar3) * 8;
		case IMAGE_RGBA8:		
		case IMAGE_BGRA8:		return sizeof(uchar4) * 8;
		case IMAGE_RGB32F:		
		case IMAGE_BGR32F:		return sizeof(float3) * 8;
		case IMAGE_RGBA32F: 	
		case IMAGE_BGRA32F:		return sizeof(float4) * 8;
		case IMAGE_GRAY8:		return sizeof(unsigned char) * 8;
		case IMAGE_GRAY32F:		return sizeof(float) * 8;
		case IMAGE_I420:
		case IMAGE_YV12:
		case IMAGE_NV12:		return 12;
		case IMAGE_UVY2:
		case IMAGE_VUY2:
		case IMAGE_Y2UV:
		case IMAGE_Y2VU:
		case IMAGE_UYVY:
		case IMAGE_VYUY:
		case IMAGE_YUYV:		
		case IMAGE_YVYU:		return 16;
		case IMAGE_BAYER_BGGR:
		case IMAGE_BAYER_GBRG:
		case IMAGE_BAYER_GRBG:
		case IMAGE_BAYER_RGGB:	return sizeof(unsigned char) * 8;
		case IMAGE_BAYER_BGGR_RAW12:
		case IMAGE_BAYER_GBRG_RAW12:
		case IMAGE_BAYER_GRBG_RAW12:
		case IMAGE_BAYER_RGGB_RAW12:	
		case IMAGE_BAYER_RCCC_RAW12:
		case IMAGE_BAYER_CCCR_RAW12:
		case IMAGE_BAYER_CRCC_RAW12:
		case IMAGE_BAYER_CCRC_RAW12:	return sizeof(unsigned char) * 12;
		case IMAGE_BAYER_BGGR_RAW16:
		case IMAGE_BAYER_GBRG_RAW16:
		case IMAGE_BAYER_GRBG_RAW16:
		case IMAGE_BAYER_RGGB_RAW16:
		case IMAGE_BAYER_RCCC_RAW16:
		case IMAGE_BAYER_CCCR_RAW16:
		case IMAGE_BAYER_CRCC_RAW16:
		case IMAGE_BAYER_CCRC_RAW16:	return sizeof(unsigned char) * 16;
        default: return 0;
	}
}


// imageFormatSize
inline size_t imageFormatSize( imageFormat format, size_t width, size_t height )
{
	return (width * height * imageFormatDepth(format)) / 8;
}


/**
 * @ingroup image
 * @internal
 */
template<typename T> struct __image_format_assert_false : std::false_type { };


// imageFormatFromType
template<typename T> inline imageFormat imageFormatFromType()	
{ 
	static_assert(__image_format_assert_false<T>::value, "invalid image format type - supported types are uchar3, uchar4, float3, float4"); 
	return IMAGE_UNKNOWN;
}

template<> inline imageFormat imageFormatFromType<uchar3>()	{ return IMAGE_RGB8; }
template<> inline imageFormat imageFormatFromType<uchar4>()	{ return IMAGE_RGBA8; }
template<> inline imageFormat imageFormatFromType<float3>()	{ return IMAGE_RGB32F; }
template<> inline imageFormat imageFormatFromType<float4>()	{ return IMAGE_RGBA32F; }


// imageFormatErrorMsg
inline void imageFormatErrorMsg( const char* prefix, const char* function, imageFormat format )
{
	printf("%s%s -- unsupported image format (%s)\n", prefix, function, imageFormatToStr(format));
	printf("%s      supported formats are:\n", prefix);
	printf("%s          * rgb8\n", prefix);		
	printf("%s          * rgba8\n", prefix);		
	printf("%s          * rgb32f\n", prefix);		
	printf("%s          * rgba32f\n", prefix);
}


#endif

