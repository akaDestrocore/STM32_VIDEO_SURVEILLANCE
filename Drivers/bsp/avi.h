#ifndef INC_AVI_H_
#define INC_AVI_H_
#include <main.h>
#include <fatfs.h>

enum avi_resolution
{
	RES_320X240 = 1,
	RES_640X480	= 2,
	RES_800X600	= 3,
};

typedef enum
{
	AVI_START 		= 1,
	AVI_PENDING 	= 2,
	AVI_CLOSED_OUT	= 3,
	AVI_READY		= 0
}AVI_state_t;

typedef unsigned long DWORD;
typedef long LONG;
typedef unsigned short WORD;
typedef unsigned char BYTE;

// AVI atoms
typedef struct {
	DWORD dwRIFF;
	DWORD dwSize;
	DWORD dwFourCC;
} RIFF;

typedef struct {
	DWORD dwFourCC;
	DWORD dwSize;
	//	BYTE* data; // dwSize in length
} CHUNK;

typedef struct {
	DWORD dwList;
	DWORD dwSize;
	DWORD dwFourCC;
	//	BYTE* data; // dwSize - 4 in length
} LIST;

typedef struct {
	DWORD dwFourCC;
	DWORD dwSize;

	DWORD dwMicroSecPerFrame;
	DWORD dwMaxBytesPerSec;
	DWORD dwPaddingGranularity;

	DWORD dwFlags;
	DWORD dwTotalFrames;
	DWORD dwInitialFrames;
	DWORD dwStreams;
	DWORD dwSuggestedBufferSize;

	DWORD dwWidth;
	DWORD dwHeight;

	DWORD dwReserved[4];
} MainAVIHeader;

typedef struct _RECT {
	LONG left;
	LONG top;
	LONG right;
	LONG bottom;
} RECT;

typedef struct {
	DWORD dwFourCC;
	DWORD dwSize;

	DWORD fccType;
	DWORD fccHandler;
	DWORD dwFlags;
	WORD  wPriority;
	WORD  wLanguage;
	DWORD dwInitialFrames;
	DWORD dwScale;
	DWORD dwRate;
	DWORD dwStart;
	DWORD dwLength;
	DWORD dwSuggestedBufferSize;
	DWORD dwQuality;
	DWORD dwSampleSize;

} AVIStreamHeader;

typedef struct {
	DWORD dwFourCC;
	DWORD dwSize;

	DWORD biSize;
	DWORD  biWidth;
	DWORD  biHeight;
	WORD  biPlanes;
	WORD  biBitCount;
	DWORD biCompression;
	DWORD biSizeImage;
	DWORD biXPelsPerMeter;
	DWORD  biYPelsPerMeter;
	DWORD biClrUsed;
	DWORD biClrImportant;
} AVI_BITMAPINFOHEADER;

typedef struct tagEXBMINFOHEADER {
	DWORD dwFourCC;
	DWORD dwSize;

	DWORD biSize;
	LONG  biWidth;
	LONG  biHeight;
	WORD  biPlanes;
	WORD  biBitCount;
	DWORD biCompression;
	DWORD biSizeImage;
	LONG  biXPelsPerMeter;
	LONG  biYPelsPerMeter;
	DWORD biClrUsed;
	DWORD biClrImportant;

} EXBMINFOHEADER;

typedef struct {
	DWORD ckid;
	DWORD dwFlags;
	DWORD dwChunkOffset;
	DWORD dwChunkLength;
} AVIINDEXENTRY;

typedef struct {
	DWORD fcc;
	DWORD cd;
	WORD wLongsPerEntry;
	char bIndexSubType;
	char bIndexType;
	DWORD nEntriesInUse;
	DWORD dwChunkId;
	DWORD dwReserved[3];
	AVIINDEXENTRY axiindex_entry;
} AVIINDEXCHUNK;

typedef struct {
	DWORD name;
	DWORD dwSize;
	DWORD dwTotalFrames;
} ODMLExtendedAVIheader;

typedef struct {
	RIFF riff_AVI;
	LIST hdrl;
	MainAVIHeader avih;
	LIST strl;
	AVIStreamHeader strh;
	EXBMINFOHEADER strf;
	LIST odml;
	ODMLExtendedAVIheader dmlh;
	LIST movi;
	CHUNK movi_data;
} avi_file;

FRESULT output_avi_header(FIL *file, uint8_t fps, uint16_t width, uint16_t height);
FRESULT start_output_mjpeg_avi(FIL *file, DCMI_HandleTypeDef *hdcmi, uint8_t fps, uint8_t resolution);
void stop_output_mjpeg_avi(FIL *file, FIL* temp_idx1);
void set_avi_output_status(uint8_t stat);
void decode_jpeg_to_tft(uint8_t *buff, uint32_t len);
uint8_t read_avi_output_status();
#endif /* INC_AVI_H_ */
