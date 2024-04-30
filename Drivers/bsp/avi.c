#define JPEG_INTERNALS
#include <jinclude.h>
#include <jpeglib.h>

#include <avi.h>
#include <stdlib.h>
#include <ov2640.h>
#include <string.h>
#include <unistd.h>
#include <simple_delay.h>
#include <stm32f4xx_hal_dcmi.h>

#define MAX_AVI_BUFF		20480 //20k for motion jpeg
#define MAX_PICTURE_BUFF	92160 //90k for stikk jepg

extern I2C_Handle_t hi2c1;

struct jpeg_decompress_struct cinfo;
JSAMPROW buffer[2] = {0};
uint8_t rowBuff[1024];

typedef struct RGB
{
  uint8_t B;
  uint8_t G;
  uint8_t R;
}RGB_typedef;

struct jpeg_error_mgr jerr;

RGB_typedef *RGB_matrix;

uint16_t RGB16PixelColor;

uint8_t bAviStartRecording = AVI_READY; // 1: start, 2: pending, 3: closed output file , 0: ready for recording

DWORD totalLen=0;
DWORD frames=0;




/********************************************************************************************************/
/* @function name 		- LSBtoDWORD																	*/
/*																										*/
/* @brief				- converts a 4-byte array to a DWORD (32-bit unsigned integer) assuming 		*/
/* 						  little-endian byte order														*/
/*																										*/
/* @parameter[in]		- pointer to the 4-byte array													*/
/*																										*/
/* @return				- the resulting DWORD value														*/
/*																										*/
/* @Note					- This function utilizes the I2C_Mem_Write function for communication			*/
/********************************************************************************************************/
DWORD LSBtoDWORD(char* lsb)
{
	return lsb[0] | lsb[1]<<8 | lsb[2]<<16 | lsb[3] << 24;
}

/********************************************************************************************************/
/* @function name 		- fwrite_DWORD																	*/
/*																										*/
/* @brief				- writes a DWORD (32-bit unsigned integer) to a file							*/
/*																										*/
/* @parameter[in]		- the DWORD value to write														*/
/*																										*/
/* @return				- the status code of the operation												*/
/*																										*/
/* @Note					- This function writes exactly 4 bytes to the file								*/
/********************************************************************************************************/
FRESULT fwrite_DWORD(FIL * file, DWORD word)
{
	unsigned char * p;
	UINT bw;

	p = (unsigned char *)&word;

	return (f_write(file, p, 4, &bw));
}

/********************************************************************************************************/
/* @function name 		- fwrite_WORD																	*/
/*																										*/
/* @brief				- writes a WORD (16-bit unsigned integer) to a file								*/
/*																										*/
/* @parameter[in]		- pointer to the FIL object that specifies the file								*/
/*																										*/
/* @parameter[in]		- the WORD value to write														*/
/*																										*/
/* @return				- the status code of the operation												*/
/*																										*/
/* @Note					- This function writes exactly 2 bytes to the file								*/
/********************************************************************************************************/
FRESULT fwrite_WORD(FIL * file, WORD word)
{
	unsigned char * p;
	UINT bw;

	p = (unsigned char*)&word;
	return (f_write(file, p, 2, &bw));

}

/********************************************************************************************************/
/* @function name 		- set_avi_output_status															*/
/*																										*/
/* @brief				- sets the value of status variable for AVI output								*/
/*																										*/
/* @parameter[in]		- the status value to set														*/
/*																										*/
/* @return				- none																			*/
/*																										*/
/* @Note					- This function updates a global variable that controls AVI recording			*/
/********************************************************************************************************/
void set_avi_output_status(uint8_t stat)
{
	bAviStartRecording = stat;
}

/********************************************************************************************************/
/* @function name 		- read_avi_output_status														*/
/*																										*/
/* @brief				- reads the current status of AVI output										*/
/*																										*/
/* @return				- current status of AVI output													*/
/*																										*/
/* @Note					- This function returns the value of a global variable that controls recording	*/
/********************************************************************************************************/
uint8_t read_avi_output_status()
{
	return bAviStartRecording;
}

/********************************************************************************************************/
/* @function name 		- decode_jpeg_to_tft															*/
/*																										*/
/* @brief				- decodes a JPEG image and converts to raw TFT display data						*/
/*																										*/
/* @parameter[in]		- pointer to the buffer containing the JPEG image								*/
/*																										*/
/* @parameter[in]		- size of the buffer															*/
/*																										*/
/* @return				- none																			*/
/*																										*/
/* @Note					- This function decodes the JPEG image and handles the display process			*/
/********************************************************************************************************/
void decode_jpeg_to_tft(uint8_t *buff, uint32_t len)
{
	uint32_t line_counter = 0;
	uint32_t i = 0;

	buffer[0] = rowBuff;

	// JPEG must begin with 0xFFD8 and end with 0xFFD9
	if (!(buff[0] == 0xFF && buff[1] == 0xD8 && buff[len-2]==0xFF && buff[len-1] == 0xD9))
		return;
	cinfo.err = jpeg_std_error(&jerr);

	jpeg_create_decompress(&cinfo);

	jpeg_mem_src(&cinfo, buff, len);
	jpeg_read_header(&cinfo, TRUE);
	//cinfo.scale_num=1;
	//cinfo.scale_denom=2;
	cinfo.dct_method = JDCT_IFAST;

	jpeg_start_decompress(&cinfo);
	//while (cinfo.output_scanline < cinfo.output_height && line_counter < 240)

	while (cinfo.output_scanline < cinfo.output_height && line_counter < 228) //240-12 (12 for menu)
	{
		(void)jpeg_read_scanlines(&cinfo, buffer, 1);

		RGB_matrix=(RGB_typedef*)buffer[0];
		for(i = 0; i < cinfo.output_width ; i++)
		{
			RGB16PixelColor = (uint16_t)
			(
				((RGB_matrix[i].R & 0x00F8) >> 3)|
				((RGB_matrix[i].G & 0x00FC) << 3)|
				((RGB_matrix[i].B & 0x00F8) << 8)
			);

		}

	  	line_counter++;

	}

	jpeg_finish_decompress(&cinfo);

	jpeg_destroy_decompress(&cinfo);
}

/********************************************************************************************************/
/* @function name 		- output_avi_header																*/
/*																										*/
/* @brief				- writes the header for an AVI file to the specified file						*/
/*																										*/
/* @parameter[in]		- pointer to the file object representing the AVI file							*/
/*																										*/
/* @parameter[in]		- frames per second for the AVI video											*/
/*																										*/
/* @parameter[in]		- width of the video frames in pixels											*/
/*																										*/
/* @parameter[in]		- height of the video frames in pixels 											*/
/*																										*/
/* @return				- status code indicating the result of the operation							*/
/*																										*/
/* @Note					- This function initializes the AVI file with the necessary headers and prepares*/
/*						  it for video data to be written. The header includes the RIFF chunk, AVI main */
/*						  header (avih), and stream header (strh) among others. The dwSize fields are   */
/*						placeholders and should be updated with actual values during the writing process*/
/********************************************************************************************************/
FRESULT output_avi_header(FIL *file, uint8_t fps, uint16_t width, uint16_t height)
{
	FRESULT res=FR_OK;
	UINT bw;
	RIFF RIFF_LIST;


	RIFF_LIST.dwRIFF = 'RIFF';
	res = f_write(file, "RIFF", 4, &bw); //offset=0

	//RIFF_LIST.dwSize =  150 + 12 + len + 8*frames + 8 + 4*4*frames;
	RIFF_LIST.dwSize =  0;
	// must rewrite when stop_output at file offset  4;
	res = fwrite_DWORD(file, RIFF_LIST.dwSize); //offset+4=4

	RIFF_LIST.dwFourCC = 'AVI ';
	res = f_write(file, "AVI ", 4, &bw); //offset+4=8
	// 	RIFF_LIST.data = WAIT WITH THIS

	LIST hdrl;
	hdrl.dwList = 'LIST';
	res = f_write(file, "LIST", 4, &bw); //offset+4=12

	hdrl.dwSize = 208;
	res = fwrite_DWORD(file, hdrl.dwSize);//offset+4=16

	hdrl.dwFourCC = 'hdrl';
	res = f_write(file, "hdrl", 4, &bw);//offset+4=20

	MainAVIHeader avih;

	avih.dwFourCC = 'avih';
	res = f_write(file, "avih", 4, &bw); //offset+4=24

	avih.dwSize = 56;
	res = fwrite_DWORD(file, avih.dwSize);//offset+4=28

	avih.dwMicroSecPerFrame = 1000000/fps;
	res = fwrite_DWORD(file, avih.dwMicroSecPerFrame);//offset+4=32

	avih.dwMaxBytesPerSec = 7000;
	res = fwrite_DWORD(file, avih.dwMaxBytesPerSec);//offset+4=36

	avih.dwPaddingGranularity = 0;
	res = fwrite_DWORD(file, avih.dwPaddingGranularity);//offset+4=40

	// dwFlags set to 16, do not know why!
	avih.dwFlags = 16;
	res = fwrite_DWORD(file, avih.dwFlags);//offset+4=44
///////////////////////////////
	avih.dwTotalFrames = 0;  // frames, offset ?, must rewrite

	res = fwrite_DWORD(file, avih.dwTotalFrames);//offset+4=48

	avih.dwInitialFrames = 0;
	res = fwrite_DWORD(file, avih.dwInitialFrames);//offset+4=52

	avih.dwStreams = 1;
	res = fwrite_DWORD(file, avih.dwStreams);//offset+4=56

	avih.dwSuggestedBufferSize = 0;
	res = fwrite_DWORD(file, avih.dwSuggestedBufferSize);//offset+4=60

	avih.dwWidth = width;
	res = fwrite_DWORD(file, avih.dwWidth);//offset+4=64

	avih.dwHeight = height;
	res = fwrite_DWORD(file, avih.dwHeight);//offset+4=68

	avih.dwReserved[0] = 0;
	res = fwrite_DWORD(file, avih.dwReserved[0]);//offset+4=72
	avih.dwReserved[1] = 0;
	res = fwrite_DWORD(file, avih.dwReserved[1]);//offset+4=76
	avih.dwReserved[2] = 0;
	res = fwrite_DWORD(file, avih.dwReserved[2]);//offset+4=80
	avih.dwReserved[3] = 0;
	res = fwrite_DWORD(file, avih.dwReserved[3]);//offset+4=84

	LIST strl;
	strl.dwList = 'LIST';
	res = f_write(file, "LIST", 4, &bw);//offset+4=88

	strl.dwSize = 132;
	res = fwrite_DWORD(file, strl.dwSize);//offset+4=92

	strl.dwFourCC = 'strl';
	res = f_write(file, "strl", 4, &bw);//offset+4=96

	AVIStreamHeader strh;
	strh.dwFourCC = 'strh';
	res = f_write(file, "strh", 4, &bw);//offset+4=100

	strh.dwSize = 48;
	res = fwrite_DWORD(file, strh.dwSize);//offset+4=104

	strh.fccType = 'vids';
	res = f_write(file, "vids", 4, &bw);//offset+4=108

	strh.fccHandler = 'MJPG';
	res = f_write(file, "MJPG", 4,&bw);//offset+4=112

	strh.dwFlags = 0;
	res = fwrite_DWORD(file, strh.dwFlags);//offset+4=116

	strh.wPriority = 0; // +2 = 14
	res = fwrite_WORD(file, strh.wPriority);//offset+4=120

	strh.wLanguage = 0; // +2 = 16
	res = fwrite_WORD(file, strh.wLanguage);//offset+2=122

	strh.dwInitialFrames = 0; // +4 = 20
	res = fwrite_DWORD(file, strh.dwInitialFrames);//offset+2=124

	strh.dwScale = 1; // +4 = 24
	res = fwrite_DWORD(file, strh.dwScale);//offset+4=128

	// insert FPS
	strh.dwRate = fps; // +4 = 28
	res = fwrite_DWORD(file, strh.dwRate);//offset+4=132

	strh.dwStart = 0; // +4 = 32
	res = fwrite_DWORD(file, strh.dwStart);//offset+4=136
	//////////////////////
	// insert nbr of jpegs, must rewrite, offset ?
	strh.dwLength = 0; //nbr_of_jpgs;  +4 = 36
	res = fwrite_DWORD(file, strh.dwLength);//offset+4=140

	strh.dwSuggestedBufferSize = 0; // +4 = 40
	res = fwrite_DWORD(file, strh.dwSuggestedBufferSize);//offset+4=144

	strh.dwQuality = 0; // +4 = 44
	res = fwrite_DWORD(file, strh.dwQuality);//offset+4=148
	// Specifies the size of a single sample of data.
	// This is set to zero if the samples can vary in size.
	// If this number is nonzero, then multiple samples of data
	// can be grouped into a single chunk within the file.
	// If it is zero, each sample of data (such as a video frame) must be in a separate chunk.
	// For video streams, this number is typically zero, although
	// it can be nonzero if all video frames are the same size.
	//
	strh.dwSampleSize = 0; // +4 = 48
	res = fwrite_DWORD(file, strh.dwSampleSize);//offset+4=152

	EXBMINFOHEADER strf;

	strf.dwFourCC = 'strf';
	res = f_write(file, "strf", 4, &bw);//offset+4=156

	strf.dwSize = 40;
	res = fwrite_DWORD(file, strf.dwSize);//offset+4=160

	strf.biSize = 40;
	res = fwrite_DWORD(file, strf.biSize);//offset+4=164

	strf.biWidth = width;
	res = fwrite_DWORD(file, strf.biWidth);//offset+4=168
	strf.biHeight = height;
	res = fwrite_DWORD(file, strf.biHeight);//offset+4=172
	strf.biPlanes = 1;
	res = fwrite_WORD(file, strf.biPlanes);//offset+4=176
	strf.biBitCount = 24;
	res = fwrite_WORD(file, strf.biBitCount);//offset+2=178
	strf.biCompression = 'MJPG';
	res = f_write(file, "MJPG", 4, &bw);//offset+2=180

	strf.biSizeImage = ((strf.biWidth*strf.biBitCount/8 + 3)&0xFFFFFFFC)*strf.biHeight;
	res = fwrite_DWORD(file, strf.biSizeImage);//offset+4=184
	strf.biXPelsPerMeter = 0;
	res = fwrite_DWORD(file, strf.biXPelsPerMeter);//offset+4=188
	strf.biYPelsPerMeter = 0;
	res = fwrite_DWORD(file, strf.biYPelsPerMeter);//offset+4=192
	strf.biClrUsed = 0;
	res = fwrite_DWORD(file, strf.biClrUsed);//offset+4=196
	strf.biClrImportant = 0;
	res = fwrite_DWORD(file, strf.biClrImportant);//offset+4=200

	res = f_write(file, "LIST", 4, &bw);//offset+4=204

	DWORD ddww = 16;
	res = fwrite_DWORD(file, ddww);//offset+4=208
	res = f_write(file, "odml", 4, &bw);//offset+4=212
	res = f_write(file, "dmlh", 4, &bw);//offset+4=216

	DWORD szs = 4;
	res = fwrite_DWORD(file, szs);//offset+4=220
////////////////////////////////////////
	// nbr of jpgs
	DWORD totalframes = 0; //nbr_of_jpgs; must rewrite, offset ?;
	res = fwrite_DWORD(file, totalframes);//offset+4=224

	LIST movi;
	movi.dwList = 'LIST';
	res = f_write(file, "LIST", 4, &bw);//offset+4=228

	//////////////////////////////
	//movi.dwSize = len + 4 + 8*nbr_of_jpgs;   must rewrite, offset?, len, nbr_of_jpgs;
	movi.dwSize = 0;
	res = fwrite_DWORD(file, movi.dwSize);//offset+4=232
	movi.dwFourCC = 'movi';
	res = f_write(file, "movi", 4, &bw);//offset+4=236
	return res;
}


/********************************************************************************************************/
/* @function name 		- start_output_mjpeg_avi														*/
/*																										*/
/* @brief				- initiates the output of an MJPEG AVI file using data from DCMI interface		*/
/*																										*/
/* @parameter[in]		- pointer to the file object where the AVI file will be written					*/
/*																										*/
/* @parameter[in]		- pointer to the DCMI_HandleTypeDef structure for camera interface configuration*/
/*																										*/
/* @parameter[in]		- frames per second for the AVI video											*/
/*																										*/
/* @parameter[in]		- resolution setting for the camera 											*/
/*																										*/
/* @return				- status code indicating the result of the operation							*/
/*																										*/
/* @Note				   - This function configures the camera, initializes the AVI file header,and starts*/
/*						 the recording process. It captures frames from the camera, checks for valid JPEG*/
/*						 headers, writes the data to the file, and updates the index. 					*/
/*						 The recording continues until the stop condition is met						*/
/********************************************************************************************************/
FRESULT start_output_mjpeg_avi(FIL *file, DCMI_HandleTypeDef *hdcmi, uint8_t fps, uint8_t resolution)
{
	FRESULT res = FR_OK;
	FIL temp_idx1;
	uint32_t next_idx1_offset;
	uint8_t dmabuff[MAX_AVI_BUFF];
	uint16_t width, height;
	uint16_t f_begin = 0;
	uint32_t f_end = 0;
	uint32_t idx = 0;

	uint32_t sublen=0;
	uint8_t headerFinder=0;
	UINT bw;
	uint16_t f,r;
	CHUNK data;
	uint8_t *buff;
	uint8_t frame_ok=0;

	uint32_t lastSuccessTick;
	uint16_t timeout=0;
	uint8_t adjTime=0;

	unsigned long beginTick;
	long time_diff=0;


	bAviStartRecording = AVI_START;
	HAL_DCMI_Stop(hdcmi);
	ov2640_Init(0x60, CAMERA_Movie);

	width = 320;
	height = 240;

	 res = f_open(&temp_idx1, "0:/temp_idx1", FA_CREATE_ALWAYS|FA_WRITE|FA_READ);
	 if (res != FR_OK) return res;

	 if (output_avi_header(file, fps, width, height) != FR_OK)  //default fps = 15
		return res;

	totalLen=0;
	frames=0;

	next_idx1_offset=4;
	lastSuccessTick = HAL_GetTick();
	while(bAviStartRecording == AVI_START)
	{
		memset(dmabuff,0, MAX_AVI_BUFF);
		headerFinder=0;
		//if (frame_ok 0)
		if (frame_ok && adjTime  == 0)
		{
			time_diff = HAL_GetTick()-lastSuccessTick;

			//if (1000/fps > time_diff && time_diff > 0)
			if (1000 > time_diff)
				delay_ms(1000 - time_diff);
		}

		beginTick = HAL_GetTick();

		HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_SNAPSHOT, ((uint32_t)dmabuff), MAX_AVI_BUFF/4);

		timeout=0;
		while((hdcmi->Instance->CR.reg & 0x03) == 3)
		{
			if (timeout > 1000)
			{
				break; //max timeout: 1 seconds
			}
			delay_ms(1);
			timeout++;
		}
		if(timeout <= 1000)
		{
			frame_ok=0;
			for (idx = 0; idx<MAX_AVI_BUFF-4;idx++)
			{
				if (headerFinder == 0 && dmabuff[idx] == 0xff && dmabuff[idx+1]==0xD8)
				{
					headerFinder=1;
					f_begin=idx;
				}
				if (headerFinder==1 && dmabuff[idx]== 0xFF && dmabuff[idx+1] == 0xD9 )
				{
					if (adjTime == 0) // 1 sec
						lastSuccessTick=beginTick;

					adjTime = (adjTime+1)%fps; // 1 sec

					frame_ok=1;
					headerFinder=0;
					f_end = idx+1;

					buff = dmabuff + f_begin;

					data.dwFourCC = '00db';
					f_write(file, "00db", 4, &bw);
					sublen = f_end-f_begin+1;

					data.dwSize = sublen;
					fwrite_DWORD(file, data.dwSize);
					f = sublen / 512;
					r = sublen % 512;
					for (int i = 0; i < f; i++)
					{
						res=f_write(file, buff, 512, &bw);
						buff += 512;
					}

					if (r > 0)
					{
						res=f_write(file, buff, r, &bw);

					}

					buff = dmabuff + f_begin;

					decode_jpeg_to_tft(buff, sublen);

					if (sublen%2)
					{
						sublen++;
						res=f_write(file, '\0', 1, &bw);
					}
					frames++;
					totalLen += (sublen);
					f_write(&temp_idx1, "00db", 4, &bw);
					fwrite_DWORD(&temp_idx1, 16);//AVI_KEYFRAME=16
					fwrite_DWORD(&temp_idx1, next_idx1_offset);
					fwrite_DWORD(&temp_idx1, sublen);
					next_idx1_offset += (8+sublen);

					break;
				}

			}

		}
	}
	if (bAviStartRecording == AVI_PENDING)
	{
		stop_output_mjpeg_avi(file, &temp_idx1);
	}
	//bAviStartRecording = AVI_PENDING; //wait for rewriting some parameters and to close file;

	return res;
}

/********************************************************************************************************/
/* @function name 		- stop_output_mjpeg_avi															*/
/*																										*/
/* @brief				- finalizes the output of an MJPEG AVI file and updates the file's headers      */
/*																										*/
/* @parameter[in]		- pointer to the file object where the AVI file has been written                */
/*																										*/
/* @parameter[in]		- pointer to the temporary index file used during AVI file creation             */
/*																										*/
/* @return				- none																			*/
/*																										*/
/* @Note					- This function is called to finalize the AVI file after recording is complete. */
/*						  It writes the 'idx1' chunk, updates the file size, frame count, and other     */
/*						 header information. It also closes the temporary index file and the main AVI   */
/*						 file, and sets the recording status to closed.                                 */
/********************************************************************************************************/
void stop_output_mjpeg_avi(FIL *file, FIL* temp_idx1)
{
	UINT br;
	UINT bw;
	unsigned char buf[512];
	if (bAviStartRecording != AVI_PENDING) return;

	f_write(file, "idx1", 4, &br);
	fwrite_DWORD(file, 4*4*frames);
	f_lseek(temp_idx1, 0);
	do {
		f_read(temp_idx1, buf, 512, &br);
		f_write(file, buf, br, &bw);
	} while (br == 512);
	f_close(temp_idx1);
	f_unlink("0:/temp_idx1");
	// rewrite file size, frames
	// file size offset 4
	DWORD size;

	//RIFF_LIST.dwSize =  150 + 12 + len + 8*frames + 8 + 4*4*frames;

	//size = 150 + 12 + totalLen + 8*frames + 8 + 4*4*frames(idx1);
	size = 150 + 12 + totalLen + 8*frames + 8 + 4*4*frames;
	f_lseek(file, 4);
	fwrite_DWORD(file, size);

	//DWORD avih  totalframes = 0; nbr_of_jpgs;;
	size = frames;
	f_lseek(file, 48);
	fwrite_DWORD(file, size);//offset+4=48

	//strh.dwLength = nbr_of_jpgs;  +4 = 36
	size=frames;
	f_lseek(file, 140); //140
	fwrite_DWORD(file, size);


	//totalFrames(dmlh)
	size=frames;
	f_lseek(file, 224); //140
	fwrite_DWORD(file, size);


	//movi.dwSize = len + 4 + 8*nbr_of_jpgs;
	size = totalLen + 4 + 8*frames;
	f_lseek(file, 232);
	fwrite_DWORD(file, size);//offset+4=232
	f_close(temp_idx1);
	f_close(file);
	bAviStartRecording = AVI_CLOSED_OUT;
}
