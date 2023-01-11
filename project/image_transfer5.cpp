
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <Windows.h>

#include "image_transfer5.h"

using namespace std;

int save_rgb_image5(char file_name[], image &a)
{
	size_t size;
	int height, width;

	width = a.width; 
	height = a.height;
	
	if( a.type == RGB_IMAGE ) {
		size = width * height * 3;
	} else {
		cout << "\nerror in save_image(...) - image must be RGB image type";
		return 1;
	}
	
    HANDLE hf = CreateFileA(
            file_name, GENERIC_WRITE, FILE_SHARE_READ, NULL,
            CREATE_ALWAYS, NULL, NULL );

    if( hf == INVALID_HANDLE_VALUE ){
		cout << "\nerror in save_image(...) - file open error\n";
		return 1;
	}

    // write out the file header
    BITMAPFILEHEADER bfh;
    memset( &bfh, 0, sizeof( bfh ) );
    bfh.bfType = 'MB';
    bfh.bfSize = sizeof( bfh ) + size + sizeof( BITMAPINFOHEADER );
    bfh.bfOffBits = sizeof( BITMAPINFOHEADER ) + sizeof( BITMAPFILEHEADER );

    DWORD dwWritten = 0;
    WriteFile( hf, &bfh, sizeof( bfh ), &dwWritten, NULL );

    // write out the bitmap format
    BITMAPINFOHEADER bih;
    memset( &bih, 0, sizeof( bih ) );
    bih.biSize = sizeof( bih );
    bih.biWidth = width;
    bih.biHeight = height;
    bih.biPlanes = 1;
    bih.biBitCount = 24;

    WriteFile( hf, &bih, sizeof( bih ), &dwWritten, NULL );

    // write out the bits themselves
    WriteFile( hf, a.pdata, size, &dwWritten, NULL );

    CloseHandle( hf );

	return 0;
}


int load_rgb_image5(char file_name[], image &a)
{
	size_t size;
	int height, width;

	width = a.width; 
	height = a.height;

    HANDLE hf = CreateFileA(
            file_name, GENERIC_READ, FILE_SHARE_READ, NULL,
            OPEN_ALWAYS, NULL, NULL );

    if( hf == INVALID_HANDLE_VALUE ) {
		cout << "\nerror in load_image(...) - file open error\n";
		return 1;
	}

    // read the file header
    BITMAPFILEHEADER bfh;
    DWORD dwRead;
    ReadFile( hf, &bfh, sizeof( bfh ), &dwRead, NULL );

	int W, H;
	
    // read the bitmap format
    BITMAPINFOHEADER bih;
    ReadFile( hf, &bih, sizeof( bih ), &dwRead, NULL );
	W = (int)(bih.biWidth);
    H = (int)(bih.biHeight);
	
	// check for compatibility

	if( a.type == RGB_IMAGE ) {
		size = width * height * 3;
	} else {
		cout << "\nerror in load_image(...) - image must be RGB image type";
		return 1;
	}
	
	if( (W != width) || (H != height) ) {
		cout << "\nerror in load_image(...) - image and bitmap size are different";
		return 1;
	}

    // read the bits themselves
	ReadFile( hf, a.pdata, size, &dwRead, NULL );

    CloseHandle( hf );

	return 0;
}


int allocate_image5(image &a)
// allocate image memory
{
	size_t size;

	// calculate the image size (in bytes)
	// 3 bytes per pixel for RGB (RGB)
	if( a.type == RGB_IMAGE ) { // 3 bytes/pixel (B-G-A)
		size = (size_t)a.width * a.height * 3;
	} else if( a.type == GREY_IMAGE ) {
		// 1 byte/pixel (0-255)
		size = (size_t)a.width * a.height;
	} else if( a.type == LABEL_IMAGE ) { // 2 bytes/pixel (0-65535)
		size = (size_t)a.width * a.height * 2;
	} else {
		cout << "\nerror in allocate_image(...) - unknown image type";
		return 1;
	}

	// allocate size bytes of dynamic memory
	a.pdata = (ibyte *)malloc(size*sizeof(ibyte));

	if( a.pdata == NULL ) {
		cout << "\nmemory allocation error in allocate_image(...)";
		return 1;
	}

	return 0;
}


int free_image5(image &a)
// free image memory
{
	if( a.pdata == NULL ) {
		cout << "\nerror in free_image(...) - memory already freed";
		return 1;
	}

	// free image memory so other functions can use it
	free(a.pdata);

	// set the pointer to NULL so we know the memory has been freed
	a.pdata = NULL; 
	
	return 0;
}

