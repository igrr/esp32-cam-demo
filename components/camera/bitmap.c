//https://stackoverflow.com/a/23303847
#include "bitmap.h"
#include <string.h>
#include <stdlib.h>


char *bmp_create_header(int w, int h)
{
	bitmap *pbitmap  = (bitmap*)calloc(1, sizeof(bitmap));
	int _pixelbytesize = w * h * _bitsperpixel/8;
	int _filesize = _pixelbytesize+sizeof(bitmap);
	strcpy((char*)pbitmap->fileheader.signature, "BM");
	pbitmap->fileheader.filesize = _filesize;
	pbitmap->fileheader.fileoffset_to_pixelarray = sizeof(bitmap);
	pbitmap->bitmapinfoheader.dibheadersize = sizeof(bitmapinfoheader);
	pbitmap->bitmapinfoheader.width = w;
	pbitmap->bitmapinfoheader.height = h;
	pbitmap->bitmapinfoheader.planes = _planes;
	pbitmap->bitmapinfoheader.bitsperpixel = _bitsperpixel;
	pbitmap->bitmapinfoheader.compression = _compression;
	pbitmap->bitmapinfoheader.imagesize = _pixelbytesize;
	pbitmap->bitmapinfoheader.ypixelpermeter = _ypixelpermeter ;
	pbitmap->bitmapinfoheader.xpixelpermeter = _xpixelpermeter ;
	pbitmap->bitmapinfoheader.numcolorspallette = 0;
	return (char *)pbitmap;
}