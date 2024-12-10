#include <stdio.h>
#include <stdlib.h>
#define tiff_short 3
#define tiff_long 4
#define tiff_rational 5
struct tiff_header
{
    /* data */
    long ImageWidth;                    //256
    long ImageLength;                   //257
    short BitsPerSample;                //258
    short Compression;                  //259
    short PhotometricInterpretation;    //262
    //long StripOffset;                  //273
    short SamplesPerPixel;              //277
    short RowsPerStrip;                 //278
    //long StripByteCounts;               //279
    long XResolution;                  //282
    long YResolution;                  //283
    short ResolutionUnit;               //296
    long blockSize;
};
struct simple_buffer{
    u_int32_t length;
    u_int32_t index;
    uint8_t* buffer;
};
void putu8(uint8_t in, struct simple_buffer* buffer){
    buffer->buffer[buffer->index]=in;
    buffer->index++;

}
void putu16(u_int16_t in, struct simple_buffer* buffer){
    putu8(in>>8, buffer);
    putu8(in,buffer);
}
void putlong(u_int32_t in, struct simple_buffer* buffer){
    putu8(in>>24,buffer);
    putu8(in>>16,buffer);
    putu8(in>>8,buffer);
    putu8(in,buffer);
}

void write_header(u_int8_t* rawbuffer, struct tiff_header header){
    struct simple_buffer headerbuffer;
    headerbuffer.index=0;
    
    uint32_t startimage = 32768;//8*header.ImageLength+256;
    headerbuffer.length=startimage;
    headerbuffer.buffer=rawbuffer;
    putu8('M',&headerbuffer);
    putu8('M',&headerbuffer);
    putu16(42,&headerbuffer);
    //start of idf location
    putlong(8,&headerbuffer);
    //start of idf
    putu16(12,&headerbuffer);
    //start of fields
    putu16(256,&headerbuffer);putu16(tiff_long,&headerbuffer);putlong(1,&headerbuffer);putlong(header.ImageWidth,&headerbuffer);
    putu16(257,&headerbuffer);putu16(tiff_long,&headerbuffer);putlong(1,&headerbuffer);putlong(header.ImageLength,&headerbuffer);
    //bits per pixel op adress 200
    putu16(258,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(3,&headerbuffer);putlong(200,&headerbuffer);//bits per sample
    putu16(259,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(1,&headerbuffer);putu16(header.Compression,&headerbuffer);putu16(0,&headerbuffer);
    putu16(262,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(1,&headerbuffer);putu16(header.PhotometricInterpretation,&headerbuffer);putu16(0,&headerbuffer);
    //ofssets
    putu16(273,&headerbuffer);putu16(tiff_long,&headerbuffer);putlong(header.ImageLength,&headerbuffer);putlong(256,&headerbuffer);//StipOffsets
    putu16(277,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(1,&headerbuffer);putu16(header.SamplesPerPixel,&headerbuffer);putu16(0,&headerbuffer);
    putu16(278,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(1,&headerbuffer);putu16(header.RowsPerStrip,&headerbuffer);putu16(0,&headerbuffer);
    putu16(279,&headerbuffer);putu16(tiff_long,&headerbuffer);putlong(header.ImageLength,&headerbuffer);putlong(256+header.ImageLength*4,&headerbuffer);//StripByteCounts
    
    putu16(282,&headerbuffer);putu16(tiff_rational,&headerbuffer);putlong(1,&headerbuffer);putlong(220,&headerbuffer);
    putu16(283,&headerbuffer);putu16(tiff_rational,&headerbuffer);putlong(1,&headerbuffer);putlong(228,&headerbuffer);
    putu16(296,&headerbuffer);putu16(tiff_short,&headerbuffer);putlong(1,&headerbuffer);putu16(header.ResolutionUnit,&headerbuffer);putu16(0,&headerbuffer);
    putlong(0,&headerbuffer);
    headerbuffer.index=200;
    //fseek(fp,200,SEEK_SET);
    putu16(header.BitsPerSample,&headerbuffer);putu16(header.BitsPerSample,&headerbuffer);putu16(header.BitsPerSample,&headerbuffer);
    //fseek(fp,220,SEEK_SET);
    headerbuffer.index=220;
    putlong(header.XResolution,&headerbuffer);putlong(1,&headerbuffer);
    putlong(header.YResolution,&headerbuffer);putlong(1,&headerbuffer);
    //fseek(fp,256,SEEK_SET);
    headerbuffer.index=256;
    
    for(long i =0;i<header.ImageLength;i++){
        putlong(startimage+i*header.blockSize,&headerbuffer);
    }
    for(long i =0;i<header.ImageLength;i++){
        putlong(6*header.ImageWidth,&headerbuffer);
    }
    //fwrite(headerbuffer.buffer,1,headerbuffer.index+1,fp);
//     uint8_t*  imagebuffer;
//     imagebuffer = (uint8_t*) malloc(header.ImageWidth*6);
//     for(uint16_t i = 0;i<header.ImageLength;i++){
//         for(uint16_t j =0;j<header.ImageWidth;j++){
// //            putu16(j*256,&headerbuffer);putu16(j*256,&headerbuffer);putu16(i*256,&headerbuffer);
//             imagebuffer[6*j] = (j*32)>>8;
//             imagebuffer[6*j+1]= j*32;
//             imagebuffer[6*j+2] = (i*32)>>8;
//             imagebuffer[6*j+3]= i*32;
//             imagebuffer[6*j+4] = (i*32)>>8;
//             imagebuffer[6*j+5]= i*32;
//         }
//         fwrite(imagebuffer,1,6*header.ImageWidth,fp);
//     }

}

// int main(){
//     char filename[]="plaatje.tiff";
//     FILE* fp = fopen(filename, "wb");
//     if (!fp) {
//         fprintf(stderr, "Error: could not open file '%s' for writing\n", filename);
//     }
//     else{
//         printf("hij werkt");
//     }
//     struct tiff_header  tiff_file;
//     tiff_file.ImageWidth=1024*32;
//     tiff_file.ImageLength=1024*32;
//     tiff_file.BitsPerSample=16;
//     tiff_file.Compression=1;
//     tiff_file.PhotometricInterpretation=2;
//     tiff_file.RowsPerStrip=1;
//     tiff_file.SamplesPerPixel=3;
//     tiff_file.XResolution= 300;
//     tiff_file.YResolution=300;
//     tiff_file.ResolutionUnit=2;
//     write_header(fp,tiff_file);
//     fclose(fp);
//     /*
//     Ideetjes
//     Defineer de header(struct?)
//     blokken locatie en lengte berekenen
//     Bereken header size

    
//     Scrhijf de header naar buffer
//     buffer naar bestand

//     loop over theoretische image data
//     schrijf de content lijn voor lijn vanuit buffer naar bestand

//     */
// }
