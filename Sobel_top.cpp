#include <stdint.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include "composting_data.hpp"

typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;

#define WIDTH  1280
#define HEIGHT 720

#define COMP_WIDTH 128 // 32 for 32x32, 64 for 64x64, 128 for 128x128
#define COMP_HEIGHT 128 // 32 for 32x32, 64 for 64x64, 128 for 128x128
#define COMP_X (WIDTH - COMP_WIDTH - 120) // 480 for 32x32, 240 for 64x64, 120 for 128x128
#define COMP_Y 120 // 480 for 32x32, 240 for 64x64, 120 for 128x128


#define GR(v) ((v)&0xFF)
#define GG(v) (((v)&0xFF00)>>8)
#define GB(v) (((v)&0xFF0000)>>16)
#define SR(v) ((v)&0xFF)
#define SG(v) (((v)&0xFF)<<8)
#define SB(v) (((v)&0xFF)<<16)

//const int16_t gauss_7x7[7][7] = {
//    {1,   6,  15,  20,  15,   6,  1},
//    {6,  36,  90, 120,  90,  36,  6},
//    {15, 90, 225, 300, 225,  90, 15},
//    {20, 120, 300, 400, 300, 120, 20},
//    {15, 90, 225, 300, 225,  90, 15},
//    {6,  36,  90, 120,  90,  36,  6},
//    {1,   6,  15,  20,  15,   6,  1}
//};

// Stronger Blur with sigma = 3
const int16_t gauss_7x7[7][7] = {
		{16, 19, 21, 22, 21, 19, 16},
		{19, 22, 25, 26, 25, 22, 19},
		{21, 25, 28, 29, 28, 25, 21},
		{22, 26, 29, 30, 29, 26, 22},
		{21, 25, 28, 29, 28, 25, 21},
		{19, 22, 25, 26, 25, 22, 19},
		{16, 19, 21, 22, 21, 19, 16}
};

const int8_t sobel_gx[3][3] = {
		{-1,0,1},
		{-2,0,2},
		{-1,0,1}
};

const int8_t sobel_gy[3][3] = {
		{-1,-2,-1},
		{0,0,0},
		{1,2,1}
};

void sobel(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS BIND_STORAGE variable=satisfied_data type=rom_1p impl=bram
#pragma HLS BIND_STORAGE variable=unsatisfied_data type=rom_1p impl=bram

    static uint16_t x = 0;
    static uint16_t y = 0;
    static uint32_t edge_count = 0;
    static uint32_t frame_counter = 0;
    static bool satisfied = false;

    static uint8_t gray_lines[6][WIDTH];
#pragma HLS ARRAY_PARTITION variable=gray_lines complete dim=1
//#pragma HLS BIND_STORAGE variable=gray_lines type=ram_s2p impl=bram

    static uint8_t sobel_lines[2][WIDTH];
#pragma HLS ARRAY_PARTITION variable=sobel_lines complete dim=1
//#pragma HLS BIND_STORAGE variable=sobel_lines type=ram_s2p impl=bram

    static uint8_t rgb_r_lines[6][WIDTH];
#pragma HLS ARRAY_PARTITION variable=rgb_r_lines complete dim=1
//#pragma HLS BIND_STORAGE variable=rgb_r_lines type=ram_s2p impl=bram

    static uint8_t rgb_g_lines[6][WIDTH];
#pragma HLS ARRAY_PARTITION variable=rgb_g_lines complete dim=1
//#pragma HLS BIND_STORAGE variable=rgb_g_lines type=ram_s2p impl=bram

    static uint8_t rgb_b_lines[6][WIDTH];
#pragma HLS ARRAY_PARTITION variable=rgb_b_lines complete dim=1
//#pragma HLS BIND_STORAGE variable=rgb_b_lines type=ram_s2p impl=bram

    static uint8_t gauss_win[7][7];
#pragma HLS ARRAY_PARTITION variable=gauss_win complete dim=0

    static uint8_t sobel_win[3][3];
#pragma HLS ARRAY_PARTITION variable=sobel_win complete dim=0

    static uint8_t gray_wr = 0;
    static uint8_t sobel_wr = 0;
    static uint8_t rgb_wr = 0;

    pixel_data p;
    src >> p;

    if (p.user) {
        x = y = 0;
        gray_wr = 0;
        sobel_wr = 0;
        rgb_wr = 0;
    }

    //<--------------- Grayscale --------------->
    uint8_t P_r = GR(p.data);
    uint8_t P_g = GG(p.data);
    uint8_t P_b = GB(p.data);
    uint8_t gray = (77*P_r + 150*P_g + 29*P_b) >> 8;

    //<--------------- Gaussian Blur --------------->
    // Shift window horizontally
    for (int i = 0; i < 7; i++) {
#pragma HLS UNROLL
        for (int j = 0; j < 6; j++) {
#pragma HLS UNROLL
            gauss_win[i][j] = gauss_win[i][j+1];
        }
    }

    for (int i = 0; i < 6; i++) {
#pragma HLS UNROLL
        uint8_t line_id;
        if (gray_wr >= i) {
            line_id = gray_wr - i;
        }
        else{
            line_id = gray_wr + 6 - i;
        }
        gauss_win[i][6] = gray_lines[line_id][x];
    }
    gauss_win[6][6] = gray;

    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
#pragma HLS UNROLL
        for (int j = 0; j < 7; j++) {
#pragma HLS UNROLL
            sum += (gauss_win[i][j] * gauss_7x7[i][j]);
        }
    }
    uint8_t blurred = sum >> 10;

    gray_lines[gray_wr][x] = gray;
    sobel_lines[sobel_wr][x] = blurred;
    rgb_r_lines[rgb_wr][x] = P_r;
    rgb_g_lines[rgb_wr][x] = P_g;
    rgb_b_lines[rgb_wr][x] = P_b;
    //<--------------- Sobel --------------->
    for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
        for (int j = 0; j < 2; j++) {
#pragma HLS UNROLL
            sobel_win[i][j] = sobel_win[i][j+1];
        }
    }

    uint8_t sobel_prev = sobel_wr^1;
    //uint8_t sobel_prev = (sobel_wr == 0) ? 1 : 0;
    sobel_win[0][2] = sobel_lines[sobel_prev][x];
    sobel_win[1][2] = sobel_lines[sobel_wr][x];
    sobel_win[2][2] = blurred;

    int16_t gx = 0, gy = 0;
    for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
        for (int j = 0; j < 3; j++) {
#pragma HLS UNROLL
            gx += sobel_win[i][j] * sobel_gx[i][j];
            gy += sobel_win[i][j] * sobel_gy[i][j];
        }
    }
    int16_t abs_gx = (gx < 0) ? -gx : gx;
    int16_t abs_gy = (gy < 0) ? -gy : gy;
    uint16_t magnitude = abs_gx + abs_gy;
    uint8_t edge = (magnitude > 255) ? 255 : (uint8_t)magnitude;
    uint8_t output = (x >= 7 && y >= 7) ? edge : 0;

    //<--------------- RGB --------------->
    uint8_t rgb_rd = (rgb_wr == 5) ? 0 : (rgb_wr + 1);
    uint8_t RGB_r = rgb_r_lines[rgb_rd][x];
    uint8_t RGB_g = rgb_g_lines[rgb_rd][x];
    uint8_t RGB_b = rgb_b_lines[rgb_rd][x];

    //<--------------- Edge count --------------->
    if (output > 20) {
    	edge_count++;


    }
    if (p.last && y == HEIGHT - 1){
    	//satisfied = (edge_count > 420000); // To check unsatisfied threshold, very high.
    	satisfied = (edge_count > ((WIDTH-7)*(HEIGHT-7))>>3); // To check parrot or in general threshold
    	//satisfied = (edge_count > 56727);
    	edge_count = 0;
    	frame_counter++;
    }
    //<--------------- Output --------------->
    //p.data = SR(gray) + SG(gray) + SB(gray); // <----- UNCOMMENT JUST FOR GRAY SCALE
    //p.data = SR(blurred) + SG(blurred) + SB(blurred); // <----- UNCOMMENT JUST FOR GRAY SCALE & BLUR
    //p.data = SR(output) + SG(output) + SB(output); // <----- UNCOMMENT FOR SOBEL
    //p.data = SR(RGB_r) + SG(RGB_g) + SB(RGB_b); // <----- UNCOMMENT JUST FOR RGB
    // TO TEST INDIVIDUAL THINGS, COMMENT FROM HERE TILL LINE 230
    if ((x >= COMP_X && x < COMP_X + COMP_WIDTH) && (y >= COMP_Y && y < COMP_Y + COMP_HEIGHT)){
    	if(frame_counter>0){
    		uint16_t comp_x = x - COMP_X;
        	uint16_t comp_y = y - COMP_Y;
        	uint32_t comp_p = satisfied?satisfied_data[comp_y][comp_x]:unsatisfied_data[comp_y][comp_x];
        	uint8_t comp_r = (comp_p >> 16) & 0xFF;
        	uint8_t comp_g = (comp_p >> 8) & 0xFF;
        	uint8_t comp_b = comp_p & 0xFF;
        	p.data = SR(comp_r) + SG(comp_g) + SB(comp_b);
    	}
    }
    else{
    	if (output > 20){
    		p.data = SR(255) + SG(255) + SB(255);
    	}
    	else{
    		p.data = SR(RGB_r) + SG(RGB_g) + SB(RGB_b);
    	}
    }
    // COMMENT TILL HERE TO TEST THE REST
    dst << p;
    if (p.last) {
        x = 0;
        y++;
        gray_wr = (gray_wr == 5) ? 0 : (gray_wr + 1);
        sobel_wr ^= 1;
        //sobel_wr = (sobel_wr == 0) ? 1 : 0;
        rgb_wr = (rgb_wr == 5) ? 0 : (rgb_wr + 1);
    }
    else {
        x++;
    }
}

//void stream(pixel_stream &src, pixel_stream &dst, int frame){
//#pragma HLS INTERFACE ap_ctrl_none port=return
//    sobel(src, dst);
//}
