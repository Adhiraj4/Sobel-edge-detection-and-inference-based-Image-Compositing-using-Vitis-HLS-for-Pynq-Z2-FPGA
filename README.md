# Sobel-edge-detection-and-inference-based-Image-Compositing-using-Vitis-HLS-for-Pynq-Z2-FPGA
Sobel edge Detection was done for image edge Detection. Based on the edge density of the entire frame, a small 128 x 128 image is composited on the top right corner of the frame. The incoming pixel is first converted to gray scale, and the gray scale image is blurred using a 7x7 Gaussian Blur kernel with a standard deviation ($\sigma$) of 3. 
The 7x7 window slides horizontally and vertically. Row buffers are created for this purpose, to make the design algorithm efficient, circular buffering is used, instead of shifting row-wise in memory. 

Using the same principles of circular row buffering, a 3x3 sliding window is used to get the x and y gradients for Sobel Edge detection. Furthermore, the initial coloured pixel value is stored through the pipeline and depending on the density of the edge, either the colour pixel is displayed or a completely white pixel, giving a non-traditional visual of edges on a coloured image. 

Finally, after a whole frame has passed, the total edge density of the pixel image is computed, and based on a low/ high density, different images are composited on the top right corner. 

To convert the images into pixel data, a Python (.py) program is provided; one can use it as they like in order to get their own custom image. 
Applications include detecting brain tumours, road-lane monitoring, etc. 

The image data is in compositing.hpp file. 

The project is done for a Xilinix PYNQ-Z2 board, and the selected part is \textbf{xc7z020clg400-1}. The design.bd file is [AXI Video DMA (AXI VDMA)](https://www.fpgadeveloper.com/2014/08/using-the-axi-dma-in-vivado.html/). In the video stream, and in between the hdmi\_in and the axi\_vdma blocks, the Sobel IP must be put. An extra AXI master pin must be added and connected to Interconnect IP inside the video stream IP of the VDMA. The IP has to be connected to the 142 MHz clock and the corresponding reset signals present in the design. 

Port type s_axilite was used. 

For the users ease, a synthesized base.bit (bitstream) and base.hwh (hardware handover) files have been provided as well as the .ipynb file to run the video on xilinx pynq's provided [Jupyter productivity platform](https://pynq.readthedocs.io/en/v2.6.1/getting_started.html)
