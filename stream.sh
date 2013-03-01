#!/bin/bash

#gst-launch-0.10 v4l2src device=/dev/video0  ! 'video/x-raw-yuv,width=320,height=240' !  x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=129.241.103.209 port=1234


#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=320,height=240, framerate=15/1' ! x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=129.241.103.209 port=1234

#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=320,height=240' ! TIVidenc1 codecName=h264enc engineName=codecServer encodingPreset=3 genTimeStamps=TRUE byteStream=TRUE ! rtph264pay ! udpsink host=129.241.103.209 port=1234

#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=640,height=480, framerate=30/1' ! dspmp4venc ! rtpmp4vpay ! udpsink host=129.241.103.209 port=1234



gst-launch-0.10 playbin uri=fd://0 always-copy=FALSE ! 'video/x-raw-yuv,width=640,height=480, framerate=30/1' ! dspmp4venc ! rtpmp4vpay ! udpsink host=10.0.0.100 port=1234
#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=640,height=480, framerate=30/1' ! dspmp4venc ! rtpmp4vpay ! udpsink host=10.0.0.100 port=1234
#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=320,height=240, framerate=30/1' ! dspmp4venc ! rtpmp4vpay ! udpsink host=129.241.103.209 port=1234
#gst-launch-0.10 v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=640,height=480, framerate=30/1' ! ffenc_mpeg4 ! rtpmp4vpay ! udpsink host=129.241.103.209 port=1234
#gst-dsp-parse v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=320,height=240, framerate=15/1' ! dsph264enc ! rtph264pay ! udpsink host=129.241.103.209 port=1234


#gst-launch-0.10 -v videotestsrc ! dspvdec ! filesink location=output.m4v

