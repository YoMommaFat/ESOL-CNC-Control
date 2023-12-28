# https://itecnote.com/tecnote/python-sending-opencv-output-to-vlc-stream/
# https://gist.github.com/okdimok/465039e582f2b9810674952eeda85436

import numpy as np
import sys
import cv2

cap = cv2.VideoCapture(0)
fps = cap.get(cv2.CAP_PROP_FPS)
frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
sys.stderr.write("OpenCV camera data: {}x{} @{}FPS\n".format(frame_size[0], frame_size[1], fps))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        sys.stdout.buffer.write(frame.tobytes())
#        sys.stdout.buffer.write( frame.tostring() )
    else:
        break

cap.release()


"""
python streamVLC.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout "#display"
OK6 python streamVLC.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout '#transcode{vcodec=h264,vb=1024}:standard{access=http{mime=video/x-flv},mux=ffmpeg{mux=flv},dst=:1234}'
OK2 python streamVLC.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout '#transcode{vcodec=theo,vb=1024}:standard{access=http,mux=ogg,dst=:1234}'
NOK python streamVLC.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout '#transcode{vcodec=mp4v,acodec=none}:standard{access=http,mux=ts,dst=:1234}'
NOK python streamVLC.py | cvlc -v v4l2:///dev/video0:chroma=mp2v --v4l2-width 640 --v4l2-height 480 --sout '#transcode{vcodec=mp2v,acodec=mpga,fps=30}:rtp{mux=ts,sdp=rtsp://:1234/}'
#vlc http://192.168.1.6:1234
OKN python streamVLC.py | ffmpeg -r 8 -f rawvideo -pix_fmt bgr24 -s 640x480 -i - -c:v libx264 -f matroska -listen 1 tcp://192.168.1.6:1234
#vlc  tcp://192.168.1.6:1234
"""
"""
raspivid -o - -t 0     -w 1280-h 720    -fps 25 -g 75 -fl           | ffmpeg -f lavfi -i anullsrc=channel_layout=stereo:sample_rate=44100 -i pipe:0 -c:v copy -c:a aac -strict experimental -f flv -f flv rtmp://ip:port/app/stream
raspivid -o - -t 0 -n                                               | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264
raspivid -o - -t 0 -vf -w 800 -h 400 -n -fps 24                     | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8160}' :demux=h264
raspivid -o - -t 0     -w 800 -h 600    -fps 12                     | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8080/}' :demux=h264
raspivid -o - -t 0     -w 640 -h 480    -fps 25 -b 30000000 -ev -10 | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264
raspivid -o - -t 0 -hf -w 800 -h 400    -fps 24                     | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8160}' :demux=h264
http://192.168.1.6:8081/stream.flv
"""
