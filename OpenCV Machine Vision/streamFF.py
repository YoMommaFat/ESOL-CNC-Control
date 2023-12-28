import ffmpeg
import cv2
import subprocess

server_url = "http://192.168.1.5:1234"

"""
OK6 python opencvx.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout '#transcode{vcodec=h264,vb=1024}:standard{access=http{mime=video/x-flv},mux=ffmpeg{mux=flv},dst=:1234}'
OK2 python opencvx.py | cvlc --demux=rawvideo --rawvid-fps=30 --rawvid-width=640 --rawvid-height=480 --rawvid-chroma=RV24 - --sout '#transcode{vcodec=theo,vb=1024}:standard{access=http,mux=ogg,dst=:1234}'
"""

def start_streaming(width, height, fps):
    process = (
        ffmpeg
        .input('pipe:', format='rawvideo', codec="rawvideo", pix_fmt='bgr24', s='{}x{}'.format(width, height))
        .output(
            server_url + '/stream',
            #codec = "copy", # use same codecs of the original video
            codec = "h264",
            listen=1, # enables HTTP server
            pix_fmt="yuv420p",
            preset="ultrafast",
            f="flv"
        )
        .overwrite_output()
        .run_async(pipe_stdin=True)
    )
    return process

def init_cap():
    cap = cv2.VideoCapture(0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, width, height

def run():
    cap, width, height = init_cap()
    fps = cap.get(cv2.CAP_PROP_FPS)
    streaming_process = start_streaming(width, height,fps)
    while True:
        ret, frame = cap.read()
        if ret:
            streaming_process.stdin.write(frame.tobytes())
        else:
            break
    streaming_process.stdin.close()
    streaming_process.wait()
    cap.release()

if __name__ == "__main__":
    run()
