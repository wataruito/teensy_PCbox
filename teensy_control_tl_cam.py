"""
teensy_control_tl_cam.py
================================================================================
2022/07/25 wi
    Start modifying from Sentech_camera_control.py
================================================================================
Function:
    Intended to use Teledyne FLIR camera to capture movies, using Spinnaker SDK.
        Spinnnaker_2.7.0.128
        spinnaker_python-2.7.0.128-cp38-cp38-win_amd64

Create Python environment:
    $ conda create --name spinnaker
	$ conda activate spinnaker
    $ conda install python=3.8 anaconda
    $ conda install -c conda-forge keyboard                                     # spinnaker test code requires keyboard
    $ python  -m pip install spinnaker_python-2.x.x.x-cp3x-cp3x-win_amd64.whl   # install downloaded whl file
    $ pip install opencv-contrib-python                                         # OpenCV
    $ pip install scikit-video                                                  # for mp4 video output
    $ pip install EasyPySpin                                                    # python wrapper of the wrapper for Spinnaker SDK
    $ pip install pyserial                                                      # communicate with teensy via serial (Jupyter)
"""
from numba import jit
import EasyPySpin
import cv2
import skvideo.io
import threading
import time


class camThread(threading.Thread):
    ################################################################################
    # camThread class for multi-threading camera acquisition
    ################################################################################
    def __init__(self, camID, output_file, trig_mode=False, gain=1.0, fps=4, exposure=1000.0):
        threading.Thread.__init__(self)
        self.previewName = camID[0]
        self.cam_id = camID[2]
        self.cam_type = camID[1]
        self.output_file = output_file
        self.fps = fps
        self.gain = gain
        self.exposure = exposure
        self.trig_mode = trig_mode
        self.new_gain = gain
        self.new_exposure = exposure
        self.new_fps = fps
        self.camera_on = True

    def run(self):
        self.acquire_movie()

    ################################################################################
    # acquire_movie
    #   Each thread doing the followings:
    #       1. Prepare the output movie file
    #       2. Initialize the camera
    #       3. Set camera parameters
    #       4. Set up preview window
    #       5. Acquire movie
    #       6. Clean up
    ################################################################################
    @jit
    def acquire_movie(self):
        print("Starting " + self.previewName)

        # Prepare the video writer through skvideo.io
        if self.output_file != "":
            out = skvideo.io.FFmpegWriter(self.output_file, outputdict={
                '-r': str(self.fps),
                '-vcodec': 'libx264',  # use the h.264 codec
                # '-crf': '0',           #set the constant rate factor to 0, which is lossless
                # '-preset':'veryslow'   #the slower the better compression, in principle, try
                # other options see https://trac.ffmpeg.org/wiki/Encode/H.264
            }, inputdict={
                '-r': str(self.fps)
            })

        # Initialize the camera
        cap = EasyPySpin.VideoCapture(self.cam_id)

        if not cap.isOpened():
            print("Camera can't open\nexit")
            return -1

        # auto white balance
        if self.cam_type == 'color':
            cap.set(cv2.CAP_PROP_AUTO_WB, True)

        if self.trig_mode:
            # gain 0-48
            cap.set(cv2.CAP_PROP_GAIN, self.gain)
            print('gain: ', cap.get(cv2.CAP_PROP_GAIN))

            cap.set_pyspin_value("TriggerMode", "On")
            # cap.set_pyspin_value("ExposureMode", "TriggerWidth")
            cap.set_pyspin_value("TriggerSelector", "FrameStart")
            cap.set_pyspin_value("TriggerSource", "Line3")

        else:
            cap.set_pyspin_value("ExposureMode", "Timed")
            cap.set_pyspin_value("TriggerMode", "Off")
            # cap.set_pyspin_value("TriggerSource", "Line3")

            # Set camera parameters
            # fps
            cap.set(cv2.CAP_PROP_FPS, self.fps)

            # gain 0-48
            cap.set(cv2.CAP_PROP_GAIN, self.gain)
            print('gain: ', cap.get(cv2.CAP_PROP_GAIN))
            # exposure
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
            print('exposure: ', cap.get(cv2.CAP_PROP_EXPOSURE))
            print('ExposureAuto: ', cap.get_pyspin_value('ExposureAuto'))
            print('ExposureTime: ', cap.get_pyspin_value('ExposureTime'))

        # cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
        # cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto

        # Set up preview window
        cv2.namedWindow(self.previewName)

        # For real-time fps calculation
        prev_frame_time = 0
        new_frame_time = 0
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Acquire movie
        while self.camera_on:
            if self.gain != self.new_gain:
                cap.set(cv2.CAP_PROP_GAIN, self.new_gain)
                self.gain = self.new_gain

            if not self.trig_mode:
                if self.exposure != self.new_exposure:
                    cap.set(cv2.CAP_PROP_EXPOSURE, self.new_exposure)
                    self.exposure = self.new_exposure

                if self.fps != self.new_fps:
                    cap.set(cv2.CAP_PROP_FPS, self.new_fps)
                    self.fps = self.new_fps

            _ret, frame = cap.read()

            # the color-space is BGR, so we need to change the color-space
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # compute realtime fps
            new_frame_time = time.time()
            real_fps = 1/(new_frame_time-prev_frame_time)
            prev_frame_time = new_frame_time
            real_fps = int(real_fps)
            real_fps = str(real_fps)
            cv2.putText(frame, real_fps, (7, 70), font,
                        3, (100, 255, 0), 3, cv2.LINE_AA)

            # img_show = cv2.resize(frame, None, fx=0.25, fy=0.25)
            img_show = frame

            # Append the frame to the video writer
            if self.output_file != "":
                out.writeFrame(frame)
            # Display the frame
            cv2.imshow(self.previewName, img_show)
            # key monitoring
            key = cv2.waitKey(30)
            if key == ord("q"):
                break

        # Clean up
        if self.trig_mode:
            # cap.set_pyspin_value("ExposureMode", "Timed")
            cap.set_pyspin_value("TriggerMode", "Off")
        if self.output_file != "":
            out.close()
        cap.release()
        # cv2.destroyAllWindows()
        cv2.destroyWindow(self.previewName)


def live_movie(gain=1.0, exposure=40000.0, fps=4, trig_mode=True):
    # Note:
    #   <gain> 0-48
    #   <fps> is in the range of 1-59 and used as the frame rate of the movie file.
    #       At 60, frame drops may occur.
    #   <exposure> should be smaller than (1/fps * 1000000) in microseconds.
    #   <trig_mode> is for triggering the camera
    #       When trig_mode=False, need to set both exposure and gain.
    #       When trig_mode=True, need to set only fps.

    camera_1 = ['Firefly FFY-U3-16S2C', 'color', '21040292']
    camera_2 = ['Firefly FFY-U3-16S2M', 'bw', '20216234']

    cam_1_movie = 'cam_1.mp4'
    cam_2_movie = 'cam_2.mp4'

    thread1 = camThread(camera_1, cam_1_movie,
                        trig_mode=trig_mode, gain=gain, fps=fps, exposure=exposure)
    thread1.start()

    thread2 = camThread(camera_2, cam_2_movie,
                        trig_mode=trig_mode, gain=gain, fps=fps, exposure=exposure)
    thread2.start()


if __name__ == "__main__":
    live_movie()
