# jupyter_interface.ipynb
2022/07/28 wi<BR>
    
**Function:**<BR>
Control Teledyne FLIR cameras through Jupyter<BR>
    
**Create Python environment:**
```
$ conda create --name spinnaker
$ conda activate spinnaker
$ conda install python=3.8 anaconda
$ conda install -c conda-forge keyboard               # spinnaker test code requires keyboard
# install the downloaded whl file
$ python  -m pip install spinnaker_python-2.x.x.x-cp3x-cp3x-win_amd64.whl
$ pip install opencv-contrib-python                   # OpenCV
$ pip install scikit-video                            # for mp4 video output
$ pip install EasyPySpin                              # python wrapper of the wrapper for Spinnaker SDK
$ pip install pyserial                                # communicate with teensy via serial (Jupyter)
```
