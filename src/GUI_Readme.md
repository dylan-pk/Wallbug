# Plant GUI

To run the Plant GUI first ensure that both plant_gui.py and plantColour.cpp are in the same directory.

Make sure all dependencies are installed:
```
# Python deps
pip install PySide6 opencv-python numpy
```
```
# System libs for GUI
sudo apt update && sudo apt install -y libgl1 libglib2.0-0 libx11-6 libxcb1 libxkbcommon0 libxext6 libxi6 libsm6 libxrender1 libxrandr2
```
```
# C++ build deps
sudo apt install -y build-essential cmake pkg-config libopencv-dev
g++ plantColour.cpp -o plantColour `pkg-config --cflags --libs opencv4`

```

From there in the terminal run:
`python3 ~/path/to/directory/plant_gui.py`
(Make sure to put in the actual directory of the file)
