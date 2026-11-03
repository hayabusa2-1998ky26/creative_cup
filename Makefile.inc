APPL_COBJS += 

# 1. ここには「メインファイル（app）」以外を、拡張子「.o」のまま指定します
APPL_CXXOBJS += pybricks_ev3rt.o run.o all_calibration.o color_calibration.o motion_sequence.o motor_control.o

# 2. メイン言語が C++ であることを指定（これにより app.cpp が自動でコンパイルされます）
SRCLANG := c++

