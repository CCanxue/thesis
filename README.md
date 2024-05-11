### 上位机程序代码
- **QT** 版本为 **6.6.2** ，用 **Cmake** 编译的 **Widget** 界面
- **Cmakelists** 文件中保存了 **Cmake** 编译方法，主要包含了对相机库、OpenCV库、串口通信库的引用，注意对文件路径的修改
- **Main** 函数用于界面显示，调用 **Widget.cpp** ，不需要修改
- **Widget.cpp** 为源文件
- **Widget.h** 为头文件
- **Widget.ui** 为显示界面控件代码
