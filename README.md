
# 写在前面的话

我自己把项目代码全部自己重写了一遍，然后照猫画虎地在 `img_manager` 包下添加了两个组件 `VirtualWriter` 和 `ImgProcUtil` 以及 `include` 和 `config` 目录。


> 小吐槽：Lucy 发给我们的虚拟机环境有问题，排查了半天以为是自己代码的问题结果重装 Ubuntu 自己配环境之后就好了 :）

---

# 使用方法：

和学长的项目一样，运行这三行命令即可：

```bash
colcon build
source install/setup.bash
ros2 launch img_manager manager.launch.py
```

修改两个包下的 `config/*.xml` 以实现不同功能。

### `manager.xml`

1. `export_mode`: 可以设置为 `"play"` 或 `"write"`，分别表示直接在窗口中播放和导出为视频文件。

2. `arg`: 对获取到的图像的处理方式，设置为 `"ALL"` 时实现作业要求的完整功能。其余可以设置的值可参阅 `src/ImgProcUtil.cpp`。

3. `target_path`, `fps`, `width`, `height`: 对应导出视频的路径，帧数，图像尺寸。


### `camera.xml`

1. `source_path`: 要操作的视频文件。

2. `fps`: 帧率，决定了 `CameraNode` 广播图像消息的频率。

---

# 项目结构分析

## 1. `camera_node` 部分 

### 1.1 `VirtualCamera` 类

作用是访问目标图片/视频，并调用 `get_frame()` 获取 `cv::Mat` 类型的图像文件。

- 成员变量（均为 `private`）：

    - `std::string source_path;` 

        虚拟摄像头指向的源文件路径。文件应该是支持的图片或视频格式。

    - `static const std::set<std::string> PICTURE_TYPE = {"jpg", "jpeg", "png"};` 
        
        列举可识别的图片文件类型。如果目标文件是以这几个后缀名结尾则用图片形式打开。

    - `static const std::set<std::string> VIDEO_TYPE = {"avi", "mp4"};` 
        
        列举可识别的视频文件类型。如果目标文件是以这几个后缀名结尾则用视频形式打开。

    - `std::string open_type;` 

        值为 `"VIDEO"` 或 `"PICTURE"`。表示源路径对应文件的打开方式。

    - `cv::VideoCapture cam;`

    - `bool is_opened;` 

        Tips: 如果 `open_type` 为 `"PICTURE"`，那么此值常设为 `true`；否则表示 `cam` 是否为 `open` 状态。

- 成员函数（均为 `public`）：

    - 构造/析构函数

        构造时必须传入参数 `source_path` 并判断文件类型，如果类型不支持则报告错误并退出，否则写入到 `open_type` 中。
    
    - `int open(void)`

        开启虚拟摄像头。使用 `try-catch` 尝试打开视频文件。若源文件是图片文件则无需操作。返回值为 $0$ 时开启正常，为 $-1$ 时开启异常。

    - `int close(void)`

        关闭虚拟摄像头。析构函数就是这个函数的封装。关闭不会出问题但是还是要报告状态，所以返回 $0$。

    - `cv::Mat get_frame(void)`

        **记得开启摄像头**。返回源文件图片或源文件视频帧。记得判断 `frame.empty()` 并报告错误。

### 1.2 `CameraNode` 类 ： 继承自 `rclcpp::Node` 类

内置一个 `VirtualCamera` 类，通过 RCLCPP 发布携带图像信息的 ROS 消息。

- 成员变量（均为 `private`）：

    - `std::shared_ptr<VirtualCamera> cam;`

    - `std::string source_path;`

        就是 `cam->source_path`。

    -  `int fps;`

    -  `rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_`

        发布图像消息的图像发布器。

    - `rclcpp::TimerBase::SharedPtr timer_`

        计时器。是用来接收 `Node::create_wall_timer()` 的返回值的。

        > &nbsp;
        >
        > Tips: `Node::create_wall_timer()` 函数的用法
        >
        > create_wall_timer的主要作用是让你能够**周期性地执行某段代码**。它创建的定时器会严格按照你设定的 period参数，基于系统的真实时间（挂钟时间）周期性触发。
        >
        > 参数（按顺序）：
        >
        > > `std::chrono::duration period`：表示定时器触发的时间间隔。可以使用 `std::chrono::milliseconds(int)` 兼容。
        >
        > > `callback`：可调用对象（函数或 lambda 表达式），定时器到期时执行的回调函数。以使用 `std::bind()` 为例：
        > > > `std::bind(成员函数指针, 对象实例（指针或引用）, 参数...)`
        > > >
        > > > 例如：`std::bind(&CameraNode::get_image, this)`
        > >
        > > 即可返回一个可用的 `callback` 参数。
        >
        > &nbsp;

        若想关闭计时器的定期回调，只需调用 `timer_->cancel()`。

- 成员函数（均为 `public`）：

    - `void get_image()`

        1. 调用 `cam->get_frame()`；

        2. 创建 `std_msgs::msg::Header header`；

            分为 `set__frame_id()` 与 `set__stamp()` 两部分。

        3. 使用 `cv_bridge::CvImage` 转化为 ROS 消息并发布；

---

## 2. `img_manager` 部分

### 2.1 `ManagerNode` 类 ： 继承自 `rclcpp::Node` 类

- 成员变量（均为 `private`）：

    - `rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;`

        接收 `virtual_camera` 的 `CmaeraNode` 节点定时发送的 ROS 图像消息，并在每次收到消息时回调函数 `[this](... msg){ this->image_callback(msg); }` 以实现处理并展示图像效果。

- 成员函数：

    - `private void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);`

        接受 ROS 消息并转换为 `cv::Mat` 类型变量。随后调用 `mark_light_strip()` 处理图像，最后通过 `cv::imshow()` 展示。

### 2.2 `VirtualWriter` 类

作为封装的将图像写入视频的类。将 `src/img_manager/config/manager.yml` 配置文件中 `export_mode` 改为 `"write"` 即可使用。

有 `target_path`, `fps`, `frame_size`, `writer` 等参数。意义比较明确，不再阐述。

### 2.3 `ImgProcUtil` 类

静态工具类（其实完全可以用命名空间但是我喜欢这样写），禁止创建实例。内置了若干种 `cv::Mat->cv::Mat` 的函数用于操作图像，有一个 `img_proc_all` 函数集成了本次作业需要实现的所有功能，将 `src/img_manager/config/manager.yml` 配置文件中 `arg` 改为 `"ALL"` 即可使用。

---