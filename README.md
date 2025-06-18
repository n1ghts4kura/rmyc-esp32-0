# RMYC-ESP32

# *__FROM EOR TEAM IN DGZX.__*

## 制作组
> **2024届EOR战队**

程序设计:

- *deepseek / claude / chatgpt*

- **夜樱n1ghts4kura**
    - [传送门 Github](https://github.com/n1ghts4kura)   **/**   [传送门 bilibili](https://space.bilibili.com/438325806)

- **Aunnno**
    - [传送门 Github](https://github.com/Aunnno)

> 开发过程请查看 **dev-1** 分支

## 环境配置

1. 安装**VS-Code**

2. 安装插件
- C/C++ & C/C++ Excension Pack & CMake Tools
- ESP-IDF
- (选用)TODO Highlight

3. 配置**ESP-IDF**插件
这一步可以到网上找教程 此处不赘述  
注意 请文件安装到一个**你找得到的地方** 并将地址记录下来 **后面有用**

> 有两个地址 一个是 *esp-container* (地址1)  一个是*esp-tools* (地址2)

4. 下载项目 & 使用VS-Code打开
```sh
git clone <url>
```

5. 代码补全优化

- 打开.vscode/c_cpp_properties.json
```json
{
    "includePath": [
        // 在这里加上你刚刚所记录下来的 地址1
        // 示范如下
        "D:\\2025Project\\ESPIDF\\ESPIDF_CONTAINER\\v5.4.1\\esp-idf\\components",
        "D:/2025Project/ESPIDF/ESPIDF_CONTAINER/v5.4.1/esp-idf/components/log/include",
    ],

    "browse": {
        "path": [
            "D:\\2025Project\\ESPIDF\\ESPIDF_CONTAINER\\v5.4.1\\esp-idf\\components",
        ]
    },
}
```

## 使用说明

代码部分自行阅读

硬件直接烧录即可

GPIO口 **4号/5号** 为 **TX/RX** 接口
> 实际上到底怎么接入自己尝试  
> 我们的经验是
> - **4口**连接机器人 左1(RX)  
> - **5口**连接机器人 左2(TX)  
> - GND 连接 左3(GND)