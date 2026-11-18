# 智能婴儿陪护舱 BabyCabin

大学生创新训练项目结题成果。系统由 STM32H743、ESP32 图传、CI1302 离线语音、DHT11、电脑端网页以及鸿蒙 APP 组成。当前交付是无人桌面演示原型，不用于婴儿或载人，不是医疗或监护设备。

| 成果 | 位置 |
| --- | --- |
| 结题报告、答辩 PPT、商业计划书、演示视频 | [closeout](closeout/) |
| STM32H743 CubeIDE 工程与 F407 早期工程 | [H743](fetal-baby/) / [F407](firmware-f407/) |
| GB01 台架快照、语音控制候选及模块源码 | [firmware-snapshots](firmware-snapshots/) / [firmware-candidates](firmware-candidates/) |
| 四层 KiCad 原理图、PCB、封装库及询价材料 | [hardware](hardware/) |
| 专用语音词表、模型训练与验证材料 | [voice](voice/) / [vision](vision/) |
| 电脑端网页、摄像头与识别服务 | [babycabin-desktop-web](https://github.com/13261688309/babycabin-desktop-web) |
| 鸿蒙 APP | [babycabin-harmony-app](https://github.com/13261688309/babycabin-harmony-app) |

## 复现与边界

- STM32CubeIDE 导入对应目录下的工程，保留原第三方组件许可证。GB01 台架快照和语音候选不是同一个固件；候选构建记录不能替代整机上板或运动验收。
- 四层板为设计与询价版本，Gerber、BOM、贴片坐标只供报价核对，不代表制造批准、装配确认或电气验收。
- 电脑端运行视觉推理，ESP32 提供画面，STM32 和手机端处理各自的状态或操作。Cry 等类别是图像信号，不是诊断；模型对照的开发集不是独立实机准确率评估。
- 不包含原始训练影像、现场串口日志、整片 Flash 备份、账号配置、配网密钥或厂商原始工具包。外部摄像头的厂商固件不在项目源码内。
