# 视觉模型训练与验证

YOLO 标注包含 Cry、Happy、Neutral、Sleep 四类目标框。按相似图像与来源线索归组后，训练集 1,880 张、开发验证集 384 张、预留测试集 418 张。候选使用 YOLO11s 预训练初始化、640 像素、batch 4、AdamW；[训练曲线](candidate-training-results.csv)与[候选权重](candidate-best.pt)保留在此。

同一开发验证集、置信度 0.60、匹配 IoU 0.50：现用模型精确率 91.74%、召回率 78.14%；候选分别为 89.26%、73.12%。现用模型未替换，数据存在历史训练重叠可能。本比较不是独立实机准确率测试。

[对照方法](../docs/engineering/MODEL_COMPARISON.txt) · [复现实用脚本](https://github.com/13261688309/babycabin-desktop-web/tree/main/tools)
