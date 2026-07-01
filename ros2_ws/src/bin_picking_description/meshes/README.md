# meshes/ — 零件 CAD 模型

把你 3D 打印的零件 CAD 放在这里，供感知节点做位姿估计、抓取节点做标注。

- 格式：`.stl` / `.obj` / `.ply`
- 命名：如 `part.stl`
- 单位：CAD 常用 **mm**，感知节点参数 `model_scale: 0.001` 会换算成 m。
- 建议：零件设计**避免完全对称**、带可区分特征，利于位姿估计消歧。

感知节点引用方式（launch 参数）：
```
cad_model_path:=/abs/path/.../bin_picking_description/meshes/part.stl
```
