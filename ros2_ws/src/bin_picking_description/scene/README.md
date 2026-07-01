# scene/ — 工作场景碰撞体

放料框、相机支架等的尺寸/网格，用于 MoveIt2 Planning Scene 碰撞避障（阶段3）。

建议内容：
- `bin.stl` 或在代码里用长方体盒近似料框
- 料框相对 base_link 的位姿（写进 grasp/bringup 配置）

阶段3 会在抓取执行前把这些加入 MoveIt Planning Scene，规划无碰撞轨迹。
