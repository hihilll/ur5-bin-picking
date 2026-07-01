# 单个被识别零件的 6D 位姿（位姿估计结果）
string object_id                    # 匹配到的 CAD 模型标识
geometry_msgs/PoseStamped pose      # 零件位姿（header.frame_id 通常为相机系或基座系）
float32 fitness                     # 配准重叠度 (0~1, 越大越好)
float32 inlier_rmse                 # 内点均方根误差 (m, 越小越好)
