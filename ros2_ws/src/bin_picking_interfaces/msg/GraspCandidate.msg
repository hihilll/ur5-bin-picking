# 单个抓取候选（夹爪 TCP 目标位姿）
geometry_msgs/PoseStamped grasp_pose   # 夹爪 TCP 目标位姿（基座系）
float32 width                          # 夹爪开口宽度 (m)
float32 score                          # 抓取质量评分 (越大越好)
string object_id                       # 关联的零件
