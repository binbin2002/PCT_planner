from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSpiral():
    pcd = ScenePCD()
    pcd.file_name = 'spiral0.3_2.pcd'

    map = SceneMap()
    map.resolution = 0.20
    map.ground_h = 0.0
    map.slice_dh = 0.5

    trav = SceneTrav()
    # 内核大小，用于形态学分析
    trav.kernel_size = 7
    # 最小可行走间隔，单位米
    trav.interval_min = 0.50
    # 机器人正常工作高度间隔，单位米
    trav.interval_free = 0.65
    # 最大允许坡度 (弧度)
    trav.slope_max = 0.40
    # 设置机器狗最大跨越台阶高度，单位米
    trav.step_max = 0.30
    # 可站立区域比例阈值
    trav.standable_ratio = 0.40
    # 障碍物代价
    trav.cost_barrier = 50.0
    # 安全边距，单位米
    trav.safe_margin = 1.2
    # 膨胀距离，单位米
    trav.inflation = 0.2

