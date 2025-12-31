import os
import sys
import pickle
import numpy as np

from utils import *

current_file=os.path.abspath(__file__)
current_dir=os.path.dirname(current_file)
planner_dir=os.path.dirname(current_dir)

#
sys.path.insert(0,planner_dir)

from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'
print(f"RSG root: {rsg_root}")


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        """ Load tomogram data from pickle file and initialize planner """
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1] # 相邻层可行性差值
        diff_g = np.abs(elev_g[1:] - elev_g[:-1]) # 相邻层几何高度差值
        
        # 识别上下坡道位置
        gateway_up = np.zeros_like(trav, dtype=bool) 
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos):
        """
        支持起点/终点在不同 slice 的规划。
        输入 start_pos/end_pos 可以是 [x,y] 或 [x,y,z]（z 用于计算 slice）。
        """
        #self.start_idx[1:] = self.pos2idx(start_pos)
        #self.end_idx[1:] = self.pos2idx(end_pos)

         # 提取平面坐标并转为索引（pos2idx 返回 [y_idx, x_idx] 的浮点数组）
        s_xy_raw = np.asarray(self.pos2idx(np.asarray(start_pos[:2], dtype=np.float64)), dtype=np.float64)
        e_xy_raw = np.asarray(self.pos2idx(np.asarray(end_pos[:2], dtype=np.float64)), dtype=np.float64)

        # 显式取整并裁剪到合法范围，避免越界
        s_xy = np.floor(s_xy_raw).astype(int)
        e_xy = np.floor(e_xy_raw).astype(int)
        s_xy = np.clip(s_xy, [0, 0], [self.map_dim[0] - 1, self.map_dim[1] - 1])
        e_xy = np.clip(e_xy, [0, 0], [self.map_dim[0] - 1, self.map_dim[1] - 1])

        self.start_idx[1:] = s_xy
        self.end_idx[1:] = e_xy

        # 计算 slice（如果提供 z 则使用，否则默认 0）
        if len(start_pos) >= 3:
            self.start_idx[0] = self.pos2slice(start_pos[2])
        else:
            self.start_idx[0] = 0

        if len(end_pos) >= 3:
            self.end_idx[0] = self.pos2slice(end_pos[2])
        else:
            self.end_idx[0] = 0


        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def pos2slice(self, z_pos):
        slice_idx = int(np.round((z_pos - self.slice_h0) / self.slice_dh))
        slice_idx = np.clip(slice_idx, 0, self.n_slice - 1)
        return slice_idx
    
    def get_slice_height(self, slice_idx):
        return self.slice_h0 + slice_idx * self.slice_dh
    
    
    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx