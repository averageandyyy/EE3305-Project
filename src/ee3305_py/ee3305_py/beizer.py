import numpy as np

class BezierSmoother:
    def __init__(self, costmap, origin_x, origin_y, resolution, cols, rows, max_access_cost):
        self.costmap_ = costmap
        self.origin_x_ = origin_x
        self.origin_y_ = origin_y
        self.res_ = resolution
        self.cols_ = cols
        self.rows_ = rows
        self.max_cost_ = max_access_cost

    # --- map helpers (same math as your planners) ---
    def _xy_to_cr(self, x, y):
        c = int((x - self.origin_x_) / self.res_ - 0.5)
        r = int((y - self.origin_y_) / self.res_ - 0.5)
        return c, r

    def _in_map(self, c, r):
        return (0 <= c < self.cols_) and (0 <= r < self.rows_)

    def _is_free_xy(self, x, y):
        c, r = self._xy_to_cr(x, y)
        if not self._in_map(c, r):
            return False
        idx = r * self.cols_ + c
        return self.costmap_[idx] < self.max_cost_

    # --- small utilities ---
    def _unit(self, v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else v

    def _downsample_path(self, pts, min_dist=0.5, angle_thresh_deg=60.0):
        """
        Downsample path by removing points that are too close or don't have significant turns.
        
        Args:
            pts: list of (x,y) tuples
            min_dist: minimum distance between kept points (meters)
            angle_thresh_deg: minimum turn angle to keep a point (degrees)
        """
        if len(pts) <= 2:
            return pts[:]
        out = [pts[0]]
        last = np.array(pts[0], dtype=float)

        def turn_angle(a, b, c):
            v1 = np.array(b) - np.array(a)
            v2 = np.array(c) - np.array(b)
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-6 or n2 < 1e-6: return 0.0
            cosang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
            return np.degrees(np.arccos(cosang))

        for i in range(1, len(pts) - 1):
            b = np.array(pts[i], dtype=float)
            if np.linalg.norm(b - last) >= min_dist or turn_angle(last, b, pts[i+1]) >= angle_thresh_deg:
                out.append(tuple(b))
                last = b
        out.append(pts[-1])
        return out

    def _cubic_bezier(self, P0, P1, P2, P3, samples):
        t = np.linspace(0.0, 1.0, samples)
        B0 = (1 - t) ** 3
        B1 = 3 * (1 - t) ** 2 * t
        B2 = 3 * (1 - t) * t ** 2
        B3 = t ** 3
        x = B0 * P0[0] + B1 * P1[0] + B2 * P2[0] + B3 * P3[0]
        y = B0 * P0[1] + B1 * P1[1] + B2 * P2[1] + B3 * P3[1]
        return np.stack([x, y], axis=1)

    def _collision_free_polyline(self, pts):
        for (x, y) in pts:
            if not self._is_free_xy(x, y):
                return False
        return True
    
    def _resample_polyline_by_dist(self, pts, step=0.04):
        if len(pts) < 2:
            return pts[:]
        out = [tuple(pts[0])]
        acc = 0.0
        for i in range(1, len(pts)):
            x0, y0 = out[-1]
            x1, y1 = pts[i]
            dx, dy = x1 - x0, y1 - y0
            seglen = (dx*dx + dy*dy) ** 0.5
            if seglen < 1e-9:
                continue
            # place points every 'step' along this segment
            t = step - acc
            while t <= seglen + 1e-9:
                nx = x0 + (dx / seglen) * t
                ny = y0 + (dy / seglen) * t
                out.append((nx, ny))
                t += step
            acc = seglen - (t - step)
        if out[-1] != tuple(pts[-1]):
            out.append(tuple(pts[-1]))
        return out

    
    # offset frac (0.2-0.4) controls how "tight" the curve is, but risk collision
    # samples_per_seg controls smoothness for each segment (how dense, 50-120)
    def smooth(self, raw_pts, offset_frac=0.3, samples_per_seg=10,
               start_yaw=None, end_yaw=None, yaw_bias=0.6, 
               target_spacing=0.04, max_points=800,
               min_dist=0.5, angle_thresh_deg=60.0):
        """
        Apply Bezier curve smoothing to a raw path.
        
        Args:
            raw_pts: list[(x,y)] polyline from A*/Dijkstra/RRT*
            offset_frac: controls curve tightness (0.2-0.4), higher = smoother but more collision risk
            samples_per_seg: number of samples per bezier segment (affects curve density)
            start_yaw: optional starting yaw angle (radians)
            end_yaw: optional ending yaw angle (radians)
            yaw_bias: how much to bias toward start/end yaw (0=ignore, 1=full bias)
            target_spacing: target spacing between output points (meters)
            max_points: maximum number of points in output path
            min_dist: minimum distance for downsampling (meters)
            angle_thresh_deg: minimum angle for downsampling (degrees)
            
        Returns:
            list[(x,y)] smoothed polyline (or raw if smoothing collides)
        """
        if len(raw_pts) < 3:
            return raw_pts[:]
        # tune angle_thresh_dist (increase if want fewer anchor points, smoother)
        key = self._downsample_path(raw_pts, min_dist=min_dist, angle_thresh_deg=angle_thresh_deg)
        if len(key) < 3:
            return raw_pts[:]

        # tangents per key point
        tangents = []
        N = len(key)
        for i in range(N):
            if i == 0:
                tvec = np.array(key[1]) - np.array(key[0])
                if start_yaw is not None:
                    v = np.array([np.cos(start_yaw), np.sin(start_yaw)])
                    tvec = (1 - yaw_bias) * tvec + yaw_bias * v
            elif i == N - 1:
                tvec = np.array(key[-1]) - np.array(key[-2])
                if end_yaw is not None:
                    v = np.array([np.cos(end_yaw), np.sin(end_yaw)])
                    tvec = (1 - yaw_bias) * tvec + yaw_bias * v
            else:
                t1 = np.array(key[i]) - np.array(key[i-1])
                t2 = np.array(key[i+1]) - np.array(key[i])
                tvec = self._unit(t1) + self._unit(t2)
                if np.linalg.norm(tvec) < 1e-9:
                    tvec = self._unit(t2)
            tangents.append(self._unit(tvec))

        # build cubic segments
        smoothed = []
        for i in range(N - 1):
            P0 = np.array(key[i], dtype=float)
            P3 = np.array(key[i+1], dtype=float)
            L = np.linalg.norm(P3 - P0)
            d = offset_frac * L
            P1 = P0 + d * tangents[i]
            P2 = P3 - d * tangents[i+1]

            seg = self._cubic_bezier(P0, P1, P2, P3, samples_per_seg)
            if i > 0:
                seg = seg[1:]  # avoid duplicates
            smoothed.append(seg)

        # safety: if any point is in collision, keep raw path
        # return smoothed if self._collision_free_polyline(smoothed) else raw_pts[:]
        smoothed_dense = np.vstack(smoothed).tolist()

        # Safety first: check the dense curve
        if not self._collision_free_polyline(smoothed_dense):
            return raw_pts[:]

        # NEW: resample to fixed spatial spacing for RViz/controller
        smoothed = self._resample_polyline_by_dist(smoothed_dense, step=target_spacing)

        # Optional: cap the total poses to keep RViz snappy
        if len(smoothed) > max_points:
            stride = max(1, len(smoothed) // max_points)
            smoothed = smoothed[::stride]

        return smoothed