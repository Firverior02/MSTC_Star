import bisect
import math

import networkx as nx
import numpy as np

#V_MOV = 0.5  # m/s
#V_ROT = 140  # deg/s
V_MOV = 0.5  # m/s
V_ROT = 45.84  # rad/s

SQRT_2 = math.sqrt(2)
PI = math.pi
YAW = {(1, 0): ('E', 0), (1, 1): ('NE', PI/4), (0, 1): ('N', PI/2),
       (-1, 1): ('NW', 3*PI/4), (-1, 0): ('W', PI), (-1, -1): ('SW', 5*PI/4),
       (0, -1): ('S', 3*PI/2), (1, -1): ('SE', 7*PI/4)}


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, ts=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.ts = ts


class Robot:
    def __init__(self, path, G):
        self.state = path[0]
        self.S = path
        self.N = len(self.S)
        self._travel_cache = {}
        self._turn_cache = {}
        self.T, self.V = self.__init_time_series(G)

        

    def get_cur_state(self, ts):
        """ locate ts in time series T using binary search """
        if ts >= self.T[-1]:
            self.state.ts = ts
            return self.N-1, self.state

        ti = bisect.bisect(self.T, ts) - 1
        dx = self.S[ti+1][0] - self.S[ti][0]
        dy = self.S[ti+1][1] - self.S[ti][1]
        k = (ts - self.T[ti]) / (self.T[ti+1] - self.T[ti])
        x_sign = 0 if dx == 0 else (1 if dx > 0 else -1)
        y_sign = 0 if dy == 0 else (1 if dy > 0 else -1)

        self.state = State(
            x=self.S[ti][0] + k*dx, y=self.S[ti][1] + k*dy,
            yaw=YAW[(x_sign, y_sign)][1], ts=ts)

        return ti, self.state

    # def __init_time_series(self, G: nx.Graph):
    #     T, V = [0.0] * self.N, [0.0] * (self.N-1)

    #     cur = self.state
    #     for i, nxt in enumerate(self.S[1:]):
    #         dist = 1.0 if cur[0] == nxt[0] or cur[1] == nxt[1] else SQRT_2
    #         V[i] = V_MOV
    #         T[i+1] = T[i] + dist / V[i]
    #         cur = nxt

    #     return T, V
    
    # def __init_time_series(self, G: nx.Graph):
    #     S = self.S
    #     N = len(S)
    #     T = [0.0] * N
    #     V = [0.0] * (N - 1)

    #     vmax = V_MOV       # e.g., 0.5 m/s
    #     a = 0.6            # m/s²
    #     omega = 0.8        # rad/s

    #     def euclidean(p1, p2):
    #         return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    #     for i in range(N - 1):
    #         p1, p2 = S[i], S[i + 1]
    #         dist = euclidean(p1, p2)
    #         V[i] = vmax

    #         # Travel time: acceleration-limited vs constant speed
    #         if dist <= (vmax ** 2) / a:
    #             travel_time = math.sqrt(dist / a)
    #         else:
    #             travel_time = dist / vmax + vmax / a

    #         rotation_time = 0
    #         if i != 0:
    #             dx1, dy1 = S[i][0] - S[i - 1][0], S[i][1] - S[i - 1][1]
    #             dx2, dy2 = S[i + 1][0] - S[i][0], S[i + 1][1] - S[i][1]
    #             if (dx1, dy1) != (dx2, dy2):                    
    #                 a_point = np.array(S[i - 1])
    #                 b_point = np.array(S[i + 0])
    #                 c_point = np.array(S[i + 1])
                    
    #                 ba = a_point - b_point
    #                 bc = c_point - b_point
    #                 cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    #                 angle = np.pi - np.arccos(cosine_angle)
                    
    #                 rotation_time = angle / omega

    def __init_time_series(self, G: nx.Graph):
        S = self.S
        T = [0.0] * self.N
        V = [0.0] * (self.N - 1)

        vmax = 0.5
        a = 0.6
        omega = 0.8
        total_time = 0

        def euclidean(p1, p2):
            return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5

        def travel_time(p1, p2):
            key = (p1, p2)
            if key in self._travel_cache:
                return self._travel_cache[key]
            d = euclidean(p1, p2)
            if d <= vmax**2 / a:
                t = math.sqrt(d / a)
            else:
                t = d / vmax + vmax / a
            self._travel_cache[key] = t
            return t

        def is_turn(p1, p2, p3):
            key = (p1, p2, p3)
            if key in self._turn_cache:
                return self._turn_cache[key]
            dx1, dy1 = p2[0] - p1[0], p2[1] - p1[1]
            dx2, dy2 = p3[0] - p2[0], p3[1] - p2[1]
            result = (dx1, dy1) != (dx2, dy2)
            self._turn_cache[key] = result
            return result

        total_turn = 0
        turns = [S[0]]
        for i in range(1, self.N - 1):
            if is_turn(S[i - 1], S[i], S[i + 1]):
                turns.append(S[i])
                           
                a_point = np.array(S[i - 1])
                b_point = np.array(S[i + 0])
                c_point = np.array(S[i + 1])
                
                ba = a_point - b_point
                bc = c_point - b_point
                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle = np.pi - np.arccos(cosine_angle)
                
                total_turn += angle

        turns.append(S[-1])

        n = len(turns)
        for j in range(n-1):
            total_time += travel_time(turns[j], turns[j + 1])

        
        total_time += (total_turn) / (omega)
        avg_time = total_time / self.N
        next_time = 0
        
        for i in range(self.N):
            T[i] = next_time
            next_time += avg_time
            
        return T, V


    #         T[i + 1] = T[i] + travel_time + rotation_time

        # Count turns 63.47, 55.74s
        # num_turns = 0
        # for i in range(1, N - 1):
        #     dx1, dy1 = S[i][0] - S[i - 1][0], S[i][1] - S[i - 1][1]
        #     dx2, dy2 = S[i + 1][0] - S[i][0], S[i + 1][1] - S[i][1]
        #     if (dx1, dy1) != (dx2, dy2):
        #         num_turns += 1
                   
        #         a = np.array(S[i - 1])
        #         b = np.array(S[i + 0])
        #         c = np.array(S[i + 1])
                
        #         ba = a - b
        #         bc = c - b
        #         cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        #         angle = np.pi - np.arccos(cosine_angle)
                
        #         T[i] += angle / omega

        #turn_time = (num_turns * math.pi) / (2 * omega)
        #T[-1] += turn_time

        # Optionally print for debugging
        #print(f"Turn time: {turn_time:.2f}s for {num_turns} turns")
        #print(f"Total time: {T[-1]:.2f}s")



