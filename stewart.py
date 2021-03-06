import collections
import yaml
import numpy as np


EPS = 1e-5

PlatformState = collections.namedtuple(
    'PlatformState',
    [
        'base',
        'servos',
        'platform',
        'legs',
        'angles',
    ])


TranslationArea = collections.namedtuple(
    'TranslationArea',
    [
        'x_grid',
        'y_grid',
        'z_min_grid',
        'z_max_grid',
        'not_empty',
        'box',
    ])


class StewartPlatform(object):
    """ 
             
           _||_
            ||  /\
  hookah => || // \
            ||// 
           (  )
          (    )
    ======PLATFORM======
                         * <= swivel joint (D)
                         |
                servo    |
                bridge   |  
                         |  <= upper leg
                    |    |
    bearing         v    |
   joint (B) => .--------* <= swivel joint  (C) 
                |   
                | <= servo leg
       [SERVO] -| <= (A)
    
    ====BASE====
    """
    def __init__(self, config):
        self.servo_psi = np.zeros(6, np.float)
        self.servos = np.zeros((6, 3), np.float)
        self.real_servos = np.zeros((6, 3), np.float)
        self.joints = np.zeros((6, 3), np.float)
        # todo(savegor): use clearer names in configs!
        self.base_radius = config['base']['radius']
        self.servo_length = config['leg']['servo_length']
        self.middle_length = config['leg']['middle_length']
        self.leg_length = config['leg']['length']
        constraints = config.get('constraints') or {}
        angle = constraints.get('max_servo_angle') or 90.0 
        self.max_servo_angle = self._to_radian(angle)
        assert 'home_position' in config
        home = config['home_position']
        assert 'translation' in home
        assert 'rotation' in home
        self.home_translation = home['translation']
        self.home_rotation = [self._to_radian(a) for a in home['rotation']]

        def from_polar_cfg(cfg, delta_r):
            r = cfg['R'] + delta_r
            phi = self._to_radian(cfg['phi'])
            return [r * np.cos(phi), r * np.sin(phi), 0.0] 

        self.alpha0 = []
        self.to_servo = []
        self.to_world = []
        for i in range(6):
            assert i in config['base']['servos']
            servo = config['base']['servos'][i]
            assert i in config['platform']['joints']
            joint = config['platform']['joints'][i]
            self.servo_psi[i] = self._to_radian(servo['phi'])
            self.real_servos[i] = from_polar_cfg(servo, 0.0)
            self.servos[i] = from_polar_cfg(servo, self.middle_length)
            self.joints[i] = from_polar_cfg(joint, 0.0)
            assert 'alpha0' in servo
            self.alpha0.append(self._to_radian(servo['alpha0']))

            self.to_servo.append(self._roll(-self.servo_psi[i]))
            self.to_world.append(self._roll(self.servo_psi[i]))

    @staticmethod
    def _to_radian(degree):
        return degree / 180.0 * np.pi

    @staticmethod
    def _roll(psi):
        roll = np.eye(3)
        sin_psi, cos_psi = np.sin(psi), np.cos(psi)
        roll[0, 0] = cos_psi
        roll[1, 1] = cos_psi
        roll[0, 1] = -sin_psi
        roll[1, 0] = sin_psi
        return roll

    @staticmethod
    def _pitch(theta):
        pitch = np.eye(3)
        sin_theta, cos_theta = np.sin(theta), np.cos(theta)
        pitch[0, 0] = cos_theta
        pitch[2, 2] = cos_theta
        pitch[0, 2] = sin_theta
        pitch[2, 0] = -sin_theta
        return pitch

    @staticmethod
    def _yaw(phi):
        yaw = np.eye(3)
        sin_phi, cos_phi = np.sin(phi), np.cos(phi)
        yaw[1, 1] = cos_phi
        yaw[2, 2] = cos_phi
        yaw[1, 2] = -sin_phi
        yaw[2, 1] = sin_phi
        return yaw

    @staticmethod
    def _rotation_matrix(psi, theta, phi):
        roll = StewartPlatform._roll(psi)
        pitch = StewartPlatform._pitch(theta)
        yaw = StewartPlatform._yaw(phi)
        return np.dot(np.dot(roll, pitch), yaw)

    def calc_leg_state(self, leg_index, translation, rot):
        q_relative = self.joints[leg_index]
        q_world = translation + np.dot(rot, q_relative)
        b_world = self.servos[leg_index]

        l = np.linalg.norm(q_world - b_world)
        if l > self.leg_length + self.servo_length:
            # it is not possible to achieve the provided position
            return None, None, None

        # relatively leg_index-th servo coordinate system
        to_servo = self.to_servo[leg_index]
        q = np.dot(to_servo, q_world)

        b = np.dot(to_servo, self.servos[leg_index])
        assert abs(b[1]) < EPS
        assert abs(b[2]) < EPS

        x, y, z = q
        a = self.servo_length
        s = self.leg_length
        r = b[0]

        gamma = (a * a + (r - x) * (r - x) + y * y + z * z - s * s) / (2 * a)

        denom = np.sqrt(y * y + z * z)

        if y != 0:
            cos_t = gamma / denom
            if abs(cos_t) > 1.0:
                return None, None, None
            acos_t = np.arccos(cos_t)
            atan_t = np.arctan(z / y)

            # Note:
            # atan_t is in [-0.5 * pi, 0.5 * pi] with period of pi
            # acos_t is in [0, pi] with period of 2 * pi
            # as cos(t) = cos(-t) we should use both acos_t, -acos_t
            # so (atan_t + acos_t) is in [-1.5 * pi, 1.5 * pi] with period of pi
            # as the home position for the servo could be equal to pi
            # we should take into account shifts by pi and 2 * pi.
            possible_alphas = [atan_t + k1 * acos_t + k2 * np.pi
                               for k1 in [-1.0, 1.0]
                               for k2 in [0.0, 1.0, 2.0]]
        else:
            assert z != 0
            sin_t = gamma / z
            if abs(sin_t) > 1.0:
                return None, None, None
            
            asin_t = np.arcsin(sin_t)

            # Note:
            # asin_t is in [-pi/2, pi/2] with 2 * pi period
            # as the home position for the servo could be equal to pi
            # (asin_t + 2 * pi) is in [1.5 * pi, 2.5 * pi] so we also
            # should take it into account shift by 2 * pi.
            possible_alphas = [asin_t, asin_t + np.pi * 2]

        def check_alpha(alpha):
            servo_shift = np.array([0, np.cos(alpha) * a, np.sin(alpha) * a])
            point_c_rel = b + servo_shift
            point_b_rel = point_c_rel - [self.middle_length, 0, 0]
            return abs(np.linalg.norm(point_c_rel - q) - self.leg_length) < EPS

        alphas = [alpha for alpha in possible_alphas if check_alpha(alpha)]
        if not alphas:
            # Actually this should not have hapenned...
            # Should we raise an exception instead?
            return None, None, None

        alpha = sorted(alphas,
            key=lambda a: abs(a - self.alpha0[leg_index]))[0]


        if abs(alpha - self.alpha0[leg_index]) > self.max_servo_angle:
            return None, None, None
        
        servo_shift = np.array([0, np.cos(alpha) * a, np.sin(alpha) * a])        
        point_c_rel = b + servo_shift
        point_b_rel = point_c_rel - [self.middle_length, 0, 0]
        back_to_world = self.to_world[leg_index]
        
        point_c_world = np.dot(back_to_world, point_c_rel)
        point_b_world = np.dot(back_to_world, point_b_rel)
        return alpha, point_b_world, point_c_world

    def calc_state(self, translation, rotation):
        rot = self._rotation_matrix(rotation[0], rotation[1], rotation[2])

        base = (self.base_radius, np.array([0.0, 0.0, 0.0]))

        platform = []
        for leg_index in range(6): 
            q_relative = self.joints[leg_index]
            q_world = translation + np.dot(rot, q_relative)
            platform.append(q_world)
        platform = np.array(platform)

        legs = []
        angles = []
        servos = []
        
        for i in range(6):
            alpha, point_b, point_c = self.calc_leg_state(i, translation, rot)
            point_a = self.real_servos[i]
            point_d = platform[i] 

            legs.append([point_a, point_b, point_c, point_d])
            angles.append(alpha)
            servos.append(self.real_servos[i])

        return PlatformState(base, servos, platform, legs, angles)

    def check_state_is_possible(self, translation, rotation_matrix):
        for i in range(6):
            alpha, point_b, point_c = self.calc_leg_state(
                i, translation, rotation_matrix)
            if alpha is None:
                return False
        return True

    def calc_possible_translations(self, rotation, n=20, progress_callback=None):
        rot = self._rotation_matrix(rotation[0], rotation[1], rotation[2])

        max_shift = self.servo_length + self.leg_length

        surface_delta = self.leg_length
        min_x = self.home_translation[0] - surface_delta
        max_x = self.home_translation[0] + surface_delta
        min_y = self.home_translation[1] - surface_delta
        max_y = self.home_translation[1] + surface_delta
        min_z = self.leg_length - self.servo_length
        max_z = self.leg_length + self.servo_length

        dz = (max_z - min_z) / (3 * n)
        upper_z = np.zeros((n, n), np.float32)
        lower_z = np.zeros((n, n), np.float32)
        not_empty = np.zeros((n, n), np.bool)

        minimax = np.zeros((2, 3), np.float32)
        minimax[0] = 1e9
        minimax[1] = -1e9

        x_grid = np.linspace(min_x, max_x, n)
        y_grid = np.linspace(min_y, max_y, n)

        for i, x in enumerate(x_grid):
            if progress_callback is not None:
                progress_callback(i)
            for j, y in enumerate(y_grid):
                z = max_z

                not_found = False
                while not self.check_state_is_possible([x, y, z], rot):
                    z -= dz
                    if z < min_z:
                        not_found = True
                        break

                if not_found:
                    continue

                not_empty[i, j] = True

                upper_z[i, j] = z

                while self.check_state_is_possible([x, y, z], rot):
                    minimax[0][2] = min(minimax[0][2], z)
                    minimax[1][2] = max(minimax[1][2], z)
                    z -= dz

                z += dz
                lower_z[i, j] = z

                minimax[0][0] = min(minimax[0][0], x)
                minimax[0][1] = min(minimax[0][1], y)

                minimax[1][0] = max(minimax[1][0], x)
                minimax[1][1] = max(minimax[1][1], y)

        if progress_callback is not None:
            progress_callback(n)

        #print("LIMITS ", (minimax[1] - minimax[0]) * 100.0, "cm")

        return TranslationArea(x_grid, y_grid, lower_z, upper_z, not_empty, minimax)



if __name__ == '__main__':
    cfg = yaml.load(open('configs/platform_shuyak.yml', 'r'))

    platform = StewartPlatform(cfg)

    state = platform.calc_state(
        platform.home_translation,
        platform.home_rotation)

    assert all(state.angles)

    # todo(savegor): add several geometry tests

