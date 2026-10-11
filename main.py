import math
import os
import time

import pygame

import ddr_core as core

WIDTH, HEIGHT = 800, 600
FPS = 60
DT = 1.0 / FPS

BACKGROUND = (0, 0, 0)
TRAJECTORY = (0, 255, 0)
TARGET_COLOR = (255, 0, 0)
START_COLOR = (120, 120, 255)
STOP_COLOR = (255, 220, 0)
TEXT_COLOR = (235, 235, 235)
DIM_TEXT = (150, 150, 150)

START_POSE = (100.0, 300.0, 0.0)
DEFAULT_TARGET = (400, 400)
SPRITE_SIZE = 70

def load_robot_sprite():
    path = os.path.join(os.path.dirname(__file__), "asset", "robot.png")
    try:
        image = pygame.image.load(path).convert_alpha()
        return pygame.transform.smoothscale(image, (SPRITE_SIZE, SPRITE_SIZE))
    except (pygame.error, FileNotFoundError):
        surface = pygame.Surface((SPRITE_SIZE, SPRITE_SIZE), pygame.SRCALPHA)
        pygame.draw.circle(surface, (200, 210, 225), (SPRITE_SIZE // 2, SPRITE_SIZE // 2), SPRITE_SIZE // 2 - 4)
        pygame.draw.line(surface, (20, 20, 20), (SPRITE_SIZE // 2, SPRITE_SIZE // 2),
                         (SPRITE_SIZE - 6, SPRITE_SIZE // 2), 4)
        return surface

class Simulation:

    def __init__(self):
        self.mode = "PID"
        self.target = DEFAULT_TARGET
        self.paused = False
        self.reset()

    def reset(self):
        """Put the robot back at the start pose and clear the trajectory."""
        self.robot = core.DifferentialDriveRobot(*START_POSE)
        self.start_xy = START_POSE[:2]
        self.controllers = core.ControllerSet(self.mode)
        self._restart_controllers()
        self.trajectory = [(START_POSE[0], START_POSE[1])]
        self.elapsed = 0.0
        self.arrived = False
        self.stop_position = None
        self.arrival_time = None
        self.v_cmd = self.w_cmd = 0.0
        self.distance_error = self.heading_error = 0.0

    def _restart_controllers(self):
        dx = self.target[0] - self.robot.x
        dy = self.target[1] - self.robot.y
        e_d = math.hypot(dx, dy)
        e_h = core.wrap_angle(math.atan2(dy, dx) - self.robot.theta)
        self.controllers.reset(e_d, e_h)

    def set_mode(self, mode):
        self.mode = mode
        self.reset()

    def set_target(self, pos):
        self.target = pos
        self.arrived = False
        self.stop_position = None
        self.arrival_time = None
        self.elapsed = 0.0
        self.trajectory = [(self.robot.x, self.robot.y)]
        self.start_xy = (self.robot.x, self.robot.y)
        self._restart_controllers()

    def update(self):
        if self.paused or self.arrived:
            return

        v_cmd, w_cmd, e_d, e_h = self.controllers.compute(self.robot, self.target, DT)
        self.distance_error, self.heading_error = e_d, e_h

        if core.has_arrived(e_d, self.robot.speed):
            self.robot.stop()
            self.arrived = True
            self.arrival_time = self.elapsed
            self.stop_position = (self.robot.x, self.robot.y)
            self.v_cmd = self.w_cmd = 0.0
            return

        self.v_cmd, self.w_cmd = v_cmd, w_cmd
        self.robot.step(v_cmd, w_cmd, DT)
        self.elapsed += DT
        self.trajectory.append((self.robot.x, self.robot.y))

    def draw(self, screen, sprite, font, small):
        screen.fill(BACKGROUND)

        if len(self.trajectory) > 1:
            pygame.draw.lines(screen, TRAJECTORY, False,
                              [(int(x), int(y)) for x, y in self.trajectory], 3)

        pygame.draw.circle(screen, START_COLOR, (int(self.start_xy[0]), int(self.start_xy[1])), 6, 2)
        pygame.draw.circle(screen, TARGET_COLOR, self.target, 10)
        pygame.draw.circle(screen, (90, 0, 0), self.target, int(core.STOP_DISTANCE) + 8, 1)

        rotated = pygame.transform.rotate(sprite, -math.degrees(self.robot.theta))
        screen.blit(rotated, rotated.get_rect(center=(self.robot.x, self.robot.y)))

        tip = (self.robot.x + 42 * math.cos(self.robot.theta),
               self.robot.y + 42 * math.sin(self.robot.theta))
        pygame.draw.line(screen, (255, 255, 255), (self.robot.x, self.robot.y), tip, 2)

        if self.stop_position is not None:
            sx, sy = int(self.stop_position[0]), int(self.stop_position[1])
            pygame.draw.line(screen, STOP_COLOR, (sx - 10, sy - 10), (sx + 10, sy + 10), 3)
            pygame.draw.line(screen, STOP_COLOR, (sx - 10, sy + 10), (sx + 10, sy - 10), 3)

        self._draw_hud(screen, font, small)

    def _draw_hud(self, screen, font, small):
        state = "ARRIVED" if self.arrived else ("PAUSED" if self.paused else "RUNNING")
        lines = [
            (f"Controller : {self.controllers.label}", TEXT_COLOR),
            (f"State      : {state}", STOP_COLOR if self.arrived else TEXT_COLOR),
            (f"Time       : {self.elapsed:6.2f} s", TEXT_COLOR),
            (f"Distance e : {self.distance_error:6.1f} px", TEXT_COLOR),
            (f"Heading e  : {math.degrees(self.heading_error):6.1f} deg", TEXT_COLOR),
            (f"v / omega  : {self.robot.speed:6.1f} px/s  {self.w_cmd:5.2f} rad/s", TEXT_COLOR),
            (f"wheels L/R : {self.robot.v_left:6.1f} / {self.robot.v_right:6.1f}", TEXT_COLOR),
        ]
        if self.arrived:
            lines.append((f"Final error: {self.distance_error:6.2f} px  (t = {self.arrival_time:.2f} s)", STOP_COLOR))
        for i, (text, color) in enumerate(lines):
            screen.blit(font.render(text, True, color), (14, 12 + i * 20))

        help_text = "1 P   2 PD   3 PID   |   click: new target   SPACE pause   R reset   S screenshot   ESC quit"
        screen.blit(small.render(help_text, True, DIM_TEXT), (14, HEIGHT - 22))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Intelligent Differential Drive Robot Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    small = pygame.font.SysFont("consolas", 13)
    sprite = load_robot_sprite()

    sim = Simulation()
    mode_keys = {pygame.K_1: "P", pygame.K_2: "PD", pygame.K_3: "PID"}

    running = True
    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in mode_keys:
                    sim.set_mode(mode_keys[event.key])
                elif event.key == pygame.K_SPACE:
                    sim.paused = not sim.paused
                elif event.key == pygame.K_r:
                    sim.reset()
                elif event.key == pygame.K_s:
                    os.makedirs("screenshots", exist_ok=True)
                    name = f"screenshots/{sim.mode}_{time.strftime('%H%M%S')}.png"
                    pygame.image.save(screen, name)
                    print("saved", name)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                sim.set_target(event.pos)

        sim.update()
        sim.draw(screen, sprite, font, small)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
