#![allow(unused)]
#![allow(dead_code)]

use std::f32::consts::PI;

use raylib::{ffi::GetFPS, prelude::*};

const WINDOW_WIDTH: i32 = 480 * 2;
const WINDOW_HEIGHT: i32 = 480 * 2;

const PIXEL_RESOLUTION_W: usize = 24;
const PIXEL_RESOLUTION_H: usize = 24;

const SQUARE_PIXEL_SIDE: usize = 40;
const PIXEL_WIDTH: i32 = WINDOW_WIDTH / PIXEL_RESOLUTION_W as i32;
const PIXEL_HEIGHT: i32 = WINDOW_HEIGHT / PIXEL_RESOLUTION_H as i32;
const MOVEMENT_DISTANCE: i32 = 5;
const UNIT_DIRECTION_VECTOR_SIZE: i32 = 20;

// const PIXEL_COUNTS_W: usize = WINDOW_WIDTH as usize / SQUARE_PIXEL_SIDE;
// const PIXEL_COUNTS_H: usize = WINDOW_HEIGHT as usize / SQUARE_PIXEL_SIDE;

const INITIAL_POS_X: i32 = WINDOW_WIDTH / 2;
const INITIAL_POS_Y: i32 = WINDOW_HEIGHT / 2;

const OBJECT_RADIUS: i32 = 13;
const OBJECT_COLOR: Color = Color::RED;
const FOV_ANGLE: f32 = -60.0;
const RAY_LENGTH: i32 = 220;

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(WINDOW_WIDTH, WINDOW_HEIGHT)
        .title("Rscast")
        .build();

    let mut game_state = GameState::default();

    rl.set_target_fps(60);

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        // data pipeline
        game_state.mount_motion_listener(&mut d);

        // rendering pipeline
        d.clear_background(Color::BLACK);
        // d.draw_text(
        //     &format!(
        //         "x: {} y:{}",
        //         game_state.object_pos.x, game_state.object_pos.y
        //     ),
        //     10,
        //     10,
        //     20,
        //     Color::CYAN,
        // );
        d.draw_fps(10, 10);
        draw_grid(&mut d);
        let mouse_pos_x = d.get_mouse_x();
        let mouse_pos_y = d.get_mouse_y();

        game_state.looking_at_line_end = Pos::new(mouse_pos_x, mouse_pos_y);

        draw_movable_object(&mut d, game_state.object_pos);
        // game_state.singular_raycast.draw(&mut d);
        game_state.fov_raycast_start.draw(&mut d);
        game_state
            .fov_raycast_end
            .draw_color(&mut d, Color::LIGHTCYAN);
        game_state.draw_looking_at_line(&mut d);

        // draw_walls(&mut d, game_state.walls.clone());
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pos {
    pub x: i32,
    pub y: i32,
}

impl Pos {
    pub const ZERO: Self = Self { x: 0, y: 0 };

    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn to_vec2(&self) -> Vector2 {
        Vector2 {
            x: self.x as f32,
            y: self.y as f32,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RayCast {
    pub start: Pos,
    pub end: Pos,
}

impl RayCast {
    pub const ZERO: Self = Self {
        start: Pos::ZERO,
        end: Pos::ZERO,
    };
    pub fn new(object_pos: Pos) -> Self {
        Self {
            start: Pos::new(object_pos.x, object_pos.y + OBJECT_RADIUS),
            // TODO: modify it to reach the end of "screen"(Modify this further for collision)
            end: Pos::new(WINDOW_WIDTH / 2, WINDOW_HEIGHT),
        }
    }

    pub fn start_end(start: Pos, end: Pos) -> Self {
        Self { start, end }
    }

    pub fn draw(&self, d: &mut RaylibDrawHandle) {
        d.draw_line_ex(
            self.start.to_vec2(),
            self.end.to_vec2(),
            2.0,
            Color::YELLOWGREEN,
        );
    }

    pub fn draw_color(&self, d: &mut RaylibDrawHandle, color: Color) {
        d.draw_line_ex(self.start.to_vec2(), self.end.to_vec2(), 2.0, color);
    }

    pub fn move_ray(&mut self, origin: &Pos, delta_angle: f32) {
        self.start = *origin;
    }

    pub fn rotate(&mut self, center_pos: Pos, theta: f32) {
        dbg!(theta.to_degrees());
        let theta = theta + (PI / 4.0);
        let angle_from_x_axis = 0;
        println!("rotating...");
        let dx = (self.end.x - center_pos.x) as f32;
        let dy = (self.end.y - center_pos.y) as f32;
        dbg!(dx);
        dbg!(dy);

        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        let rotated_x = cos_theta * dx - sin_theta * dy + center_pos.x as f32;
        let rotated_y = sin_theta * dy + cos_theta * dy + center_pos.y as f32;

        self.end.x = rotated_x as i32;
        self.end.y = rotated_y as i32;
        dbg!(self.end);
    }
}

#[derive(Debug)]
pub struct GameState {
    pub object_pos: Pos,

    pub fov_raycast_start: RayCast,
    pub fov_raycast_end: RayCast,
    pub fov_angle: f32,
    pub looking_at_line_end: Pos,
    pub direction_vector: Vector2,

    pub walls: Vec<Vec<i32>>,
    camera_plane: Vector2,
}

impl GameState {
    pub fn new() -> Self {
        let object_pos = Pos::new(INITIAL_POS_X, INITIAL_POS_Y);
        let mut walls = vec![vec![0; PIXEL_RESOLUTION_W]; PIXEL_RESOLUTION_H];

        walls[0] = vec![1; PIXEL_RESOLUTION_W];
        walls[PIXEL_RESOLUTION_H - 1] = vec![1; PIXEL_RESOLUTION_W];
        (0..PIXEL_RESOLUTION_H).for_each(|idx| {
            walls[idx][0] = 1;
            walls[idx][PIXEL_RESOLUTION_W - 1] = 1;
        });

        let fov_angle = FOV_ANGLE / 2.0;
        let fov_angle_rad = fov_angle.to_radians();

        // angle of first raycast with respect to x-axis;
        let angle_of_firstrc_x = (90.0 - fov_angle / 2.0);

        let ray_length = RAY_LENGTH as f32;
        let fov_raycast_start = RayCast::start_end(
            object_pos,
            Pos {
                x: object_pos.x + (ray_length * fov_angle_rad.cos()) as i32,
                y: object_pos.y + (ray_length * fov_angle_rad.sin()) as i32,
            },
        );

        let fov_raycast_end = RayCast::start_end(
            object_pos,
            Pos {
                x: (2 * object_pos.x) - fov_raycast_start.end.x,
                y: fov_raycast_start.end.y,
            },
        );

        Self {
            object_pos,
            fov_raycast_start,
            fov_raycast_end,
            fov_angle,
            looking_at_line_end: Pos::new(object_pos.x, object_pos.y + 20),
            direction_vector: Vector2::new(-1.0, 0.66),
            camera_plane: Vector2::new(0.0, 0.66),

            walls,
        }
    }

    pub fn draw_looking_at_line(&self, d: &mut RaylibDrawHandle) {
        d.draw_line_ex(
            self.object_pos.to_vec2(),
            self.looking_at_line_end.to_vec2(),
            2.0,
            Color::CYAN,
        );
    }

    pub fn reset(&mut self) {
        self.object_pos = Pos::new(INITIAL_POS_X, INITIAL_POS_Y);
    }

    #[allow(clippy::identity_op)]
    pub fn move_object(&mut self, x: i32, y: i32) {
        if self.object_pos.x >= WINDOW_WIDTH {
            self.object_pos.x = 0 + SQUARE_PIXEL_SIDE as i32;
            return;
        }

        if self.object_pos.y > WINDOW_HEIGHT {
            self.object_pos.y = 0 + SQUARE_PIXEL_SIDE as i32;
            return;
        }

        if self.object_pos.x <= 0 {
            self.object_pos.x = WINDOW_WIDTH - SQUARE_PIXEL_SIDE as i32;
            return;
        }

        if self.object_pos.y <= 0 {
            self.object_pos.y = WINDOW_HEIGHT - SQUARE_PIXEL_SIDE as i32;
            return;
        }

        let fov_angle = FOV_ANGLE / 2.0;
        let fov_angle_rad = fov_angle.to_radians();
        let ray_length = RAY_LENGTH as f32;

        self.object_pos.x += x;
        self.object_pos.y += y;
        self.fov_raycast_start.start = self.object_pos;
        self.fov_raycast_start.end = Pos {
            x: self.object_pos.x + (ray_length * fov_angle_rad.cos()) as i32,
            y: self.object_pos.y + (ray_length * fov_angle_rad.sin()) as i32,
        };

        self.fov_raycast_end.start = self.object_pos;
        self.fov_raycast_end.end = Pos {
            x: (2 * self.object_pos.x) - self.fov_raycast_start.end.x,
            y: self.fov_raycast_start.end.y,
        };
    }

    pub fn mount_motion_listener(&mut self, d: &mut RaylibDrawHandle) {
        // object motion
        if d.is_key_down(KeyboardKey::KEY_L) || d.is_key_down(KeyboardKey::KEY_RIGHT) {
            self.move_object(MOVEMENT_DISTANCE, 0);
        }
        if d.is_key_down(KeyboardKey::KEY_H) || d.is_key_down(KeyboardKey::KEY_LEFT) {
            self.move_object(-MOVEMENT_DISTANCE, 0);
        }
        if d.is_key_down(KeyboardKey::KEY_K) || d.is_key_down(KeyboardKey::KEY_UP) {
            self.move_object(0, -MOVEMENT_DISTANCE);
        }
        if d.is_key_down(KeyboardKey::KEY_J) || d.is_key_down(KeyboardKey::KEY_DOWN) {
            self.move_object(0, MOVEMENT_DISTANCE);
        }

        // rotate ray
        if d.is_key_pressed(KeyboardKey::KEY_U) {
            self.fov_raycast_start
                .rotate(self.object_pos, 60.0_f32.to_radians());

            self.fov_raycast_end.end =
                horizontal_reflection(self.fov_raycast_start.end, self.object_pos.x);
        }

        if d.is_key_pressed(KeyboardKey::KEY_R)
            && (d.is_key_down(KeyboardKey::KEY_LEFT_SHIFT)
                || d.is_key_down(KeyboardKey::KEY_RIGHT_SHIFT))
        {
            self.reset();
        }
    }
}

impl Default for GameState {
    fn default() -> Self {
        Self::new()
    }
}

/// draw the unit pixel grid
fn draw_grid(d: &mut RaylibDrawHandle) {
    // draw vertical lines
    (0..PIXEL_RESOLUTION_W).for_each(|i| {
        d.draw_line(
            i as i32 * PIXEL_WIDTH,
            0,
            i as i32 * PIXEL_WIDTH,
            WINDOW_HEIGHT,
            Color::WHITE,
        );
    });

    // draw horizontal lines
    (0..PIXEL_RESOLUTION_H).for_each(|i| {
        d.draw_line(
            0,
            (i as i32 * PIXEL_HEIGHT),
            WINDOW_WIDTH,
            (i as i32 * PIXEL_HEIGHT),
            Color::WHITE,
        );
    })
}

// draw the movable object
fn draw_movable_object(d: &mut RaylibDrawHandle, object_pos: Pos) {
    d.draw_circle(object_pos.x, object_pos.y, 10.0, OBJECT_COLOR);
}

// TODO: 1. draw singular ray from the object
fn draw_ray_from_object(d: &mut RaylibDrawHandle, ray_pos: Pos) {
    d.draw_line(
        ray_pos.x,
        ray_pos.y,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        Color::GREEN,
    );
}

// emit rays from the object
fn draw_rays_from_object(d: &mut RaylibDrawHandle, fov_angle: i32, object_pos: Pos) {}

fn rotate_point_around_center_at_angle(point_pos: Pos, pivot_pos: Pos, angle: f32) -> Pos {
    // shift everything back to origin
    let origin_shifted_point_x = point_pos.x - pivot_pos.x;
    let origin_shifted_point_y = point_pos.y - pivot_pos.y;

    // rotate around origin
    let origin_rotated_position_x = (origin_shifted_point_x as f32 * angle.cos())
        - (origin_shifted_point_y as f32 * angle.sin());

    let origin_rotated_position_y = (origin_shifted_point_x as f32 * angle.sin())
        + (origin_shifted_point_y as f32 * angle.cos());

    // shift back to pivot pos
    let shifted_to_pos_x = origin_rotated_position_x + pivot_pos.x as f32;
    let shifted_to_pos_y = origin_rotated_position_y + pivot_pos.y as f32;

    Pos {
        x: shifted_to_pos_x as i32,
        y: shifted_to_pos_y as i32,
    }
}

pub fn map_pixel_unit_to_screen_pixel(x: usize, y: usize) -> (Pos, Pos) {
    let start_rect_pos = Pos {
        x: (SQUARE_PIXEL_SIDE * x) as i32,
        y: (SQUARE_PIXEL_SIDE * y) as i32,
    };

    let end_rect_pos = Pos {
        x: ((SQUARE_PIXEL_SIDE * x) + SQUARE_PIXEL_SIDE) as i32,
        y: ((SQUARE_PIXEL_SIDE * y) + SQUARE_PIXEL_SIDE) as i32,
    };

    (start_rect_pos, end_rect_pos)
}

pub fn draw_walls(d: &mut RaylibDrawHandle, wall_coordinates: Vec<Vec<i32>>) {
    (0..PIXEL_RESOLUTION_H).for_each(|y| {
        (0..PIXEL_RESOLUTION_W).for_each(|x| {
            if wall_coordinates[y][x] == 1 {
                let (start_pos, _) = map_pixel_unit_to_screen_pixel(x, y);
                d.draw_rectangle(
                    start_pos.x,
                    start_pos.y,
                    SQUARE_PIXEL_SIDE as i32,
                    SQUARE_PIXEL_SIDE as i32,
                    Color::RED,
                );
            }
        });
    });
}

fn horizontal_reflection(object_pos: Pos, x_axis: i32) -> Pos {
    Pos {
        x: 2 * x_axis - object_pos.x,
        y: object_pos.y,
    }
}

fn rotate_line_around_center(
    center_pos: Pos,
    line_start: Pos,
    line_end: Pos,
    angle: f32,
) -> (Pos, Pos) {
    todo!()
}

const MAP: [[u8; 24]; 24] = [
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 0, 0, 0, 3, 0, 3, 0, 3, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 2, 2, 0, 2, 2, 0, 0, 0, 0, 3, 0, 3, 0, 3, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 0, 0, 0, 0, 5, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 0, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 4, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    ],
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    ],
];

#[cfg(test)]
mod test {
    use crate::{Pos, rotate_point_around_center_at_angle};

    #[test]
    fn test_point_rotation() {
        let pos = Pos::new(5, -1);

        let pos = rotate_point_around_center_at_angle(pos, Pos { x: 2, y: 2 }, 30.0);
    }
}
