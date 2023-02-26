//https://grantshandy.github.io/posts/raycasting/
// don't depend on the standard library because it's big
#![no_std]

use core::{arch::wasm32, panic::PanicInfo};

use libm::{ceilf, cosf, fabsf, floorf, sinf, sqrtf, tanf};
use core::f32::consts::{PI, FRAC_PI_2};

// initialize an external function call
extern "C" {
    fn vline(x: i32, y: i32, len: u32);
}

// needed if you don't compile with std
// we don't actually do anything with the info, but the panic handler function needs to take one argument of the PanicInfo type, hence the placeholder name and lifetime
//it complains that this is already defined but it's defined in the standard library
#[panic_handler]
fn phandler(_: &PanicInfo<'_>) -> ! {
    wasm32::unreachable();
}


// https://wasm4.org/docs/guides/user-input
const GAMEPAD1: *const u8 = 0x16 as *const u8;

const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;

const BUTTON_LEFT: u8 = 16;  // 00010000
const BUTTON_RIGHT: u8 = 32; // 00100000
const BUTTON_UP: u8 = 64;    // 01000000
const BUTTON_DOWN: u8 = 128; // 10000000

/// bitmaps lowkey no cap
const MAP: [u16; 8] = [
    0b1111111111111111,
    0b1000001010000101,
    0b1011100000110101,
    0b1000111010010001,
    0b1010001011110111,
    0b1011101001100001,
    0b1000100000001101,
    0b1111111111111111,
];

/// arbitrary magic number that controls the amplitude of the arrow key effect
const STEP_SIZE: f32 = 0.045;

const FOV: f32 = PI / 2.7;
const HALF_FOV: f32 = FOV * 0.5;
const ANGLE_STEP: f32 = FOV / 160.0; // angle between each ray
const WALL_HEIGHT: f32 = 100.0;

/// stores the player state across update() calls
struct State {
    player_x: f32,
    player_y: f32,
    player_angle: f32
}

impl State {
    //move the character
    pub fn update(&mut self, up: bool, down: bool, left: bool, right:bool) {
        // store the last position for hacky stuff
        let previous_position = (self.player_x, self.player_y);


        // sinf is negative because 0 is at the top and y is at the bottom
        
        if up {
            self.player_x += cosf(self.player_angle) * STEP_SIZE;
            self.player_y += -sinf(self.player_angle) * STEP_SIZE;
        }

        if down {
            self.player_x -= cosf(self.player_angle) * STEP_SIZE;
            self.player_y -= sinf(self.player_angle) * STEP_SIZE;
        }

        if right {
            self.player_angle -= STEP_SIZE;
        }

        if left {
            self.player_angle += STEP_SIZE;
        }

        //if we gonna hit a wall, don't
        if point_in_wall(self.player_x, self.player_y) {
            (self.player_x, self.player_y) = previous_position;
        }
    }

    /// Return the nearest wall the ray intersects with on a horizontal gridline
    fn horizontal_intersection(&self, angle: f32) -> f32 {
        //determine if the angle is facing up
        let up = fabsf(floorf(angle / PI) % 2.0) != 0.0;

        // `first_x` and `first_y` are all the first grid intersections
        // the ray interacts with
        // basically if the player is facing up at a y of 2/3, then the
        // first gridline would be 1/3 away
        let first_y = if up {
            ceilf(self.player_y) - self.player_y
        } else {
            floorf(self.player_y) - self.player_y
        };
        let first_x = -first_y / tanf(angle);
        
        //ray extension values
        let dy = if up { 1.0 } else { -1.0 };
        let dx = -dy / tanf(angle);

        // next_x and next_y are mutable values which will keep track
        // of how far away the ray is from the player.
        let mut next_x = first_x;
        let mut next_y = first_y;

        //keep extending the ray until a wall is hit or we're out of bounds
        for _ in 0..256 {
            // current_x and current_y are here the ray is currently
            // next_x and nex_y are relative coords to the player
            // current_x and current_y are absolute
            let current_x = next_x + self.player_x;
            let current_y = if up {
                next_y + self.player_y
            } else {
                next_y + self.player_y - 1.0
            };

            //break the loop if a wall is found
            if point_in_wall(current_x, current_y) {
                break;
            }

            // if wall is not hit on this extension, add
            // dx and dy to our current position and keep going
            next_x += dx;
            next_y += dy;
        }

        //return the distance from the next x and y to the player
        distance(next_x, next_y)
    }

    /// return the nearest wall the ray intersects with on a vertical gridline
    fn vertical_intersection(&self, angle: f32) -> f32 {
        // determine if an angle is "facing right"
        let right: bool = fabsf(floorf((angle - FRAC_PI_2) / PI) % 2.0) != 0.0;

        // first_x and first_y are the first grid intersections that the ray intersects with
        let first_x = if right {
            ceilf(self.player_x) - self.player_x
        } else {
            floorf(self.player_x) - self.player_x
        };
        let first_y = -tanf(angle) * first_x;

        // dx and dy are the ray extension values (how far to jump until a vert intersection is hit)
        let dx = if right { 1.0 } else {-1.0 };
        let dy = dx * -tanf(angle);

        let mut next_x = first_x;
        let mut next_y = first_y;

        //yada yada go check the other function
        for _ in 0..256 {
            let current_x = if right {
                next_x + self.player_x
            } else {
                //imma be honest I got no clue why 1.0 is subtracted
                next_x + self.player_x - 1.0
            };
            let current_y = next_y + self.player_y;

            if point_in_wall(current_x, current_y) {
                break;
            }

        // if we didn't hit a wall keep going
        next_x += dx;
        next_y += dy;

        }

        //return the distance till the next y wall
        distance(next_x, next_y)
    }

    /// Returns 160 wall heights from the player's perspective and whether or not there needs to be a shadow
    pub fn get_view(&self) -> [(i32, bool); 160] {
        // the player's FOV is split in half by their viewing angle
        // in order to get the ray's first angle
        // we must add half the FOV to the player's angle to get the edge of the FOV
        let starting_angle = self.player_angle + HALF_FOV;

        let mut walls = [(0, false); 160];

        for (idx, wall) in walls.iter_mut().enumerate() {
            //idx is the number of ray, wall is a mutable reference to the value in `walls`
            let angle = starting_angle - idx as f32 * ANGLE_STEP;

            //get both the closest vertical and horizontal wall intersections for the angle
            let h_dist = self.horizontal_intersection(angle);
            let v_dist = self.vertical_intersection(angle);
            //draw a shadow on vertical walls
            let (min_dist, shadow) = if h_dist < v_dist {
                (h_dist, false)
            } else {
                (v_dist, true)
            };


            //determine which wall is closer and convert it to a height
            //also fisheye correction is a W
            //*wall = (WALL_HEIGHT / f32::min(h_dist, v_dist)) as i32;
            *wall = ((WALL_HEIGHT / (min_dist * cosf(angle - self.player_angle)) ) as i32, shadow);    
        }

        walls
    }
}

//as contradictory as it may seem to have a static mutable mut, this is a mutable struct with a *static* lifetime.
//all access must be unsafe lol unless it's from a `State` method
static mut STATE: State = State {
        // walls are on integers, so this spawns the player in the middle of the "square"
        player_x: 1.5,
        player_y: 1.5,
        player_angle: 0.0,
};

/// check if the map contains a wall at a point
fn point_in_wall(x: f32, y: f32) -> bool {
    //steps of selector algorithm
    // move down to the y line
    // | 0010
    // | 0100
    // |>0101
    //   1000
    //
    // 01b is our selector bit
    // boot it left until it's on the coordinate we're trying to check
    // 0001 selector bit
    // boot to where we want to check
    // 
    // 0001 <- x2 == 0100
    // 
    // 0100 selector bit after shift
    // 0101 line to check against
    // VVVV AND the two together (only make the last bit a 1 if both of the bits are a 1)
    // 0100 if this number is *not* 0, then the selected bit is a wall


    //get() is used instead of []
    match MAP.get(y as usize) {
        Some(line) => (line & (0b1 << x as usize)) != 0,
        //if the index is out of bounds, assume it's a wall
        None => true,
    }
}

///calculate the distance from 0,0 to a point
fn distance(a: f32, b: f32) -> f32 {
    sqrtf((a * a) + (b * b))
}

// no mangle keeps the name constant to allow the function to be called from the library
// this is called once a frame, the entry point
#[no_mangle]
unsafe fn update() {
    //see if the button pressed down is pressed down
    STATE.update(
        *GAMEPAD1 & BUTTON_UP != 0,
        *GAMEPAD1 & BUTTON_DOWN != 0,
        *GAMEPAD1 & BUTTON_LEFT != 0,
        *GAMEPAD1 & BUTTON_RIGHT != 0,
    ); 


    for (x, wall) in STATE.get_view().iter().enumerate() {
        let (height, shadow) = wall;

        if *shadow {
            *DRAW_COLORS = 0x2;
        } else {
            *DRAW_COLORS = 0x3;
        }
        vline(x as i32, 80 - (height / 2), *height as u32);
    }
}




