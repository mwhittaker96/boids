use std::{ops::RangeInclusive, time::Instant};

use egui::{Color32, Pos2, Rect, Ui, Vec2, Visuals};
use log::{error, info};
use rand::Rng;

const SIMULATION_AREA_WIDTH: f32 = 800.0;
const SIMULATION_AREA_HEIGHT: f32 = 600.0;

const LEFT: f32 = -SIMULATION_AREA_WIDTH / 2.0;
const RIGHT: f32 = SIMULATION_AREA_WIDTH / 2.0;
const TOP: f32 = -SIMULATION_AREA_HEIGHT / 2.0;
const BOTTOM: f32 = SIMULATION_AREA_HEIGHT / 2.0;

const NEIGHBOR_RADIUS: f32 = 50.0;

const COHESION_COLOR: Color32 = Color32::BLUE;
const SEPARATION_COLOR: Color32 = Color32::RED;
const ALIGNMENT_COLOR: Color32 = Color32::GREEN;

const FRAME_TIME: f32 = 1.0 / 60.0;

// Separation

// Alignment

// Cohesion

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct BoidsApp {
    #[serde(skip)]
    boids: Vec<Boid>,

    // config stuff
    #[serde(skip)]
    num_boids: usize,
    #[serde(skip)]
    max_speed: f32,
    #[serde(skip)]
    max_force: f32,
    #[serde(skip)]
    last_update_time: std::time::Instant,

    #[serde(skip)]
    draw_debug_perimeters: bool,
    #[serde(skip)]
    separation_weight: f32,
    #[serde(skip)]
    alignment_weight: f32,

    #[serde(skip)]
    cohesion_weight: f32,
}

impl Default for BoidsApp {
    fn default() -> Self {
        Self {
            boids: Vec::new(),
            num_boids: 0,
            max_force: 0.5,
            max_speed: 5.0,
            last_update_time: Instant::now(),
            draw_debug_perimeters: true,
            separation_weight: 1.0,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
        }
    }
}

impl BoidsApp {
    pub fn update_boids(&mut self, dt: f32) {
        // SIMULATION LOGIC
        if self.boids.len() > self.num_boids {
            // Remove some boids
            for _ in [0..self.boids.len() - self.num_boids] {
                self.boids.remove(self.boids.len() - 1);
            }
        } else if self.boids.len() < self.num_boids {
            // Insert some boids

            let mut rng: rand::prelude::ThreadRng = rand::thread_rng();

            // Pick a random point in our field
            let pos = Pos2::new(
                rng.gen_range(-SIMULATION_AREA_WIDTH / 2.0..SIMULATION_AREA_WIDTH / 2.0),
                rng.gen_range(-SIMULATION_AREA_HEIGHT / 2.0..SIMULATION_AREA_HEIGHT / 2.0),
            );
            // Set a random initial velocity
            let rand_x_vel = rng.gen_range(-self.max_speed..self.max_speed);
            let rand_y_vel = rng.gen_range(-self.max_speed..self.max_speed);
            let random_velocity = Vec2::new(rand_x_vel, rand_y_vel);

            self.boids.push(Boid::new(pos, random_velocity));
        }

        self.update_forces();
        self.update_boids_position(dt);
    }

    fn update_boids_position(&mut self, dt: f32) {
        // Update positions from velocity/acceleration
        for boid in &mut self.boids {
            // Apply the acceleration to the velocity
            boid.velocity = boid.velocity + boid.acceleration;
            // clamp the velocity - can do length squared if needed here
            if boid.velocity.length() > self.max_speed {
                boid.velocity = boid.velocity.normalized() * self.max_speed;
            }
            // Zero out the acceleration
            boid.acceleration = Vec2::ZERO;

            boid.position += boid.velocity; //* dt;

            // wrap position
            if boid.position.x > RIGHT {
                boid.position.x = LEFT;
            }
            if boid.position.x < LEFT {
                boid.position.x = RIGHT;
            }
            if boid.position.y > BOTTOM {
                boid.position.y = TOP;
            }
            if boid.position.y < TOP {
                boid.position.y = BOTTOM;
            }
        }
    }

    pub fn update_forces(&mut self) {
        let mut separation_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());
        let mut cohesion_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());
        let mut alignment_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());

        for boid in &self.boids {
            separation_forces.push(boid.calculate_separation_force(
                &self.boids,
                self.separation_weight,
                self.max_force,
            ));

            alignment_forces.push(boid.calculate_alignment_force(
                &self.boids,
                self.alignment_weight,
                self.max_speed,
                self.max_force,
            ));

            cohesion_forces.push(boid.calculate_cohesion_force(
                &self.boids,
                self.cohesion_weight,
                self.max_force,
                self.max_speed,
            ));
        }

        for i in 0..self.boids.len() {
            self.boids[i].acceleration += separation_forces[i];
            self.boids[i].acceleration += alignment_forces[i];
            self.boids[i].acceleration += cohesion_forces[i];

            if separation_forces[i].length_sq() > alignment_forces[i].length_sq()
                && separation_forces[i].length_sq() > cohesion_forces[i].length_sq()
            {
                self.boids[i].color = SEPARATION_COLOR;
            } else if alignment_forces[i].length_sq() > separation_forces[i].length_sq()
                && alignment_forces[i].length_sq() > cohesion_forces[i].length_sq()
            {
                self.boids[i].color = ALIGNMENT_COLOR;
            } else if cohesion_forces[i].length_sq() > alignment_forces[i].length_sq()
                && cohesion_forces[i].length_sq() > separation_forces[i].length_sq()
            {
                self.boids[i].color = COHESION_COLOR;
            }
        }
    }
}

impl BoidsApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        cc.egui_ctx.set_visuals(Visuals::dark());
        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            info!("had storage");
            return eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default();
        }

        Default::default()
    }
}

impl eframe::App for BoidsApp {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let dt = Instant::now()
            .saturating_duration_since(self.last_update_time)
            .as_secs_f32();
        if dt >= FRAME_TIME {
            self.last_update_time = Instant::now();
            self.update_boids(dt);
            ctx.request_repaint();
        }

        // DRAW LOGIC

        // HACK! Idk why i ended up needing to do this in the update loop
        ctx.set_visuals(Visuals::dark());
        // Put your widgets into a `SidePanel`, `TopBottomPanel`, `CentralPanel`, `Window` or `Area`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            // The top panel is often a good place for a menu bar:

            egui::menu::bar(ui, |ui| {
                // NOTE: no File->Quit on web pages!
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
                        if ui.button("Config").clicked() {
                            error!("Config clicked! Implement me!");
                        }
                        if ui.button("Quit").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });
                    ui.add_space(16.0);
                }
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, _response) = ui.allocate_exact_size(
                egui::vec2(SIMULATION_AREA_WIDTH, SIMULATION_AREA_HEIGHT),
                egui::Sense::hover(),
            );

            if ui.is_rect_visible(rect) {
                // Draw some lines around the box to help with visualization
                if self.draw_debug_perimeters {
                    draw_perimeter(ui, &rect);
                }

                for boid in &self.boids {
                    let adjusted_pos = boid.position + rect.center().to_vec2();
                    draw_boid(ui, &rect, adjusted_pos, boid.color);
                }
            }
        });

        egui::SidePanel::right("config_panel").show(ctx, |ui| {
            ui.label("Configuration Panel");
            ui.label("Number of Boids");
            ui.add(egui::Slider::new(
                &mut self.num_boids,
                RangeInclusive::new(0, 1000),
            ));

            ui.checkbox(&mut self.draw_debug_perimeters, "Draw Debug Areas");
            ui.separator();

            ui.label("Max Velocity");
            ui.add(egui::DragValue::new(&mut self.max_speed));

            ui.label("Max Force");
            ui.add(egui::DragValue::new(&mut self.max_force));

            ui.label("Separation Weight");
            ui.add(egui::DragValue::new(&mut self.separation_weight));
            ui.label("Cohesion Weight");
            ui.add(egui::DragValue::new(&mut self.cohesion_weight));
            ui.label("Alignment Weight");
            ui.add(egui::DragValue::new(&mut self.alignment_weight));
        });
    }
}

fn draw_boid(ui: &mut Ui, rect: &Rect, center: Pos2, color: Color32) {
    let painter = ui.painter_at(*rect);
    let radius = 5.0;

    painter.circle_filled(center, radius, color);
}

fn draw_perimeter(ui: &mut Ui, rect: &Rect) {
    let painter = ui.painter_at(*rect);

    let top_left = rect.min;
    let bottom_right = rect.max;
    let top_right = Pos2::new(rect.max.x, rect.min.y);
    let bottom_left = Pos2::new(rect.min.x, rect.max.y);

    let stroke = egui::Stroke::new(2.0, Color32::YELLOW);

    painter.line_segment([top_left, top_right], stroke);
    painter.line_segment([top_left, bottom_left], stroke);
    painter.line_segment([top_right, bottom_right], stroke);
    painter.line_segment([bottom_left, bottom_right], stroke);
}

// TODO: Move me to another file
struct Boid {
    pub velocity: Vec2,
    pub position: Pos2,
    pub acceleration: Vec2,

    pub color: Color32,
}

impl Boid {
    pub fn new(position: Pos2, initial_velocity: Vec2) -> Self {
        Boid {
            velocity: initial_velocity,
            position,
            acceleration: Vec2::ZERO,
            color: Color32::WHITE,
        }
    }

    pub fn calculate_separation_force(
        &self,
        boids: &[Boid],
        separation_weight: f32,
        max_force: f32,
    ) -> Vec2 {
        let mut steering_force = Vec2::ZERO;
        let mut count = 0;

        for other in boids {
            let distance = (self.position - other.position).length();

            // If the other boid is within some small radius
            if distance > 0.0 && distance < 15.0 {
                // Try to move away from them
                let dir_to_move: Vec2 = (self.position - other.position).normalized();
                // distance;
                steering_force += dir_to_move;
                count += 1;
            }
        }

        if count > 0 {
            steering_force /= count as f32;
        }

        if steering_force.length() > max_force {
            steering_force.normalized() * max_force * separation_weight
        } else {
            steering_force * separation_weight
        }
    }

    pub fn calculate_cohesion_force(
        &self,
        boids: &[Boid],
        cohesion_weight: f32,
        max_force: f32,
        max_speed: f32,
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;

        for other in boids {
            let distance = (self.position - other.position).length();
            if distance > 0.0 && distance < NEIGHBOR_RADIUS {
                sum += other.position.to_vec2();
                count += 1;
            }
        }

        if count > 0 {
            sum /= count as f32;
            return self.seek(sum, max_speed, max_force) * cohesion_weight;
        }

        Vec2::ZERO
    }

    fn seek(&self, target_pos: Vec2, max_speed: f32, max_force: f32) -> Vec2 {
        let desired = (target_pos - self.position.to_vec2()).normalized() * max_speed;
        let mut steer = desired - self.velocity;
        if steer.length() > max_force {
            steer = steer.normalized() * max_force;
        }
        steer
    }

    pub fn calculate_alignment_force(
        &self,
        boids: &[Boid],
        alignment_weight: f32,
        max_speed: f32,
        max_force: f32,
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;

        // Trying to match the average of its neighbors velocity
        for other in boids {
            let distance = (self.position - other.position).length();
            if distance > 0.0 && distance < NEIGHBOR_RADIUS {
                sum += other.velocity;
                count += 1;
            }
        }

        if count > 0 {
            sum /= count as f32;
            sum = sum.normalized() * max_speed;

            let steer = sum - self.velocity;
            // If the force exceeds our max force, make sure to cap it
            if steer.length() > max_force {
                steer.normalized() * max_force * alignment_weight
            } else {
                steer * alignment_weight
            }
        } else {
            Vec2::ZERO
        }
    }
}
