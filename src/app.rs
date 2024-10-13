use std::time::Instant;

use egui::{Color32, Pos2, Rect, Stroke, Ui, Vec2, Visuals};
use rand::Rng;

use crate::{boid::Boid, boids_simulation::BoidsSimulationParameters};

const SIMULATION_AREA_WIDTH: f32 = 1700.0;
const SIMULATION_AREA_HEIGHT: f32 = 950.0;

const LEFT: f32 = -SIMULATION_AREA_WIDTH / 2.0;
const RIGHT: f32 = SIMULATION_AREA_WIDTH / 2.0;
const TOP: f32 = -SIMULATION_AREA_HEIGHT / 2.0;
const BOTTOM: f32 = SIMULATION_AREA_HEIGHT / 2.0;

const COHESION_COLOR: Color32 = Color32::BLUE;
const SEPARATION_COLOR: Color32 = Color32::YELLOW;
const ALIGNMENT_COLOR: Color32 = Color32::GREEN;
const AVOIDANCE_COLOR: Color32 = Color32::RED;

const FRAME_TIME: f32 = 1.0 / 60.0;

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct BoidsApp {
    #[serde(skip)]
    boids: Vec<Boid>,
    #[serde(skip)]
    last_update_time: std::time::Instant,
    #[serde(skip)]
    paused: bool,
    #[serde(skip)]
    predator_pos: Option<Pos2>,
    #[serde(default)]
    params: BoidsSimulationParameters,
}

impl Default for BoidsApp {
    fn default() -> Self {
        Self {
            boids: Vec::new(),
            predator_pos: None,
            last_update_time: Instant::now(),
            paused: false,
            params: BoidsSimulationParameters::default(),
        }
    }
}

impl BoidsApp {
    pub fn update_boids(&mut self) {
        // SIMULATION LOGIC
        if self.boids.len() > self.params.num_boids {
            // Remove some boids
            for _ in [0..self.boids.len() - self.params.num_boids] {
                self.boids.remove(self.boids.len() - 1);
            }
        } else if self.boids.len() < self.params.num_boids {
            // Insert some boids

            let mut rng: rand::prelude::ThreadRng = rand::thread_rng();

            // Pick a random point in our field
            let pos = Pos2::new(
                rng.gen_range(-SIMULATION_AREA_WIDTH / 2.0..SIMULATION_AREA_WIDTH / 2.0),
                rng.gen_range(-SIMULATION_AREA_HEIGHT / 2.0..SIMULATION_AREA_HEIGHT / 2.0),
            );
            // Set a random initial velocity
            let rand_x_vel = rng.gen_range(-self.params.max_speed..self.params.max_speed);
            let rand_y_vel = rng.gen_range(-self.params.max_speed..self.params.max_speed);
            let random_velocity = Vec2::new(rand_x_vel, rand_y_vel);

            self.boids.push(Boid::new(pos, random_velocity));
        }

        self.update_forces();
        self.update_boids_position();
    }

    fn update_boids_position(&mut self) {
        // Update positions from velocity/acceleration
        for boid in &mut self.boids {
            boid.apply_forces(self.params.max_speed);
            // screen wrap
            boid.screen_wrap(LEFT, RIGHT, TOP, BOTTOM);
        }
    }

    pub fn update_forces(&mut self) {
        let mut separation_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());
        let mut cohesion_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());
        let mut alignment_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());
        let mut avoidance_forces: Vec<Vec2> = Vec::with_capacity(self.boids.len());

        for boid in &self.boids {
            separation_forces.push(boid.calculate_separation_force(
                &self.boids,
                self.params.separation_weight,
                self.params.max_force,
                self.params.max_speed,
                self.params.neighbor_radius,
            ));

            alignment_forces.push(boid.calculate_alignment_force(
                &self.boids,
                self.params.alignment_weight,
                self.params.max_speed,
                self.params.max_force,
                self.params.neighbor_radius,
            ));

            cohesion_forces.push(boid.calculate_cohesion_force(
                &self.boids,
                self.params.cohesion_weight,
                self.params.max_force,
                self.params.max_speed,
                self.params.neighbor_radius,
            ));

            if let Some(predator_position) = self.predator_pos {
                avoidance_forces.push(boid.calculate_avoidance_force(
                    predator_position,
                    self.params.avoidance_weight,
                    self.params.max_force,
                    self.params.max_speed,
                    self.params.avoidance_radius,
                ));
            } else {
                avoidance_forces.push(Vec2::ZERO);
            }
        }

        for i in 0..self.boids.len() {
            self.boids[i].acceleration += separation_forces[i];
            self.boids[i].acceleration += alignment_forces[i];
            self.boids[i].acceleration += cohesion_forces[i];
            self.boids[i].acceleration += avoidance_forces[i];

            let separation_dominant = separation_forces[i].length_sq()
                > alignment_forces[i].length_sq()
                && separation_forces[i].length_sq() > cohesion_forces[i].length_sq()
                && separation_forces[i].length_sq() > avoidance_forces[i].length_sq();
            let alignment_dominant = alignment_forces[i].length_sq()
                > separation_forces[i].length_sq()
                && alignment_forces[i].length_sq() > cohesion_forces[i].length_sq()
                && alignment_forces[i].length_sq() > avoidance_forces[i].length_sq();
            let cohesion_dominant = cohesion_forces[i].length_sq()
                > alignment_forces[i].length_sq()
                && cohesion_forces[i].length_sq() > separation_forces[i].length_sq()
                && cohesion_forces[i].length_sq() > avoidance_forces[i].length_sq();
            let avoidance_dominant = avoidance_forces[i].length_sq()
                > alignment_forces[i].length_sq()
                && avoidance_forces[i].length_sq() > cohesion_forces[i].length_sq()
                && avoidance_forces[i].length_sq() > separation_forces[i].length_sq();

            if separation_dominant {
                self.boids[i].color = SEPARATION_COLOR;
            } else if alignment_dominant {
                self.boids[i].color = ALIGNMENT_COLOR;
            } else if cohesion_dominant {
                self.boids[i].color = COHESION_COLOR;
            } else if avoidance_dominant {
                self.boids[i].color = AVOIDANCE_COLOR;
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
        // App Update
        let dt = Instant::now()
            .saturating_duration_since(self.last_update_time)
            .as_secs_f32();
        if dt >= FRAME_TIME && !self.paused {
            self.last_update_time = Instant::now();
            self.update_boids();
            ctx.request_repaint();
        }

        // DRAW LOGIC

        // HACK! Idk why i ended up needing to do this in the update loop
        ctx.set_visuals(Visuals::dark());

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                // NOTE: no File->Quit on web pages!
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
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

            if let Some(mouse_pos) = ctx.input(|i| i.pointer.hover_pos()) {
                if rect.contains(mouse_pos) {
                    self.predator_pos = Some(mouse_pos - rect.center().to_vec2());
                    let painter: egui::Painter = ui.painter_at(rect);
                    painter.circle_filled(mouse_pos, 5.0, Color32::RED);
                    painter.circle_stroke(
                        mouse_pos,
                        self.params.avoidance_radius,
                        Stroke::new(5.0, Color32::RED),
                    );
                } else {
                    self.predator_pos = None;
                }
            } else {
                self.predator_pos = None;
            }

            if ui.is_rect_visible(rect) {
                // Draw some lines around the box to help with visualization
                draw_perimeter(ui, &rect);

                for boid in &self.boids {
                    boid.draw(ui, &rect);
                }
            }
        });

        egui::SidePanel::right("config_panel").show(ctx, |ui| {
            ui.label("Configuration Panel");
            ui.checkbox(&mut self.paused, "Pause Simulation");
            ui.separator();
            self.params.draw_panel(ui);
        });
    }
}

fn draw_perimeter(ui: &mut Ui, rect: &Rect) {
    let painter: egui::Painter = ui.painter_at(*rect);

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
