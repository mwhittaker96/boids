use std::ops::RangeInclusive;

use egui::Ui;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct BoidsSimulationParameters {
    #[serde(skip)]
    pub num_boids: usize,
    #[serde(default)]
    pub max_speed: f32,
    #[serde(default)]
    pub max_force: f32,
    // Weights
    #[serde(default)]
    pub separation_weight: f32,
    #[serde(default)]
    pub alignment_weight: f32,
    #[serde(default)]
    pub avoidance_weight: f32,
    #[serde(default)]
    pub cohesion_weight: f32,
    // Radii
    #[serde(default)]
    pub neighbor_radius: f32,
    #[serde(default)]
    pub avoidance_radius: f32,
}

impl Default for BoidsSimulationParameters {
    fn default() -> Self {
        Self {
            num_boids: 100,
            max_speed: 5.0,
            max_force: 0.5,
            separation_weight: 1.0,
            alignment_weight: 1.0,
            avoidance_weight: 1.0,
            cohesion_weight: 1.0,
            neighbor_radius: 50.0,
            avoidance_radius: 75.0,
        }
    }
}

impl BoidsSimulationParameters {
    pub fn draw_panel(&mut self, ui: &mut Ui) {
        ui.label("Number of Boids");
        ui.add(egui::Slider::new(
            &mut self.num_boids,
            RangeInclusive::new(0, 1000),
        ));

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
        ui.label("Avoidance Weight");
        ui.add(egui::DragValue::new(&mut self.avoidance_weight));

        ui.separator();

        ui.label("Neighbor Radius");
        ui.add(egui::DragValue::new(&mut self.neighbor_radius));
        ui.label("Avoidance Radius");
        ui.add(egui::DragValue::new(&mut self.avoidance_radius));

        if ui.button("Reset").clicked() {
            self.reset();
        }
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}
