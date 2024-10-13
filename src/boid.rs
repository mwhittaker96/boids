use egui::{Color32, Pos2, Rect, Ui, Vec2};

pub struct Boid {
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

    pub fn draw(&self, ui: &mut Ui, rect: &Rect) {
        let painter = ui.painter_at(*rect);
        let size = 10.0;

        // TODO: Fix me - arrow points in wrong direction/starts in wrong pos
        let stroke = egui::Stroke::new(2.0, self.color);
        let adjusted_pos = self.position + rect.center().to_vec2();
        painter.arrow(adjusted_pos, self.velocity.normalized() * size, stroke);
    }

    pub fn apply_forces(&mut self, max_speed: f32) {
        // Apply the acceleration to the velocity
        self.velocity = self.velocity + self.acceleration;
        // clamp the velocity - can do length squared if needed here
        if self.velocity.length() > max_speed {
            self.velocity = self.velocity.normalized() * max_speed;
        }
        // Zero out the acceleration
        self.acceleration = Vec2::ZERO;

        self.position += self.velocity;
    }

    pub fn wrap(&mut self, left: f32, right: f32, top: f32, bottom: f32) {
        if self.position.x > right {
            self.position.x = left;
        }
        if self.position.x < left {
            self.position.x = right;
        }
        if self.position.y > bottom {
            self.position.y = top;
        }
        if self.position.y < top {
            self.position.y = bottom;
        }
    }

    pub fn calculate_separation_force(
        &self,
        boids: &[Boid],
        separation_weight: f32,
        max_force: f32,
        neighbor_radius: f32,
    ) -> Vec2 {
        let mut steering_force = Vec2::ZERO;
        let mut count = 0;

        for other in boids {
            let distance = (self.position - other.position).length();

            // If the other boid is within some small radius
            if distance > 0.0 && distance < neighbor_radius {
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
        neighbor_radius: f32,
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;

        for other in boids {
            let distance = (self.position - other.position).length();
            if distance > 0.0 && distance < neighbor_radius {
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
        neighbor_radius: f32,
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;

        // Trying to match the average of its neighbors velocity
        for other in boids {
            let distance = (self.position - other.position).length();
            if distance > 0.0 && distance < neighbor_radius {
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

    pub fn calculate_avoidance_force(
        &self,
        predator_position: Pos2,
        avoidance_weight: f32,
        max_force: f32,
        avoidance_radius: f32,
    ) -> Vec2 {
        let distance = (self.position - predator_position).length();

        if distance < avoidance_radius {
            let steer = (self.position - predator_position).normalized();
            // TODO: THis is fishy
            if steer.length() > max_force {
                steer.normalized() * max_force * avoidance_weight
            } else {
                steer * avoidance_weight
            }
        } else {
            Vec2::ZERO
        }
    }
}

// Add vision cone
// Switch boids to triangles
// Add goals for groups
// Add predator prey reaction
