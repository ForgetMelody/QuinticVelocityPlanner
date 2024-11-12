#[derive(Debug, Clone)]
pub struct PolynomialQuintic {
    pub t0: f64,
    pub t1: f64,
    tmax: f64,
    q0: f64,
    q1: f64,
    v0: f64,
    v1: f64,
    a0: f64,
    a1: f64,
    max_acc: Option<f64>,
    max_vel: Option<f64>,
    pub poly: Vec<f64>,
}


impl PolynomialQuintic {
    /// Example usage:
    /// ```
    /// let tmax = 10.0;
    /// let q0 = 0.0;
    /// let q1 = 10.0;
    /// let v0 = 0.0;
    /// let v1 = 0.0;
    /// let a0 = 0.0;
    /// let a1 = 0.0;
    /// let max_acc = Some(1.0);
    /// let max_vel = Some(2.0);
    /// let poly = PolynomialQuintic::new(tmax, q0, q1, v0, v1, a0, a1, max_acc, max_vel);
    ///
    /// let t_values: Vec<f64> = (0..100).map(|i| poly.t0 + poly.t1 * i as f64 / 99.0).collect();
    /// let velocities: Vec<f64> = t_values.iter().map(|&t| poly.compute_vel(t)).collect();
    /// let positions: Vec<f64> = t_values.iter()
    /// .map(|&t|
    /// poly.poly[0] +
    /// poly.poly[1] * t +
    /// poly.poly[2] * t.powi(2) +
    /// poly.poly[3] * t.powi(3) +
    /// poly.poly[4] * t.powi(4) +
    /// poly.poly[5] * t.powi(5)  // 确保使用到五次方
    /// )
    /// .collect();
    /// let accelerations: Vec<f64> = t_values.iter().map(|&t| poly.compute_acc(t)).collect();
    /// println!("t_values: {:?}, velocities: {:?}, positions: {:?}, accelerations: {:?}", t_values, velocities, positions, accelerations);
    /// ```


    pub fn get_dault_pos_plan(q_start: f64, q_end: f64) -> (usize,Option<Vec<f64>>){
        const CALCULATE_FREQUENCY:f64 = 66.0;
        let tmax = 10.0;
        let q0 = q_start;
        let q1 = q_end;
        let v0 = 0.0;
        let v1 = 0.0;
        let a0 = 0.0;
        let a1 = 0.0;
        let max_acc = Some(5000.0);    //83.3/50 rev/s^2             54578
        //54578刻度/s^2
        let max_vel = Some(4000.0);   //max:33/50 rev/s(ja52)   50/50 rev/s(ja40)     对于91码盘值1度的，一圈为32760
        //21621刻度/s             32760刻度/s
        let poly = PolynomialQuintic::new(tmax, q0, q1, v0, v1, a0, a1, max_acc, max_vel);

        if tmax == poly.t1{
            eprintln!("Unable to find the plan in {} seconds", tmax);
            return (0,None);
        }
        let num_segments = (poly.t1 * CALCULATE_FREQUENCY).round() as usize;
        let t_values: Vec<f64> = (0..num_segments).map(|i| poly.t0 + poly.t1 * i as f64 / (num_segments-1)as f64).collect();
        let velocities: Vec<f64> = t_values.iter().map(|&t| poly.compute_vel(t)).collect();
        let positions: Vec<f64> = t_values.iter()
            .map(|&t|
            poly.poly[0] +
                poly.poly[1] * t +
                poly.poly[2] * t.powi(2) +
                poly.poly[3] * t.powi(3) +
                poly.poly[4] * t.powi(4) +
                poly.poly[5] * t.powi(5)
            )
            .collect();
        let accelerations: Vec<f64> = t_values.iter().map(|&t| poly.compute_acc(t)).collect();
        println!("t_values: {:?},\n velocities: {:?}, \npositions: {:?}, \naccelerations: {:?}", t_values, velocities, positions, accelerations);
        (num_segments,Some(positions))
    }



    pub fn new(tmax: f64, q0: f64, q1: f64, v0: f64, v1: f64, a0: f64, a1: f64, max_acc: Option<f64>, max_vel: Option<f64>) -> Self {
        let mut instance = PolynomialQuintic {
            t0: 0.0,
            tmax,
            q0,
            q1,
            v0,
            v1,
            a0,
            a1,
            max_acc,
            max_vel,
            poly: Vec::new(),
            t1: 0.0,
        };
        instance.t1 = instance.optimize_time();
        instance.poly = instance.compute_quintic_coeffs(instance.t0, instance.t1);
        instance
    }

    fn compute_quintic_coeffs(&self, t0: f64, t1: f64) -> Vec<f64> {
        let delta_t = t1 - t0;
        let delta_t2 = delta_t * delta_t;
        let h = self.q1 - self.q0;
        let k0 = self.q0;
        let k1 = self.v0;
        let k2 = 0.5 * self.a0;
        let k3 = (20.0 * h - (8.0 * self.v1 + 12.0 * self.v0) * delta_t - (3.0 * self.a0 - self.a1) * delta_t2) / (2.0 * delta_t * delta_t2);
        let k4 = (-30.0 * h + (14.0 * self.v1 + 16.0 * self.v0) * delta_t + (3.0 * self.a0 - 2.0 * self.a1) * delta_t2) / (2.0 * delta_t2 * delta_t2);
        let k5 = (12.0 * h - 6.0 * (self.v1 + self.v0) * delta_t + (self.a1 - self.a0) * delta_t2) / (2.0 * delta_t2 * delta_t2 * delta_t);
        vec![k0, k1, k2, k3, k4, k5]
    }

    pub fn compute_acc(&self, t: f64) -> f64 {
        20.0 * self.poly[5] * t.powi(3) + 12.0 * self.poly[4] * t.powi(2) + 6.0 * self.poly[3] * t + 2.0 * self.poly[2]
    }

    pub fn compute_vel(&self, t: f64) -> f64 {
        5.0 * self.poly[5] * t.powi(4) + 4.0 * self.poly[4] * t.powi(3) + 3.0 * self.poly[3] * t.powi(2) + 2.0 * self.poly[2] * t + self.poly[1]
    }

    fn check_constraints(&mut self, t: f64) -> bool {
        let coeffs = self.compute_quintic_coeffs(self.t0, self.t0 + t);
        self.poly = coeffs;

        let t_points: Vec<f64> = (0..100).map(|i| self.t0 + t * i as f64 / 99.0).collect();
        let accs: Vec<f64> = t_points.iter().map(|&t| self.compute_acc(t)).collect();
        let vels: Vec<f64> = t_points.iter().map(|&t| self.compute_vel(t)).collect();

        let max_acc_current = accs.iter().map(|&a| a.abs()).fold(0.0, f64::max);
        let max_vel_current = vels.iter().map(|&v| v.abs()).fold(0.0, f64::max);

        if let Some(max_acc) = self.max_acc {
            if max_acc_current > max_acc {
                return false;
            }
        }
        if let Some(max_vel) = self.max_vel {
            if max_vel_current > max_vel {
                return false;
            }
        }
        true
    }

    fn optimize_time(&mut self) -> f64 {
        let mut t_min = 0.1;
        let mut t_max = self.tmax;
        let tolerance = 1e-3;

        while t_max - t_min > tolerance {
            let t_mid = (t_min + t_max) / 2.0;
            if self.check_constraints(t_mid) {
                t_max = t_mid;
            } else {
                t_min = t_mid;
            }
        }
        t_max
    }
}
