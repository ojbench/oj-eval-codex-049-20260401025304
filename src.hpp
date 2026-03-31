#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include "monitor.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
        step_counter = 0;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Per-robot local counter; no globals/statics allowed.
    long long step_counter;

    // Check safety assuming others keep their last-step velocity (predictive).
    bool velocity_safe_against_all(const Vec &v_candidate) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec other_pos = monitor->get_pos_cur(j);
            Vec other_v_pred = monitor->get_v_cur(j);
            Vec delta_pos = pos_cur - other_pos;
            Vec delta_v = v_candidate - other_v_pred;

            double delta_v_norm = delta_v.norm();
            double min_dis_sqr;
            if (delta_v_norm <= 1e-12) {
                min_dis_sqr = delta_pos.norm_sqr();
            } else {
                double project = delta_pos.dot(delta_v);
                if (project >= 0) {
                    // Moving away
                    min_dis_sqr = delta_pos.norm_sqr();
                } else {
                    double along = (-project) / delta_v_norm;
                    if (along < delta_v_norm * TIME_INTERVAL) {
                        min_dis_sqr = delta_pos.norm_sqr() - along * along;
                    } else {
                        Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                        min_dis_sqr = end_delta.norm_sqr();
                    }
                }
            }

            double delta_r = r + monitor->get_r(j);
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return false;
            }
        }
        return true;
    }

public:

    Vec get_v_next() {
        // If already at target, stop.
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            ++step_counter;
            return Vec();
        }

        // If last step reported collisions and this robot was involved,
        // yield to the smallest id among the colliding group.
        if (monitor->get_warning()) {
            auto involved = monitor->get_collision(id);
            if (!involved.empty()) {
                int min_id = id;
                for (int k : involved) min_id = std::min(min_id, k);
                if (id != min_id) {
                    ++step_counter;
                    return Vec();
                }
            }
        }
        ++step_counter;

        // Desired speed towards the target, capped to avoid overshoot in this interval.
        Vec dir = to_tar.normalize();
        double s_max = v_max;
        s_max = std::min(s_max, dist / TIME_INTERVAL);
        if (s_max <= 1e-12) {
            return Vec();
        }

        // Find the maximum safe speed with a small binary search.
        double lo = 0.0, hi = s_max;
        for (int it = 0; it < 25; ++it) {
            double mid = 0.5 * (lo + hi);
            Vec v_try = dir * mid;
            if (velocity_safe_against_all(v_try)) lo = mid; else hi = mid;
        }
        return dir * lo;
    }
};


#endif //PPCA_SRC_HPP
