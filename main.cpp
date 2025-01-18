#include <cassert>
#include <cmath>
#include <format>
#include <optional>
#include <random>
#include <ranges>
#include <vector>

#include <raylib.h>
#include <raymath.h>

namespace stdv = std::views;

constexpr float GRAVITY = 3e2f;
constexpr float DIST_EPS = 1e-4f;
constexpr float COLL_EPS = 1.0f;

constexpr int WIDTH = 1000;
constexpr int HEIGHT = 600;

struct Body {
    float mass;
    float radius;
    Vector2 pos;
    Vector2 vel;
    Color color;

    void update(Vector2 acc, float dt) {
        vel += acc * dt;
        pos += vel * dt;
    }
};

static Color get_random_color(std::default_random_engine& eng) {
    std::uniform_int_distribution color_dist(0, 0xFFFFFF);
    return GetColor((color_dist(eng) << 8) + 0xFF);
}

static std::vector<Body> get_random_system(std::default_random_engine& eng, int min_planets,
                                           int max_planets) {
    assert(1 <= min_planets && min_planets <= max_planets);
    std::binomial_distribution planet_num_dist(max_planets - min_planets);
    int n_planets = planet_num_dist(eng) + min_planets;

    const float max_rad = std::min(WIDTH, HEIGHT) / 2.0f;
    // outermost orbit approaches max_rad (edge) as n_planets increases
    const float mean_distance = max_rad / (n_planets + 1);
    std::normal_distribution distance_dist(mean_distance, mean_distance * 0.1f);
    std::uniform_real_distribution angle_dist(0.0f, 2.0f * PI);

    constexpr float SOLAR_MASS = 1e4f;
    const float sun_radius = /* starts at 0.2x mean distance and gradually approaches 0.5x */
        mean_distance * (std::pow(0.3f, n_planets) - std::pow(0.6f, n_planets) + 0.5f);
    const float density = SOLAR_MASS / (sun_radius * sun_radius);

    const Vector2 center {WIDTH / 2.0f, HEIGHT / 2.0f};
    std::vector<Body> bodies(n_planets + 1);
    bodies[0] = {SOLAR_MASS, sun_radius, center, {}, get_random_color(eng)};

    float distance = 0.0f;
    for (auto& body : bodies | stdv::drop(1)) {
        distance += distance_dist(eng);
        Vector2 pos_dir = Vector2Rotate(Vector2UnitX, angle_dist(eng));
        body.pos = center + pos_dir * distance;

        // outer planets are larger on average
        const float mean_rad = (distance / max_rad * 0.4f + 0.2f) * sun_radius;
        std::normal_distribution rad_dist(mean_rad, mean_rad * 0.2f);
        body.radius = rad_dist(eng);
        body.mass = body.radius * body.radius / density;

        Vector2 vel_dir = Vector2Rotate(pos_dir, PI / 2.0f);
        body.vel = vel_dir * std::sqrt(GRAVITY * SOLAR_MASS / distance);
        body.color = get_random_color(eng);
    }
    return bodies;
}

int main() {
    std::random_device r;
    std::default_random_engine e(r());
    auto bodies = get_random_system(e, 2, 15);
    float sys_density = bodies[0].mass / (bodies[0].radius * bodies[0].radius);

    std::vector<Vector2> acc(bodies.size());
    std::optional<Body> new_body {};

    InitWindow(WIDTH, HEIGHT, "nbody");
    SetTargetFPS(60);

    const Vector2 margin = {WIDTH / 100.0f, HEIGHT / 100.0f};
    const auto reset_text = "reset system";
    const Vector2 reset_size = MeasureTextEx(GetFontDefault(), reset_text, 20.0f, 2.0f);
    const Vector2 reset_inner_margin = reset_size / 10.0f;
    const Vector2 reset_btn_size = reset_size + reset_inner_margin * 2.0f;
    const Rectangle reset_btn {WIDTH - reset_btn_size.x - margin.x,
                               HEIGHT - reset_btn_size.y - margin.y, reset_btn_size.x,
                               reset_btn_size.y};
    enum { NONE, HOVER, DOWN } reset_btn_state = NONE;

    while (!WindowShouldClose()) {
        Vector2 mouse_pos = GetMousePosition();
        reset_btn_state = CheckCollisionPointRec(mouse_pos, reset_btn) ? HOVER : NONE;

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (new_body.has_value()) {
                new_body->vel = mouse_pos - new_body->pos;
            } else if (reset_btn_state == HOVER) {
                reset_btn_state = DOWN;
            } else {
                new_body = {
                    .mass = 10.0f,
                    .radius = 20.0f,
                    .pos = mouse_pos,
                    .vel = {},
                    .color = get_random_color(e),
                };
            }
        } else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (reset_btn_state == HOVER) {
                bodies = get_random_system(e, 2, 15);
                sys_density = bodies[0].mass / (bodies[0].radius * bodies[0].radius);
                acc.clear();
                acc.resize(bodies.size());
                reset_btn_state = NONE;
            } else {
                assert(new_body.has_value());
                bodies.push_back(new_body.value());
                acc.push_back({});
                new_body.reset();
            }
        }

        const float dt = GetFrameTime();
        for (auto [i, body] : stdv::enumerate(bodies)) {
            for (const auto [j, other] : stdv::enumerate(bodies)) {
                if (i == j) {
                    continue;
                }
                const Vector2 xrel = Vector2AddValue(other.pos - body.pos, DIST_EPS);
                const float dist_sqr = Vector2LengthSqr(xrel);
                if (std::sqrt(dist_sqr) < body.radius + other.radius - COLL_EPS) {
                    const Vector2 vrel = body.vel - other.vel;
                    const float v_proj_mag = -Vector2DotProduct(vrel, xrel) / dist_sqr;
                    const float idt = dt ? 1.0f / dt : 0.0f;
                    const float m_tot = body.mass + other.mass;
                    const Vector2 coll_acc = xrel * 2.0f * other.mass / m_tot * v_proj_mag * idt;
                    acc[i] += coll_acc;
                }

                const Vector2 grav_acc = xrel * GRAVITY * other.mass / std::pow(dist_sqr, 1.5f);
                acc[i] += grav_acc;
            }
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        for (auto [i, body] : stdv::enumerate(bodies)) {
            body.update(acc[i], dt);
            acc[i] = {};

            DrawCircleV(body.pos, body.radius, body.color);
            if (new_body.has_value()) {
                DrawCircleV(new_body->pos, new_body->radius, new_body->color);
                DrawLineEx(new_body->pos, new_body->pos + new_body->vel, 2.0f, new_body->color);
            }
        }
        const auto& counter =
            std::format("{} bod{}", bodies.size(), bodies.size() == 1 ? "y" : "ies");
        const int counter_size = MeasureText(counter.c_str(), 20);
        DrawText(counter.c_str(), WIDTH - counter_size - margin.x, margin.y, 20, GRAY);

        const Color reset_btn_colors[] = {
            {150, 150, 150, 100}, {150, 150, 150, 130}, {150, 150, 150, 180}};
        DrawRectangleRec(reset_btn, reset_btn_colors[reset_btn_state]);
        DrawText(reset_text, WIDTH - reset_size.x - reset_inner_margin.x - margin.x,
                 HEIGHT - reset_size.y - reset_inner_margin.y - margin.y, 20, GRAY);
        EndDrawing();

        std::erase_if(bodies, [](Body b) {
            return b.pos.x > WIDTH + b.radius || b.pos.x < -b.radius ||
                   b.pos.y > HEIGHT + b.radius || b.pos.y < -b.radius;
        });
    }

    CloseWindow();
    return 0;
}
