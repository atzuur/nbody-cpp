#include <cassert>
#include <cmath>
#include <format>
#include <optional>
#include <random>
#include <ranges>
#include <vector>

#include <raylib.h>
#include <raymath.h>

constexpr float GRAVITY = 6.6743e2f;
constexpr int BODY_RADIUS = 20;
constexpr int BODY_DIAM = 2 * BODY_RADIUS;

constexpr float DIST_EPS = 1e-4f;
constexpr float COLL_EPS = 1.0f;

constexpr int WIDTH = 1000;
constexpr int HEIGHT = 600;

struct Body {
    float mass;
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

[[maybe_unused]] static std::vector<Body> get_random_bodies(std::default_random_engine& eng,
                                                            int n_bodies) {
    std::uniform_int_distribution x_dist(0, WIDTH);
    std::uniform_int_distribution y_dist(0, HEIGHT);

    std::vector<Body> bodies(n_bodies);
    for (auto& body : bodies) {
        body.mass = 100.0f;
        body.pos = Vector2(x_dist(eng), y_dist(eng));
        body.color = get_random_color(eng);
    }
    return bodies;
}

int main() {
    namespace stdv = std::views;

    std::random_device r;
    std::default_random_engine e(r());
    // auto bodies = get_random_bodies(e, 30);

    const Vector2 center = {WIDTH / 2.0f, HEIGHT / 2.0f};
    std::vector<Body> bodies = {
        {1000.0f, {center.x, center.y}, {0.0f, 0.0f}, BLUE},
        {10.0f, {center.x, center.y + 100.0f}, {81.69f, 0.0f}, GREEN},
        {100.0f, {center.x + 200.0f, center.y + 200.0f}, {-60.0f, -60.0f}, RED},
    };
    std::vector<Vector2> acc(bodies.size());
    std::optional<Body> new_body {};

    InitWindow(WIDTH, HEIGHT, "nbody");
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 mouse_pos = GetMousePosition();
            if (new_body.has_value()) {
                new_body->vel = mouse_pos - new_body->pos;
            } else {
                new_body = {
                    .mass = 200.0f,
                    .pos = mouse_pos,
                    .vel = {},
                    .color = get_random_color(e),
                };
            }
        } else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            assert(new_body.has_value());
            bodies.push_back(new_body.value());
            acc.push_back({});
            new_body.reset();
        }

        const float dt = GetFrameTime();
        for (auto [i, body] : stdv::enumerate(bodies)) {
            for (const auto [j, other] : stdv::enumerate(bodies)) {
                if (i == j) {
                    continue;
                }
                const Vector2 xrel = Vector2AddValue(other.pos - body.pos, DIST_EPS);
                const float dist_sqr = Vector2LengthSqr(xrel);
                if (std::sqrt(dist_sqr) < BODY_DIAM - COLL_EPS) {
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
            DrawCircleV(body.pos, BODY_RADIUS, body.color);
            if (new_body.has_value()) {
                DrawCircleV(new_body->pos, BODY_RADIUS, new_body->color);
                DrawLineEx(new_body->pos, new_body->pos + new_body->vel, 2.0f, new_body->color);
            }
        }
        const auto& counter =
            std::format("{} bod{}", bodies.size(), bodies.size() == 1 ? "y" : "ies");
        DrawText(counter.c_str(), WIDTH - (counter.size() + 1) * 10, 5, 20, GRAY);
        EndDrawing();

        std::erase_if(bodies, [](Body b) {
            return b.pos.x > WIDTH + BODY_RADIUS || b.pos.x < -BODY_RADIUS ||
                   b.pos.y > HEIGHT + BODY_RADIUS || b.pos.y < -BODY_RADIUS;
        });
    }

    CloseWindow();
    return 0;
}
