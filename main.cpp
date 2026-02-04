#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <array>
#include <queue>
#include <random>
#include <algorithm>
#include <vector>
#include <deque>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>
#include <set>
#include <bit>

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#define ASIO_STANDALONE
#include <asio.hpp>

constexpr bool ENABLE_ROTATION_ON_SPOT = false;
constexpr int DIRECTION_COUNT          = 4;
constexpr int PADDING                  = 30;
constexpr int COORD_RANGE              = 200; // NOTE: corresponds to 200cm
constexpr int RANDOMIZER_PADDING       = 20;  // NOTE: corresponds to 20cm
constexpr int SQUARES_PER_LINE         = 20;
constexpr float ROBOT_WIDTH            = 20.0f;
constexpr int FPS                      = 60;
constexpr int IMAGE_COUNT              = 7; // WARN: switch to 6/7/8 (?) for actual
constexpr int FONT_SIZE                = 20;
constexpr int DOUBLE_BORDER_WIDTH      = 6;
constexpr float STEP_DURATION          = 2.0f;
constexpr float EPS                    = 1e-6;
constexpr float INF                    = 1e9;
constexpr Color BG_COLOR               = { 230, 224, 216, 255 };

bool is_robot_visble(float x, float y) {
    const float l = ROBOT_WIDTH / 2, r = COORD_RANGE - ROBOT_WIDTH / 2;
    return x > l - EPS && y > l - EPS && x < r + EPS && y < r + EPS;
}

enum class Instruction {
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
    ForwardLeft,
    ForwardRight,
    BackwardLeft,
    BackwardRight,
    Scan,
};

enum class Direction {
    North,
    South,
    East,
    West,
};

float get_angle_from_direction(Direction direction) {
    switch (direction) {
        case Direction::East:
            return 0.0f;
        case Direction::North:
            return PI / 2;
        case Direction::West:
            return PI;
        case Direction::South:
            return 3 * PI / 2;
        default:
            return 0.0f;
    }
}

std::string get_prefix(Instruction instruction) {
    switch (instruction) {
        case Instruction::Forward:
            return "FW";
        case Instruction::Backward:
            return "BW";
        case Instruction::TurnLeft:
            return "TL";
        case Instruction::TurnRight:
            return "TR";
        case Instruction::ForwardLeft:
            return "FL";
        case Instruction::ForwardRight:
            return "FR";
        case Instruction::BackwardLeft:
            return "BL";
        case Instruction::BackwardRight:
            return "BR";
        case Instruction::Scan:
            return "SCAN";
        default:
            return "?";
    }
}

Direction get_opposite_direction(Direction dir) {
    switch (dir) {
        case Direction::North:
            return Direction::South;
        case Direction::South:
            return Direction::North;
        case Direction::East:
            return Direction::West;
        case Direction::West:
            return Direction::East;
        default:
            return Direction::North;
    }
}

void render_direction_indicator(float x, float y, float angle, float radius, Color color) {
    const float deg_l = DEG2RAD * 140.0f;
    const float deg_r = DEG2RAD * 220.0f;
    Vector2 pt_x      = { x + cosf(angle) * radius, y + sinf(angle) * radius };
    Vector2 pt_y      = { x + cosf(angle + deg_l) * radius, y + sinf(angle + deg_l) * radius };
    Vector2 pt_z      = { x + cosf(angle + deg_r) * radius, y + sinf(angle + deg_r) * radius };
    DrawTriangle(pt_z, pt_y, pt_x, color);
}

void render_direction_indicator(float x, float y, Direction direction, float radius, Color color) {
    const float angle = get_angle_from_direction(direction);
    render_direction_indicator(x, y, angle, radius, color);
}

struct DirectedPoint {
    float x, y;
    Direction direction;
};

std::random_device device;
std::mt19937 gen;

Font font;
int selected_idx = IMAGE_COUNT;

constexpr const char *SOLUTION_NOT_FOUND = "Failed to find a solution.";
constexpr const char *SOLUTION_FOUND     = "Solution found in ";

std::string solution_status;

class Grid {
    int squares_per_line;
    float grid_line_thickness;
    std::vector<DirectedPoint> grid_points;

    void draw_border(int offset) const {
        float top_left_offset = GRID_START - offset;
        float width           = GRID_WIDTH + 2 * offset;
        Rectangle rect        = { top_left_offset, top_left_offset, width, width };
        DrawRectangleLinesEx(rect, BORDER_THICKNESS, BORDER_COLOR);
    }

    void draw_grid_lines() const {
        PerspectiveModifier perspective_modifier;

        for (int i = 1; i <= squares_per_line - 1; i++) {
            const float offset = i * block_size;
            const float bound  = COORD_RANGE;

            Vector2 bottom_pt = { offset, 0 };
            Vector2 top_pt    = { offset, bound };

            Vector2 left_pt  = { 0, offset };
            Vector2 right_pt = { bound, offset };

            DrawLineEx(bottom_pt, top_pt, grid_line_thickness, GRID_LINE_COLOR);
            DrawLineEx(left_pt, right_pt, grid_line_thickness, GRID_LINE_COLOR);
        }
    }

  public:
    class PerspectiveModifier {
      public:
        PerspectiveModifier() {
            rlDrawRenderBatchActive();
            rlPushMatrix();
            rlTranslatef(GRID_START, GRID_START + GRID_WIDTH, 0);
            rlScalef(SCALE, -SCALE, 1.0f);
            rlSetCullFace(RL_CULL_FACE_FRONT);
        }
        ~PerspectiveModifier() {
            rlDrawRenderBatchActive();
            rlSetCullFace(RL_CULL_FACE_BACK);
            rlPopMatrix();
        }
        PerspectiveModifier(const PerspectiveModifier &)             = delete;
        PerspectiveModifier &operator=(const PerspectiveModifier &)  = delete;
        PerspectiveModifier(const PerspectiveModifier &&)            = delete;
        PerspectiveModifier &operator=(const PerspectiveModifier &&) = delete;
    };

    static constexpr float BORDER_THICKNESS    = 2.0f;
    static constexpr float GRID_LINE_THICKNESS = 1.0f;
    static constexpr int GRID_WIDTH            = 800;
    static constexpr int GRID_START            = 3 * PADDING;
    static constexpr float SCALE               = static_cast<float>(GRID_WIDTH) / COORD_RANGE;
    static constexpr Color GRID_COLOR          = { 245, 240, 238, 255 };
    static constexpr Color BORDER_COLOR        = { 56, 46, 46, 165 };
    static constexpr Color GRID_LINE_COLOR     = { 64, 54, 54, 65 };

    const int block_size;

    Grid(int squares_per_line = SQUARES_PER_LINE)
        : squares_per_line(squares_per_line)
        , grid_line_thickness(GRID_LINE_THICKNESS / SCALE)
        , block_size(COORD_RANGE / squares_per_line) {
        for (int i = 0; i < squares_per_line; i++) {
            for (int j = 0; j < squares_per_line; j++) {
                for (int k = 0; k < DIRECTION_COUNT; k++) {
                    float x = (static_cast<float>(i) + 0.5) * block_size;
                    float y = (static_cast<float>(j) + 0.5) * block_size;
                    grid_points.push_back({ x, y, static_cast<Direction>(k) });
                }
            }
        }
    }

    void render() const {
        DrawRectangle(GRID_START, GRID_START, GRID_WIDTH, GRID_WIDTH, GRID_COLOR);

        draw_border(0);
        draw_border(DOUBLE_BORDER_WIDTH);

        draw_grid_lines();
    }

    const std::vector<DirectedPoint> &get_grid_points() { return grid_points; }
};

constexpr int WINDOW_WIDTH  = Grid::GRID_WIDTH + 4 * PADDING;
constexpr int WINDOW_HEIGHT = Grid::GRID_WIDTH + 4 * PADDING;

Grid grid;

class Obstacle {
    using Face = std::pair<Vector2, Vector2>;

  public:
    enum class State {
        Hidden,
        Active,
        Visited,
    };

  private:
    bool found_stop_pos    = false;
    std::string label      = "";
    float x                = static_cast<float>(COORD_RANGE) / 2;
    float y                = static_cast<float>(COORD_RANGE) / 2;
    State state            = State::Active;
    State last_state       = State::Active;
    float border_thickness = BORDER_THICKNESS / Grid::SCALE;
    Direction direction    = Direction::West;
    DirectedPoint stop_pos = { 0.0f, 0.0f, Direction::North };

    void build_stop_pos() {
        found_stop_pos = false;

        const float offset = OBSTACLE_WIDTH / 2 + IMAGE_RECOGNITION_DISTANCE + ROBOT_WIDTH / 2;
        float min_dist     = INF;

        for (const auto &point : grid.get_grid_points()) {
            if (!is_robot_visble(point.x, point.y)) continue;

            DirectedPoint candidate = { point.x, point.y, get_opposite_direction(direction) };
            bool okay               = true;

            switch (direction) {
                case Direction::North:
                    okay = (point.y > y + offset - EPS);
                    break;
                case Direction::South:
                    okay = (point.y < y - offset + EPS);
                    break;
                case Direction::East:
                    okay = (point.x > x + offset - EPS);
                    break;
                case Direction::West:
                    okay = (point.x < x - offset + EPS);
                    break;
                default:
                    break;
            }

            if (!okay) continue;

            float dist = Vector2Distance({ x, y }, { point.x, point.y });

            if (dist < min_dist - EPS) {
                found_stop_pos = true;
                min_dist       = dist;
                stop_pos       = candidate;
            }
        }
    }

    Face get_directed_face() const {
        Face wall;
        const float offset = OBSTACLE_WIDTH / 2;
        switch (direction) {
            case Direction::North:
                wall.first  = { x - offset, y + offset };
                wall.second = { x + offset, y + offset };
                break;
            case Direction::South:
                wall.first  = { x - offset, y - offset };
                wall.second = { x + offset, y - offset };
                break;
            case Direction::East:
                wall.first  = { x + offset, y - offset };
                wall.second = { x + offset, y + offset };
                break;
            case Direction::West:
                wall.first  = { x - offset, y - offset };
                wall.second = { x - offset, y + offset };
                break;
        }
        return wall;
    }

  public:
    static constexpr float BORDER_THICKNESS              = 3.0f;
    static constexpr float OBSTACLE_WIDTH                = 10.0f;
    static constexpr float VIRTUAL_OBSTACLE_BUFFER       = 8.0f;
    static constexpr float IMAGE_RECOGNITION_DISTANCE    = 10.0f;
    static constexpr float INDICATOR_RADIUS              = 2.5f;
    static constexpr Color OBSTACLE_BORDER_COLOR         = { 162, 112, 21, 255 };
    static constexpr Color VIRTUAL_OBSTACLE_BORDER_COLOR = { 220, 222, 102, 255 };
    static constexpr Color FACE_COLOR                    = { 235, 116, 52, 255 };
    static constexpr Color VISITED_COLOR                 = { 86, 200, 36, 255 };
    static constexpr Color ACTIVE_BUTTON_COLOR           = Grid::BORDER_COLOR;
    static constexpr Color VISITED_BUTTON_COLOR          = VISITED_COLOR;
    static constexpr Color HIDDEN_BUTTON_COLOR           = { 230, 80, 80, 255 };
    static constexpr short OPACITY                       = 52;
    static constexpr int LABEL_SIZE                      = FONT_SIZE;

    Obstacle() = default;

    Vector2 get_position() const { return { x, y }; }

    float get_angle() const { return get_angle_from_direction(direction); }

    State get_state() const { return state; }

    State get_last_state() const { return last_state; }

    bool stop_pos_exists() const { return found_stop_pos; }

    DirectedPoint get_stop_pos() const { return stop_pos; }

    void set_position(float x_, float y_) {
        const float offset = OBSTACLE_WIDTH / 2;

        x = std::clamp(x_, offset, COORD_RANGE - offset);
        y = std::clamp(y_, offset, COORD_RANGE - offset);

        build_stop_pos();
    }

    void set_position(Vector2 pos) { set_position(pos.x, pos.y); }

    void set_position_x(float x_) { set_position(x_, y); }

    void set_position_y(float y_) { set_position(x, y_); }

    void set_state(State state_) { state = state_; }

    void set_last_state(State state_) { last_state = state_; }

    void set_direction(Direction direction_) {
        direction = direction_;
        build_stop_pos();
    }

    void set_label(std::string label_) { label = label_; }

    void reset_gui_style() const {
        Color normal_color;

        switch (state) {
            case State::Visited:
                normal_color = VISITED_BUTTON_COLOR;
                break;
            case State::Hidden:
                normal_color = HIDDEN_BUTTON_COLOR;
                break;
            case State::Active:
                normal_color = ACTIVE_BUTTON_COLOR;
                break;
            default:
                normal_color = ACTIVE_BUTTON_COLOR;
                break;
        }

        Color focused_color = normal_color;
        focused_color.a     = OPACITY;

        Color pressed_color = focused_color;

        GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(BLANK));
        GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(focused_color));
        GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(pressed_color));

        GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(normal_color));

        GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(normal_color));
    }

    void render_label() const {
        if (state == State::Hidden) return;

        Color label_color = state == State::Visited ? VISITED_COLOR : OBSTACLE_BORDER_COLOR;

        float window_x = Grid::GRID_START + Grid::SCALE * x;
        float window_y = Grid::GRID_START + Grid::GRID_WIDTH - Grid::SCALE * y;

        const float spacing = GuiGetStyle(DEFAULT, TEXT_SPACING);

        Vector2 label_size = MeasureTextEx(font, label.c_str(), LABEL_SIZE, spacing);
        Vector2 label_pos  = { window_x - label_size.x / 2.0f, window_y - label_size.y / 2.0f };

        DrawTextEx(font, label.c_str(), label_pos, LABEL_SIZE, spacing, label_color);
    }

    void render_shape() const {
        if (state == State::Hidden) return;

        Grid::PerspectiveModifier perspective_modifier;

        Rectangle obstacle_rect = { x - OBSTACLE_WIDTH / 2, y - OBSTACLE_WIDTH / 2, OBSTACLE_WIDTH, OBSTACLE_WIDTH };
        Color border_color      = state == State::Visited ? VISITED_COLOR : OBSTACLE_BORDER_COLOR;
        Color obstacle_color    = border_color;
        obstacle_color.a        = OPACITY;

        DrawRectangleRec(obstacle_rect, obstacle_color);
        DrawRectangleLinesEx(obstacle_rect, border_thickness, border_color);

        if (state == State::Visited) return;

        Rectangle virtual_obstacle_rect{ x - OBSTACLE_WIDTH / 2 - VIRTUAL_OBSTACLE_BUFFER,
                                         y - OBSTACLE_WIDTH / 2 - VIRTUAL_OBSTACLE_BUFFER,
                                         OBSTACLE_WIDTH + 2 * VIRTUAL_OBSTACLE_BUFFER,
                                         OBSTACLE_WIDTH + 2 * VIRTUAL_OBSTACLE_BUFFER };
        Color virtual_obstacle_color = VIRTUAL_OBSTACLE_BORDER_COLOR;
        virtual_obstacle_color.a     = OPACITY;

        DrawRectangleRec(virtual_obstacle_rect, virtual_obstacle_color);

        Face face = get_directed_face();

        DrawLineEx(face.first, face.second, 2 * border_thickness, FACE_COLOR);

        if (found_stop_pos) {
            render_direction_indicator(stop_pos.x, stop_pos.y, stop_pos.direction, INDICATOR_RADIUS, FACE_COLOR);
        }
    }

    friend bool collide(const Obstacle &obstacle_x, const Obstacle &obstacle_y) {
        const float offset = OBSTACLE_WIDTH / 2;
        float min_x        = std::min(obstacle_x.x + offset, obstacle_y.x + offset);
        float max_x        = std::max(obstacle_x.x - offset, obstacle_y.x - offset);
        float min_y        = std::min(obstacle_x.y + offset, obstacle_y.y + offset);
        float max_y        = std::max(obstacle_x.y - offset, obstacle_y.y - offset);
        return min_x - max_x > EPS || min_y - max_y > EPS;
    }

    bool collide(Vector2 start_pos, Vector2 end_pos) const {
        const float offset = (ROBOT_WIDTH / 2.0f) + VIRTUAL_OBSTACLE_BUFFER + (OBSTACLE_WIDTH / 2.0f);

        float min_x = x - offset;
        float max_x = x + offset;
        float min_y = y - offset;
        float max_y = y + offset;

        float t_min = 0.0f;
        float t_max = 1.0f;

        float dx = end_pos.x - start_pos.x;
        float dy = end_pos.y - start_pos.y;

        if (fabsf(dx) > EPS) {
            float t1 = (min_x - start_pos.x) / dx;
            float t2 = (max_x - start_pos.x) / dx;
            t_min    = fmaxf(t_min, fminf(t1, t2));
            t_max    = fminf(t_max, fmaxf(t1, t2));
        } else if (start_pos.x < min_x || start_pos.x > max_x) {
            return false;
        }

        if (fabsf(dy) > EPS) {
            float t1 = (min_y - start_pos.y) / dy;
            float t2 = (max_y - start_pos.y) / dy;
            t_min    = fmaxf(t_min, fminf(t1, t2));
            t_max    = fminf(t_max, fmaxf(t1, t2));
        } else if (start_pos.y < min_y || start_pos.y > max_y) {
            return false;
        }

        return t_max > t_min || FloatEquals(t_max, t_min);
    }

    void randomize_position() {
        const float offset = OBSTACLE_WIDTH / 2 + VIRTUAL_OBSTACLE_BUFFER + RANDOMIZER_PADDING;
        std::uniform_real_distribution distribution(offset, COORD_RANGE - offset);
        set_position(distribution(gen), distribution(gen));
    }

    void randomize_direction() {
        std::uniform_int_distribution distribution(0, DIRECTION_COUNT - 1);
        set_direction(static_cast<Direction>(distribution(gen)));
    }
};

class Robot {
    float x                = ROBOT_WIDTH - static_cast<float>(grid.block_size) / 2;
    float y                = ROBOT_WIDTH - static_cast<float>(grid.block_size) / 2;
    int scan_offset        = 0;
    float angle            = get_angle_from_direction(Direction::North);
    float border_thickness = BORDER_THICKNESS / Grid::SCALE;

  public:
    static constexpr float BORDER_THICKNESS    = 3.0f;
    static constexpr float INDICATOR_RADIUS    = 3.5f;
    static constexpr float SCAN_OFFSET_DIVISOR = 16.0f;
    static constexpr Color BORDER_COLOR        = { 52, 150, 240, 255 };
    static constexpr Color SCAN_COLOR          = { 52, 150, 240, 68 };
    static constexpr short OPACITY             = 52;

    Vector2 get_position() const { return { x, y }; }
    float get_angle() const { return angle; }

    void set_position(float x_, float y_) {
        const float offset = ROBOT_WIDTH / 2;

        x = std::clamp(x_, offset, COORD_RANGE - offset);
        y = std::clamp(y_, offset, COORD_RANGE - offset);
    }

    void set_position(Vector2 pos) { set_position(pos.x, pos.y); }

    void set_position_x(float x_) { set_position(x_, y); }

    void set_position_y(float y_) { set_position(x, y_); }

    void set_angle(float angle_) { angle = angle_; }

    void set_direction(Direction direction_) { set_angle(get_angle_from_direction(direction_)); }

    void set_scan_offset(int offset_) { scan_offset = offset_; }

    void reset_gui_style() const {
        Color normal_color = Obstacle::FACE_COLOR;

        Color focused_color = normal_color;
        focused_color.a     = OPACITY;

        Color pressed_color = focused_color;

        GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(BLANK));
        GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(focused_color));
        GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(pressed_color));

        GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(normal_color));

        GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(normal_color));
    }

    void render() const {
        Grid::PerspectiveModifier perspective_modifier;

        Rectangle robot_rect = { x, y, ROBOT_WIDTH, ROBOT_WIDTH };
        Vector2 origin       = { ROBOT_WIDTH / 2.0f, ROBOT_WIDTH / 2.0f };

        float rotation_degree = angle * RAD2DEG;
        const float sqrt_2    = sqrtf(2.0f);

        Color obstacle_color = BORDER_COLOR;
        obstacle_color.a     = OPACITY;

        DrawRectanglePro(robot_rect, origin, rotation_degree, obstacle_color);
        DrawPolyLinesEx({ x, y }, 4, ROBOT_WIDTH / sqrt_2, rotation_degree + 45, border_thickness, BORDER_COLOR);

        render_direction_indicator(x, y, angle, INDICATOR_RADIUS, BORDER_COLOR);

        if (scan_offset) {
            const float scan_radius = INDICATOR_RADIUS + static_cast<float>(scan_offset) / SCAN_OFFSET_DIVISOR;
            render_direction_indicator(x, y, angle, scan_radius, SCAN_COLOR);
        }
    }
};

std::array<Obstacle, IMAGE_COUNT> obstacles;
Robot robot;

class Step {
  protected:
    int frame      = 1;
    int frame_cnt  = 1;
    bool is_synced = false;

    void setup_animation(const Robot &animated_robot) {
        if (!is_synced) {
            sync(animated_robot);
            is_synced = true;
        }
        if (is_complete()) {
            return;
        }
    }

    void render_simulated_point(Vector2 point) {
        float inner_radius = 2 * SIMULATED_PATH_THICKNESS / Grid::SCALE;
        float outer_radius = 3 * SIMULATED_PATH_THICKNESS / Grid::SCALE;

        DrawCircleV(point, inner_radius, SIMULATED_PATH_COLOR);
        DrawRing(point, inner_radius, outer_radius, 0.0f, 360.0f, 36, SIMULATED_BORDER_COLOR);
    }

  public:
    static constexpr float SIMULATED_PATH_THICKNESS = 2.0f;
    static constexpr Color SIMULATED_PATH_COLOR     = { 36, 190, 165, 106 };
    static constexpr Color SIMULATED_BORDER_COLOR   = { 36, 190, 165, 166 };

    virtual std::string serialize()                           = 0;
    virtual void animate(Robot &)                             = 0;
    virtual void instant_animate(Robot &, bool is_last_point) = 0;
    virtual void sync(const Robot &)                          = 0;
    virtual bool is_complete()                                = 0;
    virtual void cleanup()                                    = 0;
    virtual ~Step() {}
};

class LinearStep : public Step {
    Instruction instruction = Instruction::Forward;
    float dist              = 0.0f;
    Vector2 start_pos       = { 0, 0 };
    Vector2 end_pos         = { 0, 0 };

  public:
    LinearStep(Instruction instruction, float dist)
        : instruction(instruction)
        , dist(dist) {}

    std::string serialize() override {
        std::string prefix = get_prefix(instruction);
        std::string suffix = std::to_string(static_cast<int>(roundf(dist)));
        return prefix + suffix;
    }

    void sync(const Robot &animated_robot) override {
        start_pos = animated_robot.get_position();
        end_pos   = start_pos;

        float angle = animated_robot.get_angle();

        float dx = dist * cosf(angle);
        float dy = dist * sinf(angle);

        if (instruction == Instruction::Backward) dx *= -1, dy *= -1;

        end_pos.x += dx;
        end_pos.y += dy;

        frame_cnt = std::max(1, static_cast<int>(roundf(dist * STEP_DURATION)));
    }

    void animate(Robot &animated_robot) override {
        setup_animation(animated_robot);
        float t         = static_cast<float>(frame) / frame_cnt;
        Vector2 cur_pos = Vector2Lerp(start_pos, end_pos, t);
        animated_robot.set_position(cur_pos);
        frame++;
    }

    void instant_animate(Robot &animated_robot, bool is_last_point) override {
        sync(animated_robot);

        float path_thickness = SIMULATED_PATH_THICKNESS / Grid::SCALE;

        animated_robot.set_position(end_pos);
        end_pos = animated_robot.get_position();
        float x = end_pos.x, y = end_pos.y, angle = animated_robot.get_angle();

        render_simulated_point(start_pos);
        DrawLineEx(start_pos, end_pos, path_thickness, SIMULATED_PATH_COLOR);
        if (is_last_point) {
            render_direction_indicator(x, y, angle, Robot::INDICATOR_RADIUS, SIMULATED_BORDER_COLOR);
        }
    }

    bool is_complete() override { return frame > frame_cnt; }

    void cleanup() override {}
};

class RotateStep : public Step {
    Instruction instruction  = Instruction::Forward;
    float angle_delta_degree = 0.0f;
    float start_angle        = robot.get_angle();

  public:
    static constexpr float ROTATION_SPEED = 6.0;

    RotateStep(Instruction instruction, float angle_delta_degree)
        : instruction(instruction)
        , angle_delta_degree(angle_delta_degree) {}

    std::string serialize() override {
        std::string prefix = get_prefix(instruction);
        std::string suffix = std::to_string(static_cast<int>(roundf(angle_delta_degree)));
        return prefix + suffix;
    }

    void sync(const Robot &animated_robot) override {
        start_angle = animated_robot.get_angle();
        frame_cnt   = std::max(1, static_cast<int>(roundf(angle_delta_degree * STEP_DURATION / ROTATION_SPEED)));
    }

    void animate(Robot &animated_robot) override {
        setup_animation(animated_robot);
        float t           = static_cast<float>(frame) / frame_cnt;
        float angle_delta = (instruction == Instruction::TurnLeft ? 1 : -1) * DEG2RAD * angle_delta_degree;
        float angle       = start_angle + t * angle_delta;
        animated_robot.set_angle(angle);
        frame++;
    }

    void instant_animate(Robot &animated_robot, bool is_last_point) override {
        sync(animated_robot);
        float angle_delta = (instruction == Instruction::TurnLeft ? 1 : -1) * DEG2RAD * angle_delta_degree;
        animated_robot.set_angle(start_angle + angle_delta);

        Vector2 pos = animated_robot.get_position();
        float x = pos.x, y = pos.y, angle = animated_robot.get_angle();

        if (is_last_point) {
            render_direction_indicator(x, y, angle, Robot::INDICATOR_RADIUS, SIMULATED_BORDER_COLOR);
        }
    }

    bool is_complete() override { return frame > frame_cnt; }

    void cleanup() override {}
};

class CircularTurnStep : public Step {
    Instruction instruction = Instruction::ForwardLeft;
    float start_angle       = 0.0f;
    Vector2 circle_center   = { 0, 0 };
    Vector2 start_pos       = { 0, 0 };
    float circle_angle      = 0.0f;

  public:
    static constexpr float ROTATION_SPEED     = 6.0;
    static constexpr float TURN_ANGLE_DEGREES = 90;
    static constexpr int TURN_BLOCK_COUNT     = 1;
    static constexpr int ARC_SEGMENTS         = 20;

    const float turn_radius = TURN_BLOCK_COUNT * grid.block_size;

    CircularTurnStep(Instruction instruction)
        : instruction(instruction) {}

    std::string serialize() override {
        std::string prefix = get_prefix(instruction);
        return prefix;
    }

    void sync(const Robot &animated_robot) override {
        Vector2 pos = animated_robot.get_position();
        start_angle = animated_robot.get_angle();
        start_pos   = pos;

        bool is_left = instruction == Instruction::ForwardLeft || instruction == Instruction::BackwardLeft;

        float center_offset = is_left ? (PI / 2.0f) : (-PI / 2.0f);

        circle_center.x = start_pos.x + turn_radius * cosf(start_angle + center_offset);
        circle_center.y = start_pos.y + turn_radius * sinf(start_angle + center_offset);

        circle_angle = atan2f(start_pos.y - circle_center.y, start_pos.x - circle_center.x);

        frame_cnt = std::max(1, static_cast<int>(roundf(TURN_ANGLE_DEGREES * STEP_DURATION / ROTATION_SPEED)));
    }

    void animate(Robot &animated_robot) override {
        setup_animation(animated_robot);
        float t = static_cast<float>(frame) / frame_cnt;

        bool is_left    = instruction == Instruction::ForwardLeft || instruction == Instruction::BackwardLeft;
        bool is_forward = instruction == Instruction::ForwardLeft || instruction == Instruction::ForwardRight;

        float swing_dir     = (is_left == is_forward) ? 1.0f : -1.0f;
        float center_offset = is_left ? (PI / 2.0f) : (-PI / 2.0f);

        float current_circle_angle = circle_angle + (swing_dir * t * (PI / 2.0f));

        Vector2 new_pos = { circle_center.x + turn_radius * cosf(current_circle_angle),
                            circle_center.y + turn_radius * sinf(current_circle_angle) };

        animated_robot.set_position(new_pos);
        animated_robot.set_angle(current_circle_angle + center_offset);

        frame++;
    }

    void instant_animate(Robot &animated_robot, bool is_last_point) override {
        sync(animated_robot);

        float path_thickness = SIMULATED_PATH_THICKNESS / Grid::SCALE;

        bool is_left    = instruction == Instruction::ForwardLeft || instruction == Instruction::BackwardLeft;
        bool is_forward = instruction == Instruction::ForwardLeft || instruction == Instruction::ForwardRight;

        float swing_dir     = (is_left == is_forward) ? 1.0f : -1.0f;
        float center_offset = is_left ? (PI / 2.0f) : (-PI / 2.0f);

        float final_circle_angle = circle_angle + (swing_dir * (PI / 2.0f));

        Vector2 end_pos = { circle_center.x + turn_radius * cosf(final_circle_angle),
                            circle_center.y + turn_radius * sinf(final_circle_angle) };

        animated_robot.set_position(end_pos);
        animated_robot.set_angle(final_circle_angle + center_offset);

        render_simulated_point(start_pos);

        Vector2 prev_p = start_pos;

        for (int i = 1; i <= ARC_SEGMENTS; i++) {
            float t                  = static_cast<float>(i) / ARC_SEGMENTS;
            float current_step_angle = circle_angle + (swing_dir * t * (PI / 2.0f));

            Vector2 current_p = { circle_center.x + turn_radius * cosf(current_step_angle),
                                  circle_center.y + turn_radius * sinf(current_step_angle) };

            DrawLineEx(prev_p, current_p, path_thickness, SIMULATED_PATH_COLOR);
            prev_p = current_p;
        }

        if (is_last_point) {
            render_direction_indicator(
                end_pos.x, end_pos.y, animated_robot.get_angle(), Robot::INDICATOR_RADIUS, SIMULATED_BORDER_COLOR);
        }
    }

    bool is_complete() override { return frame > frame_cnt; }

    void cleanup() override {}
};

class ScanStep : public Step {
    Instruction instruction = Instruction::Scan;
    int obstacle_idx        = 0;

  public:
    ScanStep(int obstacle_idx)
        : obstacle_idx(obstacle_idx) {}

    static constexpr int SCAN_DURATION = 90;

    std::string serialize() override {
        std::string prefix = get_prefix(instruction);
        return prefix;
    }

    void sync(const Robot &_) override { frame_cnt = std::max(1, SCAN_DURATION); }

    void animate(Robot &animated_robot) override {
        setup_animation(animated_robot);
        animated_robot.set_scan_offset(SCAN_DURATION - frame);
        frame++;
    }

    void instant_animate(Robot &animated_robot, bool is_last_point) override {
        sync(animated_robot);

        Vector2 pos = animated_robot.get_position();
        float x = pos.x, y = pos.y, angle = animated_robot.get_angle();

        if (is_last_point) {
            render_direction_indicator(x, y, angle, Robot::INDICATOR_RADIUS, SIMULATED_BORDER_COLOR);
        }
    }

    bool is_complete() override { return frame > frame_cnt; }

    void cleanup() override { obstacles[obstacle_idx].set_state(Obstacle::State::Visited); }
};

std::deque<std::unique_ptr<Step>> steps;

class Transmitter {
  public:
    enum class Status { Idle, InProgress, Success, Failed };

    static inline std::atomic<Status> transmission_status{ Status::Idle };
    static inline std::optional<std::jthread> router_thread;

    static constexpr const char *IP_ADDRESS = "127.0.0.1";
    static constexpr const char *PORT       = "6767";
    static constexpr short OPACITY          = 52;

    static const char *get_label_from_status(Status status) {
        switch (status) {
            case Status::Idle:
                return "Send Command(s)";
            case Status::InProgress:
                return "Sending...";
            case Status::Success:
                return "Success!";
            case Status::Failed:
                return "Failed...";
            default:
                return "";
        }
    }

    static std::string get_command_chain() {
        std::string commands, prev_command;
        size_t step_cnt = steps.size();

        for (size_t i = 0; i < step_cnt; i++) {
            std::string command = steps[i]->serialize();
            if (command == get_prefix(Instruction::Scan)) {
                command = "\n";
                if (i > 0 && prev_command != command) {
                    commands.pop_back();
                }
            }
            commands += command;
            if (i != step_cnt - 1 && command != "\n") commands += ' ';
            prev_command = command;
        }

        return commands;
    }

    static bool route_commands(std::string commands) {
        transmission_status = Status::InProgress;

        try {
            asio::io_context io_context;

            asio::ip::tcp::resolver resolver(io_context);
            auto endpoint = resolver.resolve(IP_ADDRESS, PORT);

            asio::ip::tcp::socket socket(io_context);
            asio::connect(socket, endpoint);

            asio::write(socket, asio::buffer(commands));

            transmission_status = Status::Success;
            return true;
        } catch (std::exception &e) {
            transmission_status = Status::Failed;
            return false;
        }
    }

    static void reset_gui_style() {
        Color normal_color;

        switch (transmission_status) {
            case Status::Idle:
                normal_color = Grid::BORDER_COLOR;
                break;
            case Status::InProgress:
                normal_color = Obstacle::FACE_COLOR;
                break;
            case Status::Failed:
                normal_color = Obstacle::HIDDEN_BUTTON_COLOR;
                break;
            case Status::Success:
                normal_color = Obstacle::VISITED_BUTTON_COLOR;
                break;
            default:
                normal_color = Grid::BORDER_COLOR;
                break;
        }

        Color focused_color = normal_color;
        focused_color.a     = OPACITY;

        Color pressed_color = focused_color;

        GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(BLANK));
        GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(focused_color));
        GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(pressed_color));

        GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(normal_color));

        GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(normal_color));
    }
};

class Animator {
    Robot simulation_base = robot;
    bool run_status       = false;
    void lock_animation() { run_status = false; }
    void unlock_animation() { run_status = true; }

    void render_commands() const {
        Color label_color = run_status ? Obstacle::FACE_COLOR : Grid::BORDER_COLOR;

        float window_x = 3.0 * PADDING + static_cast<float>(Grid::GRID_WIDTH) / 2.0f;
        float window_y = 1.5 * PADDING;

        const float spacing = GuiGetStyle(DEFAULT, TEXT_SPACING);

        std::string label = "No Pending Commands";

        if (steps.size()) {
            label.clear();
            size_t step_cnt = steps.size();
            for (size_t i = 0; i < step_cnt; i++) {
                if (i == COMMAND_LIMIT - 1) {
                    label += "...";
                    break;
                }
                label += steps[i]->serialize();
                if (i != step_cnt - 1) label += ' ';
            }
        }

        Vector2 label_size = MeasureTextEx(font, label.c_str(), LABEL_SIZE, spacing);
        Vector2 label_pos  = { window_x - label_size.x / 2.0f, window_y - label_size.y / 2.0f };

        DrawTextEx(font, label.c_str(), label_pos, LABEL_SIZE, spacing, label_color);
    }

    void render_simulated_path() {
        Grid::PerspectiveModifier perspective_modifier;

        Robot ghost(simulation_base);
        size_t step_cnt = steps.size();

        for (size_t i = 0; i < step_cnt; i++) {
            steps[i]->instant_animate(ghost, i == step_cnt - 1);
        }
    }

  public:
    static constexpr int LABEL_SIZE      = 20;
    static constexpr short OPACITY       = 52;
    static constexpr short COMMAND_LIMIT = 8;

    void reset_gui_style() const {
        Color normal_color = run_status || empty(steps) ? Obstacle::FACE_COLOR : Obstacle::VISITED_COLOR;

        Color focused_color = normal_color;
        focused_color.a     = OPACITY;

        Color pressed_color = focused_color;

        GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(BLANK));
        GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(focused_color));
        GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(pressed_color));

        GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(normal_color));

        GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(normal_color));
        GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(normal_color));
    }

    bool is_running() const { return run_status; }

    void toggle_animation_status() {
        if (run_status) return;
        unlock_animation();
    }

    void animate() {
        if (empty(steps)) lock_animation();

        if (!run_status) {
            simulation_base = robot;
            return;
        }

        auto &step = steps.front();

        if (step->is_complete()) {
            step->cleanup();
            simulation_base = robot;
            steps.pop_front();
            return;
        }

        step->animate(robot);
    }

    void render() {
        render_commands();
        render_simulated_path();
    }
};

Animator animator;

void randomize_obstacles_positions() {
    for (auto &obstacle : obstacles) {
        obstacle.set_state(Obstacle::State::Hidden);
    }

    for (int i = 0; i < IMAGE_COUNT; i++) {
        obstacles[i].randomize_direction();
        obstacles[i].set_state(Obstacle::State::Active);

        if (i == 0) {
            obstacles[i].randomize_position();
            continue;
        }

        size_t attempts       = 20;
        float best_separation = 0.0f;
        Vector2 best_pos      = { 0, 0 };

        while (attempts--) {
            obstacles[i].randomize_position();

            float min_distance = INF;

            for (int j = 0; j < i; j++) {
                while (collide(obstacles[i], obstacles[j])) {
                    obstacles[i].randomize_position();
                }
            }

            for (int j = 0; j < i; j++) {
                float distance = Vector2Distance(obstacles[i].get_position(), obstacles[j].get_position());
                min_distance   = std::min(min_distance, distance);
            }

            if (min_distance > best_separation + EPS) {
                best_separation = min_distance;
                best_pos        = obstacles[i].get_position();
            }
        }

        obstacles[i].set_position(best_pos);
    }
}

void render_selected_indicator() {
    if (selected_idx < IMAGE_COUNT) {
        obstacles[selected_idx].reset_gui_style();
    } else {
        robot.reset_gui_style();
    }

    float x = PADDING - DOUBLE_BORDER_WIDTH;
    float y = (3 + 2 * selected_idx) * PADDING - DOUBLE_BORDER_WIDTH;

    float width        = PADDING + 2 * DOUBLE_BORDER_WIDTH;
    Rectangle rect     = { x, y, width, width };
    Color border_color = GetColor(GuiGetStyle(BUTTON, BORDER_COLOR_NORMAL));

    DrawRectangleLinesEx(rect, Grid::BORDER_THICKNESS, border_color);
}

void render_obstacle_buttons() {
    for (int i = 0; i < IMAGE_COUNT; i++) {
        float x = PADDING, y = (3 + 2 * i) * PADDING;
        std::string label = std::to_string(i + 1);

        obstacles[i].reset_gui_style();

        if (GuiButton({ x, y, PADDING, PADDING }, label.c_str())) {
            Obstacle::State new_state = obstacles[i].get_state();

            if (new_state == Obstacle::State::Active) new_state = Obstacle::State::Visited;
            else if (new_state == Obstacle::State::Visited) new_state = Obstacle::State::Active;

            obstacles[i].set_state(static_cast<Obstacle::State>(new_state));
            selected_idx = i;
        }
    }
}

void render_robot_button() {
    float x = PADDING, y = (3 + 2 * IMAGE_COUNT) * PADDING;
    const char *label = "P";

    robot.reset_gui_style();

    if (GuiButton({ x, y, PADDING, PADDING }, label)) {
        selected_idx = IMAGE_COUNT;
    }
}

void render_solution_status() {
    float text_x = 1.8f * PADDING;
    float text_y = WINDOW_HEIGHT - 3 * PADDING;

    Vector2 position = { text_x, text_y };

    const char *status  = solution_status.c_str();
    const float spacing = GuiGetStyle(DEFAULT, TEXT_SPACING);

    Vector2 status_size = MeasureTextEx(GuiGetFont(), status, FONT_SIZE, spacing);
    Vector2 origin      = { 0, status_size.y };

    Color color = (strcmp(status, SOLUTION_NOT_FOUND) == 0 ? Obstacle::HIDDEN_BUTTON_COLOR : Grid::BORDER_COLOR);

    DrawTextPro(font, status, position, origin, -90.0f, FONT_SIZE, spacing, color);
}

void render_play_button() {
    float x = PADDING, y = WINDOW_HEIGHT - 2 * PADDING;
    const char *label = "#119#";

    animator.reset_gui_style();

    if (GuiButton({ x, y, PADDING, PADDING }, label)) {
        animator.toggle_animation_status();
    }
}

void render_position_controller() {
    float x = 3 * PADDING;
    float y = PADDING;

    const int offset          = roundf(Obstacle::OBSTACLE_WIDTH) / 2.0f;
    const float label_padding = 16.0f;

    const float box_width = 50;

    // WARN: confine user inputs to `int` for now (easier parsing)
    static int input_x = 0;
    static int input_y = 0;

    static bool in_edit_x = false;
    static bool in_edit_y = false;

    if (IsKeyPressed(KEY_J) || IsKeyPressed(KEY_K)) {
        in_edit_x = false;
        in_edit_y = false;
    }

    if (!in_edit_x) {
        if (selected_idx < IMAGE_COUNT) input_x = obstacles[selected_idx].get_position().x;
        else input_x = robot.get_position().x;
    }
    if (!in_edit_y) {
        if (selected_idx < IMAGE_COUNT) input_y = obstacles[selected_idx].get_position().y;
        else input_y = robot.get_position().y;
    }

    if (GuiValueBox(
            { x + label_padding, y, box_width, PADDING }, "X  ", &input_x, offset, COORD_RANGE - offset, in_edit_x)) {
        in_edit_x = !in_edit_x;
        if (!in_edit_x) {
            if (selected_idx < IMAGE_COUNT) obstacles[selected_idx].set_position_x(input_x);
            else robot.set_position_x(input_x);
        }
    }

    if (GuiValueBox(
            { x + box_width + PADDING + 2 * label_padding, y, box_width, PADDING },
            "Y  ",
            &input_y,
            offset,
            COORD_RANGE - offset,
            in_edit_y)) {
        in_edit_y = !in_edit_y;
        if (!in_edit_y) {
            if (selected_idx < IMAGE_COUNT) obstacles[selected_idx].set_position_y(input_y);
            else robot.set_position_y(input_y);
        }
    }
}

void render_transmitter_button() {
    const char *label = Transmitter::get_label_from_status(Transmitter::transmission_status);

    const float width = 6 * PADDING;
    const float x = WINDOW_WIDTH - PADDING - width, y = PADDING;

    Transmitter::reset_gui_style();

    if (Transmitter::transmission_status == Transmitter::Status::InProgress) {
        GuiLock();
    }

    if (GuiButton({ x, y, width, PADDING }, label)) {
        std::string commands = Transmitter::get_command_chain();

        Transmitter::router_thread = std::jthread([c = std::move(commands)]() mutable -> void {
            if (Transmitter::route_commands(std::move(c))) {
            }
        });
    }

    GuiUnlock();
}

void render() {
    ClearBackground(BG_COLOR);

    grid.render();

    for (const auto &obstacle : obstacles) {
        obstacle.render_shape();
        obstacle.render_label();
    }

    render_obstacle_buttons();
    render_robot_button();
    render_solution_status();
    render_play_button();
    render_transmitter_button();

    render_selected_indicator();
    render_position_controller();

    animator.render();
    robot.render();
}

class Solver {
    struct Point {
        float x, y, angle;
    };
    struct MiniCommand {
        Instruction instruction;
        float magnitude;
    };
    struct Path {
        MiniCommand mini_command;
        float cost;
    };

    size_t n = 0, m = 0;

    std::vector<int> obstacle_idxs;
    std::vector<Point> points;
    std::vector<float> base_dist;
    std::vector<float> dist;
    std::vector<std::vector<std::vector<MiniCommand>>> path;

    void insert_step(MiniCommand mini_command) {
        Instruction instuction = mini_command.instruction;

        switch (instuction) {
            case Instruction::Forward:
                steps.push_back(std::make_unique<LinearStep>(instuction, mini_command.magnitude));
                break;
            case Instruction::Backward:
                steps.push_back(std::make_unique<LinearStep>(instuction, mini_command.magnitude));
                break;
            case Instruction::TurnLeft:
                steps.push_back(std::make_unique<RotateStep>(instuction, mini_command.magnitude));
                break;
            case Instruction::TurnRight:
                steps.push_back(std::make_unique<RotateStep>(instuction, mini_command.magnitude));
                break;
            case Instruction::ForwardLeft:
                steps.push_back(std::make_unique<CircularTurnStep>(instuction));
                break;
            case Instruction::BackwardLeft:
                steps.push_back(std::make_unique<CircularTurnStep>(instuction));
                break;
            case Instruction::ForwardRight:
                steps.push_back(std::make_unique<CircularTurnStep>(instuction));
                break;
            case Instruction::BackwardRight:
                steps.push_back(std::make_unique<CircularTurnStep>(instuction));
                break;
            default:
                break;
        }
    }

    void insert_scan(int obstacle_idx) { steps.push_back(std::make_unique<ScanStep>(obstacle_idx)); }

    Path get_turn(float angle_cur, float angle_end) {
        if (FloatEquals(angle_cur, angle_end)) {
            return { { Instruction::TurnLeft, 0.0f }, 0.0f };
        }

        float delta = angle_end - angle_cur;

        if (delta < -PI - EPS) delta += 2.0f * PI;
        if (delta > PI + EPS) delta -= 2.0f * PI;

        MiniCommand mini_command;

        float delta_degrees = fabsf(delta) > EPS ? fabsf(delta * RAD2DEG) : 0.0f;

        if (delta >= -EPS) {
            mini_command = { Instruction::TurnLeft, delta_degrees };
        } else {
            mini_command = { Instruction::TurnRight, delta_degrees };
        }

        return { mini_command, TURN_PENALTY * fabsf(delta) };
    }

    Path get_linear(Instruction instruction, Vector2 pos_cur, Vector2 pos_end) {
        float euclid_dist = Vector2Distance(pos_cur, pos_end);
        return { MiniCommand({ instruction, euclid_dist }), euclid_dist };
    }

    std::optional<Path> get_circular(Point start_pos, Point target_pos) {
        auto instructions = {
            Instruction::ForwardLeft,
            Instruction::ForwardRight,
            Instruction::BackwardLeft,
            Instruction::BackwardRight,
        };

        const float turn_radius = CircularTurnStep::TURN_BLOCK_COUNT * grid.block_size;

        for (auto instruction : instructions) {
            Vector2 circle_center = { 0.0f, 0.0f };
            float circle_angle    = 0.0f;

            float start_angle = start_pos.angle;

            bool is_left    = instruction == Instruction::ForwardLeft || instruction == Instruction::BackwardLeft;
            bool is_forward = instruction == Instruction::ForwardLeft || instruction == Instruction::ForwardRight;

            float swing_dir     = (is_left == is_forward) ? 1.0f : -1.0f;
            float center_offset = is_left ? (PI / 2.0f) : (-PI / 2.0f);

            circle_center.x = start_pos.x + turn_radius * cosf(start_angle + center_offset);
            circle_center.y = start_pos.y + turn_radius * sinf(start_angle + center_offset);
            circle_angle    = atan2f(start_pos.y - circle_center.y, start_pos.x - circle_center.x);

            float final_circle_angle = circle_angle + (swing_dir * (PI / 2.0f));

            Vector2 end_pos = { circle_center.x + turn_radius * cosf(final_circle_angle),
                                circle_center.y + turn_radius * sinf(final_circle_angle) };

            float end_angle = Wrap(final_circle_angle + center_offset, 0, 2.0f * PI);

            if (FloatEquals(end_pos.x, target_pos.x) && FloatEquals(end_pos.y, target_pos.y)
                && FloatEquals(end_angle, Wrap(target_pos.angle, 0, 2.0f * PI))) {
                return Path({ MiniCommand({ instruction, 0.0f }), TURN_PENALTY });
            }
        }

        return std::nullopt;
    }

    void build() {
        points.clear();
        obstacle_idxs.clear();
        m = 0;

        Vector2 robot_pos = robot.get_position();
        points.push_back({ robot_pos.x, robot_pos.y, Wrap(robot.get_angle(), 0, 2.0f * PI) });

        for (int i = 0; i < IMAGE_COUNT; i++) {
            DirectedPoint stop_pos = obstacles[i].get_stop_pos();
            if (obstacles[i].get_state() != Obstacle::State::Active) continue;
            points.push_back({ stop_pos.x, stop_pos.y, get_angle_from_direction(stop_pos.direction) });
            obstacle_idxs.push_back(i);
            m++;
        }

        for (const auto &point : grid.get_grid_points()) {
            if (!is_robot_visble(point.x, point.y)) continue;
            points.push_back({ point.x, point.y, get_angle_from_direction(point.direction) });
        }

        n = points.size();

        dist.assign(n * n, INF);
        base_dist = dist;
        path.assign(n, std::vector(n, std::vector<MiniCommand>()));
    }

  public:
    static constexpr float TURN_PENALTY     = 12.0f;
    static constexpr float CIRCULAR_PENALTY = 172.0f;
    static constexpr float MOVE_PENALTY     = 5.0f;

    void solve() {
        auto start_time = std::chrono::high_resolution_clock::now();

        steps.clear();
        build();

        if (m == 0) {
            solution_status = SOLUTION_FOUND;
            solution_status += "0 ms.";
            return;
        }

        for (const auto &obstacle : obstacles) {
            if (!obstacle.stop_pos_exists()) {
                solution_status = SOLUTION_NOT_FOUND;
                return;
            }
        }

        for (size_t i = 0; i < n; i++) {
            for (size_t j = 0; j < n; j++) {
                bool in_collision = false;

                float cur_x = points[i].x, cur_y = points[i].y;
                float end_x = points[j].x, end_y = points[j].y;

                for (const auto &obstacle : obstacles) {
                    if (obstacle.get_state() == Obstacle::State::Hidden) continue;
                    if (obstacle.collide({ cur_x, cur_y }, { end_x, end_y })) {
                        in_collision = true;
                        break;
                    }
                }

                if (in_collision) continue;

                if (!is_robot_visble(cur_x, cur_y)) {
                    continue;
                }

                if (!is_robot_visble(end_x, end_y)) {
                    continue;
                }

                if (i == j) {
                    base_dist[i * n + j] = 0.0f;
                    path[i][j].clear();
                    continue;
                }

                if (FloatEquals(cur_x, end_x) && FloatEquals(cur_y, end_y)
                    && FloatEquals(Wrap(points[i].angle, 0.0, 2.0f * PI), Wrap(points[j].angle, 0.0, 2.0f * PI))) {
                    base_dist[i * n + j] = 0.0f;
                    path[i][j].clear();
                    continue;
                }

                {
                    if (auto arc_match = get_circular(points[i], points[j]); arc_match.has_value()) {
                        Path circular  = arc_match.value();
                        float new_cost = circular.cost + MOVE_PENALTY;

                        if (new_cost < base_dist[i * n + j] - EPS) {
                            base_dist[i * n + j] = new_cost;
                            path[i][j].clear();
                            path[i][j].push_back(circular.mini_command);
                        }
                    }
                }

                if (ENABLE_ROTATION_ON_SPOT && Vector2Equals({ cur_x, cur_y }, { end_x, end_y })) {
                    Path initial_turn    = get_turn(points[i].angle, points[j].angle);
                    base_dist[i * n + j] = initial_turn.cost + MOVE_PENALTY;
                    if (initial_turn.mini_command.magnitude > EPS) path[i][j].push_back(initial_turn.mini_command);
                }

                {
                    float initial_turn_angle = Wrap(atan2f(end_y - cur_y, end_x - cur_x), 0, 2.0f * PI);

                    Path initial_turn = get_turn(points[i].angle, initial_turn_angle);
                    Path linear       = get_linear(Instruction::Forward, { cur_x, cur_y }, { end_x, end_y });
                    Path last_turn    = get_turn(initial_turn_angle, points[j].angle);

                    float new_cost = initial_turn.cost + linear.cost + last_turn.cost + 3 * MOVE_PENALTY;

                    // NOTE: Allow for solely linear steps in all cases.
                    if (initial_turn.mini_command.magnitude <= EPS && last_turn.mini_command.magnitude <= EPS) {
                        int linear_cost = new_cost - 2 * MOVE_PENALTY;

                        if (linear_cost < base_dist[i * n + j] - EPS) {
                            base_dist[i * n + j] = linear_cost;
                            path[i][j].clear();
                            if (linear.mini_command.magnitude > EPS) path[i][j].push_back(linear.mini_command);
                        }
                    }

                    if (ENABLE_ROTATION_ON_SPOT) {
                        if (new_cost < base_dist[i * n + j] - EPS) {
                            base_dist[i * n + j] = new_cost;
                            path[i][j].clear();
                            if (initial_turn.mini_command.magnitude > EPS)
                                path[i][j].push_back(initial_turn.mini_command);
                            if (linear.mini_command.magnitude > EPS) path[i][j].push_back(linear.mini_command);
                            if (last_turn.mini_command.magnitude > EPS) path[i][j].push_back(last_turn.mini_command);
                        }
                    }
                }

                if (ENABLE_ROTATION_ON_SPOT) {
                    float initial_turn_angle = Wrap(PI + atan2f(end_y - cur_y, end_x - cur_x), 0, 2.0f * PI);

                    Path initial_turn = get_turn(points[i].angle, initial_turn_angle);
                    Path linear       = get_linear(Instruction::Backward, { cur_x, cur_y }, { end_x, end_y });
                    Path last_turn    = get_turn(initial_turn_angle, points[j].angle);

                    float new_cost = initial_turn.cost + linear.cost + last_turn.cost + 3 * MOVE_PENALTY;

                    if (new_cost < base_dist[i * n + j] - EPS) {
                        base_dist[i * n + j] = new_cost;
                        path[i][j].clear();
                        if (initial_turn.mini_command.magnitude > EPS) path[i][j].push_back(initial_turn.mini_command);
                        if (linear.mini_command.magnitude > EPS) path[i][j].push_back(linear.mini_command);
                        if (last_turn.mini_command.magnitude > EPS) path[i][j].push_back(last_turn.mini_command);
                    }
                }
            }
        }

        std::vector sssp_parent(n, std::vector(n, -1));

        std::vector adj(n, std::vector<int>());

        for (size_t u = 0; u < n; u++) {
            for (size_t v = 0; v < n; v++) {
                if (FloatEquals(base_dist[u * n + v], INF)) continue;
                adj[u].push_back(v);
            }
        }

        auto build_sssp = [&](int init_u) -> void {
            if (FloatEquals(base_dist[init_u * n + init_u], INF)) return;

            const float IMPROVEMENT_THRESHOLD = 1e-6;

            using Node = std::pair<float, int>;

            std::vector seen(n, 0);
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> q;

            dist[init_u * n + init_u] = 0;
            q.push({ dist[init_u * n + init_u], init_u });

            while (q.size()) {
                auto [cur_dist, u] = q.top();
                q.pop();

                if (!FloatEquals(cur_dist, dist[init_u * n + u])) continue;
                seen[u] = true;

                for (auto v : adj[u]) {
                    if (seen[v]) continue;
                    float new_cost = cur_dist + base_dist[u * n + v];
                    if (new_cost >= dist[init_u * n + v] - IMPROVEMENT_THRESHOLD) continue;
                    dist[init_u * n + v]   = new_cost;
                    sssp_parent[init_u][v] = u;
                    q.push({ dist[init_u * n + v], v });
                }
            }
        };

        std::vector<std::jthread> sssp_threads;

        for (size_t i = 0; i <= m; i++) {
            sssp_threads.emplace_back(build_sssp, i);
        }

        sssp_threads.clear();

        auto get_path = [&](int u, int v) -> std::vector<MiniCommand> {
            std::vector<MiniCommand> commands;
            std::vector<int> nodes;

            while (v != u && v != -1) {
                nodes.push_back(v);
                v = sssp_parent[u][v];
            }
            nodes.push_back(u);

            std::reverse(std::begin(nodes), std::end(nodes));

            for (size_t i = 1; i < nodes.size(); i++) {
                for (const auto &c : path[nodes[i - 1]][nodes[i]]) {
                    commands.push_back(c);
                }
            }

            return commands;
        };

        std::vector mask_cost(1 << m, std::vector(m, INF));
        std::vector parent(1 << m, std::vector(m, std::pair<int, int>()));

        for (size_t mask = 1; mask < (1 << m); mask++) {
            for (size_t v = 0; v < m; v++) {
                if (!((mask >> v) & 1)) continue;
                if (std::popcount(mask) == 1) {
                    mask_cost[mask][v] = dist[v + 1];
                    continue;
                }
                size_t pmask = mask ^ (1 << v);
                for (size_t u = 0; u < m; u++) {
                    if (!((pmask >> u) & 1)) continue;
                    float new_cost = mask_cost[pmask][u] + dist[(u + 1) * n + v + 1];
                    if (new_cost > mask_cost[mask][v] - EPS) continue;
                    mask_cost[mask][v] = new_cost;
                    parent[mask][v]    = { pmask, u };
                }
            }
        }

        std::vector<int> visit_order(m + 1);
        size_t mask = (1 << m) - 1, take_idx = m;

        if (FloatEquals(*std::min_element(std::begin(mask_cost[mask]), std::end(mask_cost[mask])), INF)) {
            solution_status = SOLUTION_NOT_FOUND;
            return;
        }

        int l = std::min_element(std::begin(mask_cost[mask]), std::end(mask_cost[mask])) - std::begin(mask_cost[mask]);

        while (mask) {
            visit_order[take_idx--] = l + 1;
            auto [pmask, r]         = parent[mask][l];
            mask = pmask, l = r;
        }

        for (size_t i = 1; i <= m; i++) {
            auto u = visit_order[i - 1], v = visit_order[i];
            for (const auto &mini_command : get_path(u, v)) {
                insert_step(mini_command);
            }
            insert_scan(obstacle_idxs[visit_order[i] - 1]);
        }

        auto end_time      = std::chrono::high_resolution_clock::now();
        auto time_taken_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        solution_status = SOLUTION_FOUND + std::to_string(time_taken_ms.count()) + " ms.";
    }
};

Solver solver;

void handle_key_press() {
    if (selected_idx < IMAGE_COUNT) {
        if (IsKeyPressed(KEY_SPACE)) {
            const Obstacle::State state      = obstacles[selected_idx].get_state();
            const Obstacle::State last_state = obstacles[selected_idx].get_last_state();

            if (state == Obstacle::State::Hidden) {
                obstacles[selected_idx].set_state(last_state);
            } else {
                obstacles[selected_idx].set_last_state(state);
                obstacles[selected_idx].set_state(Obstacle::State::Hidden);
            }
        }

        if (IsKeyDown(KEY_LEFT)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                obstacles[selected_idx].set_direction(Direction::West);
            } else {
                obstacles[selected_idx].set_position_x(obstacles[selected_idx].get_position().x - 1.0f);
            }
        }

        if (IsKeyDown(KEY_RIGHT)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                obstacles[selected_idx].set_direction(Direction::East);
            } else {
                obstacles[selected_idx].set_position_x(obstacles[selected_idx].get_position().x + 1.0f);
            }
        }

        if (IsKeyDown(KEY_DOWN)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                obstacles[selected_idx].set_direction(Direction::South);
            } else {
                obstacles[selected_idx].set_position_y(obstacles[selected_idx].get_position().y - 1.0f);
            }
        }

        if (IsKeyDown(KEY_UP)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                obstacles[selected_idx].set_direction(Direction::North);
            } else {
                obstacles[selected_idx].set_position_y(obstacles[selected_idx].get_position().y + 1.0f);
            }
        }
    } else {
        if (IsKeyPressed(KEY_LEFT)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                robot.set_direction(Direction::West);
            } else {
                robot.set_position_x(robot.get_position().x - grid.block_size);
            }
        }

        if (IsKeyPressed(KEY_RIGHT)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                robot.set_direction(Direction::East);
            } else {
                robot.set_position_x(robot.get_position().x + grid.block_size);
            }
        }

        if (IsKeyPressed(KEY_DOWN)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                robot.set_direction(Direction::South);
            } else {
                robot.set_position_y(robot.get_position().y - grid.block_size);
            }
        }

        if (IsKeyPressed(KEY_UP)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                robot.set_direction(Direction::North);
            } else {
                robot.set_position_y(robot.get_position().y + grid.block_size);
            }
        }
    }

    if (IsKeyPressed(KEY_J)) {
        selected_idx = (selected_idx + 1) % (IMAGE_COUNT + 1);
    }

    if (IsKeyPressed(KEY_K)) {
        selected_idx = (selected_idx + IMAGE_COUNT) % (IMAGE_COUNT + 1);
    }

    // NOTE: These keybinds are solely for debugging purposes.
    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
        if (IsKeyPressed(KEY_Q)) {
            steps.push_back(std::make_unique<CircularTurnStep>(Instruction::ForwardLeft));
        }
        if (IsKeyPressed(KEY_E)) {
            steps.push_back(std::make_unique<CircularTurnStep>(Instruction::ForwardRight));
        }
        if (IsKeyPressed(KEY_Z)) {
            steps.push_back(std::make_unique<CircularTurnStep>(Instruction::BackwardLeft));
        }
        if (IsKeyPressed(KEY_C)) {
            steps.push_back(std::make_unique<CircularTurnStep>(Instruction::BackwardRight));
        }
        if (IsKeyPressed(KEY_W)) {
            steps.push_back(std::make_unique<LinearStep>(Instruction::Forward, 10.0f));
        }
        if (IsKeyPressed(KEY_X)) {
            steps.push_back(std::make_unique<LinearStep>(Instruction::Backward, 10.0f));
        }
        if (IsKeyPressed(KEY_A)) {
            steps.push_back(std::make_unique<RotateStep>(Instruction::TurnLeft, 45.0f));
        }
        if (IsKeyPressed(KEY_D)) {
            steps.push_back(std::make_unique<RotateStep>(Instruction::TurnRight, 45.0f));
        }
        if (IsKeyPressed(KEY_R)) {
            solver.solve();
        }
    }
}

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Simulator");
    SetTargetFPS(FPS);

    GuiSetStyle(DEFAULT, TEXT_SIZE, FONT_SIZE);

    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToInt(Grid::BORDER_COLOR));
    GuiSetStyle(DEFAULT, TEXT_COLOR_FOCUSED, ColorToInt(Obstacle::FACE_COLOR));
    GuiSetStyle(DEFAULT, TEXT_COLOR_PRESSED, ColorToInt(Obstacle::FACE_COLOR));

    GuiSetStyle(VALUEBOX, BASE_COLOR_NORMAL, ColorToInt(Grid::BORDER_COLOR));
    GuiSetStyle(VALUEBOX, BASE_COLOR_FOCUSED, ColorToInt(Obstacle::FACE_COLOR));
    GuiSetStyle(VALUEBOX, BASE_COLOR_PRESSED, ColorToInt(BLANK));

    GuiSetStyle(VALUEBOX, BORDER_COLOR_NORMAL, ColorToInt(Grid::BORDER_COLOR));
    GuiSetStyle(VALUEBOX, BORDER_COLOR_FOCUSED, ColorToInt(Obstacle::FACE_COLOR));
    GuiSetStyle(VALUEBOX, BORDER_COLOR_PRESSED, ColorToInt(Obstacle::FACE_COLOR));

    GuiSetStyle(VALUEBOX, TEXT_COLOR_NORMAL, ColorToInt(Grid::BORDER_COLOR));
    GuiSetStyle(VALUEBOX, TEXT_COLOR_FOCUSED, ColorToInt(Obstacle::FACE_COLOR));
    GuiSetStyle(VALUEBOX, TEXT_COLOR_PRESSED, ColorToInt(Obstacle::FACE_COLOR));

    font = GuiGetFont();

    gen.seed(device());

    for (int i = 0; i < IMAGE_COUNT; i++) {
        obstacles[i].set_label(std::to_string(i + 1));
    }

    randomize_obstacles_positions();

    while (!WindowShouldClose()) {
        handle_key_press();
        animator.animate();

        BeginDrawing();

        render();

        EndDrawing();
    }

    if (Transmitter::router_thread.has_value() && Transmitter::router_thread->joinable()) {
        Transmitter::router_thread->join();
    }

    return 0;
}
