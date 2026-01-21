#include <wx/wx.h>
#include <wx/dcbuffer.h>
#include <wx/timer.h>
#include <wx/statline.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include "Constants.h"
#include "Vector.h"
#include "Terrain.h"
#include "LanderState.h"
#include "Engine.h"
#include "Lander.h"
#include "Landing.h"
#include "AutoPilot.h"

enum class SimulationState {
    Idle,
    Running,
    Paused,
    Drawing,
    Landed,
    Crashed
};

static wxString OneDecimalString(float v)
{
    int scaled = (int)std::round(v * 10.0f);
    int whole = scaled / 10;
    int frac = std::abs(scaled % 10);

    wxString s;
    s << whole << "." << frac;
    return s;
}

class SimulationCanvas : public wxPanel {
public:
    SimulationCanvas(wxWindow* parent) : wxPanel(parent){
        SetMinSize(wxSize(1200, 1200));
        SetMaxSize(wxSize(1200, 1200));
        SetBackgroundStyle(wxBG_STYLE_PAINT);

        Bind(wxEVT_PAINT, &SimulationCanvas::OnPaint,this);
        Bind(wxEVT_MOTION, &SimulationCanvas::OnMouseMove, this);
        Bind(wxEVT_LEFT_DOWN, &SimulationCanvas::OnMouseDown, this);
        Bind(wxEVT_RIGHT_DOWN, &SimulationCanvas::OnRightMouseDown, this);

        GenerateStars();
    }
    void SetAutopilotActive(bool on) { autopilot_active = on; Refresh(); }


    void SetTerrain(const Terrain& t) {
        terrain = t;
        landing_zones = terrain.FindSafeLandingZones(Constants::WORLD_WIDTH);
        Refresh();
    }
    void SetLander(const Lander& l) {
        lander = const_cast<Lander*>(&l);
        Refresh();
    }

    void SetSimulationState(SimulationState st) {
        sim_state = st;
        Refresh();
    }

    void SetMessage(const wxString& m) {
        message = m;
        Refresh();
    }

    void ClearTrajectory() {
        trajectory_points.clear();
        Refresh();
    }

    void AddTrajectoryPoint(const Vector& pos_world) {
        trajectory_points.push_back(WorldToScreen(pos_world));
        if (trajectory_points.size() > 10000) {
            trajectory_points.erase(
                trajectory_points.begin(),
                trajectory_points.begin() + 5000
            );
        }
    }

    void EnableDrawingMode() {
        drawing_mode = true;
        sim_state = SimulationState::Drawing;
        drawn_points.clear();
        trajectory_points.clear();
        message.clear();
        Refresh();
    }

    void DisableDrawingMode() {
        drawing_mode = false;
        if (sim_state == SimulationState::Drawing)
            sim_state = SimulationState::Idle;
        Refresh();
    }

    bool IsDrawingMode() const { return drawing_mode; }

    bool ConsumeFinishDrawingRequest() {
        if (!request_finish_drawing) return false;
        request_finish_drawing = false;
        return true;
    }

    wxPoint ScreenToWorldPoint(const wxPoint& p) const {
        int w, h;
        GetClientSize(&w, &h);
        if (w <= 0 || h <= 0) return wxPoint(0, 0);

        float sx = (float)w/ Constants::WORLD_WIDTH;
        float sy = (float)h/ Constants::WORLD_HEIGHT;
        float s = std::min(sx, sy);

        float wx_= p.x / s;
        float wy_= Constants::WORLD_HEIGHT - p.y / s;
        return wxPoint((int)wx_, (int)wy_);
    }

    const std::vector<wxPoint>& GetDrawnPoints() const { return drawn_points; }

    void SetAutopilot(Autopilot* ap) { autopilot = ap; }

private:
    Terrain terrain;
    Lander* lander = nullptr;
    Autopilot* autopilot = nullptr;
    bool autopilot_active = false;


    SimulationState sim_state = SimulationState::Idle;

    std::vector<wxPoint> stars;
    std::vector<wxPoint> trajectory_points;
    std::vector<wxPoint> drawn_points;
    std::vector<LandingZone> landing_zones;

    bool drawing_mode = false;
    bool request_finish_drawing = false;
    wxString message;

    wxPoint WorldToScreen(const Vector& p) const {
        int w, h;
        GetClientSize(&w, &h);
        if (w <= 0 || h <= 0) return wxPoint(0, 0);

        float world_w = Constants::WORLD_WIDTH;
        float world_h = Constants::WORLD_HEIGHT;

        float sx = (float)w / world_w;
        float sy = (float)h / world_h;
        float scale = std::min(sx, sy);

        float offset_x = 0.5f * (w - world_w * scale);
        float offset_y = 0.5f * (h - world_h * scale);

        int px = (int)(offset_x + p.x * scale);
        int py = (int)(offset_y + (world_h - p.y) * scale);

        return wxPoint(px, py);
    }

    void GenerateStars() {
        stars.clear();
        const int coords[][2] = {
            {50,30},{150,60},{280,40},{420,80},{550,25},
            {680,70},{800,45},{920,65},{1050,35},
            {120,100},{260,120},{390,110},{540,130},{700,115},
            {880,90},{1030,120}
        };
        int n = sizeof(coords) / sizeof(coords[0]);
        for (int i = 0; i < n; ++i)
            stars.emplace_back(coords[i][0], coords[i][1]);
    }

    void DrawStars(wxDC& dc) {
        dc.SetPen(wxPen(wxColour(200, 200, 200), 1));
        dc.SetBrush(wxBrush(wxColour(200, 200, 200)));
        for (const auto& s : stars)
            dc.DrawCircle(s.x, s.y, 1);
    }

    void DrawMoonSurface(wxDC& dc) {
        dc.SetBrush(wxBrush(wxColour(169, 169, 169)));
        dc.SetPen(wxPen(wxColour(100, 100, 100), 2));

        if (terrain.GetPointCount() >= 2) {
            int count = terrain.GetPointCount() + 2;
            std::vector<wxPoint> poly(count);

            for (int i = 0; i < terrain.GetPointCount(); ++i) {
                Vector p = terrain.GetPoint(i);
                poly[i] = WorldToScreen(p);
            }

            int w, h;
            GetClientSize(&w, &h);
            poly[count - 2] = wxPoint(w, h);
            poly[count - 1] = wxPoint(0, h);

            dc.DrawPolygon(count, &poly[0]);
        }
    }

    void DrawLandingZones(wxDC& dc) {
        if (landing_zones.empty()) return;

        dc.SetBrush(wxBrush(wxColour(100, 200, 100), wxBRUSHSTYLE_FDIAGONAL_HATCH));
        dc.SetPen(wxPen(wxColour(0, 255, 0), 2));

        for (const auto& zone : landing_zones) {
            wxPoint p1 = WorldToScreen(Vector(zone.x_start, terrain.GetHeightAt(zone.x_start)));
            wxPoint p2 = WorldToScreen(Vector(zone.x_end,terrain.GetHeightAt(zone.x_end)));
            dc.DrawLine(p1.x, p1.y - 10, p2.x, p2.y - 10);
        }
    }

    void DrawTrajectory(wxDC& dc) {
        if (trajectory_points.size() < 2) return;

        dc.SetPen(wxPen(wxColour(0, 255, 150), 1));

        int start = std::max(0, (int)trajectory_points.size() - 500);
        for (size_t i = start + 1; i < trajectory_points.size(); ++i)
            dc.DrawLine(trajectory_points[i - 1], trajectory_points[i]);
    }

    void DrawTargetPoint(wxDC& dc) {
        if (sim_state != SimulationState::Running) return;
        if (!autopilot) return;

        float target_x = autopilot->GetTargetX();
        float target_z = terrain.GetHeightAt(target_x);

        wxPoint screen_pos = WorldToScreen(Vector(target_x, target_z));

        dc.SetBrush(wxBrush(wxColour(255, 50, 50)));
        dc.SetPen(wxPen(wxColour(255, 100, 100), 2));
        dc.DrawCircle(screen_pos.x, screen_pos.y, 5);

        dc.SetPen(wxPen(wxColour(255, 50, 50), 2));
        dc.DrawLine(screen_pos.x - 20, screen_pos.y, screen_pos.x + 20, screen_pos.y);
        dc.DrawLine(screen_pos.x, screen_pos.y - 20, screen_pos.x, screen_pos.y + 20);
    }

    void DrawLander(wxDC& dc) {
        if (drawing_mode) return;
        if (!lander) return; 

        const LanderState& S = lander->getState();

        
        if (autopilot) {
            const auto& rays = autopilot->GetRadarRays();
            for (int i = 0; i < rays.GetSize(); ++i) {
                const auto& ray = rays[i];

                wxColour color;
                if (!ray.hit) {
                    color = wxColour(0, 80, 0); 
                } else if (ray.dangerous) {
                    color = wxColour(255, 0, 0);
                } else {
                    color = wxColour(0, 255, 0);
                }

                dc.SetPen(wxPen(color, 2));
                wxPoint a = WorldToScreen(ray.origin);
                wxPoint b = WorldToScreen(ray.endPoint);
                dc.DrawLine(a, b);
            }
        }

        float body_h = lander->getBodyHeight();
        float leg_len = lander->getLegLength();

        // нижняя точка ног (через COM и геометрию)
        float foot_z = S.pos.y - S.com.y - leg_len - body_h * 0.5f;
        wxPoint foot_pos = WorldToScreen(Vector(S.pos.x, foot_z));

        int w, h;
        GetClientSize(&w, &h);
        float world_w = Constants::WORLD_WIDTH;
        float world_h = Constants::WORLD_HEIGHT;

        float sx = (float)w / world_w;
        float sy = (float)h / world_h;
        float pixels_per_meter = std::min(sx, sy);

        float scale = body_h * pixels_per_meter;
        float body_half_h_px = 0.5f * body_h * pixels_per_meter;
        float leg_len_px = leg_len * pixels_per_meter;

        int x = foot_pos.x;
        int y = foot_pos.y - (int)(scale * (1.0f / 3.0f + 0.5f));
        float body_angle = -S.angle;
        float cosA = std::cos(body_angle);
        float sinA = std::sin(body_angle);

        auto R = [x, y, cosA, sinA](int px, int py) {
            float dx = (float)px - (float)x;
            float dy = (float)py - (float)y;
            int rx = x + (int)(dx * cosA - dy * sinA);
            int ry = y + (int)(dx * sinA + dy * cosA);
            return wxPoint(rx, ry);
        };
        //корпус
        dc.SetBrush(wxBrush(wxColour(255, 200, 50)));
        dc.SetPen(wxPen(wxColour(200, 150, 0), 2));

        int body_left = x - (int)(scale / 2);
        int body_right = x + (int)(scale / 2);
        int body_top = y - (int)(scale / 3);
        int body_bottom = y + (int)(scale * 0.7f - scale / 3);

        wxPoint body_poly[4];
        body_poly[0] = R(body_left,body_top);
        body_poly[1] = R(body_right,body_top);
        body_poly[2] = R(body_right,body_bottom);
        body_poly[3] = R(body_left, body_bottom);
        dc.DrawPolygon(4, body_poly);

        // кабина
        dc.SetBrush(wxBrush(wxColour(100, 150, 200)));
        dc.SetPen(wxPen(wxColour(50, 100, 150), 1));
        wxPoint cabin_c = R(x, y - (int)(scale / 5));
        dc.DrawCircle(cabin_c.x, cabin_c.y, (int)(scale / 4));

        // лапки
        dc.SetBrush(wxBrush(wxColour(200, 200, 200)));
        dc.SetPen(wxPen(wxColour(100, 100, 100), 1));

        float leg_length = leg_len_px;
        int leg_y = y + (int)body_half_h_px;
        int foot_r = (int)(scale * 0.06f);

        float angle_deg = 15.0f;
        float dx_leg = leg_length * std::tan(angle_deg * 3.14159265f / 180.0f);
        float spread = scale * 0.35f;

        wxPoint left_top_unrot = wxPoint(x - (int)spread,leg_y);
        wxPoint left_foot_unrot = wxPoint(x - (int)(spread + dx_leg), leg_y + (int)leg_length);
        wxPoint left_top = R(left_top_unrot.x,  left_top_unrot.y);
        wxPoint left_foot = R(left_foot_unrot.x, left_foot_unrot.y);
        dc.DrawLine(left_top, left_foot);
        dc.DrawCircle(left_foot.x, left_foot.y, foot_r);

        wxPoint right_top_unrot = wxPoint(x + (int)spread,leg_y);
        wxPoint right_foot_unrot = wxPoint(x + (int)(spread + dx_leg), leg_y + (int)leg_length);
        wxPoint right_top = R(right_top_unrot.x, right_top_unrot.y);
        wxPoint right_foot = R(right_foot_unrot.x, right_foot_unrot.y);
        dc.DrawLine(right_top, right_foot);
        dc.DrawCircle(right_foot.x, right_foot.y, foot_r);

        
        dc.SetBrush(wxBrush(wxColour(180, 180, 180)));
        dc.SetPen(wxPen(wxColour(120, 120, 120), 1));

        int engine0_x_unrot = x - (int)(scale * 0.2f);
        int engine1_x_unrot = x + (int)(scale * 0.2f);
        int engine_bottom_y_unrot = leg_y;

        wxPoint engine0_c = R(engine0_x_unrot, engine_bottom_y_unrot);
        wxPoint engine1_c = R(engine1_x_unrot, engine_bottom_y_unrot);

        dc.DrawCircle(engine0_c.x, engine0_c.y, foot_r);
        dc.DrawCircle(engine1_c.x, engine1_c.y, foot_r);

        int side_y_unrot   = y;
        int engine2_x_unrot = x - (int)(scale * 0.6f);
        int engine3_x_unrot = x + (int)(scale * 0.6f);
        wxPoint engine2_c = R(engine2_x_unrot, side_y_unrot);
        wxPoint engine3_c = R(engine3_x_unrot, side_y_unrot);
        dc.DrawCircle(engine2_c.x, engine2_c.y, foot_r);
        dc.DrawCircle(engine3_c.x, engine3_c.y, foot_r);
    }

    void DrawDrawingMode(wxDC& dc) {
        if (!drawing_mode) return;

        dc.SetPen(wxPen(wxColour(255, 255, 0), 2));
        for (size_t i = 1; i < drawn_points.size(); ++i)
            dc.DrawLine(drawn_points[i - 1], drawn_points[i]);

        dc.SetBrush(wxBrush(wxColour(255, 255, 0)));
        for (const auto& pt : drawn_points)
            dc.DrawCircle(pt.x, pt.y, 3);
    }

    void DrawHUD(wxDC& dc) {
        wxFont font = dc.GetFont();
        font.SetPointSize(10);
        font.MakeBold();
        dc.SetFont(font);

        int margin = 10;

        wxColour text_color;
        if (sim_state == SimulationState::Crashed)
            text_color = wxColour(255, 0, 0);
        else if (sim_state == SimulationState::Landed)
            text_color = wxColour(0, 255, 0);
        else
            text_color = wxColour(0, 255, 255);

        dc.SetTextForeground(text_color);

        if (drawing_mode) {
            dc.SetTextForeground(wxColour(255, 255, 0));
            dc.DrawText(
                wxString::Format(
                    "DRAWING MODE: %d points | use LMB to draw, press button again to finish",
                    (int)drawn_points.size()),
                margin, margin
            );
        } else {
            if (!lander) return;
            const LanderState& S = lander->getState();
            float alt = S.pos.y - terrain.GetHeightAt(S.pos.x);
            wxString status = wxString::Format(
                "HEIGHT: %.0f m | VZ: %.1f m/s | VX: %.1f m/s | FUEL: %.1f kg | TIME: %.1f s",
                alt, S.v.y, S.v.x, S.getFuelMass(), S.t
            );

            if (autopilot_active && autopilot) {
                status += wxString::Format(" | PHASE: %s", autopilot->GetPhaseText());
            }

            dc.DrawText(status, margin, margin);

        }

        if (!message.empty() && !drawing_mode)
        {
            font.SetPointSize(24);
            font.MakeBold();
            dc.SetFont(font);
            dc.SetTextForeground(
                sim_state == SimulationState::Landed ? wxColour(0, 255, 0): wxColour(255, 0, 0)
            );
            int w, h;
            GetClientSize(&w, &h);
            wxSize ts = dc.GetTextExtent(message);
            dc.DrawText(message, (w - ts.x) / 2, (h - ts.y) / 2);
        }
    }


    void OnPaint(wxPaintEvent&) {
        wxAutoBufferedPaintDC dc(this);
        dc.SetBrush(wxBrush(wxColour(10, 10, 35)));
        dc.SetPen(wxPen(wxColour(10, 10, 35)));
        int w, h;
        GetClientSize(&w, &h);
        dc.DrawRectangle(0, 0, w, h);

        DrawStars(dc);

        if (!drawing_mode) {
            DrawMoonSurface(dc);
            DrawLandingZones(dc);
            DrawTrajectory(dc);
            DrawTargetPoint(dc);
            DrawLander(dc);
        }

        DrawDrawingMode(dc);
        DrawHUD(dc);
    }

    void OnMouseMove(wxMouseEvent& event) {
        if (drawing_mode && event.LeftIsDown()) {
            drawn_points.push_back(event.GetPosition());
            Refresh();
        }
    }

    void OnRightMouseDown(wxMouseEvent&) {
        if (drawing_mode) {
            request_finish_drawing = true;
            Refresh();
        }
    }

    void OnMouseDown(wxMouseEvent& event) {
        if (drawing_mode && event.LeftDown()) {
            drawn_points.push_back(event.GetPosition());
            Refresh();
        }
    }
};

class LunarFrame : public wxFrame {
public:
    LunarFrame() : wxFrame(nullptr, wxID_ANY, "Lunar Lander 2 (new physics)",
                  wxDefaultPosition, wxSize(1700, 900)), sim_timer(this){
        std::srand((unsigned)std::time(nullptr));

        wxPanel* main = new wxPanel(this);
        wxBoxSizer* root = new wxBoxSizer(wxHORIZONTAL);
        wxBoxSizer* left = new wxBoxSizer(wxVERTICAL);
        wxBoxSizer* right = new wxBoxSizer(wxVERTICAL);

        sim_canvas = new SimulationCanvas(main);
        left->Add(sim_canvas, 1, wxEXPAND | wxALL, 5);

        right->SetMinSize(wxSize(260, -1));

        start_btn = new wxButton(main, wxID_ANY, "START");
        stop_btn  = new wxButton(main, wxID_ANY, "PAUSE");
        reset_btn = new wxButton(main, wxID_ANY, "RESET");
        draw_btn  = new wxButton(main, wxID_ANY, "DRAW TERRAIN");

        start_btn->SetBackgroundColour(wxColour(100, 200, 100));
        stop_btn->SetBackgroundColour(wxColour(200, 100, 100));
        reset_btn->SetBackgroundColour(wxColour(150, 150, 150));
        draw_btn->SetBackgroundColour(wxColour(255, 200, 0));

        right->Add(start_btn, 0, wxEXPAND | wxALL, 5);
        right->Add(stop_btn,  0, wxEXPAND | wxALL, 5);
        right->Add(reset_btn, 0, wxEXPAND | wxALL, 5);
        right->Add(draw_btn,  0, wxEXPAND | wxALL, 5);

        right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);

        autopilot_checkbox = new wxCheckBox(main, wxID_ANY, "AUTOPILOT");
        autopilot_checkbox->SetValue(false);
        right->Add(autopilot_checkbox, 0, wxALL, 5);

        right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);

        right->Add(new wxStaticText(main, wxID_ANY, "Simulation speed (1..10):"), 0, wxALL, 5);

        speed_slider = new wxSlider(
            main, wxID_ANY, 1, 1, 10,
            wxDefaultPosition, wxDefaultSize,
            wxSL_HORIZONTAL | wxSL_LABELS
        );
        right->Add(speed_slider, 0, wxEXPAND | wxALL, 5);

        right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);

        right->Add(new wxStaticText(main, wxID_ANY,"Wind speed (m/s):"), 0, wxALL, 5);

        wind_slider = new wxSlider(
            main, wxID_ANY, 0, -5, 5,
            wxDefaultPosition, wxDefaultSize,
            wxSL_HORIZONTAL | wxSL_LABELS
        );
        right->Add(wind_slider, 0, wxEXPAND | wxALL, 5);

        right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);

        const char* labels[4] = {
            "Engine 0 (main L)",
            "Engine 1 (main R)",
            "Engine 2 (side L)",
            "Engine 3 (side R)"
        };
        for (int i = 0; i < 4; ++i) {
            wxStaticText* lbl = new wxStaticText(main, wxID_ANY, labels[i]);
            lbl->SetForegroundColour(*wxBLACK);
            right->Add(lbl, 0, wxLEFT | wxRIGHT | wxTOP, 5);

            engine_sliders[i] = new wxSlider(
                main, wxID_ANY, 0, 0, 100,
                wxDefaultPosition, wxDefaultSize,
                wxSL_HORIZONTAL | wxSL_LABELS
            );
            right->Add(engine_sliders[i], 0, wxEXPAND | wxALL, 5);
        }

        right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);

        status_text = new wxStaticText(main, wxID_ANY, "Status: IDLE");
        fuel_text = new wxStaticText(main, wxID_ANY, "Fuel: 0.0");

        wxFont font = status_text->GetFont();
        font.SetWeight(wxFONTWEIGHT_BOLD);
        font.SetPointSize(9);
        status_text->SetFont(font);
        fuel_text->SetFont(font);
        status_text->SetForegroundColour(*wxBLACK);
        fuel_text->SetForegroundColour(*wxBLACK);

        right->Add(status_text, 0, wxEXPAND | wxALL, 5);
        right->Add(fuel_text, 0, wxEXPAND | wxALL, 5);

        
        // right->Add(new wxStaticLine(main, wxID_ANY), 0, wxEXPAND | wxALL, 5);
        // dbg_title = new wxStaticText(main, wxID_ANY, "Autopilot debug");
        // dbg_title->SetForegroundColour(*wxBLACK);
        // right->Add(dbg_title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 5);

        // wxFont dbg_font = dbg_title->GetFont();
        // dbg_font.SetPointSize(8);
        // dbg_font.SetWeight(wxFONTWEIGHT_NORMAL);

        // auto make_dbg_line = [&](wxStaticText*& out) {
        //     out = new wxStaticText(main, wxID_ANY, "");
        //     out->SetFont(dbg_font);
        //     out->SetForegroundColour(*wxBLACK);
        //     right->Add(out, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 5);
        // };

        // make_dbg_line(dbg_phase);
        // make_dbg_line(dbg_target);
        // make_dbg_line(dbg_radar);
        // make_dbg_line(dbg_gate_like);
        // make_dbg_line(dbg_angle);
        // make_dbg_line(dbg_thrust);
        // make_dbg_line(dbg_legs);

        root->Add(left,  1, wxEXPAND);
        root->Add(right, 0, wxEXPAND | wxALL, 5);

        main->SetSizer(root);
        SetSizerAndFit(new wxBoxSizer(wxVERTICAL));
        GetSizer()->Add(main, 1, wxEXPAND);

        Bind(wxEVT_TIMER, &LunarFrame::OnTimer, this);
        sim_timer.Start(20);

        start_btn->Bind(wxEVT_BUTTON, &LunarFrame::OnStart, this);
        stop_btn->Bind (wxEVT_BUTTON, &LunarFrame::OnPause, this);
        reset_btn->Bind(wxEVT_BUTTON, &LunarFrame::OnReset, this);
        draw_btn->Bind (wxEVT_BUTTON, &LunarFrame::OnDrawTerrain, this);

        autopilot_checkbox->Bind(wxEVT_CHECKBOX, &LunarFrame::OnAutopilotToggle, this);
        terrain.Clear();
        sim_canvas->SetTerrain(terrain);

        ResetSimulation();
    }

    ~LunarFrame() override {
        if (autopilot) delete autopilot;
    }

private:
    SimulationCanvas* sim_canvas = nullptr;
    Lander lander;
    Terrain terrain;

    Autopilot* autopilot = nullptr;

    wxTimer sim_timer;
    SimulationState sim_state = SimulationState::Idle;
    bool sim_running = false;

    wxSlider* speed_slider = nullptr;
    wxSlider* wind_slider = nullptr;
    float wind_speed = 0.0f;
    wxSlider* engine_sliders[4]{};

    wxStaticText* status_text = nullptr;
    wxStaticText* fuel_text = nullptr;

    
    wxStaticText* dbg_title = nullptr;
    wxStaticText* dbg_phase = nullptr;
    wxStaticText* dbg_target = nullptr;
    wxStaticText* dbg_radar = nullptr;
    wxStaticText* dbg_gate_like = nullptr;
    wxStaticText* dbg_angle = nullptr;
    wxStaticText* dbg_thrust = nullptr;
    wxStaticText* dbg_legs = nullptr;

    LandingContact last_contact{};
    LandingStatus  last_landing_status = LandingStatus::Airborne;

    wxButton* start_btn = nullptr;
    wxButton* stop_btn  = nullptr;
    wxButton* reset_btn = nullptr;
    wxButton* draw_btn  = nullptr;

    wxCheckBox* autopilot_checkbox = nullptr;

    void ResetSimulation() {
        lander = Lander();

        LanderState& S = lander.stateRef();
        // где корабль отнно мира
        S.pos = Vector(Constants::WORLD_WIDTH  * 0.5f, Constants::WORLD_HEIGHT * 0.85f);
        S.v = Vector(0.0f, -15.0f);
        S.angle = 0.0f;
        S.omega = 0.0f;
        S.t = 0.0f;

        for (int i = 0; i < 4; ++i)
            engine_sliders[i]->SetValue(0);

        if (autopilot) delete autopilot;
        autopilot = new Autopilot(lander, terrain);
        if (sim_canvas) sim_canvas->SetAutopilot(autopilot);

        sim_canvas->ClearTrajectory();
        sim_canvas->SetMessage("");
        sim_canvas->SetLander(lander);  

        sim_state   = SimulationState::Idle;
        sim_running = false;

        sim_canvas->SetSimulationState(sim_state);
        if (sim_canvas) sim_canvas->SetAutopilotActive(false);


        UpdateUI();
    }

    void UpdateUI() {
        const LanderState& S = lander.getState();

        wxString st;
        switch (sim_state) {
            case SimulationState::Idle:st = "Status: IDLE";break;
            case SimulationState::Running: st = "Status: RUNNING"; break;
            case SimulationState::Paused: st = "Status: PAUSED";  break;
            case SimulationState::Drawing: st = "Status: DRAWING"; break;
            case SimulationState::Landed: st = "Status: LANDED";  break;
            case SimulationState::Crashed: st = "Status: CRASHED"; break;
        }
        status_text->SetLabel(st);

        wxString fuel_str = "Fuel: ";
        fuel_str += OneDecimalString(S.getFuelMass());
        fuel_text->SetLabel(fuel_str);

        
        if (dbg_phase) {
            if (autopilot_checkbox && autopilot_checkbox->GetValue() && autopilot) {
                const char* phase = autopilot->GetPhaseText();
                const float target_x = autopilot->GetTargetX();
                const float x = S.pos.x;
                const float alt = S.pos.y - terrain.GetHeightAt(S.pos.x);
                const float dx = target_x - x;

                
                bool radar_danger = false;
                float h_min = 1e9f;
                float dist_min = 1e9f;
                const auto& rays = autopilot->GetRadarRays();
                for (int i = 0; i < rays.GetSize(); ++i) {
                    const auto& r = rays[i];
                    if (!r.hit) continue;
                    radar_danger = radar_danger || r.dangerous;
                    h_min = std::min(h_min, r.verticalClearance);
                    float d = (r.hitPoint - r.origin).len();
                    dist_min = std::min(dist_min, d);
                }
                if (!std::isfinite(h_min) || h_min > 1e8f) h_min = -1.0f;
                if (!std::isfinite(dist_min) || dist_min > 1e8f) dist_min = -1.0f;

                dbg_phase->SetLabel(wxString::Format(
                    "PHASE: %s | alt=%.1f | vx=%.2f vy=%.2f",
                    phase, alt, S.v.x, S.v.y
                ));
                dbg_target->SetLabel(wxString::Format(
                    "Target: x=%.1f | dx=%.1f",
                    target_x, dx
                ));
                dbg_radar->SetLabel(wxString::Format(
                    "Radar: danger=%d | h_min=%.1f | dist_min=%.1f",
                    (int)radar_danger, h_min, dist_min
                ));

                
                dbg_gate_like->SetLabel(wxString::Format(
                    "Gate hints: |dx|=%.2f |vx|=%.2f |vy|=%.2f",
                    std::fabs(dx), std::fabs(S.v.x), std::fabs(S.v.y)
                ));

                dbg_angle->SetLabel(wxString::Format(
                    "Angle: theta=%.1f deg | omega=%.2f",
                    S.angle * Constants::RAD2DEG, S.omega
                ));

                dbg_thrust->SetLabel(wxString::Format(
                    "Engines: mainL=%.0f mainR=%.0f sideL=%.0f sideR=%.0f",
                    lander.getEngineThrottle(0), lander.getEngineThrottle(1),
                    lander.getEngineThrottle(2), lander.getEngineThrottle(3)
                ));

                dbg_legs->SetLabel(wxString::Format(
                    "Legs: L touch=%d pen=%.3f in=%d | R touch=%d pen=%.3f in=%d | status=%d",
                    (int)last_contact.left_touch, last_contact.left_penetration, (int)last_contact.left_in_zone,
                    (int)last_contact.right_touch, last_contact.right_penetration, (int)last_contact.right_in_zone,
                    (int)last_landing_status
                ));
            } else {
                dbg_phase->SetLabel("");
                dbg_target->SetLabel("");
                dbg_radar->SetLabel("");
                dbg_gate_like->SetLabel("");
                dbg_angle->SetLabel("");
                dbg_thrust->SetLabel("");
                dbg_legs->SetLabel("");
            }
        }
    }

    void OnTimer(wxTimerEvent&) {
        if (sim_canvas && sim_canvas->ConsumeFinishDrawingRequest()) {
            wxCommandEvent dummy;
            OnDrawTerrain(dummy);
            return;
        }

        if (!sim_running) return;

        int speed = speed_slider->GetValue();
        if (speed < 1) speed = 1;

        bool use_autopilot = autopilot_checkbox && autopilot_checkbox->GetValue();
        if (sim_canvas) sim_canvas->SetAutopilotActive(use_autopilot);


        LandingStatus final_status = LandingStatus::Airborne;

        for (int step = 0; step < speed; ++step) {
            if (use_autopilot && autopilot) {
                autopilot->Update();
            } else {
                for (int i = 0; i < 4; ++i) {
                    float t = (float)engine_sliders[i]->GetValue();
                    lander.setEngineThrottle(i, t);
                }
                lander.setTargetAngleDeg(0.0f);
            }
            float WIND_STRENGTH = 0.2f;      //среда
            float WIND_CROSS_SECTION = 2.5f; // площадь поперечного сечения
            float WIND_OPPOSITION = 1.2f;      // прямоугольник
            float WIND_COEFF = WIND_STRENGTH * WIND_CROSS_SECTION * WIND_OPPOSITION;


            wind_speed = wind_slider ? wind_slider->GetValue() : 0.0f;
            if (std::fabs(wind_speed) > 0.01f) {
                LanderState& Ws = lander.stateRef();
                Ws.v.x += (wind_speed * WIND_COEFF / Constants::DRY_MASS) * Constants::DT ;
            }

            lander.update(Constants::DT);

            LandingContact C = LandingSolver::computeContact(lander, terrain);
            LandingSolver::resolve(lander, C, terrain);
            final_status = LandingSolver::getFinalStatus(lander, C);

            
            last_contact = C;
            last_landing_status = final_status;

            static int landed_ticks = 0;

            bool contact = (final_status == LandingStatus::TwoLegsContact) || (final_status == LandingStatus::OneLegContact);

            const LanderState& S = lander.getState();

            bool slow = std::fabs(S.v.x) <= 1.5f && std::fabs(S.v.y) <= 1.0f && std::fabs(S.omega) <= 0.35f;

            if (contact && slow) {
                landed_ticks++;
                if (landed_ticks >= 8) {
                    final_status = LandingStatus::Landed;
                }
            } else if (!contact) {
                landed_ticks = 0;
            }


            sim_canvas->AddTrajectoryPoint(lander.getState().pos);

            if (final_status == LandingStatus::Landed || final_status == LandingStatus::Crashed) break;
        }

        {
            for (int i = 0; i < 4; ++i) {
                float t = lander.getEngineThrottle(i);
                if (!std::isfinite(t)) t = 0.0f;

                int val = (int)std::round(std::clamp(t, 0.0f, 100.0f));
                engine_sliders[i]->SetValue(val);
            }
        }

        if (final_status == LandingStatus::Landed) {
            
            lander.setMainThrottle(0.0f);
            lander.setSideThrottle(0.0f, 0.0f);
            sim_running = false;
            sim_state   = SimulationState::Landed;
            sim_canvas->SetMessage("LANDED");
        } else if (final_status == LandingStatus::Crashed) {
            lander.setMainThrottle(0.0f);
            lander.setSideThrottle(0.0f, 0.0f);
            sim_running = false;
            sim_state   = SimulationState::Crashed;
            sim_canvas->SetMessage("CRASHED");
        } else {
            sim_state = SimulationState::Running;
        }

        sim_canvas->SetLander(lander);  
        sim_canvas->SetSimulationState(sim_state);
        UpdateUI();
    }

    void OnStart(wxCommandEvent&) {
        if (sim_state == SimulationState::Idle ||
            sim_state == SimulationState::Paused ||
            sim_state == SimulationState::Landed ||
            sim_state == SimulationState::Crashed)
        {
            if (terrain.GetPointCount() < 2) {
                wxMessageBox("Draw terrain first (button 'DRAW TERRAIN')", "Error");
                return;
            }
            sim_running = true;
            sim_state = SimulationState::Running;
            sim_canvas->SetMessage("");
            UpdateUI();
        }
    }

    void OnPause(wxCommandEvent&) {
        if (sim_running) {
            sim_running = false;
            sim_state = SimulationState::Paused;
            UpdateUI();
        }
    }

    void OnReset(wxCommandEvent&) {
        ResetSimulation();
    }

    void OnAutopilotToggle(wxCommandEvent&) {
        // флаг читается в OnTimer
    }

    // рисование рельефа мышкой 
    void OnDrawTerrain(wxCommandEvent&) {
        if (sim_running) return;

        if (!sim_canvas->IsDrawingMode()) {
            terrain.Clear();
            sim_canvas->SetTerrain(terrain);
            sim_canvas->EnableDrawingMode();
            sim_canvas->SetMessage("LMB: draw terrain. Press button again to finish.");
            draw_btn->SetLabel("FINISH DRAWING");
        } else {
            // Выход из режима рисования, переносим точки в Terrain
            const auto& pts_screen = sim_canvas->GetDrawnPoints();
            if (pts_screen.size() < 2) {
                wxMessageBox("Draw at least two points.", "Error");
            } else {
                terrain.Clear();
                for (const auto& p : pts_screen) {
                    wxPoint w = sim_canvas->ScreenToWorldPoint(p);
                    terrain.AddPoint((float)w.x, (float)w.y);
                }
                sim_canvas->SetTerrain(terrain);
            }
            sim_canvas->DisableDrawingMode();
            sim_canvas->SetMessage("");
            draw_btn->SetLabel("DRAW TERRAIN");

            // пересоздаём автопилот под новый рельеф
            if (autopilot) {
                delete autopilot;
                autopilot = new Autopilot(lander, terrain);
                if (sim_canvas)
                    sim_canvas->SetAutopilot(autopilot);
            }
        }
    }
};

class LunarApp : public wxApp {
public:
    bool OnInit() override {
        if (!wxApp::OnInit()) return false;
        LunarFrame* f = new LunarFrame();
        f->Show(true);
        return true;
    }
};

wxIMPLEMENT_APP(LunarApp);
