/*******************************************************************************************
*
*
********************************************************************************************/

#include "raylib.h"
#include <math.h>
#include <float.h>

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "hodographs.h"

// add two Vector2's
Vector2 operator+(const Vector2& a, const Vector2& b)
{
    return Vector2{a.x + b.x, a.y + b.y};
}

// multiply a Vector2 by a scalar
Vector2 operator*(const Vector2& a, const float& b)
{
    return Vector2{a.x * b, a.y * b};
}

// multiply a Vector2 by a scalar
Vector2 operator*(const float& b, const Vector2& a)
{
    return Vector2{a.x * b, a.y * b};
}

// *= a Vector2 by a scalar
Vector2& operator*=(Vector2& a, const float& b)
{
    a.x *= b;
    a.y *= b;
    return a;
}

// += two Vector2's
Vector2& operator+=(Vector2& a, const Vector2& b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}


// -= two Vector2's
Vector2& operator-=(Vector2& a, const Vector2& b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

// subtract two Vector2's
Vector2 operator-(const Vector2& a, const Vector2& b)
{
    return Vector2{a.x - b.x, a.y - b.y};
}

float dot(const Vector2& a, const Vector2& b)
{
    return a.x * b.x + a.y * b.y;
}

BezierSegment translate_bezier(BezierSegment* bz, Vector2 v)
{
    BezierSegment rv;
    rv.order = bz->order;
    for (int i = 0; i <= bz->order; i++)
        rv.p[i] = bz->p[i] + v;
    return rv;
}

BezierSegment move_bezier_to_origin(BezierSegment* bz) {
    if (!bz || bz->order != 3)
        return BezierSegment{3, {{0,0}, {0,0}, {0,0}, {0,0}}};

    return translate_bezier(bz, bz->p[0] * -1.f);
}

BezierSegment scale_bezier(BezierSegment* bz, float s) {
    if (!bz)
        return BezierSegment{0, {{0,0}, {0,0}, {0,0}, {0,0}}};

    BezierSegment rv;
    rv.order = bz->order;
    for (int i = 0; i <= bz->order; i++)
        rv.p[i] = bz->p[i] * s;
    return rv;
}


// reference C++ not used
bool split_bezier2(const BezierSegment* bz, float t, BezierSegment* r1, BezierSegment* r2)
{
    if (!bz || !r1 || !r2 || bz->order != 3)
        return false;

    /// @TODO for order 2

    if (t <= 0.f || t >= 1.f) {
        return false;
    }

    Vector2 p[4] = { bz->p[0], bz->p[1], bz->p[2], bz->p[3] };

    Vector2 Q0 = p[0];
    Vector2 Q1 = (1 - t) * p[0] + t * p[1];
    Vector2 Q2 = (1 - t) * Q1 + t * ((1 - t) * p[1] + t * p[2]);
    Vector2 Q3 = (1 - t) * Q2 + t * ((1 - t) * ((1 - t) * p[1] + t * p[2]) + t * ((1 - t) * p[2] + t * p[3]));

    Vector2 R0 = Q3;
    Vector2 R2 = (1 - t) * p[2] + t * p[3];
    Vector2 R1 = (1 - t) * ((1 - t) * p[1] + t * p[2]) + t * R2;
    Vector2 R3 = p[3];

    r1->order = 3;
    r1->p[0] = Q0;
    r1->p[1] = Q1;
    r1->p[2] = Q2;
    r1->p[3] = Q3;

    r2->order = 3;
    r2->p[0] = R0;
    r2->p[1] = R1;
    r2->p[2] = R2;
    r2->p[3] = R3;

    return true;
}

//
// Given x in the interval [0, p3], and a monotonically nondecreasing
// 1-D Bezier curve, B(u), with control points (0, p1, p2, p3), find
// u so that B(u) == x.
//

// evaluate a 1d bezier whose first point is 0.
float _bezier0(float unorm, float p2, float p3, float p4)
{
    const float p1 = 0.0;
    const float z = unorm;
    const float z2 = z*z;
    const float z3 = z2*z;

    const float zmo = z-1.0;
    const float zmo2 = zmo*zmo;
    const float zmo3 = zmo2*zmo;

    return (p4 * z3) 
        - (p3 * (3.0*z2*zmo))
        + (p2 * (3.0*z*zmo2))
        - (p1 * zmo3);
}


float _findU(float x, float p1, float p2, float p3)
{
    const float MAX_ABS_ERROR = FLT_EPSILON * 2.0;
    const int MAX_ITERATIONS = 45;
    
    if (x <= 0) {
        return 0;
    }
    
    if (x >= p3) {
        return 1;
    }
    
    float _u1 = 0;
    float _u2 = 0;
    float x1 = -x; // same as: bezier0 (0, p1, p2, p3) - x;
    float x2 = p3 - x; // same as: bezier0 (1, p1, p2, p3) - x;
    
    {
        const float _u3 = 1.0 - x2 / (x2 - x1);
        const float x3 = _bezier0(_u3, p1, p2, p3) - x;
        
        if (x3 == 0)
            return _u3;
        
        if (x3 < 0)
        {
            if (1.0 - _u3 <= MAX_ABS_ERROR) {
                if (x2 < -x3)
                    return 1.0;
                return _u3;
            }
            
            _u1 = 1.0;
            x1 = x2;
        }
        else
        {
            _u1 = 0.0;
            x1 = x1 * x2 / (x2 + x3);
            
            if (_u3 <= MAX_ABS_ERROR) {
                if (-x1 < x3)
                    return 0.0;
                return _u3;
            }
        }
        _u2 = _u3;
        x2 = x3;
    }
    
    int i = MAX_ITERATIONS - 1;
    
    while (i > 0)
    {
        i -= 1;
        const float _u3 = _u2 - x2 * ((_u2 - _u1) / (x2 - x1));
        const float x3 = _bezier0 (_u3, p1, p2, p3) - x;
        
        if (x3 == 0)
            return _u3;
        
        if (x2 * x3 <= 0)
        {
            _u1 = _u2;
            x1 = x2;
        }
        else
        {
            x1 = x1 * x2 / (x2 + x3);
        }

        _u2 = _u3;
        x2 = x3;

        if (_u2 > _u1)
        {
            if (_u2 - _u1 <= MAX_ABS_ERROR)
                break;
        }
        else
        {
            if (_u1 - _u2 <= MAX_ABS_ERROR)
                break;
        }
    }

    if (x1 < 0)
        x1 = -x1;
    if (x2 < 0)
        x2 = -x2;

    if (x1 < x2)
        return _u1;
    return _u2;
}

// given a x coordinate, find the corresponding u
float find_u(BezierSegment* b, float t) {
    float v[4];
    for (int i = 0; i <= b->order; ++i) {
        v[i] = b->p[i].x - b->p[0].x;
    }
    float tp = t - b->p[0].x;
    float u = _findU(tp, v[1], v[2], v[3]);
    return u;
}


// Draw line using cubic-bezier curves in-out
void DrawLineBezierx(Vector2 origin, BezierSegment* b, int steps, float thick, Color color)
{
    if (!b)
        return;
    
    for (int i = 0; i < steps; ++i) {
        float u = (float)i / (float)steps;
        float v = (float)(i+1) / (float)steps;
        Vector2 p0 = evaluate_bezier(b, u);
        p0.y *= -1;
        p0 += origin;
        Vector2 p1 = evaluate_bezier(b, v);
        p1.y *= -1;
        p1 += origin;
        DrawLineEx(p0, p1, thick, color);
    }
}

// Draw line using cubic-bezier curves in-out
void DrawLineBezierRotated(Vector2 origin, BezierSegment* b, int steps, float thick, Color color)
{
    if (!b)
        return;

    // swap x,y, negate y
    BezierSegment r = *b;
    for (int i = 0; i < 4; ++i) {
        float t = r.p[i].x;
        r.p[i].x = -r.p[i].y;
        r.p[i].y = -t;
    }

    for (int i = 0; i < steps; ++i) {
        float u = (float)i / (float)steps;
        float v = (float)(i+1) / (float)steps;
        Vector2 p0 = evaluate_bezier(&r, u) + origin;
        Vector2 p1 = evaluate_bezier(&r, v) + origin;
        DrawLineEx(p0, p1, thick, color);
    }
}



/// @class CubicInit
/// @brief Initialization parameters to create a cubic curve with start and
///        end y-values and derivatives.
/// Start is x = 0. End is x = width_x.
struct CubicInit {
  CubicInit(const float start_y, const float start_derivative,
            const float end_y, const float end_derivative, const float width_x)
      : start_y(start_y),
        start_derivative(start_derivative),
        end_y(end_y),
        end_derivative(end_derivative),
        width_x(width_x) {}

  // Short-form in comments:
  float start_y;           // y0
  float start_derivative;  // s0
  float end_y;             // y1
  float end_derivative;    // s1
  float width_x;           // w
};


/// @class CubicCurve
/// @brief Represent a cubic polynomial of the form,
///   c_[3] * x^3  +  c_[2] * x^2  +  c_[1] * x  +  c_[0]
class CubicCurve {
 public:
  static const int kNumCoeff = 4;
  CubicCurve() {
    for (int i = 0; i < kNumCoeff; ++i)
      c_[i] = 0.f;
  }

  CubicCurve(const float c3, const float c2, const float c1, const float c0) {
    c_[3] = c3;
    c_[2] = c2;
    c_[1] = c1;
    c_[0] = c0;
  }
  CubicCurve(const float* c) {
    for (int i = 0; i < kNumCoeff; ++i)
        c_[i] = c[i];
  }

  CubicCurve(const CubicInit& init) { Init(init); }
  void Init(const CubicInit& init);

  /// Shift the curve along the x-axis: x_shift to the left.
  /// That is x_shift becomes the curve's x=0.
  void ShiftLeft(const float x_shift);

  /// Shift the curve along the x-axis: x_shift to the right.
  void ShiftRight(const float x_shift) { ShiftLeft(-x_shift); }

  /// Shift the curve along the y-axis by y_offset: y_offset up the y-axis.
  void ShiftUp(float y_offset) { c_[0] += y_offset; }

  /// Scale the curve along the y-axis by a factor of y_scale.
  void ScaleUp(float y_scale) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] *= y_scale;
    }
  }

  /// Return the cubic function's value at `x`.
  /// f(x) = c3*x^3 + c2*x^2 + c1*x + c0
  float Evaluate(const float x) const {
    /// Take advantage of multiply-and-add instructions that are common on FPUs.
    return ((c_[3] * x + c_[2]) * x + c_[1]) * x + c_[0];
  }

  /// Return the cubic function's slope at `x`.
  /// f'(x) = 3*c3*x^2 + 2*c2*x + c1
  float Derivative(const float x) const {
    return (3.0f * c_[3] * x + 2.0f * c_[2]) * x + c_[1];
  }

  /// Return the cubic function's second derivative at `x`.
  /// f''(x) = 6*c3*x + 2*c2
  float SecondDerivative(const float x) const {
    return 6.0f * c_[3] * x + 2.0f * c_[2];
  }

  /// Return the cubic function's constant third derivative.
  /// Even though `x` is unused, we pass it in for consistency with other
  /// curve classes.
  /// f'''(x) = 6*c3
  float ThirdDerivative(const float x) const {
    (void)x;
    return 6.0f * c_[3];
  }
/*
  /// Returns true if always curving upward or always curving downward on the
  /// specified x_limits.
  /// That is, returns true if the second derivative has the same sign over
  /// all of x_limits.
  bool UniformCurvature(const Range& x_limits) const;

  /// Return a value below which floating point precision is unreliable.
  /// If we're testing for zero, for instance, we should test against this
  /// Epsilon().
  float Epsilon() const {
    using std::max;
    using std::fabs;
    const float max_c =
        max(max(max(fabs(c_[3]), fabs(c_[2])), fabs(c_[1])), fabs(c_[0]));
    return max_c * kEpsilonScale;
  }*/

  /// Returns the coefficient for x to the ith power.
  float Coeff(int i) const { return c_[i]; }

  /// Overrides the coefficent for x to the ith power.
  void SetCoeff(int i, float coeff) { c_[i] = coeff; }

  /// Returns the number of coefficients in this curve.
  int NumCoeff() const { return kNumCoeff; }

  /// Equality. Checks for exact match. Useful for testing.
  bool operator==(const CubicCurve& rhs) const;
  bool operator!=(const CubicCurve& rhs) const { return !operator==(rhs); }



 private:
  float c_[kNumCoeff];  /// c_[3] * x^3  +  c_[2] * x^2  +  c_[1] * x  +  c_[0]
};

void CubicCurve::Init(const CubicInit& init) {
  //  f(x) = dx^3 + cx^2 + bx + a
  //
  // Solve for a and b by substituting with x = 0.
  //  y0 = f(0) = a
  //  s0 = f'(0) = b
  //
  // Solve for c and d by substituting with x = init.width_x = w. Gives two
  // linear equations with unknowns 'c' and 'd'.
  //  y1 = f(x1) = dw^3 + cw^2 + bw + a
  //  s1 = f'(x1) = 3dw^2 + 2cw + b
  //    ==> 3*y1 - w*s1 = (3dw^3 + 3cw^2 + 3bw + 3a) - (3dw^3 + 2cw^2 + bw)
  //        3*y1 - w*s1 = cw^2 - 2bw + 3a
  //               cw^2 = 3*y1 - w*s1 + 2bw - 3a
  //               cw^2 = 3*y1 - w*s1 + 2*s0*w - 3*y0
  //               cw^2 = 3(y1 - y0) - w*(s1 + 2*s0)
  //                  c = (3/w^2)*(y1 - y0) - (1/w)*(s1 + 2*s0)
  //    ==> 2*y1 - w*s1 = (2dw^3 + 2cw^2 + 2bw + 2a) - (3dw^3 + 2cw^2 + bw)
  //        2*y1 - w*s1 = -dw^3 + bw + 2a
  //               dw^3 = -2*y1 + w*s1 + bw + 2a
  //               dw^3 = -2*y1 + w*s1 + s0*w + 2*y0
  //               dw^3 = 2(y0 - y1) + w*(s1 + s0)
  //                  d = (2/w^3)*(y0 - y1) + (1/w^2)*(s1 + s0)
  const float one_over_w = init.width_x > 0.f ? (1.0f / init.width_x) : 1.f;
  const float one_over_w_sq = one_over_w * one_over_w;
  const float one_over_w_cubed = one_over_w_sq * one_over_w;
  c_[0] = init.start_y;
  c_[1] = init.width_x > 0.f ? init.start_derivative : 0.f;
  c_[2] = 3.0f * one_over_w_sq * (init.end_y - init.start_y) -
          one_over_w * (init.end_derivative + 2.0f * init.start_derivative);
  c_[3] = 2.0f * one_over_w_cubed * (init.start_y - init.end_y) +
          one_over_w_sq * (init.end_derivative + init.start_derivative);
}

void RunGym(int screenWidth, int screenHeight) {
    Vector2 start = { screenWidth * 0.25f, screenHeight * 0.75f };
    Vector2 end = { screenWidth * 0.75f, screenHeight * 0.75f };
    Vector2 p1 = start;
    p1.y -= screenHeight * 0.5f;
    Vector2 p2 = end;
    p2.y -= screenHeight * 0.5f;
    
    // Vector2 start = { screenWidth * 0.75f, screenHeight * 0.15f };
    // Vector2 end = { screenWidth * 0.75f, screenHeight * 0.85f };

    // Vector2 p1 = start;
    // p1.y += screenHeight * 0.15f;
    // p1.x += 150;
    // p1.x = (start.x + end.x) * 0.5f;
    // p1.y -= 30;
    // Vector2 p2 = end;
    // p2.y -= screenHeight * 0.15f;
    // p2.x -= 100;
    // p2.y += 30;
    // p2.x = (start.x + end.x) * 0.5f;

    //--------------------------------------------------------------------------------------

    bool dragging = false;
    bool mouseDown = false;
    int selected = -1;
    
    bool draw_normals = false;
    bool draw_roots = true;
    bool draw_inflections = true;
    bool draw_approx = true;
    bool draw_split = true;
    bool draw_curve = false;
    
    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            mouseDown = true;
        }
        else {
            dragging = false;
            selected = -1;
        }
        Vector2 mousePos = GetMousePosition();
        if (!dragging) {
            draw_curve = GuiCheckBox((Rectangle){ 20, 170, 20, 20 }, "Draw curve", draw_curve);
            draw_split = GuiCheckBox((Rectangle){ 20, 200, 20, 20 }, "Draw split", draw_split);
            draw_approx = GuiCheckBox((Rectangle){ 20, 230, 20, 20 }, "Draw approximation", draw_approx);
            draw_inflections = GuiCheckBox((Rectangle){ 20, 260, 20, 20 }, "Draw Inflections", draw_inflections);
            draw_normals = GuiCheckBox((Rectangle){ 20, 290, 20, 20 }, "Draw Normals", draw_normals);
            draw_roots = GuiCheckBox((Rectangle){ 20, 320, 20, 20 }, "Draw Roots", draw_roots);
        }
        //----------------------------------------------------------------------------------
        BezierSegment b = {3, start, p1, p2, end};
        BezierSegment h = compute_hodograph(&b);
        BezierSegment h2 = compute_hodograph(&h);
        Vector2 inflections = inflection_points(&b);
        Vector2 roots = bezier_roots(&h);

        float split1 = roots.x;
        if (split1 == -1) {
            split1 = inflections.x;
        }
        if (inflections.x > 0 && inflections.x < split1)
            split1 = inflections.x;

        BezierSegment s1;
        BezierSegment s2;
        if (split1 > 0)
            split_bezier(&b, split1, &s1, &s2);
        else
            draw_split = false;
        
        BezierSegment b0 = move_bezier_to_origin(&s1);
        BezierSegment h0 = compute_hodograph(&b0);
        // start dstart end dend, width
        float run_left    = b0.p[1].x - b0.p[0].x;
        float rise_left   = b0.p[1].y - b0.p[0].y;
        float slope_left  = rise_left / run_left;
        float run_right   = b0.p[3].x - b0.p[2].x;
        float rise_right  = b0.p[3].y - b0.p[2].y;
        float slope_right = rise_right / run_right;

        float cubic_width = b0.p[3].x - b0.p[0].x;
        CubicInit ci_x(b0.p[0].y, slope_left, b0.p[3].y, slope_right, cubic_width);
        CubicCurve cubic_x(ci_x);

        Vector2 origin = { 0, 0 };

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            
            DrawText("BEZIER DEMONSTRATOR", 15, 20, 20, GRAY);
            
            const int steps = 100;
            if (draw_split) {
                DrawLineBezierx(origin, &s1, steps, 2.0f, RED);
                DrawLineBezierx(origin, &s2, steps, 2.0f, DARKBROWN);
            }

            if (draw_inflections) {
                if (inflections.x > 0) {
                    Vector2 p = evaluate_bezier(&b, inflections.x);
                    DrawCircle(p.x, p.y, 5, RED);
                }
                if (inflections.y > 0) {
                    Vector2 p = evaluate_bezier(&b, inflections.y);
                    DrawCircle(p.x, p.y, 5, RED);
                }
            }

            if (draw_approx) {
                for (float x = 0; x < cubic_width; x += 2) {
                    float y = cubic_x.Evaluate(x);
                    DrawPixel(b.p[0].x + x, b.p[0].y + y, BLACK);
                }
            }

            Vector2* points[4] = { &b.p[0], &b.p[1], &b.p[2], &b.p[3] };
            
            int closest = -1;
            if (selected >= 0) {
                *(points[selected]) = mousePos;
                switch(selected) {
                    case 0:
                        start = mousePos;
                        break;
                    case 1:
                        p1 = mousePos; break;
                    case 2:
                        p2 = mousePos; break;
                    case 3:
                        end = mousePos; break;
                }
            }
            else {
                float closestDist = 1000000;
                for (int i = 0; i < 4; ++i) {
                    Vector2 dp = mousePos - *points[i];
                    float d2 = dot(dp, dp);
                    if (d2 < closestDist) {
                        closestDist = d2;
                        closest = i;
                    }
                }
                if (closestDist < 100 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    selected = closest;
                    *(points[closest]) = mousePos;
                    switch(closest) {
                        case 0:
                            start = mousePos;
                            break;
                        case 1:
                            p1 = mousePos; break;
                        case 2:
                            p2 = mousePos; break;
                        case 3:
                            end = mousePos; break;
                    }
                }
            }

            if (draw_normals) {
                for (int i = 0; i < steps; ++i) {
                    float u = (float)i / (float)20;
                    Vector2 p0 = evaluate_bezier(&h, u);
                    Vector2 b0 = evaluate_bezier(&b, u);
                    p0 += b0;
                    DrawLineEx(b0, p0, 2.0f, BLUE);
                    //DrawLineEx(b0, p1, 2.0f, BLUE);
                }
            }
            
            DrawLineEx(b.p[0], b.p[1], 2.f, GREEN);
            DrawLineEx(b.p[3], b.p[2], 2.f, GREEN);
            for (int i = 0; i < 4; ++i)
                DrawRing(*points[i], 2, 6, 0, 360, 16, (closest >= 0)? RED : GREEN);

            if (draw_curve)
                DrawLineBezierx(origin, &b, steps, 2.0f, RED);
            
            if (draw_roots) {
                Vector2 root = bezier_roots(&h);
                if (root.x >= 0.f) {
                    Vector2 r = evaluate_bezier(&b, root.x);
                    DrawRing(r, 2, 6, 0, 360, 16, DARKGREEN);
                }
                if (root.y >= 0.f) {
                    Vector2 r = evaluate_bezier(&b, root.y);
                    DrawRing(r, 2, 6, 0, 360, 16, DARKGREEN);
                }
            }
/*            if (draw_inflections) {
                Vector2 inflection = bezier_roots(&h2);
                if (inflection.x >= 0.f) {
                    Vector2 r = evaluate_bezier(&b, inflection);
                    //DrawRing(r, 2, 6, 0, 360, 16, DARKBLUE);
                }
            }*/
        }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}

void DrawNormals(Vector2 origin, BezierSegment* b, BezierSegment* h, int steps) {
    for (int i = 0; i < steps; ++i) {
        float u = (float)i / (float)steps;
        Vector2 p0 = evaluate_bezier(h, u);
        Vector2 b0 = evaluate_bezier(b, u);
        p0 += b0;
        DrawLineEx(b0 + origin, p0 + origin, 2.0f, BLUE);
    }
}

void DrawControls(Vector2 origin, BezierSegment* b, int closest) {
    Vector2 p0 = b->p[0];
    p0.y *= -1.f;
    Vector2 p1 = b->p[1];
    p1.y *= -1.f;
    DrawLineEx(p0 + origin, p1 + origin, 2.f, GREEN);
    p0 = b->p[b->order];
    p0.y *= -1.f;
    p1 = b->p[b->order - 1];
    p1.y *= -1.f;
    DrawLineEx(p0 + origin, p1 + origin, 2.f, GREEN);
    for (int i = 0; i <= b->order; ++i) {
        p0 = b->p[i];
        p0.y *= -1.f;
        DrawRing(p0 + origin, 2, 6, 0, 360, 16, (closest >= 0)? RED : GREEN);
    }
    
    DrawRing(origin, 4, 8, 0, 360, 16, RED);
}

void Swap(float& a, float& b) {
    float t = a;
    a = b;
    b = t;
}

void DrawBounds(Vector2 origin, Vector2 bmin, Vector2 bmax) {
    float y = -bmin.y + origin.y;
    float left = 50; // bmin.x - 20 + origin.x;
    DrawLineEx((Vector2){left, y}, (Vector2){bmax.x + 20 + origin.x, y}, 2.f, DARKGREEN);
    y = -bmax.y + origin.y;
    DrawLineEx((Vector2){left, y}, (Vector2){bmax.x + 20 + origin.x, y}, 2.f, DARKGREEN);
    float x = bmin.x + origin.x;
    DrawLineEx((Vector2){x, -bmin.y + 20 + origin.y}, (Vector2){x, -bmax.y - 20 + origin.y}, 2.f, DARKGREEN);
    x = bmax.x + origin.x;
    DrawLineEx((Vector2){x, -bmin.y + 20 + origin.y}, (Vector2){x, -bmax.y - 20 + origin.y}, 2.f, DARKGREEN);
}

void RunProjector(int screenWidth, int screenHeight) {
    Vector2 start = { 0, 0 };
    Vector2 end = { screenWidth * 0.5f, 0 };
    Vector2 p1 = start;
    p1.y += screenHeight * 0.5f;
    Vector2 p2 = end;
    p2.y += screenHeight * 0.5f;
    
    //--------------------------------------------------------------------------------------

    bool dragging = false;
    bool mouseDown = false;
    
    bool draw_normals = false;
    bool draw_roots = true;
    bool draw_inflections = true;
    bool draw_approx = true;
    bool draw_split = true;
    bool draw_curve = false;
    
    Vector2 origin = { screenWidth * 0.25f, screenHeight * 0.80f };
    
    BezierSegment b1 = {3, start, p1, p2, end};
    BezierSegment b2 = {3, (Vector2){0, 0}, (Vector2){20, 20}, (Vector2){50, 50}, (Vector2) {200, 200}};
    
    BezierSegment* bwork[2] = { &b1, & b2 };
    
    int selected_curve = 0;
    int selected_point = -1;
    bool selected_origin = false;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        //----------------------------------------------------------------------------------
        // mouse and GUI
        //
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            mouseDown = true;
        }
        else {
            dragging = false;
            selected_point = -1;
            selected_origin = false;
        }
        Vector2 mousePos = GetMousePosition();
        Vector2 localMousePos = mousePos - origin;
        localMousePos.y *= -1.f;

        if (!dragging) {
            selected_curve =   GuiToggleGroup((Rectangle){ 20, 120, 20, 40 }, "from;to", selected_curve);
            draw_curve =       GuiCheckBox((Rectangle){ 20, 170, 20, 20 }, "Draw curve", draw_curve);
            draw_split =       GuiCheckBox((Rectangle){ 20, 200, 20, 20 }, "Draw split", draw_split);
            draw_approx =      GuiCheckBox((Rectangle){ 20, 230, 20, 20 }, "Draw approximation", draw_approx);
            draw_inflections = GuiCheckBox((Rectangle){ 20, 260, 20, 20 }, "Draw Inflections", draw_inflections);
            draw_normals =     GuiCheckBox((Rectangle){ 20, 290, 20, 20 }, "Draw Normals", draw_normals);
            draw_roots =       GuiCheckBox((Rectangle){ 20, 320, 20, 20 }, "Draw Roots", draw_roots);
        }
        BezierSegment* b = bwork[selected_curve];

        //----------------------------------------------------------------------------------
        // manage selecting a point using the mouse
        const int steps = 100;
        int closest = -1;
        if (selected_point >= 0) {
            b->p[selected_point] = localMousePos;
        }
        else if (selected_origin) {
            origin = mousePos;
            selected_point = -1;
        }
        else {
            float closestDist = 1000000;
            for (int i = 0; i < 4; ++i) {
                Vector2 dp = localMousePos - b->p[i];
                float d2 = dot(dp, dp);
                if (d2 < closestDist) {
                    closestDist = d2;
                    closest = i;
                }
            }
            if (closestDist < 100 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                selected_point = closest;
                b->p[closest] = localMousePos;
                selected_origin = false;
            }
        }
        
        if (!selected_origin && (selected_point < 0) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            float d2 = dot(localMousePos, localMousePos);
            selected_origin = d2 < 100;
        }

        //----------------------------------------------------------------------------------
        // compute derivatives
        BezierSegment h = compute_hodograph(b);
        BezierSegment h2 = compute_hodograph(&h);
        Vector2 inflections = inflection_points(b);
        Vector2 roots = bezier_roots(&h);
        
        //----------------------------------------------------------------------------------
        // compute splits
        float splits[3] = { 1.f, 1.f, 1.f };
        int split_count = 0;
        if (roots.x > 0.f && roots.x < 1.f) {
            splits[split_count] = roots.x;
            split_count++;
        }
        if (roots.y > 0.f && roots.y < 1.f) {
            bool duplicate = false;
            for (int i = 0; i < split_count; ++i)
                duplicate |= splits[i] == roots.y;
            if (!duplicate) {
                splits[split_count] = roots.y;
                split_count++;
            }
        }
        if (inflections.x > 0.f && inflections.x < 1.f) {
            bool duplicate = false;
            for (int i = 0; i < split_count; ++i)
                duplicate |= splits[i] == inflections.x;
            if (!duplicate) {
                splits[split_count] = inflections.x;
                split_count++;
            }
        }
        for (int i = 0; i < split_count - 1; i++)
            if (splits[i] > splits[i + 1])
                Swap(splits[i], splits[i + 1]);

        // make only the first split
        BezierSegment s1;
        BezierSegment s2;
        if (split_count > 0)
            split_bezier(b, splits[0], &s1, &s2);
        
        //----------------------------------------------------------------------------------
        // compute bounds
        Vector2 bound_min = { b1.p[0].x, b2.p[0].x };
        Vector2 bound_max = { b1.p[b1.order].x, b2.p[b2.order].x };

        // shrink the bound by knot extremal values
        float knot_min_y = b1.p[0].y < b1.p[b1.order].y ? b1.p[0].y : b1.p[b1.order].y;
        float knot_max_y =  b1.p[0].y > b1.p[b1.order].y ? b1.p[0].y : b1.p[b1.order].y;
        for (int i = 0; i < split_count; ++i) {
            float y = evaluate_bezier(&b1, splits[i]).y;
            if (y > knot_max_y)
                knot_max_y = y;
            if (y < knot_min_y)
                knot_min_y = y;
        }
        if (knot_min_y > bound_min.y)
            bound_min.y = knot_min_y;
        if (knot_max_y < bound_max.y)
            bound_max.y = knot_max_y;
        
        //----------------------------------------------------------------------------------
        // compute approximation
        BezierSegment b0 = move_bezier_to_origin(&s1);
        BezierSegment h0 = compute_hodograph(&b0);
        float run_left    = b0.p[1].x - b0.p[0].x;
        float rise_left   = b0.p[1].y - b0.p[0].y;
        float slope_left  = rise_left / run_left;
        float run_right   = b0.p[3].x - b0.p[2].x;
        float rise_right  = b0.p[3].y - b0.p[2].y;
        float slope_right = rise_right / run_right;
        float cubic_width = b0.p[3].x - b0.p[0].x;
        CubicInit ci_x(b0.p[0].y, slope_left, b0.p[3].y, slope_right, cubic_width);
        CubicCurve cubic_x(ci_x);

        //----------------------------------------------------------------------------------
        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            DrawText("BEZIER DEMONSTRATOR", 15, 20, 20, GRAY);
            
            // axes
            float oy = origin.y;
            DrawLine(0, oy, screenWidth, oy, BLACK);
            DrawLine(origin.x, 0, origin.x, screenHeight, BLACK);
            
            if (draw_normals) {
                DrawNormals(origin, b, &h, steps);
            }

            if (draw_curve) {
                DrawLineBezierx(origin, &b1, steps, 2.0f, RED);
            }
            // always draw the projectee curve
            DrawLineBezierRotated(origin, &b2, steps, 2.0f, RED);

            if (draw_roots) {
                Vector2 root = bezier_roots(&h);
                if (root.x >= 0.f) {
                    Vector2 r = evaluate_bezier(b, root.x);
                    r.y *= -1.f;
                    DrawRing(r + origin, 2, 6, 0, 360, 16, DARKGREEN);
                }
                if (root.y >= 0.f) {
                    Vector2 r = evaluate_bezier(b, root.y);
                    r.y *= -1.f;
                    DrawRing(r + origin, 2, 6, 0, 360, 16, DARKGREEN);
                }
            }
            
            if (draw_split && split_count > 0) {
                DrawLineBezierx(origin, &s1, steps, 2.0f, RED);
                DrawLineBezierx(origin, &s2, steps, 2.0f, DARKBROWN);
            }

            if (draw_inflections) {
                if (inflections.x > 0) {
                    Vector2 p = evaluate_bezier(b, inflections.x);
                    p.y *= -1;
                    DrawCircle(p.x + origin.x, p.y + origin.y, 5, RED);
                }
                if (inflections.y > 0) {
                    Vector2 p = evaluate_bezier(b, inflections.y);
                    p.y *= -1;
                    DrawCircle(p.x + origin.x, p.y + origin.y, 5, RED);
                }
            }

            if (draw_approx) {
                for (float x = 0; x < cubic_width; x += 2) {
                    float y = cubic_x.Evaluate(x);
                    DrawPixel(b->p[0].x + x + origin.x, -b->p[0].y - y + origin.y, BLACK);
                }
            }
            
            if (localMousePos.x >= bound_min.x && localMousePos.x <= bound_max.x) {
                float u = find_u(&b1, localMousePos.x);
                float y = evaluate_bezier(&b1, u).y;
                DrawLineEx((Vector2){localMousePos.x, 0} + origin,
                           (Vector2){localMousePos.x, -y} + origin, 2, GRAY);

                Vector2 labelPos = {localMousePos.x - 16, 10};
                labelPos += origin;
                char buff[32];
                snprintf(buff, 31, "%2.2f", y);
                DrawText(buff, labelPos.x, labelPos.y, 15, GRAY);

                
                if (y >= bound_min.y && y <= bound_max.y) {
                    u = find_u(&b2, y);
                    float y2 = evaluate_bezier(&b2, u).y;
                    DrawLineEx((Vector2){localMousePos.x, -y} + origin,
                               (Vector2){-y2, -y} + origin, 2, GRAY);
                    DrawLineEx((Vector2){-y2, 0} + origin,
                               (Vector2){-y2, -y} + origin, 2, GRAY);

                    labelPos = {-y2 - 16, 10};
                    labelPos += origin;
                    char buff[32];
                    snprintf(buff, 31, "%2.2f", y2);
                    DrawText(buff, labelPos.x, labelPos.y, 15, GRAY);

                }
            }

            DrawBounds(origin, bound_min, bound_max);
            DrawControls(origin, b, closest);
        }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int mainx(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1000;
    const int screenHeight = 600;

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "raylib [shapes] example - cubic-bezier lines");

    //RunGym(screenWidth, screenHeight);
    RunProjector(screenWidth, screenHeight);

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return EXIT_SUCCESS;
}
