#pragma once

#include "MTR/core.h"
#include "MTR/math_defs.h"

struct A2V {
  vec3f position;
  // vec2f tex_coords;
};

struct V2F {
  // vec4f screen_pos;
  // vec3f view_pos;
  // vec2f tex_coords;
  // vec4f worldpos;
  // vec4f viewpos;
  vec4f color;
};

// virtual class for payload(vertex, fragment) struct reflection
template <class Arr2Vert, class Vert2Frag>
class IShader {
 public:
  static Arr2Vert arr2vert_t() { return Arr2Vert(); };
  static Vert2Frag vert2frag_t() { return Vert2Frag(); };

  // MTR_GENERATE_TUPLE_CONVERTION(arr2vert_cls);
  // MTR_GENERATE_TUPLE_CONVERTION(vert2frag_cls);

  // varients
  mat4f project;
  mat4f view;
  mat4f model;

  virtual vec4f vertex_shader(Arr2Vert &a2v, Vert2Frag &v2f) = 0;
  virtual vec4f fragment_shader(Vert2Frag &v2f) = 0;
  virtual Vert2Frag interpolate_attr(Vert2Frag *v2f, vec3f bc_weight) = 0;
  virtual std::string tag() { return "IShader"; };
};

template <class vecXf>
static vecXf bc_interpolate_attr(const vecXf &a, const vecXf &b, const vecXf &c,
                                 const vec3f &weight) {
  vecXf result;
  // for (int i = 0; i < a.rows(); i++) {
  //   result(i) = weight.dot(vec3f(a(i), b(i), c(i)));
  // }
  result = a * weight.x() + b * weight.y() + c * weight.z();
  return result;
}

class SoftShader : public IShader<A2V, V2F> {
 public:
  SoftShader() = default;

  virtual vec4f vertex_shader(A2V &a2v, V2F &v2f) override {
    vec4f position;
    position << a2v.position, 1.0f;
    v2f.color = position;
    // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();
    return project * model * view * position;
  }
  virtual vec4f fragment_shader(V2F &v2f) override { return v2f.color; }
  virtual V2F interpolate_attr(V2F *v2f, vec3f bc_weight) override {
    V2F result;
    // result.color = bc_interpolate_attr(v2f[0].color, v2f[1].color,
    // v2f[2].color, bc_weight);
    result.color = v2f[0].color * bc_weight(0) + v2f[1].color * bc_weight(1) +
                   v2f[2].color * bc_weight(2);
    return result;
  }
};