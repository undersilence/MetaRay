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
template <class App2Vert, class Vert2Frag>
class IShader {
 public:
  App2Vert arr2vert_cls;
  Vert2Frag vert2frag_cls;

  // MTR_GENERATE_TUPLE_CONVERTION(arr2vert_cls);
  // MTR_GENERATE_TUPLE_CONVERTION(vert2frag_cls);

  // varients
  mat4f project;
  mat4f view;
  mat4f model;

  virtual vec4f vertex_shader(App2Vert &a2v, Vert2Frag &v2f) = 0;
  virtual vec4f fragment_shader(Vert2Frag &v2f) = 0;
  virtual V2F interpolate_attr(V2F *v2f, vec3f bc_weight) = 0;
};

class SoftShader : public IShader<A2V, V2F> {
 public:
  SoftShader() = default;

  virtual vec4f vertex_shader(A2V &a2v, V2F &v2f) override;
  virtual vec4f fragment_shader(V2F &v2f) override;
  virtual V2F interpolate_attr(V2F *v2f, vec3f bc_weight) override;
};
