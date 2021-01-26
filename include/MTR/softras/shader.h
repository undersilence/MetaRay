#pragma once

#include "MTR/core.h"
#include "MTR/math_defs.h"

struct A2V {
  vec3f position;
  // mat4f project;
  // mat4f view;
  // mat4f model;
};

struct V2F {
  vec4f screen_pos;
  vec3f view_pos;
  // vec4f worldpos;
  // vec4f viewpos;
  vec4f color;
};

template <class Arr2Vert, class Vert2Frag> class IShader {
public:
  virtual vec4f vertex_shader(Arr2Vert &a2v, Vert2Frag &v2f) = 0;
  virtual vec4f fragment_shader(Vert2Frag &v2f) = 0;
};

class SoftShader : public IShader<A2V, V2F> {
public:
  // varients
  mat4f project;
  mat4f view;
  mat4f model;

  virtual vec4f vertex_shader(A2V &a2v, V2F &v2f) override;
  virtual vec4f fragment_shader(V2F &v2f) override;
};
