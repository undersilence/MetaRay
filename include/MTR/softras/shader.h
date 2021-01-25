#pragma once

#include "MTR/core.h"
#include "MTR/math_defs.h"

struct A2V {
  vec3f position;
  mat4f project;
  mat4f view;
  mat4f model;
};

struct V2F {
  vec4f screenpos;
  // vec4f worldpos;
  // vec4f viewpos;
  vec3f color;
};

class SoftShader {
public:
  struct A2V {
    vec4f pos;
  };
  struct V2F {
    vec4f color;
  };
  V2F vertex_shader(A2V &a2v);
  vec4f fragment_shader(V2F &v2f);

protected:
};

vec4f simple_vertex_shader(A2V &a2v, V2F &v2f);

vec3f simple_fragment_shader(V2F &v2f);