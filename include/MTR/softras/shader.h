#pragma once

#include "MTR/core.h"
#include "MTR/math_defs.h"

struct vertex_shader_payload {
  vec3f position;
};

struct fragment_shader_payload {
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
