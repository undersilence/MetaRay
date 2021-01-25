#include "MTR/softras/shader.h"

SoftShader::V2F SoftShader::vertex_shader(SoftShader::A2V &a2v) {
  printf("hello from vertex_shader\n");
  return {};
}

vec4f SoftShader::fragment_shader(SoftShader::V2F &v2f) {
  printf("hello from fragment_shader\n");
  return vec4f();
}

vec4f simple_vertex_shader(A2V &a2v, V2F &v2f) {
  vec4f position;
  position << a2v.position, 1.0f;
  v2f.color = a2v.position;
  return a2v.project * a2v.model * a2v.view * position;
}

vec3f simple_fragment_shader(V2F &v2f) { return v2f.color; }
