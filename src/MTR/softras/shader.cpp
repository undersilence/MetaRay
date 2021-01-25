#include "MTR/softras/shader.h"

SoftShader::V2F SoftShader::vertex_shader(SoftShader::A2V &a2v) {
  printf("hello from vertex_shader\n");
  return {};
}

vec4f SoftShader::fragment_shader(SoftShader::V2F &v2f) {
  printf("hello from fragment_shader\n");
  return vec4f();
}
