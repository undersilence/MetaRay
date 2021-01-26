#include "MTR/softras/shader.h"

vec4f SoftShader::vertex_shader(A2V &a2v, V2F &v2f) {
  vec4f position;
  position << a2v.position, 1.0f;
  v2f.color = position;
  // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();
  return project * model * view * position;
}

vec4f SoftShader::fragment_shader(V2F &v2f) { return v2f.color; }

// vec4f simple_vertex_shader(A2V &a2v, V2F &v2f) {
//   vec4f position;
//   position << a2v.position, 1.0f;
//   v2f.color = position;
//   // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();
//   return a2v.project * a2v.model * a2v.view * position;
// }

// vec4f simple_fragment_shader(V2F &v2f) { return v2f.color; }
