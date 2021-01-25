#include "MTR/core.h"
#include "MTR/math_defs.h"
#include "MTR/softras/shader.h"

class SoftRaster {
public:
  enum Primitive { Line, Triangle };
  struct arr_buf_id {
    int arr_id = 0;
  };
  struct ind_buf_id {
    int ind_id = 0;
  };

  explicit SoftRaster(int w, int h);
  void draw_arrays(Primitive mode = Primitive::Triangle);
  void draw_elements();

  // void set_shader(const std::shared_ptr<SoftShader>& shader);
  void set_vertex_shader(
      const std::function<vec4f(vertex_shader_payload &,
                                fragment_shader_payload &)> &VertexShader);
  void set_fragment_shader(
      const std::function<vec3f(fragment_shader_payload &)> &FragmentShader);

  arr_buf_id load_array(const std::vector<float> &arr);
  ind_buf_id load_indices(const std::vector<int> &indices);
  int get_next_id();

  void set_model(const mat4f &m);
  void set_view(const mat4f &v);
  void set_project(const mat4f &p);
  void set_pixel(const vec2i &coord, const vec3f &color);

  std::vector<Eigen::Vector3f> &frame_buffer() { return frame_buf; }
  std::vector<float> &depth_buffer() { return depth_buf; }

  void test() { printf("hello from SoftRaster.\n"); }

protected:
  int width, height;
  int next_id = 0;
  // std::shared_ptr<SoftShader> shader;
  std::function<vec4f(vertex_shader_payload &, fragment_shader_payload &)>
      vertex_shader;
  std::function<vec3f(fragment_shader_payload &)> fragment_shader;

  void draw_line(vec3f begin, vec3f end);
  void rasterize_triangle();

  // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI ->
  // FRAGSHADER

  mat4f model;
  mat4f view;
  mat4f project;

  // input buffer
  std::map<int, std::vector<float>> arr_bufs;
  std::map<int, std::vector<int>> ind_bufs;

  // output buffer
  std::vector<vec3f> frame_buf;
  std::vector<float> depth_buf;
};