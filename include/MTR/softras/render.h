#include "MTR/core.h"
#include "MTR/math_defs.h"
#include "MTR/softras/shader.h"

template <class TApp2Vert, class TVert2Frag>
class SoftRaster {
 public:
  enum Primitive { Line, Triangle };
  enum Buffer { Color = 1, Depth = 2 };
  struct arr_buf_id {
    int arr_id = 0;
  };
  struct ind_buf_id {
    int ind_id = 0;
  };

  explicit SoftRaster(int w, int h);
  void draw_arrays(arr_buf_id arr_id, Primitive mode = Primitive::Triangle);
  void draw_elements(ind_buf_id buf_id, Primitive mode = Primitive::Triangle);
  void clear(Buffer buffer);

  // void set_shader(const std::shared_ptr<SoftShader>& shader);
  // void set_vertex_shader(const std::function<vec4f(A2V &, V2F &)>
  // &VertexShader); void set_fragment_shader(const std::function<vec4f(V2F &)>
  // &FragmentShader);
  void set_shader(
      const std::shared_ptr<IShader<TApp2Vert, TVert2Frag>> &_shader);

  // arr_buf_id load_array(const std::vector<float> &arr);
  arr_buf_id load_array(const std::vector<TApp2Vert> &vertex_arr);
  ind_buf_id load_indices(const std::vector<int> &indices);
  int get_next_id();
  int get_index(int x, int y);

  void set_model(const mat4f &m);
  void set_view(const mat4f &v);
  void set_project(const mat4f &p);
  void set_pixel(const vec2i &coord, const vec4f &color);

  std::vector<vec4f> &frame_buffer() { return frame_buf; }
  std::vector<float> &depth_buffer() { return depth_buf; }

  void test() { printf("hello from SoftRaster.\n"); }

 protected:
  int width, height;
  int next_id = 0;
  // std::shared_ptr<SoftShader> shader;
  std::shared_ptr<IShader<TApp2Vert, TVert2Frag>> shader;

  void draw_line(vec3f begin, vec3f end);
  void rasterize_triangle(TVert2Frag tri[3], vec4f clip_pos[3]);

  // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI ->
  // FRAGSHADER

  mat4f model = mat4f::Identity();
  mat4f view = mat4f::Identity();
  mat4f project = mat4f::Identity();

  // input buffer
  std::map<int, std::vector<decltype(shader->arr2vert_cls)>> arr_bufs;
  std::map<int, std::vector<int>> ind_bufs;

  // output buffer
  std::vector<vec4f> frame_buf;
  std::vector<float> depth_buf;
};