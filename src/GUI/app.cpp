
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <iostream>

#include "GUI/shader.h"
#include "MTR/softras/render.hpp"

struct Test_A2V {
  vec3f position;
  vec3f color;
};

struct Test_V2F {
  vec3f color;
};

class Test_Shader : public IShader<Test_A2V, Test_V2F> {
 public:
  vec3f offset;
  virtual vec4f vertex_shader(Test_A2V& a2v, Test_V2F& v2f) {
    v2f.color = a2v.color;
    // v2f.view_pos = (a2v.model * a2v.view * position).head<3>();

    vec4f position;
    position << a2v.position + offset, 1.0f;
    return project * model * view * position;
  }
  virtual vec4f fragment_shader(Test_V2F& v2f) {
    vec4f result;
    result << v2f.color, 1.0f;
    return result;
  }
  virtual Test_V2F interpolate_attr(Test_V2F* v2f, vec3f bc_weight) {
    Test_V2F result;
    result.color = bc_interpolate_attr(v2f[0].color, v2f[1].color, v2f[2].color,
                                       bc_weight);
    return result;
  }
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
char window_title[256] = {"MetaRay fps "};

int main() {
  int test_width = SCR_WIDTH, test_height = SCR_HEIGHT;
  auto shader = std::make_shared<Test_Shader>();
  auto raster = std::make_shared<SoftRaster<decltype(shader->arr2vert_cls),
                                            decltype(shader->vert2frag_cls)>>(
      test_width, test_height);
  raster->test();

  raster->set_shader(shader);
  raster->set_project(perspective(
      to_radians(60.0f), test_width / (float)test_height, 0.1f, 50.0f));

  std::vector<Test_A2V> vertex_data;
  Test_A2V a, b, c;
  a.position << 0.5f, 0.5f, 1.5f;
  a.color << 0.5f, 0.0f, 0.0f;
  b.position << -0.5f, 0.5f, 2.0f;
  b.color << 0.0f, 0.5f, 0.0f;
  c.position << 0.5f, -0.5f, 2.5f;
  c.color << 0.0f, 1.0f, 1.0f;
  vertex_data.insert(vertex_data.end(), {a, b, c});

  a.position << 0.5f, 0.5f, 1.0f;
  a.color << 0.1f, 0.0f, 0.0f;
  b.position << 0.0f, 0.5f, 1.0f;
  b.color << 0.0f, 0.1f, 0.0f;
  c.position << 0.5f, 0.0f, 1.5f;
  c.color << 0.0f, 0.0f, 1.0f;
  vertex_data.insert(vertex_data.end(), {a, b, c});

  auto buffer_id = raster->load_array(vertex_data);

  // glfw: initialize and configure
  // ------------------------------
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // glfw window creation
  // --------------------
  GLFWwindow* window =
      glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, window_title, NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSwapInterval(0);
  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // build and compile our shader zprogram
  // ------------------------------------
  Shader ourShader("../shaders/quad.vs", "../shaders/quad.fs");

  // set up vertex data (and buffer(s)) and configure vertex attributes
  // ------------------------------------------------------------------
  float vertices[] = {
      // positions       // texture coords
      1.0f,  1.0f,  0.0f, 1.0f, 1.0f,  // top right
      1.0f,  -1.0f, 0.0f, 1.0f, 0.0f,  // bottom right
      -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,  // bottom left
      -1.0f, 1.0f,  0.0f, 0.0f, 1.0f   // top left
  };
  unsigned int indices[] = {
      0, 1, 3,  // first triangle
      1, 2, 3   // second triangle
  };
  unsigned int VBO, VAO, EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
               GL_STATIC_DRAW);

  // position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  // texture coord attribute
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // load and create a texture
  // -------------------------
  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D,
                texture);  // all upcoming GL_TEXTURE_2D operations now have
                           // effect on this texture object
  // set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                  GL_REPEAT);  // set texture wrapping to GL_REPEAT (default
                               // wrapping method)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  // set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // load image, create texture and generate mipmaps
  // int width, height, nrChannels;
  // The FileSystem::getPath(...) is part of the GitHub repository so we can
  // find files on any IDE/platform; replace it with your own image path.

  // raster->draw_arrays(buffer_id);
  // auto& frame_buffer = raster->encode_frame_buffer();
  // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, test_width, test_height, 0,
  // GL_RGBA,
  //              GL_UNSIGNED_BYTE, frame_buffer.data());
  // glGenerateMipmap(GL_TEXTURE_2D);

  double lastTime = glfwGetTime();
  int nbFrames = 0;
  // render loop
  // -----------
  while (!glfwWindowShouldClose(window)) {
    double currentTime = glfwGetTime();
    nbFrames++;
    if (currentTime - lastTime >=
        1.0) {  // If last prinf() was more than 1 sec ago
      // printf and reset timer
      // printf("%f ms/frame\n", 1000.0 / double(nbFrames));
      glfwSetWindowTitle(window,
                         (window_title + std::to_string(nbFrames)).c_str());
      nbFrames = 0;
      lastTime += 1.0;
    }

    // input
    // -----
    processInput(window);

    // render
    // ------
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    float curr_time = glfwGetTime();
    shader->offset =
        vec3f(0.5f * sinf(curr_time), 0.5f * cosf(curr_time), 0.0f);
    raster->clear(SoftRaster<Test_A2V, Test_V2F>::Buffer::Color |
                  SoftRaster<Test_A2V, Test_V2F>::Buffer::Depth);
    raster->draw_arrays(buffer_id);
    // frame_buffer = raster->encode_frame_buffer();

    // bind Texture
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, test_width, test_height, 0, GL_RGBA,
                 GL_FLOAT, raster->frame_buffer().data());

    // render container
    ourShader.use();
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved
    // etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &EBO);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this
// frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback
// function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  // make sure the viewport matches the new window dimensions; note that width
  // and height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}