// Relevant SDL 2 example (with emscripten):
// http://hg.libsdl.org/SDL/file/e12c38730512/test/teststreaming.c
// Scratchpixel rasterizer tutorial
// https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "SDL.h"

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

// Color with components in [0, 1]
struct Color {
  Color() {}
  Color(double r, double g, double b) : r(r), g(g), b(b) {}
  double r = 0;
  double g = 0;
  double b = 0;

  Color operator*(double v) const { return Color(r * v, g * v, b * v); }

  Color operator*(const Color& c) const {
    return Color(r * c.r, g * c.g, b * c.b);
  }

  Color operator/(double v) const { return Color(r / v, g / v, b / v); }

  Color operator+(const Color& c) const {
    return Color(r + c.r, g + c.g, b + c.b);
  }
};

struct Vertex {
  Vertex() {}
  Vertex(const Vector3d& pos, const Color& col) : position(pos), color(col) {}
  Vertex(const Vector3d& pos, const Color& col, const Vector2d& uv)
      : position(pos), color(col), uv(uv) {}
  Vector3d position;
  Color color;
  Vector2d uv;
};

struct Transform {
  Transform() {
    translation.setZero();
    rotation.setIdentity();
  }
  Vector3d translation;
  Quaterniond rotation;

  Vector3d operator*(const Vector3d& v) const {
    return rotation * v + translation;
  }
};

struct TriangleMesh {
  std::vector<Vertex> vertices;
  std::vector<std::array<int, 3>> indices;

  Transform transform;
};

double RandDouble(double min = 0, double max = 1) {
  return min + (max - min) * rand() / static_cast<double>(RAND_MAX);
}

std::vector<std::string> SplitString(const std::string& str, const char delim) {
  std::vector<std::string> out;
  std::stringstream ss(str);
  std::string tmp;
  while (std::getline(ss, tmp, delim)) {
    out.push_back(tmp);
  }
  return out;
}

// "1" => [1, -1, -1]
// "1//2" => [1, -1, 2]
// "1/2/3" => [1, 2, 3]
std::array<int, 3> ParseOBJFaceElement(const std::string& str) {
  const auto splits = SplitString(str, '/');

  // OBJ uses 1-based indexing, hence the -1
  return {std::stoi(splits[0]) - 1,
          (splits.size() > 1 && splits[1].size() > 0) ? std::stoi(splits[1]) - 1
                                                      : -1,
          (splits.size() > 2 && splits[2].size() > 0) ? std::stoi(splits[2]) - 1
                                                      : -1};
}

TriangleMesh LoadFromOBJ(const std::string& filename,
                         bool rand_vert_colors = false) {
  TriangleMesh mesh;

  std::vector<Vector3d> vertpos;
  std::vector<Vector2d> uvs;
  std::vector<Vector3d> normals;

  std::ifstream infile(filename);
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    std::string marker;
    if (!(iss >> marker)) {
      continue;
    }
    if (marker == "v") {
      double x, y, z;
      if (!(iss >> x >> y >> z)) {
        fprintf(stderr, "Failed to parse line %s\n", line.c_str());
        abort();
      }
      vertpos.push_back(Vector3d(x, y, z));
    } else if (marker == "vn") {
      double x, y, z;
      if (!(iss >> x >> y >> z)) {
        fprintf(stderr, "Failed to parse line %s\n", line.c_str());
        abort();
      }
      normals.push_back(Vector3d(x, y, z));
    } else if (marker == "vt") {
      double u, v;
      if (!(iss >> u >> v)) {
        fprintf(stderr, "Failed to parse line %s\n", line.c_str());
        abort();
      }
      uvs.push_back(Vector2d(u, v));
    } else if (marker == "f") {
      std::array<std::string, 3> elements;
      if (!(iss >> elements[0] >> elements[1] >> elements[2])) {
        fprintf(stderr, "Failed to parse line %s\n", line.c_str());
        abort();
      }
      const int nverts = mesh.vertices.size();
      for (int i = 0; i < 3; ++i) {
        const auto indices = ParseOBJFaceElement(elements[i]);
        // printf("indices: %d, %d, %d\n", indices[0], indices[1], indices[2]);
        if (indices[0] >= vertpos.size()) {
          fprintf(stderr, "Index out of bound: %d\n", indices[0]);
          abort();
        }
        Vector3d pos = vertpos[indices[0]];
        Vector2d uv = Vector2d::Zero();
        if (indices[1] != -1) {
          uv = uvs[indices[1]];
        }
        Vector3d normal = Vector3d::Zero();
        if (indices[2] != -1) {
          normal = normals[indices[2]];
        }
        Color c(1, 1, 1);
        if (rand_vert_colors) {
          c = {RandDouble(), RandDouble(), RandDouble()};
        }
        mesh.vertices.push_back(Vertex(pos, c, uv));
      }
      mesh.indices.push_back({nverts, nverts + 1, nverts + 2});
    } else {
      printf("skipping line: %s\n", line.c_str());
    }
  }

  return mesh;
}

struct Camera {
  Transform transform;
};

class Timer {
 public:
  Timer() {
    last_ = SDL_GetPerformanceCounter();
    counter_freq_ = SDL_GetPerformanceFrequency();
  }

  // tick the timer and return elapsed time in seconds since last tick
  double Tick() {
    const auto now = SDL_GetPerformanceCounter();
    const double delta = static_cast<double>(now - last_) / counter_freq_;
    last_ = now;
    return delta;
  }

 private:
  double counter_freq_;
  uint64_t last_;
};

template <class T>
class Array2D {
 public:
  Array2D(int width, int height) : width_(width), height_(height) {
    data_.resize(width * height);
  }

  T& operator()(int i, int j) { return data_[i * width_ + j]; }

  const T& operator()(int i, int j) const { return data_[i * width_ + j]; }

  T& at(int i, int j) { return this->operator()(i, j); }
  const T& at(int i, int j) const { return this->operator()(i, j); }

  int width() const { return width_; }
  int height() const { return height_; }

  void Clear(const T& c) { std::fill(data_.begin(), data_.end(), c); }

 private:
  int width_;
  int height_;
  std::vector<T> data_;
};

using Raster = Array2D<Color>;

// Loads a binary .ppm file (e.g. exported from Gimp by selecting PPM and then
// 'raw' format)
Raster LoadBinaryPPM(const std::string& filename) {
  std::ifstream infile(filename, std::ios::binary);
  int width, height;
  int maxval;

  auto nextline = [&]() {
    std::string line;
    while (true) {
      std::getline(infile, line, '\n');
      if (line.find('#') != 0) {
        return line;
      }
      if (infile.eof()) {
        fprintf(stderr, "No more data\n");
        abort();
      }
    }
  };
  {
    auto line = nextline();
    if (line != "P6") {
      fprintf(stderr, "Wrong format: %s\n", line.c_str());
      abort();
    }
  }
  {
    auto line = nextline();
    std::istringstream iss(line);
    if (!(iss >> width >> height)) {
      fprintf(stderr, "Failed to read size: %s\n", line.c_str());
      abort();
    }
  }
  {
    auto line = nextline();
    std::istringstream iss(line);
    if (!(iss >> maxval)) {
      fprintf(stderr, "Failed to read maxval: %s\n", line.c_str());
      abort();
    }
  }
  fprintf(stdout, "Texture with width=%d, height=%d, maxval=%d\n", width,
          height, maxval);
  Raster raster(width, height);
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      uint8_t val[3];
      infile.read(reinterpret_cast<char*>(val), 3);
      if (infile.eof()) {
        fprintf(stderr, "Incomplete file at i=%d, j=%d\n", i, j);
        abort();
      }
      Color c(static_cast<double>(val[0]) / maxval,
              static_cast<double>(val[1]) / maxval,
              static_cast<double>(val[2]) / maxval);
      raster(i, j) = c;
    }
  }
  return raster;
}

Raster MakeCheckerboard() {
  Raster raster(256, 256);
  for (int i = 0; i < raster.height(); ++i) {
    for (int j = 0; j < raster.width(); ++j) {
      const double u = static_cast<double>(j) / raster.width();
      const double v = static_cast<double>(i) / raster.height();
      const int M = 10;
      float p = (fmod(u * M, 1.0) > 0.5) ^ (fmod(v * M, 1.0) < 0.5);
      raster(i, j) = Color(1, 1, 1) * p;
    }
  }
  return raster;
}

struct Buffers {
  Buffers(int width, int height) : color(width, height), depth(width, height) {}

  int width() const { return color.width(); }
  int height() const { return color.height(); }

  Raster color;
  Array2D<double> depth;
};

template <class T>
T Clip(T val, T min, T max) {
  if (val < min) {
    return min;
  } else if (val > max) {
    return max;
  } else {
    return val;
  }
}

void PrintVector(const std::string& label, const Vector3d& v) {
  printf("%s (x=%f, y=%f, z=%f)\n", label.c_str(), v.x(), v.y(), v.z());
}

void DrawSquare(Raster* raster, int x, int y, int width, int height,
                const Color& color) {
  for (int i = y; i < y + height; ++i) {
    for (int j = x; j < x + width; ++j) {
      raster->at(i, j) = color;
    }
  }
}

Vector3d WorldToCamera(const Vector3d& p, const Camera& camera) {
  return camera.transform * p;
}

// Project from camera space to screen space
Vector3d CameraToScreen(const Vector3d& p) {
  return Vector3d(p.x() / p.z(), p.y() / p.z(), p.z());
}

// Project from screen space to raster space
Vector3d ScreenToRaster(const Vector3d& p, const Raster& raster) {
  return Vector3d(
      (1 + p.x()) * 0.5 * raster.width(), (1 + p.y()) * 0.5 * raster.height(),
      // Reverse z to have positive values stored in the depth buffer
      -p.z());
}

// The magnitude of the cross product between (c - a) and (b - a)
// Only considers x/y of 3D vector
double EdgeFunction(const Vector3d& a, const Vector3d& b, const Vector3d& c) {
  return (c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x());
}

// Draws a triangle with coordinate specified in raster space
void DrawTriangle(Buffers* buffers, const Vertex& v0, const Vertex& v1,
                  const Vertex& v2, const Raster& texture) {
  const Vector3d& p0 = v0.position;
  const Vector3d& p1 = v1.position;
  const Vector3d& p2 = v2.position;

  const double area = EdgeFunction(p0, p1, p2);

  int xmin = static_cast<int>(std::min(p0.x(), std::min(p1.x(), p2.x())));
  int xmax = static_cast<int>(std::max(p0.x(), std::max(p1.x(), p2.x())));
  int ymin = static_cast<int>(std::min(p0.y(), std::min(p1.y(), p2.y())));
  int ymax = static_cast<int>(std::max(p0.y(), std::max(p1.y(), p2.y())));
  xmin = Clip(xmin, 0, buffers->width());
  xmax = Clip(xmax, 0, buffers->width());
  ymin = Clip(ymin, 0, buffers->height());
  ymax = Clip(ymax, 0, buffers->height());

  // Divide vertex attribute (color) by vertex z do to perspective correct
  // interpolation
  const Color c0 = v0.color / v0.position.z();
  const Color c1 = v1.color / v1.position.z();
  const Color c2 = v2.color / v2.position.z();

  const Vector2d uv0 = v0.uv / v0.position.z();
  const Vector2d uv1 = v1.uv / v1.position.z();
  const Vector2d uv2 = v2.uv / v2.position.z();

  // Pre-compute per-vertex 1/z
  const double one_on_z0 = 1.0 / v0.position.z();
  const double one_on_z1 = 1.0 / v1.position.z();
  const double one_on_z2 = 1.0 / v2.position.z();

  for (int i = ymin; i <= ymax; ++i) {
    for (int j = xmin; j <= xmax; ++j) {
      const Vector3d p(j + 0.5, i + 0.5, 0);
      auto w0 = EdgeFunction(p1, p2, p);
      auto w1 = EdgeFunction(p2, p0, p);
      auto w2 = EdgeFunction(p0, p1, p);

      // If we force winding order, we only need to test positive or negative
      const bool all_neg = (w0 <= 0) && (w1 <= 0) && (w2 <= 0);
      const bool all_pos = (w0 >= 0) && (w1 >= 0) && (w2 >= 0);
      if (all_neg || all_pos) {
        w0 /= area;
        w1 /= area;
        w2 /= area;
        const double z =
            1.0 / (w0 * one_on_z0 + w1 * one_on_z1 + w2 * one_on_z2);
        // Depth test (our z is negative, but we reverse z in the depth buffer
        if (z < 0 || z > buffers->depth.at(i, j)) {
          continue;
        }
        // Interpolate color based on the z-weighted vertices colors
        Color color = (c0 * w0 + c1 * w1 + c2 * w2) * z;
        const Vector2d uv = (uv0 * w0 + uv1 * w1 + uv2 * w2) * z;
        int texj =
            static_cast<int>(Clip(1 - uv.x(), 0.0, 1.0) * texture.width());
        int texi =
            static_cast<int>(Clip(1 - uv.y(), 0.0, 1.0) * texture.height());
        Color texColor = texture(texi, texj);
        color = color * texColor;
        buffers->color.at(i, j) = color;
        buffers->depth.at(i, j) = z;
      }
    }
  }
}

void RenderMesh(const TriangleMesh& mesh, const Raster& texture,
                const Camera& camera, Buffers* buffers) {
  for (const std::array<int, 3>& triangle : mesh.indices) {
    const Vertex& v0 = mesh.vertices[triangle[0]];
    const Vertex& v1 = mesh.vertices[triangle[1]];
    const Vertex& v2 = mesh.vertices[triangle[2]];
    Vector3d p0 = ScreenToRaster(
        CameraToScreen(WorldToCamera(mesh.transform * v0.position, camera)),
        buffers->color);
    Vector3d p1 = ScreenToRaster(
        CameraToScreen(WorldToCamera(mesh.transform * v1.position, camera)),
        buffers->color);
    Vector3d p2 = ScreenToRaster(
        CameraToScreen(WorldToCamera(mesh.transform * v2.position, camera)),
        buffers->color);

    DrawTriangle(buffers, Vertex(p0, v0.color, v0.uv),
                 Vertex(p1, v1.color, v1.uv), Vertex(p2, v2.color, v2.uv),
                 texture);
  }
}

class Window {
 public:
  Window(const std::string& name, int width = WIDTH, int height = HEIGHT) {
    window_ = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED, width, height, 0);
    if (!window_) {
      fprintf(stderr, "Could not create window\n");
      abort();
    }
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer_) {
      fprintf(stderr, "Could not create renderer\n");
      abort();
    }

    texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA8888,
                                 SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!texture_) {
      fprintf(stderr, "Could not create texture\n");
      abort();
    }
  }

  ~Window() {
    SDL_DestroyTexture(texture_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
  }

  template <class T>
  void Display(const Array2D<T>& raster,
               const std::function<Color(const T& v)>& to_color) {
    // Display image on screen
    uint8_t* pixels;
    int pitch;
    if (SDL_LockTexture(texture_, NULL, reinterpret_cast<void**>(&pixels),
                        &pitch) < 0) {
      fprintf(stderr, "Failed to lock texture: %s\n", SDL_GetError());
      abort();
    }
    for (int row = 0; row < raster.height(); ++row) {
      for (int col = 0; col < raster.width(); ++col) {
        uint32_t* ptr = reinterpret_cast<uint32_t*>(pixels + row * pitch +
                                                    col * sizeof(uint32_t));
        const Color color = to_color(raster(row, col));
        const uint8_t r = static_cast<uint8_t>(color.r * 255);
        const uint8_t g = static_cast<uint8_t>(color.g * 255);
        const uint8_t b = static_cast<uint8_t>(color.b * 255);
        *ptr = 0x000000FF | (r << 24) | (g << 16) | (b << 8);
      }
    }

    SDL_UnlockTexture(texture_);
    SDL_SetRenderTarget(renderer_, NULL);
    SDL_RenderCopy(renderer_, texture_, NULL, NULL);
    SDL_RenderPresent(renderer_);
  }

 private:
  SDL_Window* window_;
  SDL_Texture* texture_;
  SDL_Renderer* renderer_;
};

int main() {
  SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "Could not init SDL: %s\n", SDL_GetError());
    return 1;
  }

  Window window("main");
  Window window_depth("depth buffer");

  Buffers buffers(WIDTH, HEIGHT);
  Camera camera;
  camera.transform.translation = Vector3d(0, 0, -2);

  // Raster texture = LoadBinaryPPM("../data/capsule/capsule0.ppm");
  Raster texture = MakeCheckerboard();
  auto mesh = LoadFromOBJ("../data/capsule/capsule.obj");
  mesh.transform.translation = Vector3d(0, -1, -5);
  Timer timer;

  // Uncomment this + call to display below to debug texture loading
  // Window window_texture("texture", texture.width(), texture.height());

  const double zmin = 0;
  const double zmax = 10;

  bool done = false;
  while (!done) {
    // Event management
    {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        switch (event.type) {
          case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE) {
              done = true;
            }
            break;
          case SDL_QUIT:
            done = true;
            break;
        }
      }
    }

    const double elapsedS = timer.Tick();

    // Render scene
    buffers.color.Clear(Color(0.5, 0.5, 0.5));
    buffers.depth.Clear(zmax);

    mesh.transform.rotation *=
        Quaterniond(AngleAxisd(0.1 * elapsedS * M_PI, Vector3d::UnitY()));

    RenderMesh(mesh, texture, camera, &buffers);

    window.Display<Color>(buffers.color, [](const Color& c) { return c; });
    window_depth.Display<double>(buffers.depth, [&](double v) {
      const double s = (v - zmin) / (zmax - zmin);
      return Color(s, s, s);
    });
    // window_texture.Display<Color>(texture, [](const Color& c) { return c; });
  }
  SDL_Quit();
  return 0;
}
