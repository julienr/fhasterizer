// Relevant SDL 2 example (with emscripten):
// http://hg.libsdl.org/SDL/file/e12c38730512/test/teststreaming.c
// Scratchpixel rasterizer tutorial
// https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage
#include <cstdio>
#include <string>
#include <vector>

#include "SDL.h"

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

struct Color {
  Color() {}
  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

template <class T>
struct Vec3 {
  Vec3() {}
  Vec3(const T& x, const T& y, const T& z) : x(x), y(y), z(z) {}
  T x = T();
  T y = T();
  T z = T();
};

using Vec3d = Vec3<double>;

class Raster {
 public:
  Raster(int width, int height) : width_(width), height_(height) {
    pixels_.resize(width * height);
  }

  Color& operator()(int i, int j) { return pixels_[i * width_ + j]; }

  const Color& operator()(int i, int j) const {
    return pixels_[i * width_ + j];
  }

  Color& at(int i, int j) { return this->operator()(i, j); }
  const Color& at(int i, int j) const { return this->operator()(i, j); }

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  int width_;
  int height_;
  std::vector<Color> pixels_;
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

void PrintVector(const std::string& label, const Vec3d& v) {
  printf("%s (x=%f, y=%f, z=%f)\n", label.c_str(), v.x, v.y, v.z);
}

void DrawSquare(Raster* raster, int x, int y, int width, int height,
                const Color& color) {
  for (int i = y; i < y + height; ++i) {
    for (int j = x; j < x + width; ++j) {
      raster->at(i, j) = color;
    }
  }
}

// Project from camera space to screen space
Vec3d CameraToScreen(const Vec3d& p) {
  return Vec3d(p.x / p.z, p.y / p.z, p.z);
}

// Project from screen space to raster space
Vec3d ScreenToRaster(const Vec3d& p, const Raster& raster) {
  return Vec3d((1 + p.x) * 0.5 * raster.width(),
               (1 + p.y) * 0.5 * raster.height(), p.z);
}

// The magnitude of the cross product between (c - a) and (b - a)
// Only considers x/y of 3D vector
double EdgeFunction(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

// Draws a triangle with coordinate specified in raster space
void DrawTriangle(Raster* raster, const Vec3d& p0, const Vec3d& p1,
                  const Vec3d& p2, const Color& color) {
  const double area = EdgeFunction(p0, p1, p2);

  int xmin = static_cast<int>(std::min(p0.x, std::min(p1.x, p2.x)));
  int xmax = static_cast<int>(std::max(p0.x, std::max(p1.x, p2.x)));
  int ymin = static_cast<int>(std::min(p0.y, std::min(p1.y, p2.y)));
  int ymax = static_cast<int>(std::max(p0.y, std::max(p1.y, p2.y)));
  xmin = Clip(xmin, 0, raster->width());
  xmax = Clip(xmax, 0, raster->width());
  ymin = Clip(ymin, 0, raster->height());
  ymax = Clip(ymax, 0, raster->height());
  printf("xmin=%d, xmax=%d, ymin=%d, ymax=%d\n", xmin, xmax, ymin, ymax);

  for (int i = ymin; i <= ymax; ++i) {
    for (int j = xmin; j <= xmax; ++j) {
      const Vec3d p = {j + 0.5, i + 0.5, 0};
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
        raster->at(i, j) = color;
      }
    }
  }
}

int main() {
  SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "Could not init SDL: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Window* window =
      SDL_CreateWindow("fhrasterizer", SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, 0);
  if (!window) {
    fprintf(stderr, "Could not create window\n");
    return 1;
  }
  SDL_Renderer* renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  if (!renderer) {
    fprintf(stderr, "Could not create renderer\n");
    return 1;
  }

  SDL_Texture* texture =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
  if (!texture) {
    fprintf(stderr, "Could not create texture\n");
    return 1;
  }

  Raster raster(WIDTH, HEIGHT);
  DrawSquare(&raster, 50, 70, 20, 30, Color(255, 0, 0));

  Vec3d v2 = {-48, -10, 82};
  Vec3d v1 = {29, -15, 44};
  Vec3d v0 = {13, 34, 114};
  Color c2 = {255, 0, 0};
  Color c1 = {0, 255, 0};
  Color c0 = {0, 0, 255};

  // Project to screen space
  Vec3d p0 = ScreenToRaster(CameraToScreen(v0), raster);
  Vec3d p1 = ScreenToRaster(CameraToScreen(v1), raster);
  Vec3d p2 = ScreenToRaster(CameraToScreen(v2), raster);
  PrintVector("p0", p0);
  PrintVector("p1", p1);
  PrintVector("p2", p2);
  DrawTriangle(&raster, p0, p1, p2, c0);

  DrawTriangle(&raster, Vec3d(80, 80, 1), Vec3d(100, 80, 1), Vec3d(100, 100, 1),
               Color(0, 255, 0));

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

    // Display image on screen
    {
      uint8_t* pixels;
      int pitch;
      if (SDL_LockTexture(texture, NULL, reinterpret_cast<void**>(&pixels),
                          &pitch) < 0) {
        fprintf(stderr, "Failed to lock texture: %s\n", SDL_GetError());
        return 1;
      }
      for (int row = 0; row < HEIGHT; ++row) {
        for (int col = 0; col < WIDTH; ++col) {
          uint32_t* ptr = reinterpret_cast<uint32_t*>(pixels + row * pitch +
                                                      col * sizeof(uint32_t));
          const Color& color = raster(row, col);
          *ptr =
              0x000000FF | (color.r << 24) | (color.g << 16) | (color.b << 8);
        }
      }
    }

    SDL_UnlockTexture(texture);
    SDL_SetRenderTarget(renderer, NULL);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
