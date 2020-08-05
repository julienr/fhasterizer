// Relevant SDL 2 example (with emscripten):
// http://hg.libsdl.org/SDL/file/e12c38730512/test/teststreaming.c
#include <cstdio>
#include <vector>

#include "SDL.h"

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

class Image {
 public:
  Image(int width, int height) : width_(width) {
    pixels_.resize(width * height);
  }

  Color& operator()(int i, int j) { return pixels_[i * width_ + j]; }

  const Color& operator()(int i, int j) const {
    return pixels_[i * width_ + j];
  }

 private:
  int width_;
  std::vector<Color> pixels_;
};

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

  Image image(WIDTH, HEIGHT);
  for (int i = 50; i < 55; ++i) {
    for (int j = 90; j < 100; ++j) {
      image(i, j) = {.r = 255, .g = 0, .b = 0};
    }
  }

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
          const Color& color = image(row, col);
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
