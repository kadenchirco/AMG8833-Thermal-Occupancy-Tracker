#include <unity.h>
#include "process.h"

void setup(void) {}
void tearDown(void) {}
void loop(void) {}

void test_bilinear_interpolation_corners() {
    const int in_x = 4;
    const int in_y = 4;
    const int out_x = 8;
    const int out_y = 8;

    float source_grid[in_x*in_y] = {1.0f,   5.0f,   2.0f,   0.0f,
                                    2.0f,   3.0f,   1.0f,   0.0f,
                                    0.0f,   0.0f,   2.0f,   3.0f,
                                    0.0f,   0.0f,   1.0f,   5.0f,
                                    };

    thermal_processor process(in_x, in_y, out_x, out_y, 3);

    process.read(source_grid, 0);
    process.find_centroids();
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_bilinear_interpolation_corners);
    return UNITY_END();
}
