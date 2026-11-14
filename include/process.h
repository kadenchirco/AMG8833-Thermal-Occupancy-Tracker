#include <stdint.h>
#include <vector>

#ifndef PROCESS_H
#define PROCESS_H

struct track {
    float start_pos_x; 
    float start_pos_y; 

    float velocity_x;
    float velocity_y;

    float cetroid_x;
    float cetroid_y;

    uint8_t unmatched_frames = 0;

    uint32_t last_time = 0;
    uint32_t delta_time = 0;
};

class thermal_processor
{
private:
    uint8_t input_size_x = 8;
    uint8_t input_size_y = 8;

    uint8_t interp_size_x = 32;
    uint8_t interp_size_y = 32;

    float threshold = 2.5;
    size_t min_blob_size = 4;
    float max_blob_distance = 15;
    uint8_t max_unmatched_frames = 5;
    float edge_distance = 15;
    float min_travel = 10; // how far (interp px) a track must move in y to count as crossing

    uint32_t current_time = 0;
    uint32_t last_detection = 0;

    int count = 0;

    std::vector<bool> visited;
    std::vector<float> interp;

    std::vector<track> tracks;

public:
    thermal_processor();
    thermal_processor(uint8_t inputX, uint8_t inputY, uint8_t outputX, uint8_t outputY, float threshold);

    ~thermal_processor();

    void print_output();
    void read(float *arr, uint32_t t);
    void find_centroids();
    int get_count() const { return count; }
    void set_count(int c) { count = c; }
    int get_detection_time() const { return last_detection; }
    size_t get_blobcount() {
        return tracks.size();
    };
};

#endif