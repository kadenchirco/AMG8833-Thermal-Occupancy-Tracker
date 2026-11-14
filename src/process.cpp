#include <stdint.h>
#include <vector>
#include <stack>
#include <cstdio>
#include <queue>
#include <algorithm>
#include "process.h"

#ifndef PIO_UNIT_TESTING
  #include <Arduino.h>
#endif

thermal_processor::thermal_processor()
{
    interp.resize(interp_size_x * interp_size_y);
    visited.resize(interp_size_x * interp_size_y);
}

thermal_processor::thermal_processor(uint8_t inputX, uint8_t inputY, uint8_t outputX, uint8_t outputY, float threshold)
 : input_size_x(inputX), input_size_y(inputY), interp_size_x(outputX), interp_size_y(outputY), threshold(threshold)
{
    interp.resize(interp_size_x * interp_size_y);
    visited.resize(interp_size_x * interp_size_y);
}

thermal_processor::~thermal_processor()
{
}

void thermal_processor::print_output()
{
}

struct TrackPointPair
{
    size_t trackIndex;
    size_t pointIndex;
    float distance;
};


void thermal_processor::find_centroids() {
    std::stack<size_t> pieces;

    visited.assign(interp_size_x * interp_size_y, false);

    std::vector<std::pair<float,float>> centroids;

    for (uint16_t i = 0; i < interp_size_x * interp_size_y; i++) {
        if (interp[i] > threshold && !visited[i]) {
            pieces.push(i);
        }

        visited[i] = true;
        float weight = 0.0;
        float sumX = 0.0;
        float sumY = 0.0;
        size_t blobSize = 0;
        bool blobExists = !(pieces.empty());
        while (!(pieces.empty()))
        {   
            size_t j = pieces.top();
            pieces.pop();
            
            blobSize++;

            //i = x + y * interp_size_x
            int x = j % interp_size_x;
            int y = j / interp_size_x;
            size_t prevX = std::max(x - 1, 0);
            size_t nextX = std::min(x + 1, interp_size_x - 1);
            size_t prevY = std::max(y - 1, 0);
            size_t nextY = std::min(y + 1, interp_size_y - 1);

            float prevXValue = interp[prevX + y * interp_size_x];
            float nextXValue = interp[nextX + y * interp_size_x];
            float prevYValue = interp[x + prevY * interp_size_x];
            float nextYValue = interp[x + nextY * interp_size_x];

            weight += interp[j] - threshold;
            sumX += (interp[j] - threshold) * x;
            sumY += (interp[j] - threshold) * y;

            if (prevXValue > threshold && !visited[prevX + y * interp_size_x]) {
                pieces.push(prevX + y * interp_size_x);
                visited[prevX + y * interp_size_x] = true;
            }
            if (nextXValue > threshold && !visited[nextX + y * interp_size_x]) {
                pieces.push(nextX + y * interp_size_x);
                visited[nextX + y * interp_size_x] = true;
            }
            if (prevYValue > threshold && !visited[x + prevY * interp_size_x]) {
                pieces.push(x + prevY * interp_size_x);
                visited[x + prevY * interp_size_x] = true;
            }
            if (nextYValue > threshold && !visited[x + nextY * interp_size_x]) {
                pieces.push(x + nextY * interp_size_x);
                visited[x + nextY * interp_size_x] = true;
            }
        }

        if (blobExists && blobSize >= min_blob_size) {
            float newX = sumX / weight;
            float newY = sumY / weight;
            
            centroids.push_back({newX,newY});

            #ifdef PIO_UNIT_TESTING
                std::printf("\n");
                std::printf("Blob found at %f, %f", newX, newY);
                std::fflush(stdout);
            #endif
        }
    }

    // for (track& curTrack: tracks) {
    //     if (centroids.empty()) {
    //      // do something if a track doesnt have a closest centroid
    //      continue;   
    //     }

    //     float minDist = MAXFLOAT;
    //     size_t i = 0;

    //     for (size_t j = 0; j < centroids.size(); j++) {
    //         std::pair<float,float>& curPair = centroids[j];

    //         float dX = (curTrack.cetroid_x - curPair.first);
    //         float dY = (curTrack.cetroid_y - curPair.second);

    //         float dist = (dX * dX) + (dY * dY);

    //         if (dist < minDist) {
    //             minDist = dist;
    //             i = j;
    //         }
    //     }

    //     if (minDist < max_blob_dista nce) {

    //     } else {

    //     }
    // }

    // need to do some hungarian greedy matching bull crap :(
    // actually googled it and found out i can use a min PQ sorted by distance btwn points

    auto cmp = [](const TrackPointPair &left, const TrackPointPair &right) {
        return left.distance > right.distance;
    };

    std::priority_queue<TrackPointPair,std::vector<TrackPointPair>,decltype(cmp)> minPQ(cmp);

    for (size_t i = 0; i < tracks.size(); i++) {
        track& curTrack = tracks[i];

        for (size_t j = 0; j < centroids.size(); j++) {
            std::pair<float,float> &curPair = centroids[j];

            float dX = curTrack.cetroid_x - curPair.first;
            float dY = curTrack.cetroid_y - curPair.second;

            TrackPointPair newTPP;
            newTPP.distance = (dX * dX) + (dY * dY);
            newTPP.trackIndex = i;
            newTPP.pointIndex = j;

            if (newTPP.distance > (max_blob_distance * max_blob_distance)) {
                continue;
            }
            minPQ.push(newTPP);
        }
    }

    std::vector<bool> usedTracks(tracks.size(),false);
    std::vector<bool> usedCentroids(centroids.size(),false);
    std::vector<TrackPointPair> connections;

    // reading which centroid-track pairs exist
    while (!minPQ.empty())
    {
        TrackPointPair top = minPQ.top();
        minPQ.pop();
        if (usedTracks[top.trackIndex] || usedCentroids[top.pointIndex]) {
            continue;
        }
        connections.emplace_back(top);
        usedTracks[top.trackIndex] = true;
        usedCentroids[top.pointIndex] = true;

        // getting the velocity
        track& curTrack = tracks[top.trackIndex];
        float newX = centroids[top.pointIndex].first;
        float newY = centroids[top.pointIndex].second;

        curTrack.delta_time = current_time - curTrack.last_time;

        if (curTrack.delta_time > 0) {
            float dt = curTrack.delta_time / 1000.0f;
            curTrack.velocity_x = (newX - curTrack.cetroid_x) / dt;
            curTrack.velocity_y = (newY - curTrack.cetroid_y) / dt;
        }

        curTrack.cetroid_x = newX;
        curTrack.cetroid_y = newY;
        curTrack.last_time = current_time;
    }

    // if a track is used, we reset its unmatched frames so that it stays alive
    for (size_t i = 0; i < tracks.size(); i++) {
        if (usedTracks[i]) {
            tracks[i].unmatched_frames = 0;
        }
    }

    // now we look at unmatched tracks. some are useless but some result in the count changing (person moved across)
    for (size_t i = tracks.size(); i-- > 0;) {
        if (usedTracks[i]) {
            continue;
        }

        tracks[i].unmatched_frames++;

        // if a track doesn't match with a centroid for too long, it DIES :D
        if (tracks[i].unmatched_frames < max_unmatched_frames) {
            continue;
        }

        // before it dies, check if it crossed the middle of the frame for counting ppl.
        // uses net travel from start to last position instead of the last frame's velocity,
        // since the centroid gets jittery as the blob slides off an edge (top, bottom, or side)
        float mid_y = (interp_size_y - 1) / 2.0f;
        float travel_y = tracks[i].cetroid_y - tracks[i].start_pos_y;

        bool startedTop = tracks[i].start_pos_y < mid_y;
        bool endedTop = tracks[i].cetroid_y < mid_y;

        // only count ppl going from bottom to top or from top to bottom
        if (!startedTop && endedTop && travel_y <= -min_travel) {
            count++;
        } else if (startedTop && !endedTop && travel_y >= min_travel) {
            count--;
        }

        tracks.erase(tracks.begin() + i);
    }

    // finally, we can look at centroids that dont have a corresponding track
    for (size_t j = 0; j < centroids.size(); j++) {
        if (usedCentroids[j]) {
            continue;
        }

        float x = centroids[j].first;
        float y = centroids[j].second;

        // to see if it should appear or not, we check distance to top or bottom
        bool nearTop = y <= edge_distance;
        bool nearBottom = y >= (interp_size_y - 1) - edge_distance;

        if (!nearTop && !nearBottom) {
            continue;
        }

        // make the new track!
        track newTrack;
        newTrack.start_pos_x = x;
        newTrack.start_pos_y = y;
        newTrack.velocity_x = 0;
        newTrack.velocity_y = 0;
        newTrack.cetroid_x = x;
        newTrack.cetroid_y = y;
        newTrack.last_time = current_time;
        tracks.push_back(newTrack);

        last_detection = current_time;
    }
}

void thermal_processor::read(float *arr, uint32_t t)
{
    current_time = t;

    // first, do bilinear interp
    float xScale = (input_size_x - 1.0) / (interp_size_x - 1.0);
    float yScale = (input_size_y - 1.0) / (interp_size_y - 1.0);

    for (uint8_t x = 0; x < interp_size_x; x++)
    {
        float xsrc = x * xScale;

        uint8_t left = (uint8_t)xsrc;
        uint8_t right = std::min((int)left + 1, input_size_x - 1);

        float u = (left == right) ? 0.0 : (xsrc - left);

        for (uint8_t y = 0; y < interp_size_y; y++)
        {
            float ysrc = y * yScale;

            uint8_t top = (uint8_t)ysrc;
            uint8_t bottom = std::min((int)top + 1, input_size_y - 1);

            float v = (top == bottom) ? 0.0f : (ysrc - top);

            float Q11 = arr[left + top * input_size_x];
            float Q21 = arr[right + top * input_size_x];
            float Q12 = arr[left + bottom * input_size_x];
            float Q22 = arr[right + bottom * input_size_x];

            float R1 = (1.0 - u) * Q11 + u * Q21;
            float R2 = (1.0 - u) * Q12 + u * Q22;
            float final_value = (1.0 - v) * R1 + v * R2;

            interp[x + y * interp_size_x] = final_value;
        }
    }

    #ifdef PIO_UNIT_TESTING
        for (uint8_t y = 0; y < interp_size_y; y++)
        {
            std::printf("\n");
            for (uint8_t x = 0; x < interp_size_x; x++)
            {
                std::printf("%f", interp[x + y * interp_size_x]);
                std::printf(" ");
            }
        }

        std::fflush(stdout);
    #endif
}