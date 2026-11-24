#ifndef TYI_FAST_LIO_INCREMENTAL_VOXEL_MAP_H
#define TYI_FAST_LIO_INCREMENTAL_VOXEL_MAP_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <list>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace tyi_lio {

// Sparse, incrementally updated voxel index. Neighbor search is limited to the
// selected adjacent cells, as in linear iVox (Bai et al., IEEE RA-L 2022).
// Searches may run concurrently. Map insertion runs between scan updates.
template <typename Point, std::size_t Neighbors>
class IncrementalVoxelMap {
 public:
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

  IncrementalVoxelMap(float resolution, int adjacent, std::size_t max_voxels,
                      std::size_t max_points_per_voxel)
      : resolution_(resolution), max_voxels_(max_voxels),
        max_points_per_voxel_(max_points_per_voxel) {
    const int max_axes = adjacent == 6 ? 1 : adjacent == 18 ? 2 : 3;
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          if (std::abs(x) + std::abs(y) + std::abs(z) <= max_axes) {
            offsets_.push_back({x, y, z});
          }
        }
      }
    }
  }

  bool empty() const { return cells_.empty(); }
  std::size_t size() const { return size_; }

  void AddPoints(const Points& points) {
    for (const auto& p : points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const Key key = Locate(p);
      auto it = cells_.find(key);
      if (it == cells_.end()) {
        if (cells_.size() == max_voxels_) {
          const auto oldest = cells_.find(recent_.back());
          size_ -= oldest->second.points.size();
          cells_.erase(oldest);
          recent_.pop_back();
        }
        recent_.push_front(key);
        it = cells_.emplace(key, Cell{Points(), recent_.begin()}).first;
      } else {
        recent_.splice(recent_.begin(), recent_, it->second.recency);
      }
      auto& bucket = it->second.points;
      if (bucket.size() < max_points_per_voxel_) {
        bucket.push_back(p);
        ++size_;
      } else {
        const auto center_distance_sq = [&](const Point& candidate) {
          const float dx = candidate.x - (key.x + 0.5f) * resolution_;
          const float dy = candidate.y - (key.y + 0.5f) * resolution_;
          const float dz = candidate.z - (key.z + 0.5f) * resolution_;
          return dx*dx + dy*dy + dz*dz;
        };
        auto farthest = std::max_element(bucket.begin(), bucket.end(), [&](const Point& a, const Point& b) {
          return center_distance_sq(a) < center_distance_sq(b);
        });
        if (center_distance_sq(p) < center_distance_sq(*farthest)) *farthest = p;
      }
    }
  }

  void Nearest(const Point& query, Points& points, std::vector<float>& squared_distances,
               float max_squared_distance) const {
    points.clear();
    squared_distances.clear();
    if (!std::isfinite(query.x) || !std::isfinite(query.y) || !std::isfinite(query.z)) return;
    const Key center = Locate(query);
    std::array<Candidate, Neighbors> best;
    std::size_t count = 0;
    for (const auto& offset : offsets_) {
      const auto cell = cells_.find({center.x + offset.x, center.y + offset.y, center.z + offset.z});
      if (cell == cells_.end()) continue;
      for (const auto& p : cell->second.points) {
        const float dx = p.x - query.x;
        const float dy = p.y - query.y;
        const float dz = p.z - query.z;
        const float distance = dx*dx + dy*dy + dz*dz;
        if (distance > max_squared_distance) continue;
        std::size_t pos = 0;
        while (pos < count && best[pos].distance <= distance) ++pos;
        if (pos == Neighbors) continue;
        const std::size_t last = std::min(count, Neighbors - 1);
        for (std::size_t i = last; i > pos; --i) best[i] = best[i - 1];
        best[pos] = {distance, &p};
        if (count < Neighbors) ++count;
      }
    }
    for (std::size_t i = 0; i < count; ++i) {
      points.push_back(*best[i].point);
      squared_distances.push_back(best[i].distance);
    }
  }

 private:
  struct Key {
    int x, y, z;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const {
      const std::size_t x = std::hash<int>{}(key.x);
      const std::size_t y = std::hash<int>{}(key.y);
      const std::size_t z = std::hash<int>{}(key.z);
      return (x * 73856093u) ^ (y * 19349663u) ^ (z * 83492791u);
    }
  };

  struct Cell {
    Points points;
    typename std::list<Key>::iterator recency;
  };

  struct Candidate {
    float distance;
    const Point* point;
  };

  Key Locate(const Point& p) const {
    return {static_cast<int>(std::floor(p.x / resolution_)),
            static_cast<int>(std::floor(p.y / resolution_)),
            static_cast<int>(std::floor(p.z / resolution_))};
  }

  float resolution_;
  std::size_t max_voxels_;
  std::size_t max_points_per_voxel_;
  std::size_t size_ = 0;
  std::vector<Key> offsets_;
  std::list<Key> recent_;
  std::unordered_map<Key, Cell, KeyHash> cells_;
};

}  // namespace tyi_lio
#endif
