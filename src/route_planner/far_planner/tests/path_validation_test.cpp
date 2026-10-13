#include "far_planner/path_validation.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

static NavNodePtr Node(std::size_t id, float x) {
  auto node = std::make_shared<NavNode>();
  node->id = id; node->position = Point3D(x, 0.0f, 0.0f); node->free_direct = NodeFreeDirect::CONVEX;
  return node;
}

int main() {
  using FARPathValidation::ReconstructAndValidate;
  using FARPathValidation::Status;
  auto root = Node(1, 0); auto middle = Node(2, 1); auto goal = Node(3, 2);
  middle->parent = root; goal->parent = middle;
  assert(ReconstructAndValidate(goal, false, root).status == Status::OK);
  middle->free_parent = root; goal->free_parent = middle;
  assert(ReconstructAndValidate(goal, true, root).status == Status::OK);

  auto initial_null = Node(4, 2);
  assert(ReconstructAndValidate(initial_null, false, root).status == Status::INITIAL_NULL_PARENT);
  auto deep_null = Node(5, 2); auto orphan = Node(6, 1); deep_null->parent = orphan;
  assert(ReconstructAndValidate(deep_null, false, root).status == Status::DEEP_NULL_PARENT);
  auto cycle_a = Node(7, 2); auto cycle_b = Node(8, 1); cycle_a->parent = cycle_b; cycle_b->parent = cycle_a;
  assert(ReconstructAndValidate(cycle_a, false, root).status == Status::CYCLE_DETECTED);
  auto self = Node(9, 2); self->parent = self;
  assert(ReconstructAndValidate(self, false, root).status == Status::SELF_PARENT);
  auto nonfinite = Node(10, 2); nonfinite->position.x = std::numeric_limits<float>::quiet_NaN(); nonfinite->parent = root;
  assert(ReconstructAndValidate(nonfinite, false, root).status == Status::NONFINITE_NODE);
  std::vector<NavNodePtr> long_chain; long_chain.reserve(4098); long_chain.push_back(root);
  for (std::size_t i = 1; i < 4098; ++i) { long_chain.push_back(Node(100 + i, static_cast<float>(i))); long_chain.back()->parent = long_chain[i - 1]; }
  assert(ReconstructAndValidate(long_chain.back(), false, root, 4096).status == Status::HOP_LIMIT);
  std::cout << "far_path_validation_test: PASS\n";
  return 0;
}
