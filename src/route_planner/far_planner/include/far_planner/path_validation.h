#ifndef FAR_PATH_VALIDATION_H
#define FAR_PATH_VALIDATION_H

#include "node_struct.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <vector>

namespace FARPathValidation {

enum class Status {
  OK,
  INITIAL_NULL_PARENT,
  DEEP_NULL_PARENT,
  CYCLE_DETECTED,
  SELF_PARENT,
  NONFINITE_NODE,
  HOP_LIMIT,
  PATH_TOO_SHORT,
  INVALID_PARENT_CHAIN,
  NO_VALID_PATH,
};

inline const char* ToString(Status status) {
  switch (status) {
    case Status::OK: return "OK";
    case Status::INITIAL_NULL_PARENT: return "INITIAL_NULL_PARENT";
    case Status::DEEP_NULL_PARENT: return "DEEP_NULL_PARENT";
    case Status::CYCLE_DETECTED: return "CYCLE_DETECTED";
    case Status::SELF_PARENT: return "SELF_PARENT";
    case Status::NONFINITE_NODE: return "NONFINITE_NODE";
    case Status::HOP_LIMIT: return "HOP_LIMIT";
    case Status::PATH_TOO_SHORT: return "PATH_TOO_SHORT";
    case Status::INVALID_PARENT_CHAIN: return "INVALID_PARENT_CHAIN";
    case Status::NO_VALID_PATH: return "NO_VALID_PATH";
  }
  return "INVALID_PARENT_CHAIN";
}

struct Result {
  Status status = Status::NO_VALID_PATH;
  std::vector<NavNodePtr> path;
  std::size_t hops = 0;
  std::string detail;
  bool ok() const { return status == Status::OK; }
};

inline bool IsFiniteNode(const NavNodePtr& node) {
  return node && std::isfinite(node->position.x) && std::isfinite(node->position.y) &&
         std::isfinite(node->position.z) && std::isfinite(node->position.intensity);
}

inline Result ValidateRoute(const std::vector<NavNodePtr>& route, std::size_t min_size = 2) {
  Result result;
  if (route.size() < min_size) {
    result.status = Status::PATH_TOO_SHORT;
    result.detail = "route contains fewer than two nodes";
    return result;
  }
  std::unordered_set<const NavNode*> seen;
  for (const auto& node : route) {
    if (!IsFiniteNode(node)) {
      result.status = Status::NONFINITE_NODE;
      result.detail = "route contains a null or non-finite node";
      return result;
    }
    if (!seen.insert(node.get()).second) {
      result.status = Status::CYCLE_DETECTED;
      result.detail = "route repeats a node";
      return result;
    }
  }
  result.status = Status::OK;
  result.path = route;
  return result;
}

inline Result ReconstructAndValidate(const NavNodePtr& goal, bool free_navigation,
                                     const NavNodePtr& expected_root,
                                     std::size_t max_hops = 4096) {
  Result result;
  if (!goal || !expected_root) {
    result.status = Status::INVALID_PARENT_CHAIN;
    result.detail = "goal or expected root is null";
    return result;
  }
  std::vector<NavNodePtr> reverse_chain;
  std::unordered_set<const NavNode*> seen;
  NavNodePtr current = goal;
  for (std::size_t hop = 0; hop <= max_hops; ++hop) {
    result.hops = hop;
    if (!IsFiniteNode(current)) {
      result.status = Status::NONFINITE_NODE;
      result.detail = "parent chain contains a null or non-finite node";
      return result;
    }
    if (!seen.insert(current.get()).second) {
      result.status = Status::CYCLE_DETECTED;
      result.detail = "parent chain repeats a node";
      return result;
    }
    reverse_chain.push_back(current);
    if (current.get() == expected_root.get()) break;

    const NavNodePtr parent = free_navigation ? current->free_parent : current->parent;
    if (!parent) {
      result.status = (hop == 0) ? Status::INITIAL_NULL_PARENT : Status::DEEP_NULL_PARENT;
      result.detail = "parent chain terminated before odom root";
      return result;
    }
    if (parent.get() == current.get()) {
      result.status = Status::SELF_PARENT;
      result.detail = "parent chain contains a self-parent";
      return result;
    }
    current = parent;
    if (hop == max_hops) {
      result.status = Status::HOP_LIMIT;
      result.detail = "parent chain exceeded hop limit";
      return result;
    }
  }
  if (reverse_chain.empty() || reverse_chain.back().get() != expected_root.get()) {
    result.status = Status::INVALID_PARENT_CHAIN;
    result.detail = "parent chain did not end at odom root";
    return result;
  }
  std::reverse(reverse_chain.begin(), reverse_chain.end());
  std::vector<NavNodePtr> route;
  route.reserve(reverse_chain.size());
  for (std::size_t i = 0; i < reverse_chain.size(); ++i) {
    const auto& node = reverse_chain[i];
    if (i > 0 && node->free_direct == NodeFreeDirect::CONCAVE) continue;
    route.push_back(node);
  }
  result = ValidateRoute(route);
  if (result.ok()) result.hops = reverse_chain.size() - 1;
  return result;
}

}  // namespace FARPathValidation

#endif
