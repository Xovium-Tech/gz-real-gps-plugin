#pragma once

#include <string>
#include <gz/transport/TopicUtils.hh>

namespace real_gps::gzsim
{
// Use Transport's own normalization so relative, absolute and trailing-slash
// spellings cannot bypass duplicate-publisher detection. Empty means invalid.
inline std::string CanonicalTopic(const std::string &topic)
{
  std::string qualified, partition, canonical;
  if (!gz::transport::TopicUtils::FullyQualifiedName("", "", topic, qualified) ||
      !gz::transport::TopicUtils::DecomposeFullyQualifiedTopic(qualified, partition, canonical))
    return {};
  return canonical;
}
}
