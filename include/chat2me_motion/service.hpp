#pragma once

#include <string>
#include <vector>

namespace chat2me_motion {

int serviceCommand(const std::vector<std::string> &arguments,
                   const std::string &executable_path);

}  // namespace chat2me_motion
