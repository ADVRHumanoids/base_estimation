#ifndef COMMON_H
#define COMMON_H

#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace ikbe_common
{

/**
 * @brief footFrames get frames associated to a foot
 * @param foot_prefix prefix of a foot
 * @return vector of frames associated to a foot_prefix
 */
inline std::vector<std::string> footFrames(XBot::Cartesian::CartesianInterfaceImpl& ci,
                                           const std::string& foot_prefix)
{
    using namespace XBot::Cartesian;

    std::vector<std::string> feet_tasks;
    for(auto t : ci.getTaskList())
    {
        auto cart = std::dynamic_pointer_cast<CartesianTask>(ci.getTask(t));

        if(!cart)
        {
            continue;
        }

        if(t.length() >= foot_prefix.length() &&
                t.substr(0,foot_prefix.length()) == foot_prefix)
        {
            feet_tasks.push_back(t);
        }
    }

    return feet_tasks;
}

}

namespace ocs2 {
namespace legged_robot {

// definitions from ocs2_centauro/common/Types.h
template <typename T>
using feet_array_t = std::array<T, 4>;

template <typename T>
using arms_array_t = std::array<T, 2>;

template <typename T>
using feet_arms_array_t = std::array<T, 6>;
using arm_contact_flag_t = arms_array_t<bool>;
using contact_flag_t = feet_array_t<bool>;
using locoma_contact_flag_t = feet_arms_array_t<bool>;   // loco-manipulation

// definitions from ocs2_centauro/gait/MotionPhaseDefinition.h
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  const int legModeOffset = 16;

  switch (modeNumber) {
    case 0: case (0 + legModeOffset): case (0 + 2 * legModeOffset): case (0 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, false, false};
      break;  // 0:  0-leg-stance
    case 1: case (1 + legModeOffset): case (1 + 2 * legModeOffset): case (1 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, false, true};
      break;  // 1:  RH
    case 2: case (2 + legModeOffset): case (2 + 2 * legModeOffset): case (2 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, true, false};
      break;  // 2:  LH
    case 3: case (3 + legModeOffset): case (3 + 2 * legModeOffset): case (3 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, true, true};
      break;  // 3:  RH, LH
    case 4: case (4 + legModeOffset): case (4 + 2 * legModeOffset): case (4 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, false, false};
      break;  // 4:  RF
    case 5: case (5 + legModeOffset): case (5 + 2 * legModeOffset): case (5 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, false, true};
      break;  // 5:  RF, RH
    case 6: case (6 + legModeOffset): case (6 + 2 * legModeOffset): case (6 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, true, false};
      break;  // 6:  RF, LH
    case 7: case (7 + legModeOffset): case (7 + 2 * legModeOffset): case (7 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, true, true};
      break;  // 7:  RF, LH, RH
    case 8: case (8 + legModeOffset): case (8 + 2 * legModeOffset): case (8 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, false, false};
      break;  // 8:  LF,
    case 9: case (9 + legModeOffset): case (9 + 2 * legModeOffset): case (9 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, false, true};
      break;  // 9:  LF, RH
    case 10: case (10 + legModeOffset): case (10 + 2 * legModeOffset): case (10 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, true, false};
      break;  // 10: LF, LH
    case 11: case (11 + legModeOffset): case (11 + 2 * legModeOffset): case (11 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, true, true};
      break;  // 11: LF, LH, RH
    case 12: case (12 + legModeOffset): case (12 + 2 * legModeOffset): case (12 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, false, false};
      break;  // 12: LF, RF
    case 13: case (13 + legModeOffset): case (13 + 2 * legModeOffset): case (13 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, false, true};
      break;  // 13: LF, RF, RH
    case 14: case (14 + legModeOffset): case (14 + 2 * legModeOffset): case (14 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, true, false};
      break;  // 14: LF, RF, LH
    case 15: case (15 + legModeOffset): case (15 + 2 * legModeOffset): case (15 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, true, true};
      break;  // 15: 4-leg-stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline arm_contact_flag_t modeNumber2StanceArm(const size_t& modeNumber) {
  arm_contact_flag_t stanceArms;  // {LF, RF, LH, RH}
  const int legModeOffset = 16;

  if (modeNumber < legModeOffset) {
      stanceArms = arm_contact_flag_t{false, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (legModeOffset - 1 <  modeNumber && modeNumber < 2 * legModeOffset) {
      stanceArms = arm_contact_flag_t{false, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (2 * legModeOffset - 1 < modeNumber && modeNumber < 3 * legModeOffset) {
      stanceArms = arm_contact_flag_t{true, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (3 * legModeOffset - 1 < modeNumber) {
      stanceArms = arm_contact_flag_t{true, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  return stanceArms;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline locoma_contact_flag_t modeNumber2ActiveContacts(const size_t& modeNumber) {
  locoma_contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  arm_contact_flag_t contactArmFlags = modeNumber2StanceArm(modeNumber);
  contact_flag_t contactLegFlags = modeNumber2StanceLeg(modeNumber);
  stanceLegs = locoma_contact_flag_t{contactLegFlags[0], contactLegFlags[1], contactLegFlags[2], contactLegFlags[3], contactArmFlags[0], contactArmFlags[1]};
  return stanceLegs;
}

}
}

#endif // COMMON_H
