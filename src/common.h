#ifndef COMMON_H
#define COMMON_H

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <ros/ros.h>

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

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
inline bool isArm(const std::string& vertex_frame, const ros::NodeHandle& nodehandle) {
    std::map<std::string, std::string> arm_surface_contacts;
    nodehandle.getParam("arm_surface_contacts", arm_surface_contacts);
    for (auto& armFrame : arm_surface_contacts) {
        if (vertex_frame == armFrame.first)
            return true;
    }
    return false;
}

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename SCALAR_T>
inline void transformWrench(const Eigen::Matrix<SCALAR_T, 6, 1>& oldWrench, const Eigen::Affine3d& newCFtoOldCFtransformation, Eigen::Matrix<SCALAR_T, 6, 1>& newWrench) {
    if(oldWrench.rows() != 6) {
      throw std::runtime_error("[transformWrench] old wrench is not 6d");
    }
    if (newWrench.rows() != 6)
        newWrench.resize(6);

    Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix = newCFtoOldCFtransformation.linear();
    Eigen::Matrix<SCALAR_T, 3, 1> translation = newCFtoOldCFtransformation.translation();

    Eigen::Matrix<SCALAR_T, 3, 1> reorientedF = rotationMatrix * oldWrench.segment(0,3);
    Eigen::Matrix<SCALAR_T, 3, 1> reorientedM = rotationMatrix * oldWrench.segment(3,3);

    newWrench.segment(0,3) = reorientedF;
    newWrench.segment(3,3) = reorientedM + translation.cross(reorientedF);
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
