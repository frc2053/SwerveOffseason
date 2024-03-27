#pragma once

#include <string>
#include <frc/geometry/Transform3d.h>

namespace consts {
namespace vision {
    inline const std::string FL_CAM_NAME{"fl_cam"};
    inline const frc::Transform3d FL_ROBOT_TO_CAM{
        frc::Translation3d{10.273802_in, 11.7301_in, 8.25_in},
        frc::Rotation3d{0_rad, -28.125_deg, 30_deg}
    };

    inline const std::string FR_CAM_NAME{"fr_cam"};
    inline const frc::Transform3d FR_ROBOT_TO_CAM{
        frc::Translation3d{10.273802_in, -11.7301_in, 8.25_in},
        frc::Rotation3d{0_rad, -28.125_deg, -30_deg}
    };

    inline const std::string BL_CAM_NAME{"bl_cam"};
    inline const frc::Transform3d BL_ROBOT_TO_CAM{
        frc::Translation3d{-10.273802_in, 11.7301_in, 8.25_in},
        frc::Rotation3d{0_rad, -28.125_deg, 150_deg}
    };

    inline const std::string BR_CAM_NAME{"br_cam"};
    inline const frc::Transform3d BR_ROBOT_TO_CAM{
        frc::Translation3d{-10.273802_in, -11.7301_in, 8.25_in},
        frc::Rotation3d{0_rad, -28.125_deg, -150_deg}
    };

    inline const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STD_DEV{4, 4, 8};
    inline const Eigen::Matrix<double, 3, 1> MULTI_TAG_STD_DEV{0.5, 0.5, 1};
}
}