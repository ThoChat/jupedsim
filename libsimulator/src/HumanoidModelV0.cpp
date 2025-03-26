#include "HumanoidModelV0.hpp"

#include "Ellipse.hpp"
#include "GenericAgent.hpp"
#include "Macros.hpp"
#include "Mathematics.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Simulation.hpp"
#include "HumanoidModelV0Data.hpp"

#include <Logger.hpp>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

HumanoidModelV0::HumanoidModelV0(double bodyForce_, double friction_)
    : bodyForce(bodyForce_), friction(friction_){};

OperationalModelType HumanoidModelV0::Type() const
{
    return OperationalModelType::HUMANOID_V0;
}

std::unique_ptr<OperationalModel> HumanoidModelV0::Clone() const
{
    return std::make_unique<HumanoidModelV0>(*this);
}

namespace {
    // Body parameters used in "Development and experimental validation of a humanoid pedestrian model that captures stepping behavior and body rotation"
    // i.e., leg length, trunk height, shoulder width, etc.
    constexpr std::array<double, 12> ANATOMY = {
        0.0451, 0.2522, 0.2269, 0.2/1.7, 0.1396, 0.45/1.7, 0.3495, 0.1470/2,
        0.1470/4, 0.1470*8/50, 0.1470*8/50, 0.25/1.7
    };

    // // feet position
    // // feet_position[0:2] = [x, y, z]: position of the swing foot, feet_position[3:5] = [x, y, z]: position of the support foot
    // // feet_position[2,5] = 0
    // struct GaitResult {
    //     std::array<double, 6> feet_position;
    //     Point position;
    // };

    // Denavit-Hartenberg (DH) convention
    std::array<std::array<double, 4>, 4> DHMat(double theta, double d, double a, double alpha) {
        return {{
            {cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)},
            {sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
            {0, sin(alpha), cos(alpha), d},
            {0, 0, 0, 1}
        }};
    }
    
    // matrix(size:4*4) * matrix(size:4*4)
    std::array<std::array<double, 4>, 4> MatMul4x4(const std::array<std::array<double, 4>, 4>& mat1, const std::array<std::array<double, 4>, 4>& mat2) {
        std::array<std::array<double, 4>, 4> result = {0};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < 4; ++k) {
                    result[i][j] += mat1[i][k] * mat2[k][j];
                }
            }
        }
        return result;
    }

    // matrix(size:4*4) * vector(size:4*1)
    std::array<double, 4> MatMul(const std::array<std::array<double, 4>, 4>& mat, const std::array<double, 4>& vec) {
        std::array<double, 4> result = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result[i] += mat[i][j] * vec[j];
            }
        }
        return result;
    }

    // This function is used to switch position of the support foot and swing foot
    // should be used in the double support phase, after using the function of FuncMotion

    // Input: feet_position: the position of the feet, [x_swing, y_swing, 0, x_support, y_support, 0];
    //        support_foot_orientation: the orientation of the support foot;
    std::array<double, 6> FuncFootDH(const std::array<double, 12>& link, double support_foot_orientation, const std::array<double, 6>& feet_position) {
        double length_ankle = link[0];
        std::array<double, 4> tmp = {feet_position[0], feet_position[1], feet_position[2], 1.0};

        std::array<std::array<double, 4>, 4> W = {{
            {-sin(support_foot_orientation), 0, cos(support_foot_orientation), 0},
            {cos(support_foot_orientation), 0, sin(support_foot_orientation), 0},
            {0, 1, 0, length_ankle},
            {0, 0, 0, 1}
        }};

        std::array<double, 4> O = {0, 0, 0, 1};
        std::array<double, 4> O0_0 = O;
        std::array<double, 4> O0_0w = MatMul(W, O0_0);
        for (int i = 0; i < 4; ++i) O0_0w[i] += tmp[i];

        return {feet_position[3], feet_position[4], 0, O0_0w[0], O0_0w[1], 0};
    }

    // This function is used to calculate the position of the joints;
    // Input: link: the body parameters, see ANATOMY;
    //        stepping_foot_index:      -1 == right foot stepping/left foot support,
    //                                  0 == double support,
    //                                  1 == left foot stepping/right foot support,
    //        sp: the position of the support foot, [x, y, z];
    //        support_foot_orientation: the orientation of the support foot;
    //        th: the joint angles, [th1, th2, th3, th4, th5, th6];
    //        phi: the joint angles, [phi1, phi2, phi3, phi4, phi5];
    //        Psi: the joint angles, [Psi1, Psi2, Psi3];
    // Output: y = [x, y, z, x, y, z], the position of the support foot is y[3:5], the position of the swing foot is y[0:2];
    //         p = [[x, y, z, 1], ...], the position of the joints, whose size is 4*31;
    
    // support leg: p[0:6];
    // swing leg: p[7:13];
    // left shoulder: p[14:15];
    // right shoulder: p[16:17];
    // head and trunk: p[18:20];
    // support foot: p[21:24];
    // swing foot: p[25:28];

    struct P {
        std::array<double, 4> heel_support;
        std::array<double, 4> ankle_support;
        std::array<double, 4> hip_support;
        std::array<double, 4> center_of_mass;
        std::array<double, 4> hip_swing; 
        std::array<double, 4> ankle_swing;
        std::array<double, 4> heel_swing;
        std::array<double, 4> center_of_shoulder;
        std::array<double, 4> shoulder_left;
        std::array<double, 4> shoulder_right;
        std::array<double, 4> head;
        std::array<double, 4> pc;
        std::array<double, 4> O0_0w;
        std::array<double, 4> O4_0w;
        std::array<double, 4> O5_0w;
        std::array<double, 4> O7_0w;
        std::array<double, 4> O8_0w;
        std::array<double, 4> O11_0w;
        std::array<double, 4> pf_0_1_w;
        std::array<double, 4> pf_0_2_w;
        std::array<double, 4> pf_0_3_w;
        std::array<double, 4> pf_0_4_w;
        std::array<double, 4> pf_12_1_w;
        std::array<double, 4> pf_12_2_w;
        std::array<double, 4> pf_12_3_w;
        std::array<double, 4> pf_12_4_w;
        std::array<double, 4> O19_0w;
        std::array<double, 4> O20_0w;
    };

    std::pair<std::array<double, 6>, P> FuncMotion(
        const std::array<double, 12>& link, int stepping_foot_index, const std::array<double, 3>& support_foot_position, double support_foot_orientation,
        const std::array<double, 6>& th, const std::array<double, 5>& phi, const std::array<double, 3>& Psi) {
        
        #ifndef PI
        #define PI 3.14159
        #endif

        // Joint angles
        double th1 = th[0], th3 = th[2], th4 = th[3], th6 = th[5];
        double phi1 = phi[0], phi2 = phi[1], phi3 = phi[2], phi4 = phi[3];
        double Psi1 = Psi[0], Psi2 = Psi[1], Psi3 = Psi[2];
    
        // Body parameters
        // ThoChat: what are l1 to l8? Are they the name of the length in the paper?
        // but they seems to be repeating the same values as the other lengths (i.e.,l_hd, l_sh, l_tr..)
        // Maybe I don't understan what the other lengths are?
        double length_ankle = link[0], length_shin = link[1], length_thigh = link[2], length_pelvis = link[3];
        double length_neck = link[4], length_shoulder = link[5];
        double length_trunk = link[6], length_foot_forward = link[7], length_foot_backward = link[8], length_foot_inner = link[9], length_foot_outer = link[10], l_r = link[11];
        
        std::array<double, 4> tmp = {support_foot_position[0], support_foot_position[1], support_foot_position[2], 1.0};

        // support_foot_orientation
        std::array<std::array<double, 4>, 4> W = {{
            {-sin(support_foot_orientation), 0, cos(support_foot_orientation), 0},
            {cos(support_foot_orientation), 0, sin(support_foot_orientation), 0},
            {0, 1, 0, length_ankle},
            {0, 0, 0, 1}
        }};
    
      
        std::array<double, 4> O = {0, 0, 0, 1};
    
      
        std::array<double, 4> O0_0w = MatMul(W, O);
        for (int i = 0; i < 4; ++i) O0_0w[i] += tmp[i];
    
        auto B1 = DHMat(phi1 + PI / 2, 0, 0, PI / 2);
        auto T1_0 = B1;
        auto O1_0 = MatMul(T1_0, O);
        auto O1_0w = MatMul(W, O1_0);
        for (int i = 0; i < 4; ++i) O1_0w[i] += tmp[i];
   
        auto B2 = DHMat(-th1 + PI / 2, 0, length_shin + length_thigh, 0);
        auto T2_0 = MatMul4x4(T1_0, B2);
        auto O2_0 = MatMul(T2_0, O);
        auto O2_0w = MatMul(W, O2_0);
        for (int i = 0; i < 4; ++i) O2_0w[i] += tmp[i];
    
        auto B4 = DHMat(-th3, 0, 0, -PI / 2);
        auto T4_0 = MatMul4x4(T2_0, B4);
        auto O4_0 = MatMul(T4_0, O);
        auto O4_0w = MatMul(W, O4_0);
        for (int i = 0; i < 4; ++i) O4_0w[i] += tmp[i];
     
        auto B5 = DHMat(phi2, 0, 0, 0);
        std::array<std::array<double, 4>, 4> rotMat = {{
            {0, 0, 1, 0},
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 1}
        }};
        auto T5_0 = MatMul4x4(MatMul4x4(T4_0, B5), rotMat);
        auto O5_0 = MatMul(T5_0, O);
        auto O5_0w = MatMul(W, O5_0);
        for (int i = 0; i < 4; ++i) O5_0w[i] += tmp[i];
    
        std::array<std::array<double, 4>, 4> B6;

        if (stepping_foot_index == -1) {
            B6 = DHMat(Psi1, 0, length_pelvis, 0);
        } else if (stepping_foot_index == 1) {
            B6 = DHMat(Psi1 + PI, 0, length_pelvis, 0);
        }
        auto T6_0 = MatMul4x4(T5_0, B6);
        auto O6_0 = MatMul(T6_0, O);
        auto O6_0w = MatMul(W, O6_0);
        for (int i = 0; i < 4; ++i) O6_0w[i] += tmp[i];
     
        std::array<std::array<double, 4>, 4> B7;
        std::array<std::array<double, 4>, 4> T7_0;
        if (stepping_foot_index == -1) {
            B7 = DHMat(Psi2, 0, 0, -PI / 2);
            std::array<std::array<double, 4>, 4> transMat = {{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            }};
            T7_0 = MatMul4x4(MatMul4x4(T6_0, B7), transMat);
        } else if (stepping_foot_index == 1) {
            B7 = DHMat(Psi2, 0, 0, PI / 2);
            std::array<std::array<double, 4>, 4> transMat = {{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            }};
            T7_0 = MatMul4x4(MatMul4x4(T6_0, B7), transMat);
        }
        auto O7_0 = MatMul(T7_0, O);
        auto O7_0w = MatMul(W, O7_0);
        for (int i = 0; i < 4; ++i) O7_0w[i] += tmp[i];

        auto B8 = DHMat(phi3, 0, 0, -PI / 2);
        auto T8_0 = MatMul4x4(T7_0, B8);
        auto O8_0 = MatMul(T8_0, O);
        auto O8_0w = MatMul(W, O8_0);
        for (int i = 0; i < 4; ++i) O8_0w[i] += tmp[i];

        auto B9 = DHMat(-th4 + PI, 0, length_shin + length_thigh, 0);
        auto T9_0 = MatMul4x4(T8_0, B9);
        auto O9_0 = MatMul(T9_0, O);
        auto O9_0w = MatMul(W, O9_0);
        for (int i = 0; i < 4; ++i) O9_0w[i] += tmp[i];

        auto B11 = DHMat(-th6, 0, 0, PI / 2);
        auto T11_0 = MatMul4x4(T9_0, B11);
        auto O11_0 = MatMul(T11_0, O);
        auto O11_0w = MatMul(W, O11_0);
        for (int i = 0; i < 4; ++i) O11_0w[i] += tmp[i];

        auto B12 = DHMat(phi4, 0, length_ankle, 0);
        auto T12_0 = MatMul4x4(T11_0, B12);
        auto O12_0 = MatMul(T12_0, O);
        auto O12_0w = MatMul(W, O12_0);
        for (int i = 0; i < 4; ++i) O12_0w[i] += tmp[i];

        double tip_x = O12_0w[0];
        double tip_y = O12_0w[1];
        
        std::array<double, 4> pf_0_1_w, pf_0_2_w, pf_0_3_w, pf_0_4_w;
        std::array<double, 4> pf_12_1_w, pf_12_2_w, pf_12_3_w, pf_12_4_w;
        if (stepping_foot_index == -1) {
            // the support foot
            std::array<double, 4> pf_0_1 = {length_foot_outer, -length_ankle, length_foot_forward, 1.0};
            std::array<double, 4> pf_0_2 = {-length_foot_inner, -length_ankle, length_foot_forward, 1.0};
            std::array<double, 4> pf_0_3 = {-length_foot_inner, -length_ankle, -length_foot_backward, 1.0};
            std::array<double, 4> pf_0_4 = {length_foot_outer, -length_ankle, -length_foot_backward, 1.0};

            std::array<double, 4> pf_0_1_w = MatMul(W, pf_0_1);
            std::array<double, 4> pf_0_2_w = MatMul(W, pf_0_2);
            std::array<double, 4> pf_0_3_w = MatMul(W, pf_0_3);
            std::array<double, 4> pf_0_4_w = MatMul(W, pf_0_4);

            for (int i = 0; i < 4; ++i) {
                pf_0_1_w[i] += tmp[i];
                pf_0_2_w[i] += tmp[i];
                pf_0_3_w[i] += tmp[i];
                pf_0_4_w[i] += tmp[i];
            }


            // the swing foot
            auto pf_12_1 = MatMul(T12_0, {0, length_foot_inner, length_foot_forward, 1});
            auto pf_12_2 = MatMul(T12_0, {0, -length_foot_outer, length_foot_forward, 1});
            auto pf_12_3 = MatMul(T12_0, {0, -length_foot_outer, -length_foot_backward, 1});
            auto pf_12_4 = MatMul(T12_0, {0, length_foot_inner, -length_foot_backward, 1});

            std::array<double, 4> pf_12_1_w = MatMul(W, pf_12_1);
            std::array<double, 4> pf_12_2_w = MatMul(W, pf_12_2);
            std::array<double, 4> pf_12_3_w = MatMul(W, pf_12_3);
            std::array<double, 4> pf_12_4_w = MatMul(W, pf_12_4);

            for (int i = 0; i < 4; ++i) {
                pf_12_1_w[i] += tmp[i];
                pf_12_2_w[i] += tmp[i];
                pf_12_3_w[i] += tmp[i];
                pf_12_4_w[i] += tmp[i];
            }

        } else if (stepping_foot_index == 1) {

            // the support foot
            std::array<double, 4> pf_0_1 = {length_foot_outer, -length_ankle, length_foot_forward, 1.0};
            std::array<double, 4> pf_0_2 = {-length_foot_inner, -length_ankle, length_foot_forward, 1.0};
            std::array<double, 4> pf_0_3 = {-length_foot_inner, -length_ankle, -length_foot_backward, 1.0};
            std::array<double, 4> pf_0_4 = {length_foot_outer, -length_ankle, -length_foot_backward, 1.0};

            std::array<double, 4> pf_0_1_w = MatMul(W, pf_0_1);
            std::array<double, 4> pf_0_2_w = MatMul(W, pf_0_2);
            std::array<double, 4> pf_0_3_w = MatMul(W, pf_0_3);
            std::array<double, 4> pf_0_4_w = MatMul(W, pf_0_4);

            for (int i = 0; i < 4; ++i) {
                pf_0_1_w[i] += tmp[i];
                pf_0_2_w[i] += tmp[i];
                pf_0_3_w[i] += tmp[i];
                pf_0_4_w[i] += tmp[i];
            }
    

            // the swing foot
            auto pf_12_1 = MatMul(T12_0, {0, length_foot_outer, length_foot_forward, 1});
            auto pf_12_2 = MatMul(T12_0, {0, -length_foot_inner, length_foot_forward, 1});
            auto pf_12_3 = MatMul(T12_0, {0, -length_foot_inner, -length_foot_backward, 1});
            auto pf_12_4 = MatMul(T12_0, {0, length_foot_outer, -length_foot_backward, 1});

            std::array<double, 4> pf_12_1_w = MatMul(W, pf_12_1);
            std::array<double, 4> pf_12_2_w = MatMul(W, pf_12_2);
            std::array<double, 4> pf_12_3_w = MatMul(W, pf_12_3);
            std::array<double, 4> pf_12_4_w = MatMul(W, pf_12_4);

            for (int i = 0; i < 4; ++i) {
                pf_12_1_w[i] += tmp[i];
                pf_12_2_w[i] += tmp[i];
                pf_12_3_w[i] += tmp[i];
                pf_12_4_w[i] += tmp[i];
            }

        }

      
        std::array<std::array<double, 4>, 4> B18 = {{{1, 0, 0, -length_pelvis/2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
        B18 = MatMul4x4(B18, DHMat(Psi3, 0, 0, 0));
        auto T18_0 = MatMul4x4(T6_0, B18);
        auto O18_0 = MatMul(T18_0, O);
        auto O18_0w = MatMul(W, O18_0);
        for (int i = 0; i < 4; ++i) O18_0w[i] += tmp[i];
    

        std::array<std::array<double, 4>, 4> T13_18;
        if (stepping_foot_index == -1) {
            T13_18 = {{{0, 0, -1, -length_shoulder/2}, {0, 1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        } else if (stepping_foot_index == 1) {
            T13_18 = {{{0, 0, 1, length_shoulder/2}, {0, -1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        }
        auto T13_0 = MatMul4x4(T18_0, T13_18);
        auto O13_0 = MatMul(T13_0, O);
        auto O13_0w = MatMul(W, O13_0);
        for (int i = 0; i < 4; ++i) O13_0w[i] += tmp[i];
     

        std::array<std::array<double, 4>, 4> T17_18;
        if (stepping_foot_index == -1) {
            T17_18 = {{{0, 0, -1, length_shoulder/2}, {0, 1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        } else if (stepping_foot_index == 1) {
            T17_18 = {{{0, 0, 1, -length_shoulder/2}, {0, -1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        }
        auto T17_0 = MatMul4x4(T18_0, T17_18);
        auto O17_0 = MatMul(T17_0, O);
        auto O17_0w = MatMul(W, O17_0);
        for (int i = 0; i < 4; ++i) O17_0w[i] += tmp[i];

        std::array<std::array<double, 4>, 4> T19_18;
        if (stepping_foot_index == -1) {
            T19_18 = {{{0, 0, -1, -l_r/2}, {0, 1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        } else if (stepping_foot_index == 1) {
            T19_18 = {{{0, 0, 1, l_r/2}, {0, -1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        }


        auto T19_0 = MatMul4x4(T18_0, T19_18);
        auto O19_0 = MatMul(T19_0, O);
        auto O19_0w = MatMul(W, O19_0);
        for (int i = 0; i < 4; ++i) O19_0w[i] += tmp[i];

        std::array<std::array<double, 4>, 4> T20_18;
        if (stepping_foot_index == -1) {
            T20_18 = {{{0, 0, -1, l_r/2}, {0, 1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        } else if (stepping_foot_index == 1) {
            T20_18 = {{{0, 0, 1, -l_r/2}, {0, -1, 0, 0}, {1, 0, 0, length_trunk}, {0, 0, 0, 1}}};
        }

        auto T20_0 = MatMul4x4(T18_0, T20_18);
        auto O20_0 = MatMul(T20_0, O);
        auto O20_0w = MatMul(W, O20_0);
        for (int i = 0; i < 4; ++i) O20_0w[i] += tmp[i];

        std::array<double, 4> pc;
        for (int i = 0; i < 4; ++i) {
            pc[i] = (O5_0w[i] + O6_0w[i]) / 2;
        } // pelvis point
        std::array<double, 4> pn;
        for (int i = 0; i < 4; ++i) {
            pn[i] = (O13_0w[i] + O17_0w[i]) / 2;
        } // neck point
        auto pt = MatMul(W, MatMul(T18_0, {0, 0, length_trunk + length_neck, 1}));
        for (int i = 0; i < 4; ++i) pt[i] += tmp[i]; // head point

        std::array<double, 6> y = {tip_x, tip_y, 0, O0_0w[0], O0_0w[1], 0};
        

        // All joint positions of the skeleton
        P p;

        // Support leg
        //// heel
        p.heel_support = tmp; 
        //// ankle
        p.O0_0w = O0_0w;
        p.ankle_support = O1_0w;
        //// pelvis
        p.hip_support = O2_0w;
        p.O4_0w = O4_0w;
        p.O5_0w = O5_0w;
        
        // Swing leg
        //// 
        p.hip_swing = O6_0w;
        p.O7_0w = O7_0w;
        p.O8_0w = O8_0w;
        p.ankle_swing = O9_0w;
        p.O11_0w = O11_0w;
        p.heel_swing = O12_0w;
        
        // Left shoulder
        p.shoulder_left = O13_0w;
        // Right shoulder
        p.shoulder_right = O17_0w;

        // CoM
        p.center_of_mass = O18_0w;

        // Center of the shoulder
        p.center_of_shoulder = pn; 

        // head
        p.head = pt; 

        // CoM
        p.pc = pc; 

        // The four corners of the foot * 2
        // Support foot
        p.pf_0_1_w = pf_0_1_w;
        p.pf_0_2_w = pf_0_2_w;
        p.pf_0_3_w = pf_0_3_w;
        p.pf_0_4_w = pf_0_4_w;
        // Swing foot
        p.pf_12_1_w = pf_12_1_w;
        p.pf_12_2_w = pf_12_2_w;
        p.pf_12_3_w = pf_12_3_w;
        p.pf_12_4_w = pf_12_4_w;

        // The center of the two small circles
        // used when representing the agent body as a three-circled shape
        p.O19_0w = O19_0w;
        p.O20_0w = O20_0w;

   
        return {y, p};
    }


    // // Deg to rad
    // double deg2rad(double deg) {
    //     return deg * PI / 180.0;
    // }


    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (step_length), step width (step_width), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the SINGLE support phase, which does not involve the switching of the support foot and swing foot
    std::pair<std::array<double, 6>, P> GaitSS(
        int stepping_foot_index, 
        double delta_orientation, 
        double support_foot_orientation, 
        double step_width,
        double width_shoulder_rotation, 
        double step_length, 
        const std::array<double,6>& feet_position, 
        double Height, 
        double lean_angle, 
        int rotation_index){


            std::vector<double> link(12);
            for (int i = 0; i < 12; ++i) {
                link[i] = ANATOMY[i] * Height;
            }

            double length_leg = link[1] + link[2]; // leg length
            double length_pelvis = link[3];           // pelvis length
            double length_shoulder = link[5];         // shoulder length

            double psi_t, psi_s, theta, phi_a, phi_p;

            // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle, psi_s: shoulder rotation angle (relative to the pelvis) 
            if (rotation_index == 0) {
                psi_t = 0;
                psi_s = psi_t;
        
                double tmp_1 = step_length / (2 * length_leg);
                theta = asin(tmp_1);
                double tmp_2 = (length_pelvis - step_width) / (2 * length_leg);
                phi_a = asin(tmp_2 + lean_angle);
                phi_p = -lean_angle;
            } else {
                psi_t = rotation_index * acos(step_width / length_pelvis); 
        
                psi_s = rotation_index * acos(width_shoulder_rotation / length_shoulder) - psi_t;
                double psi_t_sin = sqrt(1 - pow(step_width / length_pelvis, 2));
                double tmp_1 = (step_length - length_pelvis * psi_t_sin) / (2 * length_leg);
                theta = asin(tmp_1);
                phi_a = lean_angle; // lean angle
                phi_p = -phi_a;
            }
            
            support_foot_orientation += delta_orientation; // support_foot_orientation in Fig.2(b) in Shang et al. 2025, indicating the target direction
            double th1 = PI / 2 - theta;
            double th3 = theta;
            double th4 = PI + theta;
            double th6 = -theta;
            std::array<double, 6> th = {th1, 0, th3, th4, 0, th6};
        
            std::array<double, 5> phi;
            if (stepping_foot_index == -1) {
                phi = {-phi_a, (phi_a + phi_p), (PI / 2 + phi_a + phi_p), (-phi_a - phi_p), -phi_a};
            } else if (stepping_foot_index == 1) {
                phi = {phi_a, (-phi_a - phi_p), (-PI / 2 - phi_a - phi_p), (phi_a + phi_p), phi_a};
            }
        
            std::array<double, 3> Psi = {psi_t + delta_orientation, -psi_t, psi_s};
        
            std::vector<double> v(feet_position.begin(), feet_position.end());

            std::array<double, 3> support_foot_position = {v[3], v[4], v[5]};
            std::array<double, 12> link_array;
            std::copy(link.begin(), link.end(), link_array.begin());

            return FuncMotion(link_array, stepping_foot_index, support_foot_position, support_foot_orientation, th, phi, Psi);

        }

    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (step_length), step width (step_width), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the DOUBLE support phase, which involves the switching of the support foot and swing foot
    std::tuple<int, std::array<double, 6>, double, P> GaitDS(
        int stepping_foot_index,
        double delta_orientation, 
        double support_foot_orientation, 
        double step_width,
        double width_shoulder_rotation, 
        double step_length, 
        const std::array<double,6>& feet_position,
        double Height, 
        int rotation_index){

            std::vector<double> link(12);
            for (int i = 0; i < 12; ++i) {
                link[i] = ANATOMY[i] * Height;
            }

            double length_leg = link[1] + link[2]; // leg length
            double length_pelvis = link[3];           // pelvis length
            double length_shoulder = link[5];         // shoulder length

            double psi_t, psi_s, theta, phi_a, phi_p;

            // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle, psi_s: shoulder rotation angle (relative to the pelvis) 
            if (rotation_index == 0) {
                psi_t = 0;
                psi_s = psi_t;
        
                double tmp_1 = step_length / (2 * length_leg);
                theta = asin(tmp_1);
                double tmp_2 = (length_pelvis - step_width) / (2 * length_leg);
                phi_a = asin(tmp_2);
                phi_p = 0;
            } else {
                psi_t = rotation_index * acos(step_width / length_pelvis); 
        
                psi_s = rotation_index * acos(width_shoulder_rotation / length_shoulder) - psi_t;
                double psi_t_sin = sqrt(1 - pow(step_width / length_pelvis, 2));
                double tmp_1 = (step_length - length_pelvis * psi_t_sin) / (2 * length_leg);
                theta = asin(tmp_1);
                phi_a = 0; // lean angle
                phi_p = -phi_a;
            }

            support_foot_orientation += delta_orientation; // support_foot_orientation in Fig.2(b) in Shang et al. 2025, indicating the target direction


            double th1 = PI / 2 - theta;
            double th3 = theta;
            double th4 = PI + theta;
            double th6 = -theta;
            std::array<double, 6> th = {th1, 0, th3, th4, 0, th6};
        
            std::array<double, 5> phi;
            if (stepping_foot_index == -1) {
                phi = {-phi_a, (phi_a + phi_p), (PI / 2 + phi_a + phi_p), (-phi_a - phi_p), -phi_a};
            } else if (stepping_foot_index == 1) {
                phi = {phi_a, (-phi_a - phi_p), (-PI / 2 - phi_a - phi_p), (phi_a + phi_p), phi_a};
            }
        
            std::array<double, 3> Psi = {psi_t + delta_orientation, -psi_t, psi_s};
        
            std::vector<double> v(feet_position.begin(), feet_position.end());

            std::array<double, 3> sp = {v[3], v[4], v[5]};
            std::array<double, 12> link_array;
            std::copy(link.begin(), link.end(), link_array.begin());
            auto funcMotionResult = FuncMotion(link_array, stepping_foot_index, sp, support_foot_orientation, th, phi, Psi);
            auto motionResult = funcMotionResult.first; 
            auto position = funcMotionResult.second;

            // switch the support foot
            int new_stepping_foot_index;

            if (stepping_foot_index == 1) {
                new_stepping_foot_index = -1;
            } else {
                new_stepping_foot_index = 1;
            }
            
            
            std::array<double, 6> feet_pos;
            std::copy_n(motionResult.begin(), 6, feet_pos.begin());

            auto final_res = FuncFootDH(link_array, support_foot_orientation, feet_pos);
        
           

            return std::make_tuple(new_stepping_foot_index, final_res, support_foot_orientation, position);

        }

}




OperationalModelUpdate HumanoidModelV0::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& model = std::get<HumanoidModelV0Data>(ped.model);
    HumanoidModelV0Update update{};
    auto forces = DrivingForce(ped);

    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.pos, this->_cutOffRadius);
    Point F_rep;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        F_rep += AgentForce(ped, neighbor);  
    }
    forces += F_rep / model.mass;
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.pos);

    const auto obstacle_f = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleForce(ped, element);
        });
    forces += obstacle_f / model.mass;

    // creating update of pelvis position based on the collision avoidance model
    update.velocity = model.velocity + forces * dT;
    // update.position = ped.pos + update.velocity * dT;
    update.position = ped.pos;

    /// #### Humanoid model #####
    Point orientation = update.velocity.Normalized();
    Point normal_to_orientation = orientation.Rotate90Deg();
    const double max_step_lenght = model.height/2.5;
    // delta_orientation == 0: straight walk

    // ThoChat: We need to change the naming of all these parameters
    double delta_orientation = 0.0, support_foot_orientation = PI/2, k = 0.0, step_width = 0.2, width_shoulder_rotation = 0.45, step_length = max_step_lenght, H = model.height, lean_angle = 2/PI, min_d = 0;
    // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
    int rotation_index = 0;
    double step_duration = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));
        
    std::array<double, 6> feet_position = {model.heel_right_position.x, model.heel_right_position.y, 0.0, model.heel_left_position.x, model.heel_left_position.y, 0.0};

    // Steps computation
    if (model.step_timer == 0) {

        // # computation of the next step duration (step_timer is the step duration given as a number of time step)
        // constant stepping time of 0.5 for an agent of 1.7m
        update.step_timer = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));

        //stepping_foot_index: -1 == left foot support, 1 == right foot support
        if (model.stepping_foot_index == -1) {
            feet_position[0] = model.heel_right_position.x;
            feet_position[1] = model.heel_right_position.y;
            feet_position[2] = 0;
            feet_position[3] = model.heel_left_position.x;
            feet_position[4] = model.heel_left_position.y;
            feet_position[5] = 0;
        } else {
            feet_position[0] = model.heel_left_position.x;
            feet_position[1] = model.heel_left_position.y;
            feet_position[2] = 0;
            feet_position[3] = model.heel_right_position.x;
            feet_position[4] = model.heel_right_position.y;
            feet_position[5] = 0;
        }

        auto [output_stepping_foot_index, output_foot_position, output_support_foot_orientation, output_position] = GaitDS(model.stepping_foot_index, delta_orientation, support_foot_orientation, step_width, width_shoulder_rotation, step_length, feet_position, H, rotation_index);
        
        support_foot_orientation = output_support_foot_orientation;

        update.stepping_foot_index = output_stepping_foot_index;
        update.position.x = output_position.center_of_mass[0];
        update.position.y = output_position.center_of_mass[1];
        update.head_position.x = output_position.head[0];
        update.head_position.y = output_position.head[1];
        // update.head_position.z = output_position.head[2];
        //stepping_foot_index: -1 == left foot support, 1 == right foot support
        if (update.stepping_foot_index == -1) {
            update.heel_right_position.x = output_foot_position[0];
            update.heel_right_position.y = output_foot_position[1];
            update.heel_left_position.x = output_foot_position[3];
            update.heel_left_position.y = output_foot_position[4];
        } else {
            update.heel_right_position.x = output_foot_position[3];
            update.heel_right_position.y = output_foot_position[4];
            update.heel_left_position.x = output_foot_position[0];
            update.heel_left_position.y = output_foot_position[1];
        }
        
    } 
    else {
        
        double sl_p, lean_angle, sw_p, sw_sh_p, R_p;
        update.step_timer = model.step_timer;
        update.stepping_foot_index = model.stepping_foot_index;
        double tmp = 1 - update.step_timer/step_duration;

        // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
        if (rotation_index != 0) {
            if (update.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(tmp, 2) * (max_step_lenght + max_step_lenght) - max_step_lenght;
                lean_angle  =  2 * tmp * lean_angle;
            }
            else
            {
                sl_p = max_step_lenght - 2 * std::pow(1-tmp, 2) * (2 * max_step_lenght);
                lean_angle  =  2 * lean_angle - 2 * tmp * lean_angle;
            }
        } 
        else
        {
            if (update.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(tmp, 2) * (max_step_lenght + max_step_lenght) - max_step_lenght;
                lean_angle  =  2 * tmp * lean_angle;
            }
            else
            {
                sl_p = max_step_lenght - 2 * std::pow(1-tmp, 2) * (2 * max_step_lenght);
                lean_angle  =  2 * lean_angle - 2 * tmp * lean_angle;
            }
        }
        
        //stepping_foot_index: -1 == left foot support, 1 == right foot support

        if (model.stepping_foot_index == -1) {
            feet_position[0] = model.heel_right_position.x;
            feet_position[1] = model.heel_right_position.y;
            feet_position[2] = 0;
            feet_position[3] = model.heel_left_position.x;
            feet_position[4] = model.heel_left_position.y;
            feet_position[5] = 0;
        } else {
            feet_position[0] = model.heel_left_position.x;
            feet_position[1] = model.heel_left_position.y;
            feet_position[2] = 0;
            feet_position[3] = model.heel_right_position.x;
            feet_position[4] = model.heel_right_position.y;
            feet_position[5] = 0;
        }
        auto [output_foot_position, output_position] = GaitSS(update.stepping_foot_index, delta_orientation, support_foot_orientation, step_width, width_shoulder_rotation, sl_p, feet_position, H, lean_angle, rotation_index);
   
        update.position.x = output_position.center_of_mass[0];
        update.position.y = output_position.center_of_mass[1];
        update.head_position.x = output_position.head[0];
        update.head_position.y = output_position.head[1];
        // update.head_position.z = output_position.head[2];
        printf("output_position: %f, %f, %f\n", output_position.head[0], output_position.head[1], output_position.center_of_mass[0]);
        // printf("head_x: %f, head_y: %f\n", update.head_position.x, update.head_position.y);
        //stepping_foot_index: -1 == left foot support, 1 == right foot support
        if (model.stepping_foot_index == -1) {
            update.heel_right_position.x = output_foot_position[0];
            update.heel_right_position.y = output_foot_position[1];
            update.heel_left_position.x = output_foot_position[3];
            update.heel_left_position.y = output_foot_position[4];
        } else {
            update.heel_right_position.x = output_foot_position[3];
            update.heel_right_position.y = output_foot_position[4];
            update.heel_left_position.x = output_foot_position[0];
            update.heel_left_position.y = output_foot_position[1];
        }


        update.step_timer = model.step_timer - 1;

    }
    

    // creating update for the Humanoid model

    // ## head 
    update.head_velocity = model.head_velocity + normal_to_orientation*(0.1*Distance(update.position, ped.pos) * dT); 
    update.head_position = update.head_position;

    // ## shoulders
    update.shoulder_rotation_velocity_z = 0.0;
    update.shoulder_rotation_angle_z = model.shoulder_rotation_angle_z + update.shoulder_rotation_velocity_z * dT;

    // ## trunk
    // ### along the frontal axis (x) of this agent
    update.trunk_rotation_velocity_x = 0.0;
    update.trunk_rotation_angle_x = model.trunk_rotation_angle_x + update.trunk_rotation_velocity_x * dT;
    // ### along sagittal axis (y) of this agent
    update.trunk_rotation_velocity_y = 0.0;
    update.trunk_rotation_angle_y = model.trunk_rotation_angle_y + update.trunk_rotation_velocity_y * dT;
 
    return update;
}

void HumanoidModelV0::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const auto& upd = std::get<HumanoidModelV0Update>(update);
    // update the SFM navigation
    agent.pos = upd.position;
    model.velocity = upd.velocity;
    agent.orientation = upd.velocity.Normalized();
    // update the Humanoid model
    // # gait variables
    model.step_timer = upd.step_timer;
    model.stepping_foot_index = upd.stepping_foot_index;

    model.step_target= upd.step_target;


    // # body motion variables
    model.head_position = upd.head_position;
    model.head_velocity = upd.head_velocity;
    model.shoulder_rotation_angle_z = upd.shoulder_rotation_angle_z;
    model.shoulder_rotation_velocity_z = upd.shoulder_rotation_velocity_z;
    model.trunk_rotation_angle_x = upd.trunk_rotation_angle_x;
    model.trunk_rotation_velocity_x = upd.trunk_rotation_velocity_x;
    model.trunk_rotation_angle_y = upd.trunk_rotation_angle_y;
    model.trunk_rotation_velocity_y = upd.trunk_rotation_velocity_y;
    model.heel_right_position = upd.heel_right_position;
    model.heel_right_velocity = upd.heel_right_velocity;
    model.heel_left_position = upd.heel_left_position;
    model.heel_left_velocity = upd.heel_left_velocity;
}



void HumanoidModelV0::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    // none of these constraint are given by the paper but are useful to create a simulation that
    // does not break immediately
    auto throwIfNegative = [](double value, std::string name) {
        if(value < 0) {
            throw SimulationError(
                "Model constraint violation: {} {} not in allowed range, "
                "{} needs to be positive",
                name,
                value,
                name);
        }
    };

    const auto& model = std::get<HumanoidModelV0Data>(agent.model);

    const auto mass = model.mass;
    throwIfNegative(mass, "mass");

    const auto desiredSpeed = model.desiredSpeed;
    throwIfNegative(desiredSpeed, "desired speed");

    const auto reactionTime = model.reactionTime;
    throwIfNegative(reactionTime, "reaction time");

    const auto radius = model.radius;
    throwIfNegative(radius, "radius");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.pos, 2);
    for(const auto& neighbor : neighbors) {
        const auto distance = (agent.pos - neighbor.pos).Norm();

        if(model.radius >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}, "
                "radius {}",
                agent.pos,
                neighbor.pos,
                distance,
                model.radius);
        }
    }
    const auto maxRadius = model.radius / 2;
    const auto lineSegments = geometry.LineSegmentsInDistanceTo(maxRadius, agent.pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance <= "
            "{}/2",
            agent.pos,
            model.radius);
    }
}

Point HumanoidModelV0::DrivingForce(const GenericAgent& agent)
{
    const auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const Point e0 = (agent.destination - agent.pos).Normalized();
    return (e0 * model.desiredSpeed - model.velocity) / model.reactionTime;
};
double HumanoidModelV0::PushingForceLength(double A, double B, double r, double distance)
{
    return A * exp((r - distance) / B);
}

Point HumanoidModelV0::AgentForce(const GenericAgent& ped1, const GenericAgent& ped2) const
{
    const auto& model1 = std::get<HumanoidModelV0Data>(ped1.model);
    const auto& model2 = std::get<HumanoidModelV0Data>(ped2.model);

    const double total_radius = model1.radius + model2.radius;

    return ForceBetweenPoints(
        ped1.pos,
        ped2.pos,
        model1.agentScale,
        model1.forceDistance,
        total_radius,
        model2.velocity - model1.velocity);
};

Point HumanoidModelV0::ObstacleForce(const GenericAgent& agent, const LineSegment& segment) const
{
    const auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return ForceBetweenPoints(
        agent.pos, pt, model.obstacleScale, model.forceDistance, model.radius, model.velocity);
}

Point HumanoidModelV0::ForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double A,
    const double B,
    const double radius,
    const Point velocity) const
{
    // todo reduce range of force to 180 degrees
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_length = PushingForceLength(A, B, radius, dist) * 0; // if this is 0 agents bumps into each other 
    double friction_force_length = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radius) {
        pushing_force_length += this->bodyForce * (radius - dist);
        friction_force_length =
            this->friction * (radius - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_length + tangent * friction_force_length;
}