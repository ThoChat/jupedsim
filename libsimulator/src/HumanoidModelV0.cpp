// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
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
// #include <cmath> 

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
    constexpr std::array<double, 21> ANATOMY = {
        0.0451, 0.2522, 0.2269, 0, 0.2/1.7, 0, 0.2269, 0.2522, 0.0451,
        0.16, 0.1531, 0.16, 0.1531, 0.1396, 0.45/1.7, 0.3495, 0.1470/2,
        0.1470/4, 0.1470*8/50, 0.1470*8/50, 0.25/1.7
    };

    // feet position
    // res[0:2] = [x, y, z]: position of the swing foot, res[3:5] = [x, y, z]: position of the support foot
    // res[2,5] = 0
    struct GaitResult {
        std::array<double, 6> res;
        Point position;
    };

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
    std::array<double, 6> FuncFootDH(const std::array<double, 21>& link, double Gamma, const std::array<double, 6>& sp) {
        double l0 = link[0];
        std::array<double, 4> tmp = {sp[0], sp[1], sp[2], 1.0};

        std::array<std::array<double, 4>, 4> W = {{
            {-sin(Gamma), 0, cos(Gamma), 0},
            {cos(Gamma), 0, sin(Gamma), 0},
            {0, 1, 0, l0},
            {0, 0, 0, 1}
        }};

        std::array<double, 4> O = {0, 0, 0, 1};
        std::array<double, 4> O0_0 = O;
        std::array<double, 4> O0_0w = MatMul(W, O0_0);
        for (int i = 0; i < 4; ++i) O0_0w[i] += tmp[i];

        return {sp[3], sp[4], 0, O0_0w[0], O0_0w[1], 0};
    }

    // This function is used to calculate the position of the joints;
    // Input: link: the body parameters, see ANATOMY;
    //        sf: the support foot, 'L' for left foot, 'R' for right foot;
    //        sp: the position of the support foot, [x, y, z];
    //        Gamma: the orientation of the support foot;
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
        std::array<double, 4> tmp;
        std::array<double, 4> O0_0w;
        std::array<double, 4> O1_0w;
        std::array<double, 4> O2_0w;
        std::array<double, 4> O4_0w;
        std::array<double, 4> O5_0w;
        std::array<double, 4> O18_0w;
        std::array<double, 4> O6_0w;
        std::array<double, 4> O7_0w;
        std::array<double, 4> O8_0w;
        std::array<double, 4> O9_0w;
        std::array<double, 4> O11_0w;
        std::array<double, 4> O12_0w;
        std::array<double, 4> pn;
        std::array<double, 4> O13_0w;
        std::array<double, 4> O17_0w;
        std::array<double, 4> pt;
        std::array<double, 4> pc;
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
        const std::array<double, 21>& link, int sf, const std::array<double, 3>& sp, double Gamma,
        const std::array<double, 6>& th, const std::array<double, 5>& phi, const std::array<double, 3>& Psi) {
        
        #ifndef PI
        #define PI 3.14159
        #endif

        // Joint angles
        double th1 = th[0], th3 = th[2], th4 = th[3], th6 = th[5];
        double phi1 = phi[0], phi2 = phi[1], phi3 = phi[2], phi4 = phi[3];
        double Psi1 = Psi[0], Psi2 = Psi[1], Psi3 = Psi[2];
    
        // Body parameters
        double l0 = link[0], l1 = link[1], l2 = link[2], l3 = link[3], l4 = link[4], l5 = link[5];
        double l6 = link[6], l7 = link[7], l8 = link[8], l_hd = link[13], l_sh = link[14];
        double l_tr = link[15], l_ft_for = link[16], l_ft_back = link[17], l_ft_inner = link[18], l_ft_outer = link[19], l_r = link[20];
        
        std::array<double, 4> tmp = {sp[0], sp[1], sp[2], 1.0};

        // Orientation
        std::array<std::array<double, 4>, 4> W = {{
            {-sin(Gamma), 0, cos(Gamma), 0},
            {cos(Gamma), 0, sin(Gamma), 0},
            {0, 1, 0, l0},
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
   
        auto B2 = DHMat(-th1 + PI / 2, 0, l1 + l2, 0);
        auto T2_0 = MatMul4x4(T1_0, B2);
        auto O2_0 = MatMul(T2_0, O);
        auto O2_0w = MatMul(W, O2_0);
        for (int i = 0; i < 4; ++i) O2_0w[i] += tmp[i];
    
        auto B4 = DHMat(-th3, 0, 0, -PI / 2);
        auto T4_0 = MatMul4x4(T2_0, B4);
        auto O4_0 = MatMul(T4_0, O);
        auto O4_0w = MatMul(W, O4_0);
        for (int i = 0; i < 4; ++i) O4_0w[i] += tmp[i];
     
        auto B5 = DHMat(phi2, 0, l3, 0);
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

        if (sf == -1) {
            B6 = DHMat(Psi1, 0, l4, 0);
        } else if (sf == 1) {
            B6 = DHMat(Psi1 + PI, 0, l4, 0);
        }
        auto T6_0 = MatMul4x4(T5_0, B6);
        auto O6_0 = MatMul(T6_0, O);
        auto O6_0w = MatMul(W, O6_0);
        for (int i = 0; i < 4; ++i) O6_0w[i] += tmp[i];
     
        std::array<std::array<double, 4>, 4> B7;
        std::array<std::array<double, 4>, 4> T7_0;
        if (sf == -1) {
            B7 = DHMat(Psi2, 0, 0, -PI / 2);
            std::array<std::array<double, 4>, 4> transMat = {{
                {1, 0, 0, 0},
                {0, 1, 0, l5},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            }};
            T7_0 = MatMul4x4(MatMul4x4(T6_0, B7), transMat);
        } else if (sf == 1) {
            B7 = DHMat(Psi2, 0, 0, PI / 2);
            std::array<std::array<double, 4>, 4> transMat = {{
                {1, 0, 0, 0},
                {0, 1, 0, -l5},
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

        auto B9 = DHMat(-th4 + PI, 0, l6 + l7, 0);
        auto T9_0 = MatMul4x4(T8_0, B9);
        auto O9_0 = MatMul(T9_0, O);
        auto O9_0w = MatMul(W, O9_0);
        for (int i = 0; i < 4; ++i) O9_0w[i] += tmp[i];

        auto B11 = DHMat(-th6, 0, 0, PI / 2);
        auto T11_0 = MatMul4x4(T9_0, B11);
        auto O11_0 = MatMul(T11_0, O);
        auto O11_0w = MatMul(W, O11_0);
        for (int i = 0; i < 4; ++i) O11_0w[i] += tmp[i];

        auto B12 = DHMat(phi4, 0, l8, 0);
        auto T12_0 = MatMul4x4(T11_0, B12);
        auto O12_0 = MatMul(T12_0, O);
        auto O12_0w = MatMul(W, O12_0);
        for (int i = 0; i < 4; ++i) O12_0w[i] += tmp[i];

        double tip_x = O12_0w[0];
        double tip_y = O12_0w[1];
        
        std::array<double, 4> pf_0_1_w, pf_0_2_w, pf_0_3_w, pf_0_4_w;
        std::array<double, 4> pf_12_1_w, pf_12_2_w, pf_12_3_w, pf_12_4_w;
        if (sf == -1) {
            // the support foot
            std::array<double, 4> pf_0_1 = {l_ft_outer, -l0, l_ft_for, 1.0};
            std::array<double, 4> pf_0_2 = {-l_ft_inner, -l0, l_ft_for, 1.0};
            std::array<double, 4> pf_0_3 = {-l_ft_inner, -l0, -l_ft_back, 1.0};
            std::array<double, 4> pf_0_4 = {l_ft_outer, -l0, -l_ft_back, 1.0};

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
            auto pf_12_1 = MatMul(T12_0, {0, l_ft_inner, l_ft_for, 1});
            auto pf_12_2 = MatMul(T12_0, {0, -l_ft_outer, l_ft_for, 1});
            auto pf_12_3 = MatMul(T12_0, {0, -l_ft_outer, -l_ft_back, 1});
            auto pf_12_4 = MatMul(T12_0, {0, l_ft_inner, -l_ft_back, 1});

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

        } else if (sf == 1) {

            // the support foot
            std::array<double, 4> pf_0_1 = {l_ft_outer, -l0, l_ft_for, 1.0};
            std::array<double, 4> pf_0_2 = {-l_ft_inner, -l0, l_ft_for, 1.0};
            std::array<double, 4> pf_0_3 = {-l_ft_inner, -l0, -l_ft_back, 1.0};
            std::array<double, 4> pf_0_4 = {l_ft_outer, -l0, -l_ft_back, 1.0};

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
            auto pf_12_1 = MatMul(T12_0, {0, l_ft_outer, l_ft_for, 1});
            auto pf_12_2 = MatMul(T12_0, {0, -l_ft_inner, l_ft_for, 1});
            auto pf_12_3 = MatMul(T12_0, {0, -l_ft_inner, -l_ft_back, 1});
            auto pf_12_4 = MatMul(T12_0, {0, l_ft_outer, -l_ft_back, 1});

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

      
        std::array<std::array<double, 4>, 4> B18 = {{{1, 0, 0, -l4/2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
        B18 = MatMul4x4(B18, DHMat(Psi3, 0, 0, 0));
        auto T18_0 = MatMul4x4(T6_0, B18);
        auto O18_0 = MatMul(T18_0, O);
        auto O18_0w = MatMul(W, O18_0);
        for (int i = 0; i < 4; ++i) O18_0w[i] += tmp[i];
    

        std::array<std::array<double, 4>, 4> T13_18;
        if (sf == -1) {
            T13_18 = {{{0, 0, -1, -l_sh/2}, {0, 1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        } else if (sf == 1) {
            T13_18 = {{{0, 0, 1, l_sh/2}, {0, -1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        }
        auto T13_0 = MatMul4x4(T18_0, T13_18);
        auto O13_0 = MatMul(T13_0, O);
        auto O13_0w = MatMul(W, O13_0);
        for (int i = 0; i < 4; ++i) O13_0w[i] += tmp[i];
     

        std::array<std::array<double, 4>, 4> T17_18;
        if (sf == -1) {
            T17_18 = {{{0, 0, -1, l_sh/2}, {0, 1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        } else if (sf == 1) {
            T17_18 = {{{0, 0, 1, -l_sh/2}, {0, -1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        }
        auto T17_0 = MatMul4x4(T18_0, T17_18);
        auto O17_0 = MatMul(T17_0, O);
        auto O17_0w = MatMul(W, O17_0);
        for (int i = 0; i < 4; ++i) O17_0w[i] += tmp[i];

        std::array<std::array<double, 4>, 4> T19_18;
        if (sf == -1) {
            T19_18 = {{{0, 0, -1, -l_r/2}, {0, 1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        } else if (sf == 1) {
            T19_18 = {{{0, 0, 1, l_r/2}, {0, -1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        }


        auto T19_0 = MatMul4x4(T18_0, T19_18);
        auto O19_0 = MatMul(T19_0, O);
        auto O19_0w = MatMul(W, O19_0);
        for (int i = 0; i < 4; ++i) O19_0w[i] += tmp[i];

        std::array<std::array<double, 4>, 4> T20_18;
        if (sf == -1) {
            T20_18 = {{{0, 0, -1, l_r/2}, {0, 1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
        } else if (sf == 1) {
            T20_18 = {{{0, 0, 1, -l_r/2}, {0, -1, 0, 0}, {1, 0, 0, l_tr}, {0, 0, 0, 1}}};
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
        auto pt = MatMul(W, MatMul(T18_0, {0, 0, l_tr + l_hd, 1}));
        for (int i = 0; i < 4; ++i) pt[i] += tmp[i]; // head point

        std::array<double, 6> y = {tip_x, tip_y, 0, O0_0w[0], O0_0w[1], 0};
        

        // All joint positions of the skeleton
        P p;

        // Support leg
        //// heel
        p.tmp = tmp; 
        //// ankle
        p.O0_0w = O0_0w;
        p.O1_0w = O1_0w;
        //// pelvis
        p.O2_0w = O2_0w;
        p.O4_0w = O4_0w;
        p.O5_0w = O5_0w;
        
        // Swing leg
        //// heel
        p.O6_0w = O6_0w;
        ////ankle
        p.O7_0w = O7_0w;
        p.O8_0w = O8_0w;
        //// pelvis
        p.O9_0w = O9_0w;
        p.O11_0w = O11_0w;
        p.O12_0w = O12_0w;
        
        // Left shoulder
        p.O13_0w = O13_0w;
        // Right shoulder
        p.O17_0w = O17_0w;

        // CoM
        p.O18_0w = O18_0w;

        // Center of the shoulder
        p.pn = pn; 

        // head
        p.pt = pt; 

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


    // Deg to rad
    double deg2rad(double deg) {
        return deg * PI / 180.0;
    }


    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (sl), step width (sw), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the SINGLE support phase, which does not involve the switching of the support foot and swing foot
    std::pair<std::array<double, 6>, P> GaitSS(
        int sf, double R, double Gamma, double k, double sw,
        double sw_sh, double sl, const std::array<double,6>& res, double d_min,
        double H, double lean_angle, int index){


            std::vector<double> link(21);
            for (int i = 0; i < 21; ++i) {
                link[i] = ANATOMY[i] * H;
            }

            double L_l = link[1] + link[2]; // leg length
            double L_p = link[4];           // pelvis length
            double L_sh = link[14];         // shoulder length

            double psi_t, psi_s, theta, phi_a, phi_p;

            // index = 1: walk with rotation; index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle, psi_s: shoulder rotation angle (relative to the pelvis) 
            if (index == 0) {
                psi_t = 0;
                psi_s = psi_t;
        
                double tmp_1 = sl / (2 * L_l);
                theta = asin(tmp_1);
                double tmp_2 = (L_p - sw) / (2 * L_l);
                phi_a = asin(tmp_2 + deg2rad(lean_angle));
                phi_p = -deg2rad(lean_angle);
            } else {
                psi_t = index * acos(sw / L_p); 
        
                psi_s = index * acos(sw_sh / L_sh) - psi_t;
                double psi_t_sin = sqrt(1 - pow(sw / L_p, 2));
                double tmp_1 = (sl - L_p * psi_t_sin) / (2 * L_l);
                theta = asin(tmp_1);
                phi_a = deg2rad(lean_angle); // lean angle
                phi_p = -phi_a;
            }

            // double sx = std::max(0.0, sl - d_min);

            double psi_r = 0; 
            Gamma += psi_r; // gamma in Fig.2(b) in Shang et al. 2025, indicating the target direction
            double th1 = PI / 2 - theta;
            double th3 = theta;
            double th4 = PI + theta;
            double th6 = -theta;
            std::array<double, 6> th = {th1, 0, th3, th4, 0, th6};
        
            std::array<double, 5> phi;
            if (sf == -1) {
                phi = {-phi_a, (phi_a + phi_p), (PI / 2 + phi_a + phi_p), (-phi_a - phi_p), -phi_a};
            } else if (sf == 1) {
                phi = {phi_a, (-phi_a - phi_p), (-PI / 2 - phi_a - phi_p), (phi_a + phi_p), phi_a};
            }
        
            std::array<double, 3> Psi = {psi_t + psi_r, -psi_t, psi_s};
        
            std::vector<double> v(res.begin(), res.end());

            std::array<double, 3> sp = {v[3], v[4], v[5]};
            std::array<double, 21> link_array;
            std::copy(link.begin(), link.end(), link_array.begin());

            return FuncMotion(link_array, sf, sp, Gamma, th, phi, Psi);

        }

    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (sl), step width (sw), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the DOUBLE support phase, which involves the switching of the support foot and swing foot
    std::tuple<char, std::array<double, 6>, double, P> GaitDS(
        int sf, double R, double Gamma, double k, double sw,
        double sw_sh, double sl, const std::array<double,6>& res, double d_min,
        double H, int index){


            std::vector<double> link(21);
            for (int i = 0; i < 21; ++i) {
                link[i] = ANATOMY[i] * H;
            }

            double L_l = link[1] + link[2]; // leg length
            double L_p = link[4];           // pelvis length
            double L_sh = link[14];         // shoulder length

            double psi_t, psi_s, theta, phi_a, phi_p;

            // index = 1: walk with rotation; index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle, psi_s: shoulder rotation angle (relative to the pelvis) 
            if (index == 0) {
                psi_t = 0;
                psi_s = psi_t;
        
                double tmp_1 = sl / (2 * L_l);
                theta = asin(tmp_1);
                double tmp_2 = (L_p - sw) / (2 * L_l);
                phi_a = asin(tmp_2);
                phi_p = 0;
            } else {
                psi_t = index * acos(sw / L_p); 
        
                psi_s = index * acos(sw_sh / L_sh) - psi_t;
                double psi_t_sin = sqrt(1 - pow(sw / L_p, 2));
                double tmp_1 = (sl - L_p * psi_t_sin) / (2 * L_l);
                theta = asin(tmp_1);
                phi_a = 0; // lean angle
                phi_p = -phi_a;
            }

            // double sx = std::max(0.0, sl - d_min);

          
            double psi_r = 0; 
            Gamma += psi_r; // gamma in Fig.2(b) in Shang et al. 2025, indicating the target direction


            double th1 = PI / 2 - theta;
            double th3 = theta;
            double th4 = PI + theta;
            double th6 = -theta;
            std::array<double, 6> th = {th1, 0, th3, th4, 0, th6};
        
            std::array<double, 5> phi;
            if (sf == -1) {
                phi = {-phi_a, (phi_a + phi_p), (PI / 2 + phi_a + phi_p), (-phi_a - phi_p), -phi_a};
            } else if (sf == 1) {
                phi = {phi_a, (-phi_a - phi_p), (-PI / 2 - phi_a - phi_p), (phi_a + phi_p), phi_a};
            }
        
            std::array<double, 3> Psi = {psi_t + psi_r, -psi_t, psi_s};
        
            std::vector<double> v(res.begin(), res.end());

            std::array<double, 3> sp = {v[3], v[4], v[5]};
            std::array<double, 21> link_array;
            std::copy(link.begin(), link.end(), link_array.begin());
            auto funcMotionResult = FuncMotion(link_array, sf, sp, Gamma, th, phi, Psi);
            auto motionResult = funcMotionResult.first; 
            auto position = funcMotionResult.second;

            // switch the support foot
            int new_sf;

            if (sf == 1) {
                new_sf = -1;
            } else {
                new_sf = 1;
            }
            
            
            std::array<double, 6> res_array;
            std::copy_n(motionResult.begin(), 6, res_array.begin());

            auto final_res = FuncFootDH(link_array, Gamma, res_array);
        
           

            return std::make_tuple(new_sf, final_res, Gamma, position);

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
    // R == 0: straight walk

    double R = 0.0, Gamma = PI/2, k = 0.0, sw = 0.2, sw_sh = 0.45, sl = max_step_lenght, d_min = 0.0, H = model.height, la = 2, min_d = 0;
    // index = 1: walk with rotation; index = 0: walk without rotationb (turning)
    int index = 0;
    double step_duration = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));
    std::array<double, 6> res = {model.heel_right_position.x, model.heel_right_position.y, 0.0, model.heel_left_position.x, model.heel_left_position.y, 0.0};

    // Steps computation
    if (model.step_timer == 0) {

        // here we are in a double support configuration, the next step is computed based on the current walking speed
        //test


        // # computation of the next step duration (step_timer is the step duration given as a number of time step)
        // constant stepping time of 0.5 for an agent of 1.7m
        update.step_timer = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));


        // // # update stepping foot as the further from the step target
        // const double distance_to_taget_right_heel = Distance(update.step_target,model.heel_right_position);
        // const double distance_to_taget_left_heel = Distance(update.step_target,model.heel_left_position);
        
        // if (distance_to_taget_right_heel >= distance_to_taget_left_heel) {
        //     update.stepping_foot_index = -1; 
        // } else {
        //     update.stepping_foot_index = 1;
            
        // }
        // //  -1 == right foot stepping, 0 == double stance, 1 == left foot stepping

        // // # computation of the next step location
        // update.step_target=update.position + orientation * max_step_lenght + normal_to_orientation * update.stepping_foot_index*0.15*(model.height/1.7); 

        // // save the curent feet locations
        // update.heel_right_position=model.heel_right_position;
        // update.heel_left_position=model.heel_left_position;

        //sf: -1 == left foot support, 1 == right foot support
        if (model.sf == -1) {
            res[0] = model.heel_right_position.x;
            res[1] = model.heel_right_position.y;
            res[2] = 0;
            res[3] = model.heel_left_position.x;
            res[4] = model.heel_left_position.y;
            res[5] = 0;
        } else {
            res[0] = model.heel_left_position.x;
            res[1] = model.heel_left_position.y;
            res[2] = 0;
            res[3] = model.heel_right_position.x;
            res[4] = model.heel_right_position.y;
            res[5] = 0;
        }

        auto [output_sf, output_res, output_Gamma, output_position] = GaitDS(model.sf, R, Gamma, k, sw, sw_sh, sl, res, d_min, H, index);
        Gamma = output_Gamma;

        update.sf = output_sf;
        update.position.x = output_position.O18_0w[0];
        update.position.y = output_position.O18_0w[1];
        update.head_position.x = output_position.pt[0];
        update.head_position.y = output_position.pt[1];

        //sf: -1 == left foot support, 1 == right foot support
        if (update.sf == -1) {
            update.heel_right_position.x = output_res[0];
            update.heel_right_position.y = output_res[1];
            update.heel_left_position.x = output_res[3];
            update.heel_left_position.y = output_res[4];
        } else {
            update.heel_right_position.x = output_res[3];
            update.heel_right_position.y = output_res[4];
            update.heel_left_position.x = output_res[0];
            update.heel_left_position.y = output_res[1];
        }
        printf("sf: %d\n", update.sf);
        printf("res: %f, %f, %f, %f, %f, %f\n", res[0], res[1], res[2], res[3], res[4], res[5]);
        printf("output_position: %f, %f, %f\n", output_position.pt[0], output_position.pt[1], output_position.O18_0w[0]);
    } 
    else {
        
        double sl_p, lean_angle, sw_p, sw_sh_p, R_p;
        update.step_timer = model.step_timer;
        update.sf = model.sf;
        double tmp = 1 - update.step_timer/step_duration;

        // index = 1: walk with rotation; index = 0: walk without rotationb (turning)
        if (index != 0) {
            if (update.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(tmp, 2) * (max_step_lenght + max_step_lenght) - max_step_lenght;
                lean_angle  =  2 * tmp * la;
            }
            else
            {
                sl_p = max_step_lenght - 2 * std::pow(1-tmp, 2) * (2 * max_step_lenght);
                lean_angle  =  2 * la - 2 * tmp * la;
            }
        } 
        else
        {
            if (update.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(tmp, 2) * (max_step_lenght + max_step_lenght) - max_step_lenght;
                lean_angle  =  2 * tmp * la;
            }
            else
            {
                sl_p = max_step_lenght - 2 * std::pow(1-tmp, 2) * (2 * max_step_lenght);
                lean_angle  =  2 * la - 2 * tmp * la;
            }
        }
        
        //sf: -1 == left foot support, 1 == right foot support

        if (model.sf == -1) {
            res[0] = model.heel_right_position.x;
            res[1] = model.heel_right_position.y;
            res[2] = 0;
            res[3] = model.heel_left_position.x;
            res[4] = model.heel_left_position.y;
            res[5] = 0;
        } else {
            res[0] = model.heel_left_position.x;
            res[1] = model.heel_left_position.y;
            res[2] = 0;
            res[3] = model.heel_right_position.x;
            res[4] = model.heel_right_position.y;
            res[5] = 0;
        }
        auto [output_res, output_position] = GaitSS(update.sf, R, Gamma, k, sw, sw_sh, sl_p, res, min_d, H, lean_angle, index);
   
        update.position.x = output_position.O18_0w[0];
        update.position.y = output_position.O18_0w[1];
        update.head_position.x = output_position.pt[0];
        update.head_position.y = output_position.pt[1];
        printf("output_position: %f, %f, %f\n", output_position.pt[0], output_position.pt[1], output_position.O18_0w[0]);
        // printf("head_x: %f, head_y: %f\n", update.head_position.x, update.head_position.y);
        //sf: -1 == left foot support, 1 == right foot support
        if (model.sf == -1) {
            update.heel_right_position.x = output_res[0];
            update.heel_right_position.y = output_res[1];
            update.heel_left_position.x = output_res[3];
            update.heel_left_position.y = output_res[4];
        } else {
            update.heel_right_position.x = output_res[3];
            update.heel_right_position.y = output_res[4];
            update.heel_left_position.x = output_res[0];
            update.heel_left_position.y = output_res[1];
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
    // update.sf = model.sf;
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
    model.sf = upd.sf;
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
    model.sf = upd.sf;
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
