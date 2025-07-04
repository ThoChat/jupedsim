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
#include <cmath>
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


    #include "HumanoidModelV0Data.hpp"
    // Get reference to the class instance
    const HumanoidModelV0& humanoid = HumanoidModelV0(0.0, 0.0); // Temporary to have access to DHTransformationMatrix function
        
   
    // /*** Matrix opperators using Eigen ***/
    // // Denavit-Hartenberg (DH) convention
    // Eigen::Matrix4d DHTransformationMatrix(double theta, double d, double a, double alpha) {
    //     Eigen::Matrix4d mat;
    //     mat << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
    //             sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
    //             0, sin(alpha), cos(alpha), d,
    //             0, 0, 0, 1;
    //     return mat;
    // }

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
        std::array<double, 4> O0_0w;
        std::array<double, 4> O3_0w;
        std::array<double, 4> O4_0w;
        std::array<double, 4> O6_0w;
        std::array<double, 4> O7_0w;
        std::array<double, 4> O9_0w;
        std::array<double, 4> pf_0_1_w;
        std::array<double, 4> pf_0_2_w;
        std::array<double, 4> pf_0_3_w;
        std::array<double, 4> pf_0_4_w;
        std::array<double, 4> pf_10_1_w;
        std::array<double, 4> pf_10_2_w;
        std::array<double, 4> pf_10_3_w;
        std::array<double, 4> pf_10_4_w;
        std::array<double, 4> O14_0w;
        std::array<double, 4> O15_0w;
    };


    // This function is used to calculate the position of the body Segments given the joint coordinates (theta, phi, Psi);

    std::pair<std::array<double, 6>, P> FuncMotion(
        
        double ankle_length,
        double leg_length ,
        double pelvis_length ,
        double neck_length ,
        double shoulder_length ,
        double trunk_length ,
        double trunk_width ,
        double foot_forward_length ,
        double foot_backward_length ,
        double foot_inner_length ,
        double foot_outer_length ,
        

        int stepping_foot_index, 
        Eigen:: Vector3d support_foot_position_eigen, 
        double support_foot_orientation,
        Eigen::VectorXd theta_eigen,
        Eigen::VectorXd phi_eigen, 
        Eigen::Vector3d Psi_eigen) {
        
        

        // Joint angles
        double th1 = theta_eigen[0], th3 = theta_eigen[2], th4 = theta_eigen[3], th6 = theta_eigen[5];
        double phi1 = phi_eigen[0], phi2 = phi_eigen[1], phi3 = phi_eigen[2], phi4 = phi_eigen[3];
        double Psi1 = Psi_eigen[0], Psi2 = Psi_eigen[1], Psi3 = Psi_eigen[2];
    
        // std::cout << " " << std::endl;
        // std::cout << "### old joint angles in FuncMotion" << std::endl;
        // std::cout << "stepping_foot_index = " << stepping_foot_index << std::endl;
        // if (stepping_foot_index == -1) { // right foot stepping, left foot suport
        //     std::cout << "right ankle: (" << phi4 << ", " << th6 << ", " << 0 <<  ")" << std::endl;
        //     std::cout << "right hip: (" << phi3 << ", " << th4 << ", " << Psi2 <<  ")" << std::endl;
        //     std::cout << "left hip: ("  << phi2 << ", " << th3 << ", " << Psi1 << ")" << std::endl;
        //     std::cout << "left ankle: (" << phi1 << ", " << th1 << ", " << 0 << ")" << std::endl;
        //     std::cout << "pelvis: (" << 0 << ", " << 0 << ", " << Psi3 << ")" << std::endl;
        // } else if (stepping_foot_index == 1) { // left foot stepping, right foot suport
        //     std::cout << "right ankle: (" << phi1 << ", " << th1 << ", " << 0 << ")" << std::endl;
        //     std::cout << "right hip: (" << phi2 << ", " << th3 << ", " << Psi1 << ")" << std::endl;
        //     std::cout << "left hip: (" << phi3 << ", " << th4 << ", " << Psi2 << ")" << std::endl;
        //     std::cout << "left ankle: (" << phi4 << ", " << th6 << ", " << 0 << ")" << std::endl;
        //     std::cout << "pelvis: (" << 0 << ", " << 0 << ", " << Psi3 << ")" << std::endl;
        // }
        // std::cin.get(); // pause
        // system("clear"); // clean terminal output
        

        
        //## Rewriting fuction using Eigen ##

        // support_foot_orientation
        Eigen::Vector4d heel_support_eigen = {support_foot_position_eigen[0], support_foot_position_eigen[1], support_foot_position_eigen[2], 1.0};
        Eigen::Matrix4d W_eigen ;
        W_eigen <<      -sin(support_foot_orientation), 0, cos(support_foot_orientation), 0,
                        cos(support_foot_orientation), 0, sin(support_foot_orientation), 0,
                        0, 1, 0, ankle_length,
                        0, 0, 0, 1;
                        

        Eigen::Vector4d O0_0_eigen = {0, 0, 0, 1}; 
        Eigen::Vector4d O0_0w_eigen = W_eigen * O0_0_eigen;
        O0_0w_eigen = O0_0w_eigen + heel_support_eigen;



        // To Do: add a description of what are the B1, B2, B3,... , B10 matrices
        Eigen::Matrix4d B1_eigen = humanoid.DHTransformationMatrix(phi1 + M_PI / 2, 0, 0, M_PI / 2);
        Eigen::Matrix4d T1_0_eigen = B1_eigen;
        Eigen::Vector4d O1_0_eigen = T1_0_eigen * O0_0_eigen;
        Eigen::Vector4d O1_0w_eigen = W_eigen * O1_0_eigen;
        O1_0w_eigen = O1_0w_eigen + heel_support_eigen;
        

        Eigen::Matrix4d B2_eigen = humanoid.DHTransformationMatrix(-th1 + M_PI / 2, 0, leg_length, 0);
        Eigen::Matrix4d T2_0_eigen = T1_0_eigen * B2_eigen;
        Eigen::Vector4d O2_0_eigen = T2_0_eigen * O0_0_eigen;
        Eigen::Vector4d O2_0w_eigen = W_eigen * O2_0_eigen;
        O2_0w_eigen = O2_0w_eigen + heel_support_eigen;

        
        
        Eigen::Matrix4d B3_eigen = humanoid.DHTransformationMatrix(-th3, 0, 0, -M_PI / 2);
        Eigen::Matrix4d T3_0_eigen = T2_0_eigen * B3_eigen;
        Eigen::Vector4d O3_0_eigen = T3_0_eigen * O0_0_eigen;
        Eigen::Vector4d O3_0w_eigen = W_eigen * O3_0_eigen;
        O3_0w_eigen = O3_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d B4_eigen = humanoid.DHTransformationMatrix(phi2, 0, 0, 0);
        Eigen::Matrix4d rotMat_eigen;
        rotMat_eigen << 0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;
        Eigen::Matrix4d T4_0_eigen = T3_0_eigen * B4_eigen * rotMat_eigen;
        Eigen::Vector4d O4_0_eigen = T4_0_eigen * O0_0_eigen;
        Eigen::Vector4d O4_0w_eigen = W_eigen * O4_0_eigen;
        O4_0w_eigen = O4_0w_eigen + heel_support_eigen;
        

        Eigen::Matrix4d B5_eigen;
        if (stepping_foot_index == -1) {
            B5_eigen = humanoid.DHTransformationMatrix(Psi1, 0, pelvis_length, 0);
        } else if (stepping_foot_index == 1) {
            B5_eigen = humanoid.DHTransformationMatrix(Psi1 + M_PI, 0, pelvis_length, 0);
        }
        Eigen::Matrix4d T5_0_eigen = T4_0_eigen * B5_eigen;
        Eigen::Vector4d O5_0_eigen = T5_0_eigen * O0_0_eigen;
        Eigen::Vector4d O5_0w_eigen = W_eigen * O5_0_eigen;
        O5_0w_eigen = O5_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d B6_eigen;
        Eigen::Matrix4d T6_0_eigen;
        Eigen::Matrix4d transMat_eigen;
        // ThoChat: Why should we bother keeping this identity matrix?
        transMat_eigen <<   1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
        if (stepping_foot_index == -1) {
            B6_eigen = humanoid.DHTransformationMatrix(Psi2, 0, 0, -M_PI / 2);
            T6_0_eigen = T5_0_eigen * B6_eigen * transMat_eigen;
        } else if (stepping_foot_index == 1) {
            B6_eigen = humanoid.DHTransformationMatrix(Psi2, 0, 0, M_PI / 2);
            T6_0_eigen = T5_0_eigen * B6_eigen * transMat_eigen;
        }
        Eigen::Vector4d O6_0_eigen = T6_0_eigen * O0_0_eigen;
        Eigen::Vector4d O6_0w_eigen = W_eigen * O6_0_eigen;
        O6_0w_eigen = O6_0w_eigen + heel_support_eigen;
        
        Eigen::Matrix4d B7_eigen = humanoid.DHTransformationMatrix(phi3, 0, 0, -M_PI / 2);
        Eigen::Matrix4d T7_0_eigen = T6_0_eigen * B7_eigen;
        Eigen::Vector4d O7_0_eigen = T7_0_eigen * O0_0_eigen;
        Eigen::Vector4d O7_0w_eigen = W_eigen * O7_0_eigen;
        O7_0w_eigen = O7_0w_eigen + heel_support_eigen;
        
        Eigen::Matrix4d B8_eigen = humanoid.DHTransformationMatrix(-th4 + M_PI, 0, leg_length, 0);
        Eigen::Matrix4d T8_0_eigen = T7_0_eigen * B8_eigen;
        Eigen::Vector4d O8_0_eigen = T8_0_eigen * O0_0_eigen;
        Eigen::Vector4d O8_0w_eigen = W_eigen * O8_0_eigen;
        O8_0w_eigen = O8_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d B9_eigen = humanoid.DHTransformationMatrix(-th6, 0, 0, M_PI / 2);
        Eigen::Matrix4d T9_0_eigen = T8_0_eigen * B9_eigen;
        Eigen::Vector4d O9_0_eigen = T9_0_eigen * O0_0_eigen;
        Eigen::Vector4d O9_0w_eigen = W_eigen * O9_0_eigen;
        O9_0w_eigen = O9_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d B10_eigen = humanoid.DHTransformationMatrix(phi4, 0, ankle_length, 0);
        Eigen::Matrix4d T10_0_eigen = T9_0_eigen * B10_eigen;
        Eigen::Vector4d O10_0_eigen = T10_0_eigen * O0_0_eigen;
        Eigen::Vector4d O10_0w_eigen = W_eigen * O10_0_eigen;
        O10_0w_eigen = O10_0w_eigen + heel_support_eigen;

        // support foot
        Eigen::Vector4d pf_0_1_eigen = {foot_outer_length, -ankle_length, foot_forward_length, 1.0};
        Eigen::Vector4d pf_0_2_eigen = {-foot_inner_length, -ankle_length, foot_forward_length, 1.0};
        Eigen::Vector4d pf_0_3_eigen = {-foot_inner_length, -ankle_length, -foot_backward_length, 1.0};
        Eigen::Vector4d pf_0_4_eigen = {foot_outer_length, -ankle_length, -foot_backward_length, 1.0};
        Eigen::Vector4d pf_0_1_w_eigen = W_eigen * pf_0_1_eigen;
        Eigen::Vector4d pf_0_2_w_eigen = W_eigen * pf_0_2_eigen;
        Eigen::Vector4d pf_0_3_w_eigen = W_eigen * pf_0_3_eigen;
        Eigen::Vector4d pf_0_4_w_eigen = W_eigen * pf_0_4_eigen;
        pf_0_1_w_eigen = pf_0_1_w_eigen + heel_support_eigen;
        pf_0_2_w_eigen = pf_0_2_w_eigen + heel_support_eigen;
        pf_0_3_w_eigen = pf_0_3_w_eigen + heel_support_eigen;
        pf_0_4_w_eigen = pf_0_4_w_eigen + heel_support_eigen;

        // swing foot
        Eigen::Vector4d pf_10_1_eigen = {0, foot_inner_length, foot_forward_length, 1.0};
        Eigen::Vector4d pf_10_2_eigen = {0, -foot_outer_length, foot_forward_length, 1.0};
        Eigen::Vector4d pf_10_3_eigen = {0, -foot_outer_length, -foot_backward_length, 1.0};
        Eigen::Vector4d pf_10_4_eigen = {0, foot_inner_length, -foot_backward_length, 1.0};
        Eigen::Vector4d pf_10_1_w_eigen = W_eigen * pf_10_1_eigen;
        Eigen::Vector4d pf_10_2_w_eigen = W_eigen * pf_10_2_eigen;
        Eigen::Vector4d pf_10_3_w_eigen = W_eigen * pf_10_3_eigen;
        Eigen::Vector4d pf_10_4_w_eigen = W_eigen * pf_10_4_eigen;
        pf_10_1_w_eigen = pf_10_1_w_eigen + heel_support_eigen;
        pf_10_2_w_eigen = pf_10_2_w_eigen + heel_support_eigen;
        pf_10_3_w_eigen = pf_10_3_w_eigen + heel_support_eigen;
        pf_10_4_w_eigen = pf_10_4_w_eigen + heel_support_eigen;

        Eigen::Matrix4d B11_eigen;
        B11_eigen <<    1, 0, 0, -pelvis_length/2,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1; 
        
        B11_eigen = B11_eigen * humanoid.DHTransformationMatrix(Psi3, 0, 0, 0);
        Eigen::Matrix4d T11_0_eigen = T5_0_eigen * B11_eigen;
        Eigen::Vector4d O11_0_eigen = T11_0_eigen * O0_0_eigen;
        Eigen::Vector4d O11_0w_eigen = W_eigen * O11_0_eigen;
        O11_0w_eigen = O11_0w_eigen + heel_support_eigen;
        
        // To Do: gather all if condition uder one if condition
        Eigen::Matrix4d T12_11_eigen;
        if (stepping_foot_index == -1) {
            T12_11_eigen << 0, 0, -1, -shoulder_length/2,
                            0, 1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        } else if (stepping_foot_index == 1) {
            T12_11_eigen << 0, 0, 1, shoulder_length/2,
                            0, -1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        }

        Eigen::Matrix4d T12_0_eigen = T11_0_eigen * T12_11_eigen;
        Eigen::Vector4d O12_0_eigen = T12_0_eigen * O0_0_eigen;
        Eigen::Vector4d O12_0w_eigen = W_eigen * O12_0_eigen;
        O12_0w_eigen = O12_0w_eigen + heel_support_eigen;
        

        Eigen::Matrix4d T13_11_eigen;
        if (stepping_foot_index == -1) {
            T13_11_eigen << 0, 0, -1, shoulder_length/2,
                            0, 1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        } else if (stepping_foot_index == 1) {
            T13_11_eigen << 0, 0, 1, -shoulder_length/2,
                            0, -1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        }
        Eigen::Matrix4d T13_0_eigen = T11_0_eigen * T13_11_eigen;
        Eigen::Vector4d O13_0_eigen = T13_0_eigen * O0_0_eigen;
        Eigen::Vector4d O13_0w_eigen = W_eigen * O13_0_eigen;
        O13_0w_eigen = O13_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d T14_11_eigen;
        if (stepping_foot_index == -1) {
            T14_11_eigen << 0, 0, -1, -trunk_width/2,
                            0, 1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        } else if (stepping_foot_index == 1) {
            T14_11_eigen << 0, 0, 1, trunk_width/2,
                            0, -1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        }
        Eigen::Matrix4d T14_0_eigen = T11_0_eigen * T14_11_eigen;
        Eigen::Vector4d O14_0_eigen = T14_0_eigen * O0_0_eigen;
        Eigen::Vector4d O14_0w_eigen = W_eigen * O14_0_eigen;
        O14_0w_eigen = O14_0w_eigen + heel_support_eigen;

        Eigen::Matrix4d T15_11_eigen;
        if (stepping_foot_index == -1) {
            T15_11_eigen << 0, 0, -1, trunk_width/2,
                            0, 1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        } else if (stepping_foot_index == 1) {
            T15_11_eigen << 0, 0, 1, -trunk_width/2,
                            0, -1, 0, 0,
                            1, 0, 0, trunk_length,
                            0, 0, 0, 1;
        }
        Eigen::Matrix4d T15_0_eigen = T11_0_eigen * T15_11_eigen;
        Eigen::Vector4d O15_0_eigen = T15_0_eigen * O0_0_eigen;
        Eigen::Vector4d O15_0w_eigen = W_eigen * O15_0_eigen;
        O15_0w_eigen = O15_0w_eigen + heel_support_eigen;

        // neck point
        Eigen::Vector4d center_of_shoulder_eigen = {
            (O12_0w_eigen[0] + O13_0w_eigen[0]) / 2,
            (O12_0w_eigen[1] + O13_0w_eigen[1]) / 2,
            (O12_0w_eigen[2] + O13_0w_eigen[2]) / 2,
            (O12_0w_eigen[3] + O13_0w_eigen[3]) / 2,
        };

        Eigen::Vector4d head_eigen = W_eigen * (T11_0_eigen * Eigen::Vector4d{0, 0, trunk_length + neck_length, 1});
        head_eigen = head_eigen + heel_support_eigen;

        Eigen::VectorXd y_eigen(6);
        y_eigen << O10_0w_eigen[0], O10_0w_eigen[1], 0, O0_0w_eigen[0], O0_0w_eigen[1], 0;


        // All joint positions of the skeleton
        P p;

        ///////////// Support leg///////////////
        //// heel
        p.heel_support = {heel_support_eigen(0), heel_support_eigen(1), heel_support_eigen(2), heel_support_eigen(3)}; 


        //// Ankle

        // O0_0w and O1_0w are the same, that means O0_0w = O1_0w
        // We can ignore O0_0w OR O1_0w when we just want to get the position of the ankle
        // But both of them are necessary for the calculation

        // p.O0_0w = O0_0w;
        p.ankle_support = {O1_0w_eigen(0), O1_0w_eigen(1), O1_0w_eigen(2), O1_0w_eigen(3)}; // ankle point


        //// pelvis

        // O2_0w = O3_0w = O4_0w

        p.hip_support = {O2_0w_eigen(0), O2_0w_eigen(1), O2_0w_eigen(2), O2_0w_eigen(3)}; // hip point
        // p.O3_0w = O3_0w; 
        // p.O4_0w = O4_0w;
        

        ////////// Swing leg////////////

        //// pelvis

        // O5_0w = O6_0w = O7_0w

        p.hip_swing = {O5_0w_eigen(0), O5_0w_eigen(1), O5_0w_eigen(2), O5_0w_eigen(3)}; // hip point
        // p.O6_0w = O6_0w;
        // p.O7_0w = O7_0w;

        // O8_0w = O9_0w
        p.ankle_swing = {O8_0w_eigen(0), O8_0w_eigen(1), O8_0w_eigen(2), O8_0w_eigen(3)}; // ankle point
        // p.O9_0w = O9_0w;

        p.heel_swing = {O10_0w_eigen(0), O10_0w_eigen(1), O10_0w_eigen(2), O10_0w_eigen(3)}; // heel point

        ////////////// CoM //////////////////
        p.center_of_mass = {O11_0w_eigen(0), O11_0w_eigen(1), O11_0w_eigen(2), O11_0w_eigen(3)}; // CoM point


        ////////////// Shoulder /////////////

        // Left shoulder
        p.shoulder_left = {O12_0w_eigen(0), O12_0w_eigen(1), O12_0w_eigen(2), O12_0w_eigen(3)};
        // Right shoulder
        p.shoulder_right = {O13_0w_eigen(0), O13_0w_eigen(1), O13_0w_eigen(2), O13_0w_eigen(3)};
        // Center of the shoulder
        p.center_of_shoulder = {center_of_shoulder_eigen(0), center_of_shoulder_eigen(1), center_of_shoulder_eigen(2), center_of_shoulder_eigen(3)}; // neck point

        // The center of the two small circles
        // used when representing the agent body as a three-circled shape
        p.O14_0w = {O14_0w_eigen(0), O14_0w_eigen(1), O14_0w_eigen(2), O14_0w_eigen(3)}; 
        p.O15_0w = {O15_0w_eigen(0), O15_0w_eigen(1), O15_0w_eigen(2), O15_0w_eigen(3)};

        // head
        p.head = {head_eigen(0), head_eigen(1), head_eigen(2), head_eigen(3)}; // head point

        // CoM
        // p.pc = pc; 

        ///////////// The four corners of the foot * 2 /////////////////////
        // Support foot
        p.pf_0_1_w = {pf_0_1_w_eigen(0), pf_0_1_w_eigen(1), pf_0_1_w_eigen(2), pf_0_1_w_eigen(3)};
        p.pf_0_2_w = {pf_0_2_w_eigen(0), pf_0_2_w_eigen(1), pf_0_2_w_eigen(2), pf_0_2_w_eigen(3)};  
        p.pf_0_3_w = {pf_0_3_w_eigen(0), pf_0_3_w_eigen(1), pf_0_3_w_eigen(2), pf_0_3_w_eigen(3)};
        p.pf_0_4_w = {pf_0_4_w_eigen(0), pf_0_4_w_eigen(1), pf_0_4_w_eigen(2), pf_0_4_w_eigen(3)};
        // Swing foot
        p.pf_10_1_w = {pf_10_1_w_eigen(0), pf_10_1_w_eigen(1), pf_10_1_w_eigen(2), pf_10_1_w_eigen(3)};
        p.pf_10_2_w = {pf_10_2_w_eigen(0), pf_10_2_w_eigen(1), pf_10_2_w_eigen(2), pf_10_2_w_eigen(3)};
        p.pf_10_3_w = {pf_10_3_w_eigen(0), pf_10_3_w_eigen(1), pf_10_3_w_eigen(2), pf_10_3_w_eigen(3)};
        p.pf_10_4_w = {pf_10_4_w_eigen(0), pf_10_4_w_eigen(1), pf_10_4_w_eigen(2), pf_10_4_w_eigen(3)};




   
        return {
            {y_eigen(0), y_eigen(1), y_eigen(2), y_eigen(3), y_eigen(4), y_eigen(5)},
            p
        };
    }



    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (step_length), step width (step_width), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the SINGLE support phase, which does not involve the switching of the support foot and swing foot
    std::pair<std::array<double, 6>, P> GaitSingleSupport(
        double height, // parameter
        int stepping_foot_index, // parameter
        double delta_orientation, // variable 
        double support_foot_orientation , // variable
        double step_width, // parameter
        double shoulder_width, // parameter
        double step_length, // variable
        const std::array<double,6>& feet_position, // variable
        double lean_angle, // variable
        int rotation_index){


            // limb dimensions calulation based on agent's height
            double ankle_length = height * HumanoidModelV0::ANKLE_SCALING_FACTOR;
            double leg_length = height * HumanoidModelV0::LEG_SCALING_FACTOR;
            double pelvis_length = height * HumanoidModelV0::PELVIS_WIDTH_SCALING_FACTOR;
            double neck_length = height * HumanoidModelV0::NECK_SCALING_FACTOR;
            double shoulder_length = height * HumanoidModelV0::SHOULDER_WIDTH_SCALING_FACTOR;
            double trunk_length = height * HumanoidModelV0::TRUNK_HEIGHT_SCALING_FACTOR;
            double trunk_width = height * HumanoidModelV0::TRUNK_WIDTH_SCALING_FACTOR;
            double foot_forward_length = height * HumanoidModelV0::FOOT_FORWARD_SCALING_FACTOR;
            double foot_backward_length = height * HumanoidModelV0::FOOT_BACKWARD_SCALING_FACTOR;
            double foot_inner_length = height * HumanoidModelV0::FOOT_WIDTH_SCALING_FACTOR;
            double foot_outer_length = height * HumanoidModelV0::FOOT_WIDTH_SCALING_FACTOR;
            
            

            double psi_t, psi_s, theta, phi_a, phi_p;

            // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle
            // psi_s: shoulder rotation angle (relative to the pelvis) 
            if (rotation_index == 0) {
                psi_t = 0;
                psi_s = psi_t;
                
                theta = asin(step_length / (2 * leg_length));
                phi_a = asin(((pelvis_length - step_width) / (2 * leg_length)) + lean_angle*M_PI/180);
                phi_p = -lean_angle*M_PI/180;
            } else {
                psi_t = rotation_index * acos(step_width / pelvis_length); 
        
                psi_s = rotation_index * acos(shoulder_width / shoulder_length) - psi_t;
                double psi_t_sin = sqrt(1 - pow(step_width / pelvis_length, 2));
                theta = asin((step_length - pelvis_length * psi_t_sin) / (2 * leg_length));
                phi_a = lean_angle*M_PI/180; // lean angle
                phi_p = -phi_a;
            }
            
            support_foot_orientation += delta_orientation; // support_foot_orientation in Fig.2(b) in Shang et al. 2025, indicating the target direction
            double th1 = M_PI / 2 - theta;
            double th3 = theta;
            double th4 = M_PI + theta;
            double th6 = -theta;

            
            Eigen::VectorXd theta_eigen(6);
            theta_eigen << th1, 0, th3, th4, 0, th6; 
    

            Eigen::VectorXd phi_eigen(5);
            if (stepping_foot_index == -1) {
                phi_eigen << -phi_a, 
                (phi_a + phi_p), 
                (M_PI / 2 + phi_a + phi_p), 
                (-phi_a - phi_p), 
                -phi_a;
            } else if (stepping_foot_index == 1) {
                phi_eigen << phi_a, (-phi_a - phi_p), 
                (-M_PI / 2 - phi_a - phi_p), 
                (phi_a + phi_p), phi_a;
            }
        

            Eigen::Vector3d Psi_eigen = {psi_t + delta_orientation, -psi_t, psi_s};
            // std::vector<double> v(feet_position.begin(), feet_position.end());

            Eigen::Vector3d support_foot_position_eigen = {feet_position[3], feet_position[4], feet_position[5]};

            return FuncMotion( 
                ankle_length, 
                leg_length ,
                pelvis_length ,
                neck_length ,
                shoulder_length ,
                trunk_length ,
                trunk_width ,
                foot_forward_length ,
                foot_backward_length ,
                foot_inner_length ,
                foot_outer_length ,
                
                
                stepping_foot_index, support_foot_position_eigen, support_foot_orientation, theta_eigen, phi_eigen, Psi_eigen);

        }

        
    // This function is used to calculate the joint rotaion angles using the gait parameters like step length (step_length), step width (step_width), etc.
    // and then pass them to the function of FuncMotion to calculate the position of the joints
    // This function is utilized in the DOUBLE support phase, which involves the switching of the support foot and swing foot
    std::tuple<int, std::array<double, 6>, double, P> GaitDoubleSupports(
        double height,
        
        int stepping_foot_index,
        double delta_orientation, 
        double support_foot_orientation, 
        double step_width,
        double shoulder_width, 
        double step_length, 
        const std::array<double,6>& feet_position,
        int rotation_index){

            
            
            // limb dimensions calulation based on agent's height
            double ankle_length = height * HumanoidModelV0::ANKLE_SCALING_FACTOR;
            double leg_length = height * HumanoidModelV0::LEG_SCALING_FACTOR;
            double pelvis_length = height * HumanoidModelV0::PELVIS_WIDTH_SCALING_FACTOR;
            double neck_length = height * HumanoidModelV0::NECK_SCALING_FACTOR;
            double shoulder_length = height * HumanoidModelV0::SHOULDER_WIDTH_SCALING_FACTOR;
            double trunk_length = height * HumanoidModelV0::TRUNK_HEIGHT_SCALING_FACTOR;
            double trunk_width = height * HumanoidModelV0::TRUNK_WIDTH_SCALING_FACTOR;
            double foot_forward_length = height * HumanoidModelV0::FOOT_FORWARD_SCALING_FACTOR;
            double foot_backward_length = height * HumanoidModelV0::FOOT_BACKWARD_SCALING_FACTOR;
            double foot_inner_length = height * HumanoidModelV0::FOOT_WIDTH_SCALING_FACTOR;
            double foot_outer_length = height * HumanoidModelV0::FOOT_WIDTH_SCALING_FACTOR;
        

            double psi_t, psi_s, theta, phi_a, phi_p;

            // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotationb (turning)
            // psi_t: plevis rotation angle, psi_s: shoulder rotation angle (relative to the pelvis) 
            if (rotation_index == 0) {
                psi_t = 0;
                psi_s = psi_t;
        

                theta = asin(step_length / (2 * leg_length));;
                phi_a = asin((pelvis_length - step_width) / (2 * leg_length));
                phi_p = 0;
            } else {
                psi_t = rotation_index * acos(step_width / pelvis_length); 
        
                psi_s = rotation_index * acos(shoulder_width / shoulder_length) - psi_t;
                double psi_t_sin = sqrt(1 - pow(step_width / pelvis_length, 2));
                theta = asin((step_length - pelvis_length * psi_t_sin) / (2 * leg_length));
                phi_a = 0; // lean angle
                phi_p = -phi_a;
            }

            support_foot_orientation += delta_orientation; // support_foot_orientation in Fig.2(b) in Shang et al. 2025, indicating the target direction


            double th1 = M_PI / 2 - theta;
            double th3 = theta;
            double th4 = M_PI + theta;
            double th6 = -theta;

            Eigen::VectorXd theta_eigen(6);
            theta_eigen << th1, 0, th3, th4, 0, th6; 
            
            
            Eigen::VectorXd phi_eigen(5);
            if (stepping_foot_index == -1) {
                phi_eigen << -phi_a, (phi_a + phi_p), (M_PI / 2 + phi_a + phi_p), (-phi_a - phi_p), -phi_a;
            } else if (stepping_foot_index == 1) {
                phi_eigen << phi_a, (-phi_a - phi_p), (-M_PI / 2 - phi_a - phi_p), (phi_a + phi_p), phi_a;
            }
        
            Eigen:: Vector3d Psi_eigen = {psi_t + delta_orientation, -psi_t, psi_s};
        
            // std::vector<double> v(feet_position.begin(), feet_position.end());

            Eigen:: Vector3d support_foot_position_eigen = {feet_position[3], feet_position[4], feet_position[5]};
    
            auto funcMotionResult = FuncMotion(
                ankle_length, 
                leg_length ,
                pelvis_length ,
                neck_length ,
                shoulder_length ,
                trunk_length ,
                trunk_width ,
                foot_forward_length ,
                foot_backward_length ,
                foot_inner_length ,
                foot_outer_length ,
                 stepping_foot_index, 
                 support_foot_position_eigen, 
                 support_foot_orientation, 
                 theta_eigen, 
                 phi_eigen, 
                 Psi_eigen);
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
            
            // switch position of the support foot and swing foot
            //  used in the double support phase, after using the function of FuncMotion
            Eigen::Vector4d heel_support = {feet_pos[0], feet_pos[1], feet_pos[2], 1.0};
            Eigen::Matrix4d W ;
            W <<    -sin(support_foot_orientation), 0, cos(support_foot_orientation), 0,
                    cos(support_foot_orientation), 0, sin(support_foot_orientation), 0,
                    0, 1, 0, ankle_length,
                    0, 0, 0, 1;
            Eigen::Vector4d O0_0 = {0, 0, 0, 1}; 
            Eigen::Vector4d O0_0w = W * O0_0;
            O0_0w = O0_0w + heel_support;
            std::array<double, 6> final_res =  {feet_pos[3], feet_pos[4], 0, O0_0w[0], O0_0w[1], 0};
           

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

    // initialise the updated joint positions matrix
    Eigen::MatrixXd updated_joint_positions_matrix; 
    updated_joint_positions_matrix = Eigen::MatrixXd::Zero(11, 3); // 11 joints, 3 positions each

    const double max_step_length = model.height/2.5;
    
    double delta_orientation = 0.0;
    double support_foot_orientation = M_PI/2;
    double step_width = 0.2;
    double shoulder_width = 0.45;
    // rotation_index = 1: walk with rotation; 
    // rotation_index = 0: walk without rotation (turning)
    // delta_orientation == 0: straight walk
    int rotation_index = 0;
    double step_duration = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));
    // double step_length = update.velocity.Norm() * step_duration *dT;
    double step_length = max_step_length;

    std::array<double, 6> feet_position ;

    // Steps computation
    if (model.step_timer == 0) {

        // # computation of the next step duration (step_timer is the step duration given as a number of time step)
        // constant stepping time of 0.5 for an agent of 1.7m
        update.step_timer = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));

        // Update stepping_foot_index
        if (model.stepping_foot_index == 1) {
            update.stepping_foot_index = -1; // switch to right foot stepping
        } else {
            update.stepping_foot_index = 1;  // switch to left foot stepping
        }
        ///////////

        // testing new Gait function
        updated_joint_positions_matrix = ComputeJointAnglesGait(ped, update, dT);

        

        //stepping_foot_index: -1 == left foot support, 1 == right foot support
        if (model.stepping_foot_index == -1) {
            feet_position[0] = model.heel_right_position.x;
            feet_position[1] = model.heel_right_position.y;
            feet_position[2] = model.heel_right_position.z;
            feet_position[3] = model.heel_left_position.x;
            feet_position[4] = model.heel_left_position.y;
            feet_position[5] = model.heel_left_position.z;
        } else {
            feet_position[0] = model.heel_left_position.x;
            feet_position[1] = model.heel_left_position.y;
            feet_position[2] = model.heel_left_position.z;
            feet_position[3] = model.heel_right_position.x;
            feet_position[4] = model.heel_right_position.y;
            feet_position[5] = model.heel_right_position.z;
        }
        
        

      
        
        auto [output_stepping_foot_index, output_foot_position, output_support_foot_orientation, output_position] = GaitDoubleSupports(
                model.height, // parameter
                model.stepping_foot_index, // == 0
                delta_orientation,  // == 0
                support_foot_orientation, // pi//2 - why is this angle pi/2 // along y-axis is pi/2 (or -pi/2); along x-axis is 0 (or -pi)
                step_width, // 0.2
                shoulder_width, // 0.45
                step_length, // max_step_length = model.height/2.5
                feet_position, // heel position
                rotation_index); // == 0
        

        support_foot_orientation = output_support_foot_orientation;
        update.stepping_foot_index = output_stepping_foot_index;

        // Update the position using the center of mass
        update.position.x = output_position.center_of_mass[0];
        update.position.y = output_position.center_of_mass[1];


        update.head_position.x = output_position.head[0];
        update.head_position.y = output_position.head[1];


        // I don't think this is useful
        //stepping_foot_index: -1 == left foot support, 1 == right foot support
        if (update.stepping_foot_index == -1) {
            update.heel_right_position.x = output_foot_position[0];
            update.heel_right_position.y = output_foot_position[1];
            update.heel_right_position.z = output_foot_position[2];
            update.heel_left_position.x = output_foot_position[3];
            update.heel_left_position.y = output_foot_position[4];
            update.heel_left_position.z = output_foot_position[5];
        } else {
            update.heel_right_position.x = output_foot_position[3];
            update.heel_right_position.y = output_foot_position[4];
            update.heel_right_position.z = output_foot_position[5];
            update.heel_left_position.x = output_foot_position[0];
            update.heel_left_position.y = output_foot_position[1];
            update.heel_left_position.z = output_foot_position[2];
        }
        
    } 
    else {
        
        // pass on stepping_foot_index 
        update.stepping_foot_index = model.stepping_foot_index;
        // update step timer
        update.step_timer = model.step_timer - 1;
        ///////////


        // New Single Support Gait function
        updated_joint_positions_matrix = ComputeJointAnglesGait(ped, update, dT);





        double sl_p, lean_angle = 2 ;
        double step_complition_factor = 1 - model.step_timer/step_duration; 

        // rotation_index = 1: walk with rotation; rotation_index = 0: walk without rotation (turning)
        if (rotation_index != 0) {
            if (model.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(step_complition_factor, 2) * (max_step_length + max_step_length) - max_step_length;
                lean_angle  =  2 * step_complition_factor * lean_angle;
            }
            else
            {
                sl_p = max_step_length - 2 * std::pow(1-step_complition_factor, 2) * (2 * max_step_length);
                lean_angle  =  2 * lean_angle - 2 * step_complition_factor * lean_angle;
            }
        } 
        else
        {
            if (model.step_timer > step_duration/2) {

                sl_p = 2 * std::pow(step_complition_factor, 2) * (max_step_length + max_step_length) - max_step_length;
                lean_angle  =  2 * step_complition_factor * lean_angle;
            }
            else
            {
                sl_p = max_step_length - 2 * std::pow(1-step_complition_factor, 2) * (2 * max_step_length);
                lean_angle  =  2 * lean_angle - 2 * step_complition_factor * lean_angle;
            }
        }


        //stepping_foot_index: -1 == left foot support, 1 == right foot support

        if (model.stepping_foot_index == -1) {
            feet_position[0] = model.heel_right_position.x;
            feet_position[1] = model.heel_right_position.y;
            feet_position[2] = model.heel_right_position.z;
            feet_position[3] = model.heel_left_position.x;
            feet_position[4] = model.heel_left_position.y;
            feet_position[5] = model.heel_left_position.z;
        } else {
            feet_position[0] = model.heel_left_position.x;
            feet_position[1] = model.heel_left_position.y;
            feet_position[2] = model.heel_left_position.z;
            feet_position[3] = model.heel_right_position.x;
            feet_position[4] = model.heel_right_position.y;
            feet_position[5] = model.heel_right_position.z;
        }
        auto [output_foot_position, output_position] = GaitSingleSupport(
                model.height,
                update.stepping_foot_index, 
                delta_orientation, 
                support_foot_orientation, 
                step_width, 
                shoulder_width, 
                sl_p, 
                feet_position, 
                lean_angle, 
                rotation_index);

   
        update.position.x = output_position.center_of_mass[0];
        update.position.y = output_position.center_of_mass[1];

        update.head_position.x = output_position.head[0];
        update.head_position.y = output_position.head[1];
        update.head_position.z = output_position.head[2];

        if (model.stepping_foot_index == -1) {
            update.heel_right_position.x = output_foot_position[0];
            update.heel_right_position.y = output_foot_position[1];
            update.heel_right_position.z = output_foot_position[2];
            update.heel_left_position.x = output_foot_position[3];
            update.heel_left_position.y = output_foot_position[4];
            update.heel_left_position.z = output_foot_position[5];
        } else {
            update.heel_right_position.x = output_foot_position[3];
            update.heel_right_position.y = output_foot_position[4];
            update.heel_right_position.z = output_foot_position[5];
            update.heel_left_position.x = output_foot_position[0];
            update.heel_left_position.y = output_foot_position[1];
            update.heel_left_position.z = output_foot_position[2];
        }


        

    }

    // Update position of the joints 
    //  pelvis position
    // update.position = ped.pos + update.velocity * dT;
    update.position.x = updated_joint_positions_matrix(6, 0);
    update.position.y = updated_joint_positions_matrix(6, 1);
    // std::cout << "Pelvis Position: " << update.position.x << ", " << update.position.y << std::endl;
    

    // // right heel position
    update.heel_right_position.x = updated_joint_positions_matrix(0, 0);
    update.heel_right_position.y = updated_joint_positions_matrix(0, 1);
    update.heel_right_position.z = updated_joint_positions_matrix(0, 2);
    
    // // left heel position
    update.heel_left_position.x = updated_joint_positions_matrix(5, 0);
    update.heel_left_position.y = updated_joint_positions_matrix(5, 1);
    update.heel_left_position.z = updated_joint_positions_matrix(5, 2);

    // // head position
    // update.head_position.x = updated_joint_positions_matrix(10, 0);
    // update.head_position.y = updated_joint_positions_matrix(10, 1);
    // update.head_position.z = updated_joint_positions_matrix(10, 2);
    

    // ## head 
    // update.head_position = update.head_position;

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

    // print the updated joint positions
    // std::cout << "old Joint Positions :"  << std::endl;
    // std::cout << "Pelvis: " << update.position.x <<", "<< update.position.y << std::endl;
    // std::cout << "Right Heel: " << update.heel_right_position.x << ", " 
    //                             << update.heel_right_position.y << ", " 
    //                             << update.heel_right_position.z << std::endl;
    // std::cout << "Left Heel: "  << update.heel_left_position.x << ", " 
    //                             << update.heel_left_position.y << ", " 
    //                             << update.heel_left_position.z << std::endl;

    // // pause
    // std::cin.get();  
    // // clear the system console
    // std::system("clear");

 
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


/***************** Function For Humanoid motion *****************/

/*** Matrix opperators using Eigen ***/
// Denavit-Hartenberg (DH) convention
// compute T^{n-1}_n
Eigen::Matrix4d HumanoidModelV0::DHTransformationMatrix(
    double theta, 
    double d, 
    double r, 
    double alpha) const
    {
    Eigen::Matrix4d mat;
    mat << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta),
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;
    return mat;
}

/*** rotation matrix  ***/
 

Eigen::Matrix4d HumanoidModelV0::CreateTransformationMatrix(
    const Eigen::Vector3d &translation,
    const Eigen::Vector3d &rotation
) const {
    
    double roll = rotation[0], pitch = rotation[1], yaw = rotation[2];
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond MergedRotations = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d RotationMatrix = MergedRotations.toRotationMatrix();

    // Create transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0,0,3,3) = RotationMatrix;  // Or use rotationQuaternionsMethod
    T.block(0,3,3,1) = translation;
    
    return T;
}


// In this function the motion of a step is initiated after a double support phase.
// The function computes the joint angles then the joint position 
//  based on the step length, step width, and the current model state.
// should return: update.stepping_foot_index, update.step_timer update.joint_angles_matrix update.joint_positions_matrix
Eigen::MatrixXd HumanoidModelV0::ComputeJointAnglesStepDoubleSupports(
                                        const GenericAgent& agent,
                                        double step_length, 
                                        double step_width) const
{
    // Get the model data from the agent
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_angles_matrix(11, 3); // 11 joints, 3 angles each

    

    // Rotation 
    // only walk without rotation is implemented for now (rotation_index != 0)


    double theta, phi_a;

    theta = asin(step_length / (2 * model.height * LEG_SCALING_FACTOR));;
    phi_a = asin((model.height * PELVIS_WIDTH_SCALING_FACTOR  - step_width) 
            / (2 * model.height * LEG_SCALING_FACTOR));

    
    // Update joint angles
    if (model.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
        // right ankle
        updated_joint_angles_matrix.row(1) <<    phi_a,        
                                                - theta, 
                                                0; 
        // right hip
        updated_joint_angles_matrix.row(2) <<   -phi_a, 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // left hip    
        updated_joint_angles_matrix.row(3) <<   - phi_a, 
                                                - theta, 
                                                0; 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   phi_a, 
                                                theta, 
                                                0;

    } 
    else if (model.stepping_foot_index == -1) // if support foot is the left foot, right foot stepping 
    { 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   -phi_a,        
                                                -theta, 
                                                0; 
        // left hip
        updated_joint_angles_matrix.row(3) <<   phi_a , 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // right hip    
        updated_joint_angles_matrix.row(2) <<   phi_a, 
                                                -theta, 
                                                0; 
        // right ankle
        updated_joint_angles_matrix.row(1) <<   -phi_a, 
                                                theta, 
                                                0;
    }


    // std::cout << "### New ComputeJointAnglesStepDoubleSupports function" << std::endl;
    // std::cout << "stepping_foot_index = " << model.stepping_foot_index << std::endl;
    // std::cout << "right ankle: " << updated_joint_angles_matrix.row(1) << std::endl;
    // std::cout << "right hip: " << updated_joint_angles_matrix.row(2) << std::endl;
    // std::cout << "left hip: " << updated_joint_angles_matrix.row(3) << std::endl;
    // std::cout << "left ankle: " << updated_joint_angles_matrix.row(4) << std::endl;
    // std::cout << "pelvis: " << updated_joint_angles_matrix.row(6) << std::endl;
  

    return updated_joint_angles_matrix;

    // ... thenUpdate joint positions
    // call function UpdateLimbPositions to compute the new joint positions


}




Eigen::MatrixXd HumanoidModelV0::ComputeJointAnglesGaitSingleSupport(const GenericAgent& agent,
                                        double step_length, 
                                        double step_width, 
                                        // the following argument have to be removed in the future
                                        double step_duration
                                    ) const

{
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_angles_matrix(11, 3); // 11 joints, 3 angles each


    // compute step_complition_factor
    // this represent the avancement of the curent step. this factor == 1 when the step is over.
    double step_complition_factor = 1 - model.step_timer/step_duration; 

    // Rotation 
    // only walk without rotation is implemented for now (rotation_index != 0)
    double traveled_step_length; // length traveled by the stepping foot during this time step (former "sl_p")
    double lean_angle; // What is this angle ? // It allows the head to oscillate perdendicularly to the walking direction, so the head trajectory looks like a sine wave

    if (model.step_timer > step_duration/2) {
        traveled_step_length = 2 * std::pow(step_complition_factor, 2) //
                                    * (step_complition_factor + step_length) - step_length; 
        lean_angle  =  (4 * step_complition_factor)*M_PI/180 ;
    }
    else 
    {
        traveled_step_length = step_length - 2 * std::pow(1-step_complition_factor, 2) 
                                    * (2 * step_length);
        lean_angle  =  (4 - 4 * step_complition_factor)*M_PI/180 ;
    }

    double theta, phi_a;
    // only walk without rotation is implemented for now (rotation_index != 0)  
    theta = asin(traveled_step_length / (2 * model.height * LEG_SCALING_FACTOR));
    phi_a = asin(((model.height * PELVIS_WIDTH_SCALING_FACTOR - step_width) 
            / (2 * model.height * LEG_SCALING_FACTOR))+ lean_angle);


    // Update joint angles
    if (model.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
         // right ankle
        updated_joint_angles_matrix.row(1) <<    phi_a,        
                                                - theta, 
                                                0; 
        // right hip
        updated_joint_angles_matrix.row(2) <<   -phi_a, 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // left hip    
        updated_joint_angles_matrix.row(3) <<   - phi_a, 
                                                - theta, 
                                                0; 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   phi_a, 
                                                theta, 
                                                0;

    } 
    else if (model.stepping_foot_index == -1) // if support foot is the left foot, right foot stepping 
    { 
       // left ankle
        updated_joint_angles_matrix.row(4) <<   -phi_a,        
                                                -theta, 
                                                0; 
        // left hip
        updated_joint_angles_matrix.row(3) <<   phi_a , 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // right hip    
        updated_joint_angles_matrix.row(2) <<   phi_a, 
                                                -theta, 
                                                0; 
        // right ankle
        updated_joint_angles_matrix.row(1) <<   -phi_a, 
                                                theta, 
                                                0;
    }


    // std::cout << "New ComputeJointAnglesGaitSingleSupport function" << std::endl;
    // std::cout << "stepping_foot_index = " << model.stepping_foot_index << std::endl;
    // std::cout << "right ankle: " << updated_joint_angles_matrix.row(1) << std::endl;
    // std::cout << "right hip: " << updated_joint_angles_matrix.row(2) << std::endl;
    // std::cout << "left hip: " << updated_joint_angles_matrix.row(3) << std::endl;
    // std::cout << "left ankle: " << updated_joint_angles_matrix.row(4) << std::endl;
    // std::cout << "pelvis: " << updated_joint_angles_matrix.row(6) << std::endl;
    
    
    return updated_joint_angles_matrix;
    
    // ... thenUpdate joint positions
    // call function UpdateLimbPositions to compute the new joint positions


}


Eigen::MatrixXd HumanoidModelV0::ComputeJointAnglesGait(
                                            const GenericAgent& agent,
                                            const HumanoidModelV0Update& update,
                                            double dT
                                    ) const

{
    //#### Precomputation
    // Get the model data from the agent
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_positions_matrix(11, 3); // 11 joints, 3 angles each

    double leg_length = model.height * LEG_SCALING_FACTOR + model.height * ANKLE_SCALING_FACTOR; 
    double pelvis_width = model.height * PELVIS_WIDTH_SCALING_FACTOR;

    // Step duration, required to copute stepping foot position
    double step_duration = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));
    
    // compute step_complition_factor
    // this represent the avancement of the curent step. this factor == 1 when the step is over.
    double step_complition_factor = 1 - update.step_timer/step_duration; 
    //#######

                      
    if(update.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
        
        // Step 1: computation of the next stepping foot position
        // left foot stepping
        updated_joint_positions_matrix.row(5) <<    model.heel_left_position.x + update.velocity.x * dT,
                                                    model.heel_left_position.y + update.velocity.y * dT,
                                                    -0.4*step_complition_factor*(step_complition_factor-1); 
                                                    // the vertical displacement of the stepping foot is 
                                                    // a parabola with a maximum at 0.1m/ z=0.40(t)(t - 1);
                                                     

        // Step 2: computation of the pelvis and support hip position
        // the center of the pelvis is placed between both feet
        updated_joint_positions_matrix(6, 0) = model.heel_right_position.x + (updated_joint_positions_matrix(5, 0)-model.heel_right_position.x)/2;
        updated_joint_positions_matrix(6, 1) = model.heel_right_position.y + (updated_joint_positions_matrix(5, 1)-model.heel_right_position.y)/2;

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point3D support_hip_position;

        support_hip_position.x = updated_joint_positions_matrix(6, 0) - pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = updated_joint_positions_matrix(6, 1) - pelvis_width * 0.5 * normal_orientation.y;
        support_hip_position.z = model.heel_right_position.z; // to insure a planar computation of distance_support_hip_foot_xy

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_right_position).Norm();

        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        updated_joint_positions_matrix(6, 2) =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

        // Step 3: The support foot (right foot) remain at the same place (on the ground)
        updated_joint_positions_matrix(0, 0) = model.heel_right_position.x;
        updated_joint_positions_matrix(0, 1) = model.heel_right_position.y;
        updated_joint_positions_matrix(0, 2) = 0;                               
                                                
    } 
    else // if the left foot is support foot (right foot stepping)
    {
        // Step 1: computation of the next stepping foot position
        // left foot stepping
        updated_joint_positions_matrix.row(0) <<    model.heel_right_position.x + update.velocity.x * dT,
                                                    model.heel_right_position.y + update.velocity.y * dT,
                                                    -0.4*step_complition_factor*(step_complition_factor-1); 
                                            // the vertical displacement of the stepping foot is 
                                            // a parabola with a maximum at 0.1m;

        
        // Step 2: computation of the pelvis and support hip position
        // the center of the pelvis is placed between both feet
        updated_joint_positions_matrix(6, 0) = model.heel_left_position.x + (updated_joint_positions_matrix(0, 0)-model.heel_left_position.x)/2;
        updated_joint_positions_matrix(6, 1) = model.heel_left_position.y + (updated_joint_positions_matrix(0, 1)-model.heel_left_position.y)/2;

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point3D support_hip_position;

        support_hip_position.x = updated_joint_positions_matrix(6, 0) + pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = updated_joint_positions_matrix(6, 1) + pelvis_width * 0.5 * normal_orientation.y;
        support_hip_position.z = model.heel_right_position.z; // to insure a planar computation of distance_support_hip_foot_xy

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_right_position).Norm();

        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        updated_joint_positions_matrix(6, 2) =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

                // Step 3: The support foot (right foot) remain at the same place (on the ground)
        updated_joint_positions_matrix(5, 0) = model.heel_left_position.x;
        updated_joint_positions_matrix(5, 1) = model.heel_left_position.y;
        updated_joint_positions_matrix(5, 2) = 0; 

    }

    // std::cout << "agent position : " << update.position.x << ", " << update.position.y << std::endl;
    // std::cout << "final position : " << updated_joint_positions_matrix.row(6) << std::endl;
    // std::cout << "right heel: " << updated_joint_positions_matrix.row(0) << std::endl;
    // std::cout << "left heel: " << updated_joint_positions_matrix.row(5) << std::endl;
    // std::cout << "update.stepping_foot_index: " << update.stepping_foot_index << std::endl;
    // std::cout << "step_complition_factor: " << step_complition_factor << std::endl;
    // std::cout << "update.step_timer: " << update.step_timer << std::endl;
    // std::cin.get();


    return updated_joint_positions_matrix;

}

Eigen::MatrixXd HumanoidModelV0::ComputeJointPositionsfromJointAngles (
                            const GenericAgent& agent,
                            const HumanoidModelV0Update& update, // this is the orientation angle of the agent's frame
                            Eigen::MatrixXd updated_joint_angles_matrix
                    
    ) const
{
 
    
    // Get the model data from the agent
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    
    // initialise the updated joint positions matrix
    Eigen::MatrixXd updated_joint_positions_matrix; 
    updated_joint_positions_matrix = Eigen::MatrixXd::Zero(11, 3); // 11 joints, 3 positions each


    // Compute all segment lengths based on the model height
    // double foot_forward_length = model.height * FOOT_FORWARD_LENGTH_SCALING_FACTOR;
    double foot_backward_length = model.height * FOOT_BACKWARD_SCALING_FACTOR;
    double ankle_length = model.height * ANKLE_SCALING_FACTOR;
    double leg_length = model.height * LEG_SCALING_FACTOR; 
    double pelvis_width = model.height * PELVIS_WIDTH_SCALING_FACTOR;
    double neck_length = model.height * NECK_SCALING_FACTOR;
    // double shoulder_width = model.height * SHOULDER_WIDTH_SCALING_FACTOR;
    double trunk_length = model.height * TRUNK_HEIGHT_SCALING_FACTOR;
    // double trunk_width = model.height * TRUNK_WIDTH_SCALING_FACTOR;



    // Tranformation from the Agent frame to the World frame
    double updated_orientation_angle = acos(update.velocity.Normalized().x);
    if (update.velocity.Normalized().x < 0) {
        updated_orientation_angle = 2 * M_PI - updated_orientation_angle; // adjust the angle to be in the range [0, 2*pi] 
    }
    // this transformation is used to compute the position of the joints in the world frame
    Eigen::Matrix4d T_world_agent = CreateTransformationMatrix(
        Eigen::Vector3d(update.position.x, update.position.y, leg_length+ankle_length), 
        Eigen::Vector3d(0, 0, updated_orientation_angle) // rotation vector (yaw angle)
    );
    Eigen::Vector4d origin_vector ={0,0,0,1}; // origin vector for the computation of the joint positions

    // pelvis ==> no translation, only rotation
    Eigen::Matrix4d T_agent_pelvis = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, 0), // no translation
        Eigen::Vector3d(updated_joint_angles_matrix.row(6)) // rotation of the pelvis
    );
    Eigen::Vector4d pelvis_vector =     T_world_agent   
                                        * T_agent_pelvis * origin_vector; // position of the pelvis in the world frame
    updated_joint_positions_matrix.row(6) <<    pelvis_vector[0], 
                                                pelvis_vector[1], 
                                                pelvis_vector[2]; // update the position of the pelvis

    //  right hip ==> translation of -pelvis_width/2 along the y-axis 
    Eigen::Matrix4d T_pelvis_righthip  = CreateTransformationMatrix(
        Eigen::Vector3d(-pelvis_width/2, 0, 0),  // translation of -pelvis_width/2 along the y-axis 
        Eigen::Vector3d(updated_joint_angles_matrix.row(2)) // rotation of the right hip joint
    ); 

    Eigen::Vector4d right_hip_vector =  T_world_agent 
                                        * T_agent_pelvis 
                                        * T_pelvis_righthip * origin_vector; // position of the right hip in the world frame
    updated_joint_positions_matrix.row(2) <<    right_hip_vector[0], 
                                                right_hip_vector[1], 
                                                right_hip_vector[2]; // update the position of the right hip
    
    //  right ankle ==> translation of -leg_length along the z-axis
    Eigen::Matrix4d T_righthip_rightankle = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, -leg_length), // translation of -leg_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(1)) // rotation of the right ankle joint
    );
    
    Eigen::Vector4d right_ankle_vector =    T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_righthip 
                                            * T_righthip_rightankle * origin_vector;
    updated_joint_positions_matrix.row(1) <<    right_ankle_vector[0],
                                                right_ankle_vector[1], 
                                                right_ankle_vector[2]; // update the position of the right ankle
                                                
    //  right heel ==> translation of -ankle_length along the z-axis and -foot_backward_length along the x-axis
    Eigen::Matrix4d T_rightankle_rightheel = CreateTransformationMatrix(
        Eigen::Vector3d(-foot_backward_length, 0, -ankle_length),               
        Eigen::Vector3d(0, 0, 0) // no rotation for the heel
    );
    Eigen::Vector4d right_heel_vector =     T_world_agent 
                                            * T_agent_pelvis
                                            * T_pelvis_righthip 
                                            * T_righthip_rightankle 
                                            * T_rightankle_rightheel * origin_vector;
    updated_joint_positions_matrix.row(0) <<    right_heel_vector[0],
                                                right_heel_vector[1], 
                                                right_heel_vector[2]; // update the position of the right heel

    //  left hip ==> translation of pelvis_width/2 along the y-axis
    Eigen::Matrix4d T_pelvis_lefthip = CreateTransformationMatrix(
        Eigen::Vector3d(pelvis_width/2, 0, 0), // translation of pelvis_width/2 along the y-axis 
        Eigen::Vector3d(updated_joint_angles_matrix.row(3)) // rotation of the left hip joint
    );
    
    Eigen::Vector4d left_hip_vector =      T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_lefthip * origin_vector;
                                            
    updated_joint_positions_matrix.row(3) <<    left_hip_vector[0],
                                                left_hip_vector[1], 
                                                left_hip_vector[2]; // update the position of the left hip
    //  left ankle ==> translation of -leg_length along the z-axis
    Eigen::Matrix4d T_lefthip_leftankle = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, -leg_length), // translation of -leg_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(4)) // rotation of the left ankle joint
    );
    
    Eigen::Vector4d left_ankle_vector =    T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_lefthip 
                                            * T_lefthip_leftankle * origin_vector;
    updated_joint_positions_matrix.row(4) <<    left_ankle_vector[0],
                                                left_ankle_vector[1], 
                                                left_ankle_vector[2]; // update the position of the left ankle

    //  left heel ==> translation of -ankle_length along the z-axis and -foot_backward_length along the x-axis
    Eigen::Matrix4d T_leftankle_leftheel = CreateTransformationMatrix(
        Eigen::Vector3d(-foot_backward_length, 0, -ankle_length), // translation of -foot_backward_length along the x-axis and -ankle_length along the z-axis
        Eigen::Vector3d(0, 0, 0) // no rotation for the heel
    );
    Eigen::Vector4d left_heel_vector =     T_world_agent 
                                            * T_agent_pelvis
                                            * T_pelvis_lefthip 
                                            * T_lefthip_leftankle 
                                            * T_leftankle_leftheel * origin_vector;
    updated_joint_positions_matrix.row(5) <<    left_heel_vector[0],
                                                left_heel_vector[1], 
                                                left_heel_vector[2]; // update the position of the left heel

    //  C7/trunk ==> translation of trunk_length along the z-axis 
    Eigen::Matrix4d T_pelvis_trunk = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, trunk_length), // translation of trunk_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(8)) // rotation of the trunk joint
    );
    Eigen::Vector4d trunk_vector =        T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_trunk * origin_vector;
    updated_joint_positions_matrix.row(8) <<    trunk_vector[0],
                                                trunk_vector[1], 
                                                trunk_vector[2]; // update the position of the trunk

    //  head ==> translation of neck_length along the z-axis
    Eigen::Matrix4d T_trunk_head = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, neck_length), // translation of neck_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(10)) // rotation of the head joint
    );
    Eigen::Vector4d head_vector =         T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_trunk 
                                            * T_trunk_head * origin_vector;
    updated_joint_positions_matrix.row(10) <<   head_vector[0],
                                                head_vector[1], 
                                                head_vector[2]; // update the position of the head


    // degug: plot the joint positions one by one
    // std::cout << " ###  Updated joint positions: " << std::endl;
    // std::cout << "Pelvis: " << updated_joint_positions_matrix.row(6)<< std::endl;
    // std::cout << "Right Heel: " << updated_joint_positions_matrix.row(0) << std::endl;
    // std::cout << "Left Heel: " << updated_joint_positions_matrix.row(5) << std::endl;
    // std::cout << " # \n # \n # " << std::endl;




    return updated_joint_positions_matrix;
}



