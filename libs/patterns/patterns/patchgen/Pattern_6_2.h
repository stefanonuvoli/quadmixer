#pragma once
#include "Pattern.h"
#include "ILP.h"
#include <kt84/eigen_def.h>
#include <kt84/openmesh/edgeloop.h>
#include <sstream>

/*
equation for pattern 2:
  |0|     |1|     |0|     |0|     |0|     |1|     |0|     |0|   |3|    |2|    |1|   |l0|
  |1|     |0|     |1|     |0|     |0|     |0|     |1|     |0|   |1|    |0|    |0|   |l1|
p0|0| + p1|1| + p2|0| + p3|1| + p4|0| + p5|0| + q0|0| + q3|1| + |1| + x|0| + y|0| = |l2|
  |0|     |0|     |1|     |0|     |1|     |0|     |0|     |0|   |1|    |0|    |1|   |l3|
  |0|     |0|     |0|     |1|     |0|     |1|     |0|     |1|   |1|    |0|    |0|   |l4|
  |1|     |0|     |0|     |0|     |1|     |0|     |1|     |0|   |1|    |0|    |0|   |l5|
*/
namespace patchgen {
    template <>
    struct Pattern<6, 2> {
        static Eigen::MatrixXd& get_constraint_matrix() {
            static Eigen::MatrixXd constraint_matrix;
            if (constraint_matrix.size() == 0) {
                constraint_matrix.resize(6, 10);
                constraint_matrix << 0, 1, 0, 0, 0, 1, 0, 0, 2, 1,
                                     1, 0, 1, 0, 0, 0, 1, 0, 0, 0,
                                     0, 1, 0, 1, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 1, 0, 0, 0, 0, 1,
                                     0, 0, 0, 1, 0, 1, 0, 1, 0, 0,
                                     1, 0, 0, 0, 1, 0, 1, 0, 0, 0;
            }
            return constraint_matrix;
        }
    
        static Eigen::VectorXd get_constraint_rhs(const Eigen::VectorXi& l) {
            return kt84::make_Vector6d(l[0] - 3,
                                       l[1] - 1,
                                       l[2] - 1,
                                       l[3] - 1,
                                       l[4] - 1,
                                       l[5] - 1);
        }
    
        static int& get_variable(PatchParam& param, int index) {
            if (index < 6) return param.p[index];
            if (index == 6) return param.q[0];
            if (index == 7) return param.q[3];
            if (index == 8) return param.x;
            return param.y;
        }

        static bool get_default_parameter(const Eigen::VectorXi& l, PatchParam& param) {
            auto& constraint_matrix = get_constraint_matrix();
            const int num_variables = constraint_matrix.cols();
            ILP ilp(num_variables);
            ilp.add_constraint(constraint_matrix, EQ, get_constraint_rhs(l));
        
            // arbitrary constraints
            // xmin
            ilp.set_objective(kt84::make_Vector10d(0, 0, 0, 0, 0, 0, 0, 0, 1, 0), false);
            if (!ilp.solve()) return false;
            int xmin = ilp.get_variables()[8];
            // xmax
            ilp.refresh();
            ilp.set_objective(kt84::make_Vector10d(0, 0, 0, 0, 0, 0, 0, 0, 1, 0), true);
            if (!ilp.solve()) return false;
            int xmax = ilp.get_variables()[8];
            // xmid-1<=x<=xmid+1
            ilp.refresh();
            int xmid = (xmin + xmax) / 2;
            ilp.add_constraint(kt84::make_Vector10d(0, 0, 0, 0, 0, 0, 0, 0, 1, 0), LE, xmid + 1);
            ilp.add_constraint(kt84::make_Vector10d(0, 0, 0, 0, 0, 0, 0, 0, 1, 0), GE, xmid - 1);
            // p0<=q0
            ilp.add_constraint(kt84::make_Vector10d(1, 0, 0, 0, 0, 0, -1, 0, 0, 0), LE, 0);
            // p3<=q3
            ilp.add_constraint(kt84::make_Vector10d(0, 0, 0, 1, 0, 0, 0, -1, 0, 0), LE, 0);
            // maximize p0+p1+p2+p3+p4+p5
            ilp.set_objective(kt84::make_Vector10d(1, 1, 1, 1, 1, 1, 0, 0, 0, 0), true);
            if (!ilp.solve()) return false;
        
            auto variables = ilp.get_variables();
            for (int i = 0; i < num_variables; ++i)
                get_variable(param, i) = variables[i];
        
            param.pattern_id = 2;
            return true;
        }
        template <typename PatchT>
        static void generate_subtopology(const PatchParam& param, PatchT& patch) {
            /*
        |                      v--y              
        |            C4-----------------C3       
        |           / \                 / \      
        |   q3-->  /   \               /   \  <--q3   
        |         /     \             /     \    
        |        /       \           /       \   
        |      C5         \         /         C2 
        |       \         V4-------V5         /  
        |        \       / \       / \       /   
        |         \     /   \     /   \     /    
        |   q0-->  \   /    V2---V3    \   /  <--q0
        |           \ /      |   |      \ /      
        |           C0------V0---V1------C1      
        |             x--^     ^--y   ^--x
            */
            patch.clear();
            typename PatchT::VHandle C[6];
            typename PatchT::VHandle V[6];
            for (int i = 0; i < 6; ++i) C[i] = add_tagged_vertex(patch, i, true );
            for (int i = 0; i < 6; ++i) V[i] = add_tagged_vertex(patch, i, false);
        
            patch.add_face(C[0], V[0], V[2], V[4]);
            patch.add_face(V[0], V[1], V[3], V[2]);
            patch.add_face(V[1], C[1], V[5], V[3]);
            patch.add_face(C[1], C[2], C[3], V[5]);
            patch.add_face(C[3], C[4], V[4], V[5]);
            patch.add_face(C[4], C[5], C[0], V[4]);
            patch.add_face(V[2], V[3], V[5], V[4]);
        
            auto h_insert_x = patch.halfedge_handle(V[0]);  // corresponds to V0-C0
            for (int i = 0; i < param.x; ++i)
                insert_edgeloop(patch, h_insert_x);
            auto h_insert_y = patch.halfedge_handle(V[1]);  // corresponds to V1-V0
            for (int i = 0; i < param.y; ++i)
                insert_edgeloop(patch, h_insert_y);
            auto h_insert_q0 = patch.halfedge_handle(C[0]);  // corresponds to C0-C5
            for (int i = 0; i < param.q[0]; ++i)
                insert_edgeloop(patch, h_insert_q0);
            auto h_insert_q3 = patch.halfedge_handle(C[3]);  // corresponds to C3-C2
            for (int i = 0; i < param.q[3]; ++i)
                insert_edgeloop(patch, h_insert_q3);
        }
        static VariableIndicators& get_variable_indicators() {
            static VariableIndicators variable_indicators;
            if (variable_indicators.empty()) {
                variable_indicators.resize(4);
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::C5));       // for q0
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::V4, PatchVertexTag::C4));
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::V5, PatchVertexTag::C3));
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::C2));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::C4, PatchVertexTag::C5));       // for q3
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V4, PatchVertexTag::C0));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V2, PatchVertexTag::V0));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V3, PatchVertexTag::V1));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V5, PatchVertexTag::C1));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::C3, PatchVertexTag::C2));
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::V0));       // for x
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::V4, PatchVertexTag::V2));
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::V5, PatchVertexTag::V3));
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::V1));
                variable_indicators[3].push_back(std::make_pair(PatchVertexTag::V0, PatchVertexTag::V1));       // for y
                variable_indicators[3].push_back(std::make_pair(PatchVertexTag::V2, PatchVertexTag::V3));
                variable_indicators[3].push_back(std::make_pair(PatchVertexTag::V4, PatchVertexTag::V5));
                variable_indicators[3].push_back(std::make_pair(PatchVertexTag::C4, PatchVertexTag::C3));
            }
            return variable_indicators;
        }
        static std::string get_param_str(const PatchParam& param) {
            std::stringstream ss;
            ss << "q0=" << param.q[0]
               << "_q3=" << param.q[3]
               << "_x=" << param.x
               << "_y=" << param.y;
            return ss.str();
        }
    };
}
