#pragma once
#include "Pattern.h"
#include "ILP.h"
#include <kt84/eigen_def.h>
#include <kt84/openmesh/edgeloop.h>
#include <sstream>

/*
equation for pattern 1:
  |0|     |1|     |1|     |1|     |1|   |4|    |2|   |l0|
p0|1| + p1|0| + p2|1| + q1|0| + q2|1| + |1| + x|0| = |l1|
  |1|     |1|     |0|     |1|     |0|   |1|    |0|   |l2|
*/
namespace patchgen {
    template <>
    struct Pattern<3, 1> {
        static Eigen::MatrixXd& get_constraint_matrix() {
            static Eigen::MatrixXd constraint_matrix;
            if (constraint_matrix.size() == 0) {
                constraint_matrix.resize(3, 6);
                constraint_matrix << 0, 1, 1, 1, 1, 2,
                                     1, 0, 1, 0, 1, 0,
                                     1, 1, 0, 1, 0, 0;
            }
            return constraint_matrix;
        }
        static Eigen::VectorXd get_constraint_rhs(const Eigen::VectorXi& l) {
            return Eigen::Vector3d(l[0] - 4,
                                   l[1] - 1,
                                   l[2] - 1);
        }
        static int& get_variable(PatchParam& param, int index) {
            if (index < 3) return param.p[index];
            if (index == 3) return param.q[1];
            if (index == 4) return param.q[2];
            return param.x;
        }
        static bool get_default_parameter(const Eigen::VectorXi& l, PatchParam& param) {
            auto& constraint_matrix = get_constraint_matrix();
            const int num_variables = constraint_matrix.cols();
            ILP ilp(num_variables);
            ilp.add_constraint(constraint_matrix, EQ, get_constraint_rhs(l));
        
            // arbitrary constraints and objective
            // xmin
            ilp.set_objective(kt84::make_Vector6d(0, 0, 0, 0, 0, 1), false);
            if (!ilp.solve()) return false;
            int xmin = ilp.get_variables()[5];
            // xmax
            ilp.refresh();
            ilp.set_objective(kt84::make_Vector6d(0, 0, 0, 0, 0, 1), true);
            if (!ilp.solve()) return false;
            int xmax = ilp.get_variables()[5];
            // xmid-1<=x<=xmid+1
            ilp.refresh();
            int xmid = (xmin + xmax) / 2;
            ilp.add_constraint(kt84::make_Vector6d(0, 0, 0, 0, 0, 1), LE, xmid + 1);
            ilp.add_constraint(kt84::make_Vector6d(0, 0, 0, 0, 0, 1), GE, xmid - 1);
            // p1<=q1
            ilp.add_constraint(kt84::make_Vector6d(0, 1, 0, -1, 0, 0), LE, 0);
            // p2<=q2
            ilp.add_constraint(kt84::make_Vector6d(0, 0, 1, 0, -1, 0), LE, 0);
            // maximize p0+p1+p2
            ilp.set_objective(kt84::make_Vector6d(1, 1, 1, 0, 0, 0), true);
            if (!ilp.solve()) return false;
        
            auto variables = ilp.get_variables();
            for (int i = 0; i < num_variables; ++i)
                get_variable(param, i) = variables[i];
        
            param.pattern_id = 1;
            return true;
        }

        template <typename PatchT>
        static void generate_subtopology(const PatchParam& param, PatchT& patch) {
            /*
        |               C2
        |              /| \
        |             / |  \
        |            /  |   \
        |           /   V3   \ 
        |   q1-->  /   / \    \  <--q2
        |         /   /   \    \
        |        /   /     \    \
        |       /   /       \    \
        |      /   /         \    \
        |    C0---V0----V1----V2---C1
        |   x --^ q2--^    ^--q1  ^-- x
            */
            patch.clear();
            typename PatchT::VHandle C[3];
            typename PatchT::VHandle V[4];
            for (int i = 0; i < 3; ++i) C[i] = add_tagged_vertex(patch, i, true );
            for (int i = 0; i < 4; ++i) V[i] = add_tagged_vertex(patch, i, false);
        
            patch.add_face(C[0], V[0], V[3], C[2]);
            patch.add_face(V[0], V[1], V[2], V[3]);
            patch.add_face(C[2], V[3], V[2], C[1]);
        
            auto h_insert_x = patch.halfedge_handle(V[0]);  // corresponds to V0-C0
            for (int i = 0; i < param.x; ++i)
                insert_edgeloop(patch, h_insert_x);
            auto h_insert_q1 = patch.halfedge_handle(V[2]);  // corresponds to V2-V1
            for (int i = 0; i < param.q[1]; ++i)
                insert_edgeloop(patch, h_insert_q1);
            auto h_insert_q2 = patch.halfedge_handle(V[1]);  // corresponds to V1-V0
            for (int i = 0; i < param.q[2]; ++i)
                insert_edgeloop(patch, h_insert_q2);
        }
        static VariableIndicators& get_variable_indicators() {
            static VariableIndicators variable_indicators;
            if (variable_indicators.empty()) {
                variable_indicators.resize(3);
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::C2));       // for q1
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::V0, PatchVertexTag::V3));
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::V1, PatchVertexTag::V2));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::C2));       // for q2
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V2, PatchVertexTag::V3));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::V0, PatchVertexTag::V1));
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::V0));       // for x
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::V2));
                variable_indicators[2].push_back(std::make_pair(PatchVertexTag::C2, PatchVertexTag::V3));
            }
            return variable_indicators;
        }
        static std::string get_param_str(const PatchParam& param) {
            std::stringstream ss;
            ss << "q1=" << param.q[1] 
               << "_q2=" << param.q[2]
               << "_x=" << param.x;
            return ss.str();
        }
    };
}
