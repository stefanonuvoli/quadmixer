#pragma once
#include "Pattern.h"
#include "ILP.h"
#include <kt84/eigen_def.h>
#include <kt84/openmesh/edgeloop.h>
#include <sstream>

/*
equation for pattern 1:
  |0|     |1|     |0|     |0|     |1|     |1|   |2|    |1|   |l0|
  |1|     |0|     |1|     |0|     |0|     |0|   |1|    |1|   |l1|
p0|0| + p1|1| + p2|0| + p3|1| + p4|0| + q4|0| + |1| + x|0| = |l2|
  |0|     |0|     |1|     |0|     |1|     |1|   |1|    |0|   |l3|
  |1|     |0|     |0|     |1|     |0|     |0|   |1|    |0|   |l4|
*/
namespace patchgen {
    template <>
    struct Pattern<5, 1> {
        static Eigen::MatrixXd& get_constraint_matrix() {
            static Eigen::MatrixXd constraint_matrix;
            if (constraint_matrix.size() == 0) {
                constraint_matrix.resize(5, 7);
                constraint_matrix << 0, 1, 0, 0, 1, 1, 1,
                                     1, 0, 1, 0, 0, 0, 1,
                                     0, 1, 0, 1, 0, 0, 0,
                                     0, 0, 1, 0, 1, 1, 0,
                                     1, 0, 0, 1, 0, 0, 0;
            }
            return constraint_matrix;
        }
    
        static Eigen::VectorXd get_constraint_rhs(const Eigen::VectorXi& l) {
            return kt84::make_Vector5d(l[0] - 2,
                                       l[1] - 1,
                                       l[2] - 1,
                                       l[3] - 1,
                                       l[4] - 1);
        }
    
        static int& get_variable(PatchParam& param, int index) {
            if (index < 5) return param.p[index];
            if (index == 5) return param.q[4];
            return param.x;
        }

        static bool get_default_parameter(const Eigen::VectorXi& l, PatchParam& param) {
            auto& constraint_matrix = get_constraint_matrix();
            const int num_variables = constraint_matrix.cols();
            ILP ilp(num_variables);
            ilp.add_constraint(constraint_matrix, EQ, get_constraint_rhs(l));
        
            // arbitrary constraints and objective
            // xmin
            ilp.set_objective(kt84::make_Vector7d(0, 0, 0, 0, 0, 0, 1), false);
            if (!ilp.solve()) return false;
            int xmin = ilp.get_variables()[6];
            // xmax
            ilp.refresh();
            ilp.set_objective(kt84::make_Vector7d(0, 0, 0, 0, 0, 0, 1), true);
            if (!ilp.solve()) return false;
            int xmax = ilp.get_variables()[6];
            // xmid-1<=x<=xmid+1
            ilp.refresh();
            int xmid = (xmin + xmax) / 2;
            ilp.add_constraint(kt84::make_Vector7d(0, 0, 0, 0, 0, 0, 1), LE, xmid + 1);
            ilp.add_constraint(kt84::make_Vector7d(0, 0, 0, 0, 0, 0, 1), GE, xmid - 1);
            // p4<=q4
            ilp.add_constraint(kt84::make_Vector7d(0, 0, 0, 0, 1, -1, 0), LE, 0);
            // maximize p0+p1+p2+p3+p4
            ilp.set_objective(kt84::make_Vector7d(1, 1, 1, 1, 1, 0, 0), true);
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
        |             __C3__
        |   q4--> ___/      \___
        |        /              \
        |      C4            ___C2
        |       \        ___/  /
        |        \   ___/     /  <--x
        |         \ /        /
        |         C0---V0---C1
        |         x--^    ^--q4
            */
            patch.clear();
            typename PatchT::VHandle C[5];
            for (int i = 0; i < 5; ++i) C[i] = add_tagged_vertex(patch, i, true );
            auto V0 = add_tagged_vertex(patch, 0, false);
        
            patch.add_face(C[0], V0, C[1], C[2]);
            patch.add_face(C[0], C[2], C[3], C[4]);
        
            auto h_insert_x = patch.halfedge_handle(V0);  // corresponds to V0-C0
            for (int i = 0; i < param.x; ++i)
                insert_edgeloop(patch, h_insert_x);
            auto h_insert_q4 = patch.halfedge_handle(C[1]);  // corresponds to C1-V0
            for (int i = 0; i < param.q[4]; ++i)
                insert_edgeloop(patch, h_insert_q4);
        }
        static VariableIndicators& get_variable_indicators() {
            static VariableIndicators variable_indicators;
            if (variable_indicators.empty()) {
                variable_indicators.resize(2);
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::V0));       // for q4
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::C2));
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::C3, PatchVertexTag::C4));
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::C0, PatchVertexTag::V0));       // for x
                variable_indicators[1].push_back(std::make_pair(PatchVertexTag::C1, PatchVertexTag::C2));
            }
            return variable_indicators;
        }
        static std::string get_param_str(const PatchParam& param) {
            std::stringstream ss;
            ss << "q4=" << param.q[4] 
               << "_x=" << param.x;
            return ss.str();
        }
    };
}
