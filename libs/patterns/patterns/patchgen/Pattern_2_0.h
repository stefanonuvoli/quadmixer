#pragma once
#include "Pattern.h"
#include "ILP.h"
#include <sstream>
#include <kt84/openmesh/edgeloop.h>

/*
equation for pattern 0:
p0|0| + p1|2| + |3| + x|2| + y|1| = |l0|
|2|     |0|   |1|    |0|    |1|   |l1|
actually, x can be seen the same as p1:
p0|0| + p1|2| + |3| + y|1| = |l0|
|2|     |0|   |1|    |1|   |l1|
*/
namespace patchgen {
    template <>
    struct Pattern<2, 0> {
        static Eigen::MatrixXd& get_constraint_matrix() {
            static Eigen::MatrixXd constraint_matrix;
            if (constraint_matrix.size() == 0) {
                constraint_matrix.resize(2, 3);
                constraint_matrix <<
                    0, 2, 1,
                    2, 0, 1;
            }
            return constraint_matrix;
        }
        static Eigen::VectorXd get_constraint_rhs(const Eigen::VectorXi& l) {
            return Eigen::Vector2d(l[0] - 3, l[1] - 1);
        }

        static int& get_variable(PatchParam& param, int index) {
            if (index < 2) return param.p[index];
            return param.y;
        }

        static bool get_default_parameter(const Eigen::VectorXi& l, PatchParam& param) {
            auto& constraint_matrix = get_constraint_matrix();
            const int num_variables = constraint_matrix.cols();
            ILP ilp(num_variables);
            ilp.add_constraint(constraint_matrix, EQ, get_constraint_rhs(l));

            // arbitrary constraints and objective
            // ymin
            ilp.set_objective(Eigen::Vector3d(0, 0, 1), false);
            if (!ilp.solve()) return false;
            int ymin = ilp.get_variables()[2];
            // ymax
            ilp.refresh();
            ilp.set_objective(Eigen::Vector3d(0, 0, 1), true);
            if (!ilp.solve()) return false;
            int ymax = ilp.get_variables()[2];
            // ymid-1<=y<=ymid+1
            ilp.refresh();
            int ymid = (ymin + ymax) / 2;
            ilp.add_constraint(Eigen::Vector3d(0, 0, 1), LE, ymid + 1);
            ilp.add_constraint(Eigen::Vector3d(0, 0, 1), GE, ymid - 1);
            // maximize p0+p1
            ilp.set_objective(Eigen::Vector3d(1, 1, 0), true);
            if (!ilp.solve()) return false;

            auto variables = ilp.get_variables();
            for (int i = 0; i < num_variables; ++i)
                get_variable(param, i) = variables[i];

            param.pattern_id = 0;
            return true;
        }
        template <typename PatchT>
        static void generate_subtopology(const PatchParam& param, PatchT& patch) {
            /*
            |          v--y
            |         ____
            |     ___/    \___
            |    /            \
            |   C0            C1
            |    \__        __/
            |  x--^ V0____V1 ^--x
            |          ^--y
            */
            patch.clear();
            typename PatchT::VHandle C[2];
            typename PatchT::VHandle V[2];
            for (int i = 0; i < 2; ++i) C[i] = add_tagged_vertex(patch, i, true);
            for (int i = 0; i < 2; ++i) V[i] = add_tagged_vertex(patch, i, false);

            patch.add_face(C[0], V[0], V[1], C[1]);

            auto h_insert_y = patch.halfedge_handle(V[1]);  // corresponds to V1-V0
            for (int i = 0; i < param.y; ++i)
                insert_edgeloop(patch, h_insert_y);
        }
        static VariableIndicators& get_variable_indicators() {
            static VariableIndicators variable_indicators;
            if (variable_indicators.empty()) {
                variable_indicators.resize(1);
                variable_indicators[0].push_back(std::make_pair(PatchVertexTag::V0, PatchVertexTag::V1));       // for y
            }
            return variable_indicators;
        }
        static std::string get_param_str(const PatchParam& param) {
            std::stringstream ss;
            ss << "y=" << param.y;
            return ss.str();
        }
    };

}
