#include "python_shared.h"
#include <sstream>
#include <string>
#include <fstream>

extern void python_export_vector(py::module &);
extern void python_export_igl(py::module &);

#ifdef PY_VIEWER
extern void python_export_igl_viewer(py::module &);
#endif

#ifdef PY_COMISO
extern void python_export_igl_comiso(py::module &);
#endif

#ifdef PY_TETGEN
extern void python_export_igl_tetgen(py::module &);
#endif

#ifdef PY_EMBREE
extern void python_export_igl_embree(py::module &);
#endif

#ifdef PY_TRIANGLE
extern void python_export_igl_triangle(py::module &);
#endif

#ifdef PY_CGAL
extern void python_export_igl_cgal(py::module &);
#endif

#ifdef PY_COPYLEFT
extern void python_export_igl_copyleft(py::module &);
#endif

#ifdef PY_PNG
extern void python_export_igl_png(py::module &);
#endif

PYBIND11_PLUGIN(pyigl) {
    py::module m("pyigl", R"pyigldoc(
        Python wrappers for libigl
        --------------------------

        .. currentmodule:: pyigl

        .. autosummary::
           :toctree: _generate

           AABB
           ARAPEnergyType
           MeshBooleanType
           SolverStatus
           active_set
           arap
           avg_edge_length
           barycenter
           barycentric_coordinates
           barycentric_to_global
           bbw
           boundary_conditions
           boundary_facets
           boundary_loop
           cat
           collapse_edge
           colon
           column_to_quats
           comb_cross_field
           comb_frame_field
           compute_frame_field_bisectors
           copyleft_cgal_RemeshSelfIntersectionsParam
           copyleft_cgal_mesh_boolean
           copyleft_cgal_remesh_self_intersections
           copyleft_comiso_miq
           copyleft_comiso_nrosy
           copyleft_marching_cubes
           copyleft_swept_volume
           copyleft_tetgen_tetrahedralize
           cotmatrix
           covariance_scatter_matrix
           cross_field_missmatch
           cut_mesh_from_singularities
           deform_skeleton
           directed_edge_orientations
           directed_edge_parents
           doublearea
           dqs
           edge_lengths
           edge_topology
           eigs
           embree_ambient_occlusion
           embree_reorient_facets_raycast
           find_cross_field_singularities
           fit_rotations
           floor
           forward_kinematics
           gaussian_curvature
           get_seconds
           grad
           harmonic
           hsv_to_rgb
           internal_angles
           invert_diag
           is_irregular_vertex
           jet
           lbs_matrix
           local_basis
           lscm
           map_vertices_to_circle
           massmatrix
           min_quad_with_fixed
           n_polyvector
           normalize_row_lengths
           normalize_row_sums
           parula
           per_corner_normals
           per_edge_normals
           per_face_normals
           per_vertex_normals
           planarize_quad_mesh
           png_readPNG
           png_writePNG
           point_mesh_squared_distance
           polar_svd
           principal_curvature
           quad_planarity
           randperm
           readDMAT
           readMESH
           readOBJ
           readOFF
           readTGF
           read_triangle_mesh
           remove_duplicate_vertices
           rotate_vectors
           setdiff
           signed_distance
           slice
           slice_into
           slice_mask
           slice_tets
           sortrows
           streamlines
           triangle_triangle_adjacency
           triangle_triangulate
           unique
           unproject_onto_mesh
           upsample
           winding_number
           writeMESH
           writeOBJ

    )pyigldoc");

    python_export_vector(m);
    python_export_igl(m);


    #ifdef PY_VIEWER
    python_export_igl_viewer(m);
    #endif

    #ifdef PY_COMISO
    python_export_igl_comiso(m);
    #endif

    #ifdef PY_TETGEN
    python_export_igl_tetgen(m);
    #endif

    #ifdef PY_EMBREE
    python_export_igl_embree(m);
    #endif

    #ifdef PY_TRIANGLE
    python_export_igl_triangle(m);
    #endif

    #ifdef PY_CGAL
    python_export_igl_cgal(m);
    #endif

    #ifdef PY_COPYLEFT
    python_export_igl_copyleft(m);
    #endif

    #ifdef PY_PNG
    python_export_igl_png(m);
    #endif

    return m.ptr();
}
