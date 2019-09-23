#pragma once

/*
 * SlamPP_Optimizer_Sim3_gXYZ_gXYZ - Global cameras and global XYZ landmarks
 */

#include <openMVG/vsslam/optimization/slampp/optimizers/SlamPP_Optimizer.hpp>
#include "slam/Sim3SolverBase.h" // C3DJacobians, CBAJacobians, generally useful functions for BA and SE(3), does not need to be included
#include "slam/LinearSolver_UberBlock.h"
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/3DSolverBase.h" // want C3DJacobians::Quat_to_AxisAngle() and C3DJacobians::AxisAngle_to_Quat()
#include "slam/Sim3_Types.h"
//#include "slam/Marginals.h" // included from nonlinear solver, if supported
#include "slam/NonlinearSolver_Lambda_DL.h"
#include "slam/Eigenvalues.h"


#include <map>


#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif






/**
 *	@brief bundle adjustment optimizer
 *	@note The purpose is not to have any interaction with CSystemType or CNonlinearSolverType
 *		in the header, in order to move the slow building of SLAM++ to BAOptimizer.cpp, which
 *		is rarely changed throughout the development process.
 */

extern int n_dummy_param;

namespace openMVG {
namespace vsslam {


class SlamPP_Optimizer_Sim3_gXYZ_gXYZ : public SlamPP_Optimizer {
public:
  // landmark: g_xyz
  using _TyLandmark = CVertexXYZ;
  using _TyObservation = CEdgeP2C_XYZ_Sim3_G;

  using TVertexTypelist = MakeTypelist_Safe((CVertexCamSim3, _TyLandmark));
  
  //#ifdef USE_EXPLICIT_HARD_UF
  //using TConstVertexTypelist = MakeTypelist_Safe((CVertexCamSim3, _TyLandmark));
  //#endif

  //#ifdef USE_EXPLICIT_HARD_UF
  //using TEdgeTypelist = MakeTypelist_Safe((_TyObservation, CEdgePoseCamSim3));
  //using CSystemType = CFlatSystem<CBaseVertex, TVertexTypelist, CBaseEdge, TEdgeTypelist, CNullUnaryFactorFactory, CBaseVertex, TConstVertexTypelist>;
  //#else USE_EXPLICIT_HARD_UF
  using TEdgeTypelist = MakeTypelist_Safe((_TyObservation));
  using CSystemType = CFlatSystem<CBaseVertex, TVertexTypelist, _TyObservation, TEdgeTypelist>;
  //#endif

  using CLinearSolverType = CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList>;
  using CNonlinearSolverType = CNonlinearSolver_Lambda_DL<CSystemType, CLinearSolverType>;

protected:
  CSystemType m_system;
  CNonlinearSolverType m_solver;

private:
  CSystemType::_TyConstVertexRef r_Vertex(size_t n_index) const
  {
    return m_system.r_Vertex_Pool()[n_index];
  }

  CSystemType::_TyVertexRef r_Vertex(size_t n_index)
  {
    return m_system.r_Vertex_Pool()[n_index];
  }

protected:
	std::map<size_t, size_t> m_camera_ownerships; // using a side channel now. this should be a part of the CVertexInvDepth class but I actually want to experiment a bit with global / local convergence so I'm leaving it here for now

public:
	SlamPP_Optimizer_Sim3_gXYZ_gXYZ(size_t undefined_camera_id = size_t(-1), bool b_verbose = false, bool b_use_schur = true,
		bool b_do_marginals = false, bool b_do_icra_style_marginals = false); // throw(srd::bad_alloc) // todo - add more optimization settings as required
	~SlamPP_Optimizer_Sim3_gXYZ_gXYZ();

	// Set properties
  void Set_TrustRadius(double f_trust_radius) override;
  void Set_TrustRadius_Persistence(bool b_trust_radius_persistent) override;
  void Set_UpdateThreshold(double f_update_thresh) override;
  void Set_AllBatch(bool b_all_batch) override;
  void Set_ForceIncSchur(bool b_force_inc_schur) override;
  void Set_RelinThreshold(double f_relin_thresh) override;

  // Get basic data
	size_t n_Vertex_Num() const override;
	Eigen::Map<const Eigen::VectorXd> r_Vertex_State(size_t n_index) const override; // returns const map of vertex state only
	Eigen::Map<Eigen::VectorXd> r_Vertex_State(size_t n_index) override; // returns map of vertex state only
  Eigen::Vector3d v_XYZVertex(size_t n_index) const override;

  // Add cameras
	double * Add_CamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) override; // throw(srd::bad_alloc)
	double * Add_CamVertexFixed(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) override; // throw(srd::bad_alloc)

  // Add landmarks
  double * Add_XYZVertex(size_t n_vertex_id, size_t n_owner_id, const Eigen::Vector3d &v_xyz_position) override;

  // Observations
	void Add_P2CSim3GEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
		const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) override; // throw(srd::bad_alloc)

	// Optimization
  void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01, double f_min_dl_norm = .01) override; // throw(srd::bad_alloc, std::runtime_error)

  // Covariance recovery
  Eigen::MatrixXd Get_CovarianceBlock(size_t element_1_slampp_id, size_t element_2_slampp_id) override;
  
  // Export
	void Show_Stats(bool b_calculate_eigenvalues = true) const override;
	bool Dump_State(const char *p_s_filename) const override; // note that this saves the state as Sim(3)
	bool Dump_Marginals(const char *p_s_filename) const override; // note that this saves the marginals of (relative?) Sim(3)
	bool Dump_Poses_SE3(const char *p_s_filename) const override; // only poses, not landmarks
	bool Dump_State_SE3(const char *p_s_filename) const override; // note that this converts the state to SE(3) for viewing by our utility which does not support Sim(3) now
	bool Dump_Graph_SE3(const char *p_s_filename) const override; // note that this converts the graph to SE(3) for viewing by our utility which does not support Sim(3) now


protected:
	Eigen::Vector3d v_LandmarkState_XYZ(size_t n_index) const;

private:
	SlamPP_Optimizer_Sim3_gXYZ_gXYZ(const SlamPP_Optimizer_Sim3_gXYZ_gXYZ &r_other); // no copy
	SlamPP_Optimizer_Sim3_gXYZ_gXYZ &operator =(const SlamPP_Optimizer_Sim3_gXYZ_gXYZ &r_other); // no copy
};

}
}
