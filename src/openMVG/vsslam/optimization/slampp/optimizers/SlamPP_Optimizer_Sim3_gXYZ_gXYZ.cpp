
#include <openMVG/vsslam/optimization/slampp/optimizers/SlamPP_Optimizer_Sim3_gXYZ_gXYZ.hpp>

#include <openMVG/vsslam/optimization/slampp/optimizers/SlamPP_IO.hpp>
#include <openMVG/vsslam/optimization/slampp/optimizers/SlamPP_Utils.hpp>

namespace openMVG  {
namespace vsslam  {


/*
 *								=== SlamPP_Optimizer_Sim3_gXYZ_gXYZ_Sim3_gXYZ_gXYZ ===
 */

SlamPP_Optimizer_Sim3_gXYZ_gXYZ::SlamPP_Optimizer_Sim3_gXYZ_gXYZ
(
  size_t undefined_camera_id,
  bool b_verbose,
  bool b_use_schur,
	bool b_do_marginals,
	bool b_do_icra_style_marginals) // throw(srd::bad_alloc)
	:SlamPP_Optimizer(undefined_camera_id),m_solver(m_system, TIncrementalSolveSetting(solve::nonlinear, frequency::Every(-1)),
	    TMarginalsComputationPolicy((b_do_marginals)? marginals::do_calculate : marginals::do_not_calculate,
	    frequency::Never(), mpart_Diagonal, mpart_Diagonal), // batch marginals
	    b_verbose, CLinearSolverType(), b_use_schur)
{
	n_dummy_param = 1;
  m_solver.Set_ICRA15_Style_IncrementalMargs(b_do_icra_style_marginals); // !!
}

SlamPP_Optimizer_Sim3_gXYZ_gXYZ::~SlamPP_Optimizer_Sim3_gXYZ_gXYZ(){}

// Get basic data
size_t SlamPP_Optimizer_Sim3_gXYZ_gXYZ::n_Vertex_Num() const
{
  return m_system.r_Vertex_Pool().n_Size();
	//return m_p_optimizer->r_System().r_Vertex_Pool().n_Size();
}

Eigen::Map<const Eigen::VectorXd> SlamPP_Optimizer_Sim3_gXYZ_gXYZ::r_Vertex_State(size_t n_index) const
{
  return ((CSystemType::_TyConstVertexRef)m_system.r_Vertex_Pool()[n_index]).v_StateC(); // explicit const required in windows for some reason // todo - this was working before; investigate
  //return ((CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyConstVertexRef)m_p_optimizer->r_Vertex(n_index)).v_StateC(); // explicit const required in windows for some reason // todo - this was working before; investigate
}

Eigen::Map<Eigen::VectorXd> SlamPP_Optimizer_Sim3_gXYZ_gXYZ::r_Vertex_State(size_t n_index)
{
  return m_system.r_Vertex_Pool()[n_index].v_State();
  //return m_p_optimizer->r_Vertex(n_index).v_State();
}

Eigen::Vector3d SlamPP_Optimizer_Sim3_gXYZ_gXYZ::v_XYZVertex(size_t n_index) const
{
  return v_LandmarkState_XYZ(n_index); // now the vertices are global
}

Eigen::Vector3d SlamPP_Optimizer_Sim3_gXYZ_gXYZ::v_LandmarkState_XYZ(size_t n_vertex_id) const
{
  return m_system.r_Vertex_Pool().r_At<_TyLandmark>(n_vertex_id).r_v_State();
  //return m_p_optimizer->r_System().r_Vertex_Pool().r_At<CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark>(n_vertex_id).r_v_State();
}

// Set properties
void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_TrustRadius(double f_trust_radius)
{
  m_solver.Set_StepSize(f_trust_radius);
	//m_p_optimizer->r_Solver().Set_StepSize(f_trust_radius);
}

void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_TrustRadius_Persistence(bool b_trust_radius_persistent)
{
  m_solver.Set_StepSize_Persistence(b_trust_radius_persistent);
	//m_p_optimizer->r_Solver().Set_StepSize_Persistence(b_trust_radius_persistent);
}

void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_UpdateThreshold(double f_update_thresh)
{
  m_solver.Set_UpdateThreshold(f_update_thresh);
	//m_p_optimizer->r_Solver().Set_UpdateThreshold(f_update_thresh);
}

void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_AllBatch(bool b_all_batch)
{
  m_solver.Set_AllBatch(b_all_batch);
	//m_p_optimizer->r_Solver().Set_AllBatch(b_all_batch);
}

void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_ForceIncSchur(bool b_force_inc_schur)
{
  m_solver.Set_ForceIncSchur(b_force_inc_schur);
	//m_p_optimizer->r_Solver().Set_ForceIncSchur(b_force_inc_schur);
}

void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Set_RelinThreshold(double f_relin_thresh)
{
  m_solver.Set_RelinearizationThreshold(f_relin_thresh);
	//m_p_optimizer->r_Solver().Set_RelinearizationThreshold(f_relin_thresh);
}

// Optimize
void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Optimize(size_t n_max_iteration_num /*= 5*/,
	double f_min_dx_norm /*= .01*/, double f_min_dl_norm /*= .01*/) // throw(srd::bad_alloc, std::runtime_error)
{
  m_solver.Optimize(n_max_iteration_num, f_min_dx_norm, f_min_dl_norm);
	//m_p_optimizer->r_Solver().Optimize(n_max_iteration_num, f_min_dx_norm, f_min_dl_norm);
}

// Add cameras
double * SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Add_CamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) // throw(srd::bad_alloc)
{
 CVertexCamSim3 &r_cam0 = m_system.r_Get_Vertex<CVertexCamSim3>(n_vertex_id, v_cam_state);
	//CVertexCamSim3 &r_cam0 = m_p_optimizer->r_System().r_Get_Vertex<CVertexCamSim3>(n_vertex_id, v_cam_state);
	return r_cam0.r_v_State().data();
}

// Fixed vertex is vertex with unary factor
double * SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Add_CamVertexFixed(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) // throw(srd::bad_alloc)
{
  CVertexCamSim3 &r_cam0 = m_system.r_Get_Vertex<CVertexCamSim3>(n_vertex_id, v_cam_state);
  //CVertexCamSim3 &r_cam0 = m_p_optimizer->r_System().r_Get_Vertex<CVertexCamSim3>(n_vertex_id, v_cam_state);

  // Add unary factor (UF) to make the optimized system positive definite
/*
  m_system.r_Add_Edge(CEdgePoseCamSim3(n_vertex_id, m_undefined_camera_id,
        Eigen::Vector7d::Zero(), Eigen::Matrix7d::Identity() * 100, m_system));
*/
  //m_p_optimizer->r_System().r_Add_Edge(CEdgePoseCamSim3(n_vertex_id, m_undefined_camera_id,
  //      Eigen::Vector7d::Zero(), Eigen::Matrix7d::Identity() * 100, m_p_optimizer->r_System()));

  // Return pointer to state of vertex
  return r_cam0.r_v_State().data();
}


// Add landamrks
double * SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Add_XYZVertex(size_t n_vertex_id, size_t n_owner_id, const Eigen::Vector3d &v_xyz_position)
{
  // Global landmark -> no owner
  n_owner_id = m_undefined_camera_id;

  _TyLandmark &r_landmark = m_system.r_Get_Vertex<_TyLandmark>(n_vertex_id,
	CParserBase::TVertexXYZ(int(n_vertex_id), v_xyz_position(0), v_xyz_position(1), v_xyz_position(2))); // initialize via parse primitive, let the vertex class convert to its representation

  //CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark &r_landmark = m_p_optimizer->r_System().r_Get_Vertex<CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark>(n_vertex_id,
  //  CParserBase::TVertexXYZ(int(n_vertex_id), v_xyz_position(0), v_xyz_position(1), v_xyz_position(2))); // initialize via parse primitive, let the vertex class convert to its representation

  m_camera_ownerships[n_vertex_id] = n_owner_id; // vertex defined in global space

  return r_landmark.r_v_State().data();
}

// Add observations
void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Add_P2CSim3GEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
	m_system.r_Add_Edge(_TyObservation(n_landmark_vertex_id, n_cam_vertex_id,
      v_observation, t_information, m_system));

	//m_p_optimizer->r_System().r_Add_Edge(CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyObservation(n_landmark_vertex_id, n_cam_vertex_id,
	//	v_observation, t_information, m_p_optimizer->r_System()));
}

// get covariance
Eigen::MatrixXd SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Get_CovarianceBlock(size_t element_1_slampp_id, size_t element_2_slampp_id)
{
	
	if(!m_solver.r_MarginalCovariance().r_SparseMatrix().b_Empty())
	{
  	const CUberBlockMatrix &r_marginals = m_solver.r_MarginalCovariance().r_SparseMatrix();
		return r_marginals.t_GetBlock_Log(element_1_slampp_id, element_2_slampp_id);
	}
	return Eigen::Matrix3d::Zero();
}



/**
 *  OUTPUTS
 */
void SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Show_Stats(bool b_calculate_eigenvalues) const
{
	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug
  std::vector<int> visible_points(m_system.r_Vertex_Pool().n_Size(), 0); // 0 = camera / visible, 1 = partially visible, 2 = invisible
	//std::vector<int> visible_points(m_p_optimizer->r_System().r_Vertex_Pool().n_Size(), 0); // 0 = camera / visible, 1 = partially visible, 2 = invisible
	double f_AVG_reprojection_err = 0;
	double f_RMSE_reprojection_err = 0;
	double f_AVG_reprojection_err_visible = 0;
	double f_RMSE_reprojection_err_visible = 0;
	const CSystemType::_TyEdgeMultiPool &edge_pool = m_system.r_Edge_Pool();
	//const CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyEdgeMultiPool &edge_pool = m_p_optimizer->r_System().r_Edge_Pool();
	size_t n_visible_obs_num = 0;
#ifdef SIM3_USE_ROBUST_EDGES
	double f_AVG_reprojection_err_robust = 0;
	double f_RMSE_reprojection_err_robust = 0;
	double f_robust_weight_sum = 0;
#endif // SIM3_USE_ROBUST_EDGES
	if(!edge_pool.b_Empty()) {
		for(size_t i = 0, n = edge_pool.n_Size(); i < n; ++ i) {
			bool b_visible;
			double f_weight;
#if 1
			double f_reprojection_err;
			edge_pool.For_Each(i, i + 1, CCalcRepErr(f_reprojection_err, f_weight, b_visible)); // need to derive the edge type
			size_t n_point_id = edge_pool[i].n_Vertex_Id(0); // landmark id // use facade
#else
			double f_reprojection_err = edge_pool.r_At<_TyObservation>(i).f_Reprojection_Error(b_visible); // know the edge type
			Eigen::Vector1d r; r(0) = f_reprojection_err; // can't initialize directly, the constructor is ambiguous
			f_weight = edge_pool.r_At<_TyObservation>(i).f_RobustWeight(r);
			size_t n_point_id = edge_pool.r_At<_TyObservation>(i).n_Vertex_Id(0); // landmark id // know the edge type
#endif
			visible_points[n_point_id] |= (b_visible)? 1 : 2; // need to combine
			f_AVG_reprojection_err += f_reprojection_err;
			f_RMSE_reprojection_err += f_reprojection_err * f_reprojection_err;
#ifdef SIM3_USE_ROBUST_EDGES
			f_AVG_reprojection_err_robust += f_reprojection_err * f_weight;
			f_RMSE_reprojection_err_robust += f_reprojection_err * f_reprojection_err * f_weight;
			f_robust_weight_sum += f_weight;
#endif // SIM3_USE_ROBUST_EDGES
			if(b_visible) {
				++ n_visible_obs_num;
				f_AVG_reprojection_err_visible += f_reprojection_err;
				f_RMSE_reprojection_err_visible += f_reprojection_err * f_reprojection_err;
			}
		}
		f_AVG_reprojection_err /= edge_pool.n_Size();
		f_RMSE_reprojection_err /= edge_pool.n_Size();
		f_RMSE_reprojection_err = sqrt(f_RMSE_reprojection_err);
#ifdef SIM3_USE_ROBUST_EDGES
		f_AVG_reprojection_err_robust /= f_robust_weight_sum;
		f_RMSE_reprojection_err_robust /= f_robust_weight_sum;
		f_RMSE_reprojection_err_robust = sqrt(f_RMSE_reprojection_err_robust);
#endif // SIM3_USE_ROBUST_EDGES
		f_AVG_reprojection_err_visible /= n_visible_obs_num;
		f_RMSE_reprojection_err_visible /= n_visible_obs_num;
		f_RMSE_reprojection_err_visible = sqrt(f_RMSE_reprojection_err_visible);
	}

	size_t n_fully_visible = 0;
	size_t n_fully_invisible = 0;
	size_t n_partly_invisible = 0;
	FILE *p_fw = fopen("visibility_info.txt", "w");
	for(size_t i = 0, n = visible_points.size(); i < n; ++ i) {
		if(visible_points[i] == 0) {
			// a camera, do nothing
		} else if(visible_points[i] == 1) {
			visible_points[i] = 0; // fully visible
			++ n_fully_visible;
		} else if(visible_points[i] == 2)
			++ n_fully_invisible; // fully invisible
		else if(visible_points[i] == 3) {
			visible_points[i] = 1; // visible from some cameras, invisible from the others
			++ n_partly_invisible;
		}

		if(p_fw) {
			double f_value = visible_points[i] * 10;
      size_t n_dim = m_system.r_Vertex_Pool()[i].n_Dimension();
			//size_t n_dim = m_p_optimizer->r_System().r_Vertex_Pool()[i].n_Dimension();
			for(size_t j = 0; j < n_dim; ++ j)
				fprintf(p_fw, (j + 1 < n_dim)? "%f " : "%f\n", f_value);
		}
		// write "fake marginals" that can be displayed by the graph viewer as false colours
	}
	if(p_fw)
		fclose(p_fw);
	// transform the visibility flags

	printf("reprojection error avg: %g px, RMSE: %g px (" PRIsize " observations)\n",
		f_AVG_reprojection_err, f_RMSE_reprojection_err, edge_pool.n_Size());
#ifdef SIM3_USE_ROBUST_EDGES
	printf("reprojection error avg: %g px, RMSE: %g px (robustified, " PRIsize " observations, %.2f weight)\n",
		f_AVG_reprojection_err_robust, f_RMSE_reprojection_err_robust, edge_pool.n_Size(), f_robust_weight_sum);
#endif // SIM3_USE_ROBUST_EDGES
	printf("reprojection error avg: %g px, RMSE: %g px (visible points only, " PRIsize " observations)\n",
		f_AVG_reprojection_err_visible, f_RMSE_reprojection_err_visible, n_visible_obs_num);
	printf("there were " PRIsize " points fully visible, " PRIsize
		" points partially visible and " PRIsize " points fully invisible\n",
		n_fully_visible, n_partly_invisible, n_fully_invisible);
	// print also the visibility info

	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug

	m_solver.Dump();
	//m_p_optimizer->r_Solver().Dump();
	double f_error = m_solver.f_Chi_Squared_Error_Denorm();
  //double f_error = m_p_optimizer->r_Solver().f_Chi_Squared_Error_Denorm();
	printf("denormalized chi2 error: %.2f\n", f_error);
	
	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug
  if(!m_solver.r_MarginalCovariance().r_SparseMatrix().b_Empty())
    m_solver.r_MarginalCovariance().Dump_Diagonal();

//	if(!m_p_optimizer->r_Solver().r_MarginalCovariance().r_SparseMatrix().b_Empty())
//		m_p_optimizer->r_Solver().r_MarginalCovariance().Dump_Diagonal();

	{
		Eigen::VectorXd diag;
		m_solver.r_Lambda().Get_Diagonal(diag, true);
		//m_p_optimizer->r_Solver().r_Lambda().Get_Diagonal(diag, true);
		double f_max_diag = diag.array().abs().maxCoeff();
		double f_min_diag = diag.array().abs().minCoeff();
		printf("approximate condition number of the information matrix is %.10g (min diag %g, max diag %g)\n",
			f_max_diag / f_min_diag, f_min_diag, f_max_diag);
	}
}

bool SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Dump_State(const char *p_s_filename) const
{
  return m_system.Dump(p_s_filename);
	//return m_p_optimizer->r_System().Dump(p_s_filename);
}

/**
 *	@brief saves SE(3) state file from the optimizer system
 */
bool SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Dump_Marginals(const char *p_s_filename) const
{

  if(!m_solver.r_MarginalCovariance().r_SparseMatrix().b_Empty())
    return m_solver.r_MarginalCovariance().Dump_Diagonal(p_s_filename);
	//if(!m_p_optimizer->r_Solver().r_MarginalCovariance().r_SparseMatrix().b_Empty())
	//	return m_p_optimizer->r_Solver().r_MarginalCovariance().Dump_Diagonal(p_s_filename);
	return true;
}

bool SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Dump_State_SE3(const char *p_s_filename) const
{
  const CSystemType::_TyVertexMultiPool &vertex_pool = m_system.r_Vertex_Pool();
	//const CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

  CSE3StatePrimitiveCallback<_TyLandmark> state_writer(p_fw, *this);
	//CSE3StatePrimitiveCallback<CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark> state_writer(p_fw, *this);
	vertex_pool.For_Each(state_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	fclose(p_fw);
	return true;
}

bool SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Dump_Poses_SE3(const char *p_s_filename) const
{
  const CSystemType::_TyVertexMultiPool &vertex_pool = m_system.r_Vertex_Pool();
  //const CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CSE3StatePrimitiveCallback<_TyLandmark> state_writer(p_fw, *this, true);
	//CSE3StatePrimitiveCallback<CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark> state_writer(p_fw, *this, true);
	vertex_pool.For_Each(state_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	fclose(p_fw);
	return true;
}

/**
 *	@brief saves a graph file from the optimizer system
 */
bool SlamPP_Optimizer_Sim3_gXYZ_gXYZ::Dump_Graph_SE3(const char *p_s_filename) const
{
  const CSystemType::_TyVertexMultiPool &vertex_pool = m_system.r_Vertex_Pool();
  const CSystemType::_TyEdgeMultiPool &edge_pool = m_system.r_Edge_Pool();

	//const CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();
	//const CBAOptimizerCore_Sim3_gXYZ_gXYZ::CSystemType::_TyEdgeMultiPool &edge_pool = m_p_optimizer->r_System().r_Edge_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CGraphPrimitiveCallback<_TyLandmark,_TyObservation> graph_writer(p_fw, *this);

	//CGraphPrimitiveCallback<CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyLandmark,CBAOptimizerCore_Sim3_gXYZ_gXYZ::_TyObservation> graph_writer(p_fw, *this);
	vertex_pool.For_Each(graph_writer);
	edge_pool.For_Each(graph_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	fclose(p_fw);
	return true;
}

/*
 *								=== ~SlamPP_Optimizer_Sim3_gXYZ_gXYZ ===
 */
}
}
