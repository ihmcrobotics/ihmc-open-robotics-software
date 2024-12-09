#include "mpc-problem.hpp"

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace ihmc
{
    MPCProblemImpl::MPCProblemImpl(const MPCProblemParameters& problem_parameters)
        : problem_parameters_(problem_parameters)
    {}

    bool MPCProblemImpl::setupProblem(const double* q_data,
                                      const int q_rows,
                                      const double* v_data,
                                      const int v_rows)
    {
        std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
        for (int i = 0; i < problem_parameters_.number_of_running_stages; ++i)
            running_models.emplace_back(stages_[i].getIntegratedModel());
        boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model = stages_.back().getIntegratedModel();
        problem_ = boost::make_shared<crocoddyl::ShootingProblem>(initial_state_, running_models, terminal_model);
        solver_ = boost::make_shared<crocoddyl::SolverFDDP>(problem_);

        setupCallbacks();

        setInitialState(q_data, q_rows, v_data, v_rows);

        return true;
    }

    bool MPCProblemImpl::setInitialState(const double* configuration_data,
                                         const int configuration_rows,
                                         const double* velocity_data,
                                         const int velocity_rows)
    {
        if (configuration_rows + velocity_rows != getStateSize())
        {
            std::cout << "Argument state size is wrong when setting the initial state." << std::endl;
            return false;
        }

        const VectorViewReadOnly configuration(configuration_data, configuration_rows);
        const VectorViewReadOnly velocity(velocity_data, velocity_rows);

        pinocchio::framesForwardKinematics(model_, data_, configuration);

        initial_state_ << configuration, velocity;
        problem_->set_x0(initial_state_);

        return true;
    }

    bool MPCProblemImpl::solve() const
    {
        return solver_->solve();
    }

    bool MPCProblemImpl::solve(const int iteration_limit) const
    {
        return solver_->solve(solver_->get_xs(), solver_->get_us(), iteration_limit, false, 1e-9);
    }

    std::size_t MPCProblemImpl::getIterations() const
    {
        return solver_->get_iter();
    }

    bool MPCProblemImpl::getInitialState(double* state_data_to_pack, const int rows) const
    {
        if (rows != getStateSize())
        {
            std::cout << "The number of rows in the data arguments don't match the state size " << getStateSize() <<
                    " when getting the initial state." << std::endl;
            return false;
        }

        if (state_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView state(state_data_to_pack, rows);
        state = initial_state_;

        return true;
    }

    bool MPCProblemImpl::getSolutionState(const int stage_id, double* state_data_to_pack, const int rows) const
    {
        if (rows != getStateSize())
        {
            std::cout << "The number of rows in the data arguments don't match the state size " << getStateSize() <<
                    " when getting the solution state." << std::endl;
            return false;
        }

        if (state_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView state(state_data_to_pack, rows);
        state = solver_->get_xs()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getSolutionControl(const int stage_id, double* control_data_to_pack, const int rows) const
    {
        if (rows != getControlSize())
        {
            std::cout << "The number of rows in the data arguments don't match the control size " << getControlSize() <<
                    " when getting the solution control." << std::endl;
            return false;
        }

        if (control_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView control(control_data_to_pack, rows);
        control = solver_->get_us()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getSolutionFeedbackGain(const int stage_id,
                                                 double* gain_data_to_pack,
                                                 const int rows,
                                                 const int cols) const
    {
        if (rows != getControlSize())
        {
            std::cout << "The number of rows in the data arguments don't match the control size " << getControlSize() <<
                    " when getting the solution feedback gain." << std::endl;
            return false;
        }
        if (cols != getStateDifferentialSize())
        {
            std::cout << "The number of cols in the data arguments don't match the state differential size " <<
                    getStateDifferentialSize() << " when getting the solution feedback gain." << std::endl;
            return false;
        }

        if (gain_data_to_pack == nullptr)
        {
            return false;
        }

        MatrixViewRowMajor gain(gain_data_to_pack, rows, cols);
        gain = solver_->get_K()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getQuu(const int stage_id, double* hessian_data_to_pack, const int rows, const int cols) const
    {
        if (rows != getControlSize())
        {
            std::cout << "The number of rows in the data arguments don't match the control size " << getControlSize() <<
                    " when getting Quu." << std::endl;
            return false;
        }
        if (cols != getControlSize())
        {
            std::cout << "The number of cols in the data arguments don't match the control size " << getControlSize() <<
                    " when getting Quu." << std::endl;
            return false;
        }

        if (hessian_data_to_pack == nullptr)
        {
            return false;
        }

        MatrixViewRowMajor hessian(hessian_data_to_pack, rows, cols);
        hessian = solver_->get_Quu()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getQxx(const int stage_id, double* hessian_data_to_pack, const int rows, const int cols) const
    {
        if (rows != getStateDifferentialSize())
        {
            std::cout << "The number of rows in the data arguments don't match the state differential size " <<
                    getStateDifferentialSize() << " when getting Qxx." << std::endl;
            return false;
        }
        if (cols != getStateDifferentialSize())
        {
            std::cout << "The number of cols in the data arguments don't match the state differential size " <<
                    getStateDifferentialSize() << " when getting Qxx." << std::endl;
            return false;
        }

        if (hessian_data_to_pack == nullptr)
        {
            return false;
        }

        MatrixViewRowMajor hessian(hessian_data_to_pack, rows, cols);
        hessian = solver_->get_Qxx()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getQxu(const int stage_id, double* hessian_data_to_pack, const int rows, const int cols) const
    {
        if (rows != getStateDifferentialSize())
        {
            std::cout << "The number of rows in the data arguments don't match the state differential size " <<
                    getStateDifferentialSize() << " when getting Qxu." << std::endl;
            return false;
        }
        if (cols != getControlSize())
        {
            std::cout << "The number of cols in the data arguments don't match the control size " << getControlSize() <<
                    " when getting Qxu." << std::endl;
            return false;
        }

        if (hessian_data_to_pack == nullptr)
        {
            return false;
        }

        MatrixViewRowMajor hessian(hessian_data_to_pack, rows, cols);
        hessian = solver_->get_Qxu()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getQx(const int stage_id, double* gradient_data_to_pack, const int rows) const
    {
        if (rows != getStateDifferentialSize())
        {
            std::cout << "The number of rows in the data arguments don't match the state differential size " <<
                    getStateDifferentialSize() << " when getting Qx." << std::endl;
            return false;
        }

        if (gradient_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView gradient(gradient_data_to_pack, rows);
        gradient = solver_->get_Qx()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getQu(const int stage_id, double* gradient_data_to_pack, const int rows) const
    {
        if (rows != getControlSize())
        {
            std::cout << "The number of rows in the data arguments don't match the control size " << getControlSize() <<
                    " when getting Qu." << std::endl;
            return false;
        }

        if (gradient_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView gradient(gradient_data_to_pack, rows);
        gradient = solver_->get_Qu()[stage_id];

        return true;
    }

    bool MPCProblemImpl::getSolutionFrameTranslation(const int stage_id,
                                                     const std::string& frame_name,
                                                     double* translation_data_to_pack,
                                                     const int rows) const
    {
        if (rows != 3)
        {
            std::cout <<
                    "The number of rows in the data arguments should be 3 when getting the solution frame translation."
                    << std::endl;
            return false;
        }

        if (translation_data_to_pack == nullptr)
        {
            return false;
        }
        const auto& frame_id = state_->get_pinocchio()->getFrameId(frame_name);

        VectorView translation(translation_data_to_pack, rows);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);

        translation = forward_dynamics->pinocchio.oMf[frame_id].translation();

        return true;
    }

    bool MPCProblemImpl::getSolutionFrameRotation(const int stage_id,
                                                  const std::string& frame_name,
                                                  double* rotation_data_to_pack,
                                                  const int rows,
                                                  const int cols) const
    {
        if (rows != 3 || cols != 3)
        {
            std::cout << "The size of the matrix should be 3x3 when getting the solution frame rotation" << std::endl;
            return false;
        }

        if (rotation_data_to_pack == nullptr)
        {
            return false;
        }
        const auto& frame_id = state_->get_pinocchio()->getFrameId(frame_name);
        MatrixViewRowMajor rotation(rotation_data_to_pack, rows, cols);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);

        rotation = forward_dynamics->pinocchio.oMf[frame_id].rotation();

        return true;
    }

    bool MPCProblemImpl::getSolutionFramePose(const int stage_id,
                                              const std::string& frame_name,
                                              double* translation_data_to_pack,
                                              const int translation_size,
                                              double* rotation_data_to_pack,
                                              const int rows,
                                              const int cols) const
    {
        if (translation_size != 3)
        {
            std::cout << "The size of the translation matrix should be 3 when getting the solution frame pose" <<
                    std::endl;
            return false;
        }
        if (rows != 3 || cols != 3)
        {
            std::cout << "The size of the matrix should be 3x3 when getting the solution frame pose" << std::endl;
            return false;
        }

        if (rotation_data_to_pack == nullptr || translation_data_to_pack == nullptr)
        {
            return false;
        }
        const auto& frame_id = state_->get_pinocchio()->getFrameId(frame_name);

        VectorView translation(translation_data_to_pack, translation_size);
        MatrixViewRowMajor rotation(rotation_data_to_pack, rows, cols);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);
        const auto& transform = forward_dynamics->pinocchio.oMf[frame_id];

        translation = transform.translation();
        rotation = transform.rotation();

        return true;
    }

    bool MPCProblemImpl::getSolutionFrameTwist(const int stage_id,
                                               const std::string& frame_name,
                                               double* twist_data_to_pack,
                                               const int rows) const
    {
        if (rows != 6)
        {
            std::cout << "The number of rows in the data arguments should be 6 when getting the solution frame twist."
                    << std::endl;
            return false;
        }

        if (twist_data_to_pack == nullptr)
        {
            return false;
        }
        const auto& frame_id = state_->get_pinocchio()->getFrameId(frame_name);
        VectorView twist(twist_data_to_pack, rows);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);
        const auto& data = forward_dynamics->pinocchio;
        twist = pinocchio::getFrameVelocity(model_, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();

        return true;
    }

    bool MPCProblemImpl::getSolutionContactWrench(const int stage_id,
                                                  const std::string& frame_name,
                                                  double* wrench_data_to_pack,
                                                  const int rows) const
    {
        if (rows != 6)
        {
            std::cout <<
                    "The number of rows in the data arguments should be 6 when getting the solution contact wrench." <<
                    std::endl;
            return false;
        }

        if (wrench_data_to_pack == nullptr)
        {
            return false;
        }
        VectorView wrench(wrench_data_to_pack, rows);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);
        wrench = forward_dynamics->multibody.contacts->contacts.at(frame_name)->f.toVector();

        return true;
    }

    bool MPCProblemImpl::getSolutionCoMPosition(const int stage_id, double* com_data_to_pack, const int rows) const
    {
        if (rows != 3)
        {
            std::cout << "The number of rows in the data arguments should be 3 when getting the solution com position."
                    << std::endl;
            return false;
        }

        if (com_data_to_pack == nullptr)
        {
            return false;
        }

        VectorView com_position(com_data_to_pack, rows);

        const auto& action_data_euler = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            problem_->get_runningDatas()[stage_id]);
        const auto& forward_dynamics = boost::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamics>(
            action_data_euler->differential);
        const auto& data = forward_dynamics->pinocchio;

        com_position = data.com[0]; // data.com is a vector of subtree CoMs, the first entry is for the whole model

        return true;
    }

    bool MPCProblemImpl::getCurrentFramePose(const std::string& frame_name,
                                             double* translation_data_to_pack,
                                             const int translation_size,
                                             double* rotation_data_to_pack,
                                             const int rows,
                                             const int cols) const
    {
        const auto& frame_id = model_.getFrameId(frame_name);

        VectorView translation(translation_data_to_pack, translation_size);
        MatrixViewRowMajor rotation(rotation_data_to_pack, rows, cols);

        translation = data_.oMf[frame_id].translation();
        rotation = data_.oMf[frame_id].rotation();

        return true;
    }

    const Stage& MPCProblemImpl::getStage(const int stage_id) const
    {
        return stages_[stage_id];
    }

    double MPCProblemImpl::getCostContribution(const int stage_id, const std::string& cost_name) const
    {
        const bool is_terminal = stage_id == static_cast<int>(stages_.size()) - 1;
        const auto model = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
            is_terminal ? problem_->get_terminalModel() : problem_->get_runningModels()[stage_id]);
        const auto data = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
            is_terminal ? problem_->get_terminalData() : problem_->get_runningDatas()[stage_id]);

        if (problem_parameters_.contact_frames.empty())
            return getCostContributionForActionType<crocoddyl::DifferentialActionModelFreeFwdDynamics,
                crocoddyl::DifferentialActionDataFreeFwdDynamics>(cost_name,
                                                                  is_terminal,
                                                                  model,
                                                                  data,
                                                                  problem_parameters_);
        else
            return getCostContributionForActionType<crocoddyl::DifferentialActionModelContactFwdDynamics,
                crocoddyl::DifferentialActionDataContactFwdDynamics>(cost_name,
                                                                     is_terminal,
                                                                     model,
                                                                     data,
                                                                     problem_parameters_);
    }

    double MPCProblemImpl::getCostContribution(const std::string& cost_name) const
    {
        double cost = 0.0;
        for (size_t i = 0; i < stages_.size(); ++i)
            cost += getCostContribution(i, cost_name);

        return cost;
    }

    double MPCProblemImpl::getCost() const
    {
        return solver_->get_cost();
    }

    double MPCProblemImpl::getMerit() const
    {
        return solver_->get_merit();
    }

    double MPCProblemImpl::getStoppingCriteria() const
    {
        return solver_->get_stop();
    }

    double MPCProblemImpl::getPrimalRegularization() const
    {
        return solver_->get_preg();
    }

    double MPCProblemImpl::getDualRegularization() const
    {
        return solver_->get_dreg();
    }

    double MPCProblemImpl::getStepSize() const
    {
        return solver_->get_steplength();
    }

    double MPCProblemImpl::getDynamicFeasibility() const
    {
        return solver_->get_ffeas();
    }

    double MPCProblemImpl::getInequalityFeasibility() const
    {
        return solver_->get_gfeas();
    }

    double MPCProblemImpl::getEqualityFeasibility() const
    {
        return solver_->get_hfeas();
    }

    double MPCProblemImpl::getCostReduction() const
    {
        return solver_->get_dV();
    }

    double MPCProblemImpl::getExpectedCostReduction() const
    {
        return solver_->get_dVexp();
    }

    double MPCProblemImpl::getMeritReduction() const
    {
        return solver_->get_dPhi();
    }

    double MPCProblemImpl::getExpectedMeritReduction() const
    {
        return solver_->get_dPhiexp();
    }

    double MPCProblemImpl::getKineticEnergy()
    {
        return pinocchio::computeKineticEnergy(model_, data_);
    }

    double MPCProblemImpl::getPotentialEnergy()
    {
        return pinocchio::computePotentialEnergy(model_, data_);
    }

    double MPCProblemImpl::getMechanicalEnergy()
    {
        return getKineticEnergy() + getPotentialEnergy();
    }

    int MPCProblemImpl::getConfigurationSize() const
    {
        return model_.nq;
    }

    int MPCProblemImpl::getVelocitySize() const
    {
        return model_.nv;
    }

    int MPCProblemImpl::getAccelerationSize() const
    {
        return getVelocitySize();
    }

    int MPCProblemImpl::getStateSize() const
    {
        return state_->get_nx();
    }

    int MPCProblemImpl::getStateDifferentialSize() const
    {
        return state_->get_ndx();
    }

    int MPCProblemImpl::getControlSize() const
    {
        return actuation_->get_nu();
    }

    void MPCProblemImpl::printProblemInfo() const
    {
        std::cout << *problem_ << std::endl;
    }

    void MPCProblemImpl::printStageInfo(const int stage_id) const
    {
        if (stage_id < static_cast<int>(stages_.size()) - 1)
            std::cout << *problem_->get_runningModels()[stage_id] << std::endl;
        else
            std::cout << *problem_->get_terminalModel() << std::endl;
    }

    void MPCProblemImpl::printContactInfo(const int stage_id) const
    {
        if (problem_parameters_.contact_frames.empty())
        {
            std::cout << "No contacts -- not printing any contact information." << std::endl;
            return;
        }

        if (stage_id < static_cast<int>(stages_.size()) - 1)
        {
            const auto& integrated_model = boost::static_pointer_cast<crocoddyl::IntegratedActionModelAbstract>(
                problem_->get_runningModels()[stage_id]);
            const auto& differential_model = boost::static_pointer_cast<
                crocoddyl::DifferentialActionModelContactFwdDynamics>(integrated_model->get_differential());
            const auto& contact_model_multiple = differential_model->get_contacts();
            std::cout << *contact_model_multiple << std::endl;
        }
        else
        {
            const auto& integrated_model = boost::static_pointer_cast<crocoddyl::IntegratedActionModelAbstract>(
                problem_->get_terminalModel());
            const auto& differential_model = boost::static_pointer_cast<
                crocoddyl::DifferentialActionModelContactFwdDynamics>(integrated_model->get_differential());
            const auto& contact_model_multiple = differential_model->get_contacts();
            std::cout << *contact_model_multiple << std::endl;
        }
    }

    void MPCProblemImpl::printPhaseDiagram() const
    {
        std::cout << std::endl;
        for (const auto& contact : contacts_)
        {
            for (size_t j = 0; j < stages_.size(); ++j)
            {
                if (constexpr int width = 10; j % width == 0 && j != 0)
                    std::cout << " ";

                std::cout << (stages_[j].getContactInformation().getContactStatus(contact) ? 'x' : 'o');
            }
            std::cout << std::endl;
        }
    }

    void MPCProblemImpl::buildFromModelAndData(const pinocchio::Model& model)
    {
        setupThreading();

        setupModelAndDataFromModel(model);
        setupProblemVariables();
    }

    void MPCProblemImpl::buildFromURDF()
    {
        setupThreading();

        setupModelAndDataFromURDF();
        setupProblemVariables();
    }

    void MPCProblemImpl::setupModelAndDataFromModel(const pinocchio::Model& model)
    {
        model_ = model;
        setupAdditionalFrames();
        data_ = pinocchio::Data(model_);
    }

    MPCProblemImpl::MPCProblemImpl(const pinocchio::Model& model, const MPCProblemParameters& problem_parameters)
        : MPCProblemImpl(problem_parameters)
    {
        buildFromModelAndData(model);
    }

    MPCProblemImpl::MPCProblemImpl(const std::string& urdf_path, const MPCProblemParameters& problem_parameters)
        : MPCProblemImpl(problem_parameters)
    {
        urdf_path_ = urdf_path;
        buildFromURDF();
    }

    void MPCProblemImpl::setupThreading() const
    {
        if (const std::set<int> threads = problem_parameters_.thread_ids; !threads.empty())
        {
            std::stringstream concatenated;
            for (auto it = threads.begin(); it != threads.end(); ++it)
            {
                concatenated << *it;
                if (std::next(it) != threads.end())
                    concatenated << ","; // Add comma only if it's not the last element
            }
            const std::string info = "{" + concatenated.str() + "}";
            setenv("OMP_PLACES", info.c_str(), 1); // Indices of the threads that can be used
            setenv("OMP_PROC_BIND", "TRUE", 1); // Rule that binds to just these threads
        }
        if (problem_parameters_.verbose)
            setenv("OMP_DISPLAY_ENV", "TRUE", 1);
    }

    void MPCProblemImpl::setupAdditionalFrames()
    {
        for (const auto& extra_frame : problem_parameters_.extra_frames)
        {
            for (const auto& frame : model_.frames)
            {
                if (frame.name == extra_frame.getName())
                    throw std::runtime_error("Frame " + extra_frame.getName() + " already exists in the model");
            }

            model_.addFrame(pinocchio::Frame(extra_frame.getName(),
                                             model_.getJointId(extra_frame.getParentJointName()),
                                             0,
                                             pinocchio::SE3(extra_frame.getRotationFromParent(),
                                                            extra_frame.getTranslationFromParent()),
                                             pinocchio::FrameType::OP_FRAME),
                            false);
            std::cout << "Added frame " << extra_frame.getName() << " to model" << std::endl;
            std::cout << "Translation from " << extra_frame.getParentJointName() << ":\n" << extra_frame.
                    getTranslationFromParent() << std::endl;
            std::cout << "Rotation from " << extra_frame.getParentJointName() << ":\n" << extra_frame.
                    getRotationFromParent() << std::endl;
        }
    }

    void MPCProblemImpl::setupModelAndDataFromURDF()
    {
        if (problem_parameters_.floating_base)
            pinocchio::urdf::buildModel(urdf_path_,
                                        pinocchio::JointModelFreeFlyer(),
                                        model_,
                                        problem_parameters_.verbose);
        else
            pinocchio::urdf::buildModel(urdf_path_, model_, problem_parameters_.verbose);

        setupAdditionalFrames();
        data_ = pinocchio::Data(model_);
    }

    void MPCProblemImpl::setupProblemVariables()
    {
        state_ = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model_));
        initial_state_ = Eigen::VectorXd::Zero(state_->get_nx());

        if (problem_parameters_.floating_base)
            actuation_ = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);
        else
            actuation_ = boost::make_shared<crocoddyl::ActuationModelFull>(state_);

        for (const auto& frame : problem_parameters_.contact_frames)
            contacts_.emplace_back(frame, state_, actuation_, problem_parameters_.contact_stabilization_gains);

        for (int i = 0; i < problem_parameters_.number_of_running_stages + 1; ++i) // + 1 for the terminal stage
        {
            ContactInformation stagewise_contact_information(state_, actuation_);
            for (const auto& contact : contacts_)
                stagewise_contact_information.addContact(contact);

            if (stagewise_contact_information.empty())
                stages_.emplace_back(i, state_, actuation_, problem_parameters_);
            else
                stages_.emplace_back(i, state_, actuation_, stagewise_contact_information, problem_parameters_);
        }
    }

    /**
     * @details By default callbacks are only created if the solver is in verbose mode
     */
    void MPCProblemImpl::setupCallbacks() const
    {
        if (problem_parameters_.verbose)
        {
            std::cout << "calling set up callbacks with verbosity " << problem_parameters_.verbose << std::endl;
            std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
            callbacks.emplace_back(boost::make_shared<crocoddyl::CallbackVerbose>());
            solver_->setCallbacks(callbacks);
        }
    }
}
