package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class InverseDynamicsOptimizationControlModule
{
   private static final boolean SETUP_JOINT_LIMIT_CONSTRAINTS = false;
   private static final boolean SETUP_RHO_TASKS = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final InverseDynamicsQPSolver qpSolver;
   private final MotionQPInput motionQPInput;
   private final PrivilegedMotionQPInput privilegedMotionQPInput;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final ExternalWrenchHandler externalWrenchHandler;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;

   private final double controlDT;
   private final OneDoFJoint[] oneDoFJoints;
   private final DenseMatrix64F qDDotMinMatrix, qDDotMaxMatrix;

   private final Map<OneDoFJoint, DoubleYoVariable> jointMaximumAccelerations = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointMinimumAccelerations = new HashMap<>();

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         YoVariableRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      oneDoFJoints = ScrewTools.filterJoints(jointsToOptimizeFor, OneDoFJoint.class);

      InverseDynamicsJoint rootJoint = toolbox.getRobotRootJoint();
      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      int rhoSize = WholeBodyControlCoreToolbox.rhoSize;
      int maxNPointsPerPlane = WholeBodyControlCoreToolbox.nContactPointsPerContactableBody;
      int maxNSupportVectors = WholeBodyControlCoreToolbox.nBasisVectorsPerContactPoint;

      double gravityZ = toolbox.getGravityZ();
      double wRho = momentumOptimizationSettings.getRhoPlaneContactRegularization();
      double wRhoSmoother = momentumOptimizationSettings.getRateOfChangeOfRhoPlaneContactRegularization();
      double wRhoPenalizer = momentumOptimizationSettings.getPenalizerOfRhoPlaneContactRegularization();

      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      wrenchMatrixCalculator = new PlaneContactWrenchMatrixCalculator(centerOfMassFrame, rhoSize, maxNPointsPerPlane, maxNSupportVectors, wRho, wRhoSmoother, 
            wRhoPenalizer, contactablePlaneBodies, registry);
      motionQPInput = new MotionQPInput(numberOfDoFs);
      privilegedMotionQPInput = new PrivilegedMotionQPInput(numberOfDoFs);
      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint, contactablePlaneBodies);

      motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, geometricJacobianHolder, twistCalculator, jointsToOptimizeFor, controlDT, registry);

      qDDotMinMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      qDDotMaxMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         jointMaximumAccelerations.put(joint, new DoubleYoVariable("qdd_max_qp_" + joint.getName(), registry));
         jointMinimumAccelerations.put(joint, new DoubleYoVariable("qdd_min_qp_" + joint.getName(), registry));
      }

      qpSolver = new InverseDynamicsQPSolver(numberOfDoFs, rhoSize, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
      motionQPInputCalculator.update();
   }

   public MomentumModuleSolution compute() throws MomentumControlModuleException
   {
      wrenchMatrixCalculator.computeMatrices();
      if (SETUP_RHO_TASKS)
         setupRhoTasks();
      setupWrenchesEquilibriumConstraint();
      computePrivilegedJointAccelerations();

      if (SETUP_JOINT_LIMIT_CONSTRAINTS)
      {
         computeJointAccelerationLimits();
         qpSolver.setMinJointAccelerations(qDDotMinMatrix);
         qpSolver.setMaxJointAccelerations(qDDotMaxMatrix);
      }

      NoConvergenceException noConvergenceException = null;

      try
      {
         qpSolver.solve();
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      DenseMatrix64F qDDotSolution = qpSolver.getJointAccelerations();
      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenches(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      SpatialForceVector centroidalMomentumRateSolution = motionQPInputCalculator.computeCentroidalMomentumRateFromSolution(qDDotSolution);
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();
      MomentumModuleSolution momentumModuleSolution = new MomentumModuleSolution(jointsToOptimizeFor, qDDotSolution, centroidalMomentumRateSolution,
            externalWrenchSolution, rigidBodiesWithExternalWrench);

      if (noConvergenceException != null)
      {
         throw new MomentumControlModuleException(noConvergenceException, momentumModuleSolution);
      }

      return momentumModuleSolution;
   }

   private void computeJointAccelerationLimits()
   {
      motionQPInputCalculator.computeJointAccelerationLimits(qDDotMinMatrix, qDDotMaxMatrix);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         
         double qDDotMin = qDDotMinMatrix.get(i, 0);
         double qDDotMax = qDDotMaxMatrix.get(i, 0);
         jointMinimumAccelerations.get(joint).set(qDDotMin);
         jointMaximumAccelerations.get(joint).set(qDDotMax);
      }
   }

   private void computePrivilegedJointAccelerations()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointAccelerations(privilegedMotionQPInput);
      if (success)
         qpSolver.setPrivilegedMotionInput(privilegedMotionQPInput);
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPrevious();
      DenseMatrix64F wRhoSmoother = wrenchMatrixCalculator.getWRhoSmoother();
      qpSolver.addRhoTask(rhoPrevious, wRhoSmoother);

      DenseMatrix64F rhoPreviousAverage = wrenchMatrixCalculator.getRhoPreviousAverage();
      DenseMatrix64F wRhoPenalizer = wrenchMatrixCalculator.getWRhoPenalizer();
      qpSolver.addRhoTask(rhoPreviousAverage, wRhoPenalizer);
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DenseMatrix64F centroidalMomentumMatrix = motionQPInputCalculator.getCentroidalMomentumMatrix();
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getQRho();
      DenseMatrix64F convectiveTerm = motionQPInputCalculator.getCentroidalMomentumConvectiveTerm();
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      qpSolver.setupWrenchesEquilibriumConstraint(centroidalMomentumMatrix, rhoJacobian, convectiveTerm, additionalExternalWrench, gravityWrench);
   }

   public void submitInverseDynamicsCommand(InverseDynamicsCommand<?> command)
   {
      switch (command.getCommandType())
      {
      case TASKSPACE:
         submitSpatialAccelerationCommand((SpatialAccelerationCommand) command);
         return;
      case POINT:
         submitPointAccelerationCommand((PointAccelerationCommand) command);
         return;
      case JOINTSPACE:
         submitJointspaceAccelerationCommand((JointspaceAccelerationCommand) command);
         return;
      case MOMENTUM:
         submitMomentumRateCommand((MomentumRateCommand) command);
         return;
      case PRIVILEGED_CONFIGURATION:
         submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
         return;
      case EXTERNAL_WRENCH:
         submitExternalWrenchCommand((ExternalWrenchCommand) command);
         return;
      case PLANE_CONTACT_STATE:
         submitPlaneContactStateCommand((PlaneContactStateCommand) command);
         return;
      case COMMAND_LIST:
         submitInverseDynamicsCommandList((InverseDynamicsCommandList) command);
         return;
      default:
         throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
      }
   }

   private void submitInverseDynamicsCommandList(InverseDynamicsCommandList command)
   {
      for (int i = 0; i < command.getNumberOfCommands(); i++)
         submitInverseDynamicsCommand(command.getCommand(i));
   }

   private void submitSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitJointspaceAccelerationCommand(JointspaceAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointspaceAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitPointAccelerationCommand(PointAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertPointAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitMomentumRateCommand(MomentumRateCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumRateCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      motionQPInputCalculator.updatePrivilegedConfiguration(command);
   }

   private void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.setPlaneContactStateCommand(command);
   }

   private void submitExternalWrenchCommand(ExternalWrenchCommand command)
   {
      RigidBody rigidBody = command.getRigidBody();
      Wrench wrench = command.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void setMinRho(double rhoMin)
   {
      qpSolver.setMinRho(rhoMin);
   }
}
