package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.OptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseDynamicsOptimizationControlModule
{
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_JOINT_LIMIT_CONSTRAINTS = true;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final BasisVectorVisualizer basisVectorVisualizer;
   private final InverseDynamicsQPSolver qpSolver;
   private final QPInput motionQPInput;
   private final QPInput rhoQPInput;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final WholeBodyControllerBoundCalculator boundCalculator;
   private final ExternalWrenchHandler externalWrenchHandler;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int numberOfDoFs;
   private final int rhoSize;

   private final OneDoFJoint[] oneDoFJoints;
   private final DenseMatrix64F qDDotMinMatrix, qDDotMaxMatrix;

   private final JointIndexHandler jointIndexHandler;
   private final YoDouble absoluteMaximumJointAcceleration = new YoDouble("absoluteMaximumJointAcceleration", registry);
   private final Map<OneDoFJoint, YoDouble> jointMaximumAccelerations = new HashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointMinimumAccelerations = new HashMap<>();
   private final YoDouble rhoMin = new YoDouble("ControllerCoreRhoMin", registry);
   private final MomentumModuleSolution momentumModuleSolution;

   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final YoBoolean useWarmStart = new YoBoolean("useWarmStartInSolver", registry);
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterationsInSolver", registry);

   private final DenseMatrix64F zeroObjective = new DenseMatrix64F(0, 0);

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this(toolbox, null, parentRegistry);
   }

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, DynamicsMatrixCalculator dynamicsMatrixCalculator,
                                                   YoVariableRegistry parentRegistry)
   {
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      this.dynamicsMatrixCalculator = dynamicsMatrixCalculator;

      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      rhoSize = toolbox.getRhoSize();

      double gravityZ = toolbox.getGravityZ();

      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();

      wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();

      YoGraphicsListRegistry yoGraphicsListRegistry = toolbox.getYoGraphicsListRegistry();
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, yoGraphicsListRegistry, registry);
      else
         basisVectorVisualizer = null;

      motionQPInput = new QPInput(numberOfDoFs);
      rhoQPInput = new QPInput(rhoSize);
      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, toolbox.getTotalRobotMass(), contactablePlaneBodies);

      motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      boundCalculator = toolbox.getQPBoundCalculator();

      absoluteMaximumJointAcceleration.set(optimizationSettings.getMaximumJointAcceleration());
      qDDotMinMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      qDDotMaxMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         jointMaximumAccelerations.put(joint, new YoDouble("qdd_max_qp_" + joint.getName(), registry));
         jointMinimumAccelerations.put(joint, new YoDouble("qdd_min_qp_" + joint.getName(), registry));
      }

      rhoMin.set(optimizationSettings.getRhoMin());

      momentumModuleSolution = new MomentumModuleSolution();

      boolean hasFloatingBase = toolbox.getRootJoint() != null;
      ActiveSetQPSolverWithInactiveVariablesInterface activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();
      double dt = toolbox.getControlDT();
      qpSolver = new InverseDynamicsQPSolver(activeSetQPSolver, numberOfDoFs, rhoSize, hasFloatingBase, dt, registry);
      qpSolver.setAccelerationRegularizationWeight(optimizationSettings.getJointAccelerationWeight());
      qpSolver.setJerkRegularizationWeight(optimizationSettings.getJointJerkWeight());
      qpSolver.setJointTorqueWeight(optimizationSettings.getJointTorqueWeight());
      qpSolver.setUseWarmStart(optimizationSettings.useWarmStartInSolver());
      qpSolver.setMaxNumberOfIterations(optimizationSettings.getMaxNumberOfSolverIterations());

      useWarmStart.set(optimizationSettings.useWarmStartInSolver());
      maximumNumberOfIterations.set(optimizationSettings.getMaxNumberOfSolverIterations());

      zeroObjective.reshape(wrenchMatrixCalculator.getCopTaskSize(), 1);
      zeroObjective.zero();

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
      motionQPInputCalculator.initialize();
   }

   public void resetRateRegularization()
   {
      qpSolver.resetRateRegularization();
   }

   public boolean compute()
   {
      wrenchMatrixCalculator.computeMatrices();
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.setMaxRho(wrenchMatrixCalculator.getRhoMaxMatrix());
      qpSolver.setActiveRhos(wrenchMatrixCalculator.getActiveRhoMatrix());

      setupWrenchesEquilibriumConstraint();
      computePrivilegedJointAccelerations();

      if (SETUP_JOINT_LIMIT_CONSTRAINTS)
      {
         computeJointAccelerationLimits();
         qpSolver.setMinJointAccelerations(qDDotMinMatrix);
         qpSolver.setMaxJointAccelerations(qDDotMaxMatrix);
      }

      qpSolver.setMaxNumberOfIterations(maximumNumberOfIterations.getIntegerValue());
      if (useWarmStart.getBooleanValue() && wrenchMatrixCalculator.hasContactStateChanged())
      {
         qpSolver.setUseWarmStart(useWarmStart.getBooleanValue());
         qpSolver.notifyResetActiveSet();
      }

      boolean hasConverged = qpSolver.solve();
      if (!hasConverged)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            PrintTools.warn(this, "The QP has not converged. Only showing this once if it happens repeatedly.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();
      }

      DenseMatrix64F qDDotSolution = qpSolver.getJointAccelerations();
      DenseMatrix64F rhoSolution = qpSolver.getRhos();

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      SpatialForceVector centroidalMomentumRateSolution = motionQPInputCalculator.computeCentroidalMomentumRateFromSolution(qDDotSolution);
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();

      momentumModuleSolution.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);
      momentumModuleSolution.setExternalWrenchSolution(externalWrenchSolution);
      momentumModuleSolution.setJointAccelerations(qDDotSolution);
      momentumModuleSolution.setRhoSolution(rhoSolution);
      momentumModuleSolution.setJointsToOptimizeFor(jointsToOptimizeFor);
      momentumModuleSolution.setRigidBodiesWithExternalWrench(rigidBodiesWithExternalWrench);

      return hasConverged;
   }

   public MomentumModuleSolution getMomentumModuleSolution()
   {
      return momentumModuleSolution;
   }

   private void computeJointAccelerationLimits()
   {
      boundCalculator.computeJointAccelerationLimits(absoluteMaximumJointAcceleration.getDoubleValue(), qDDotMinMatrix, qDDotMaxMatrix);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double qDDotMin = qDDotMinMatrix.get(jointIndex, 0);
         double qDDotMax = qDDotMaxMatrix.get(jointIndex, 0);
         jointMinimumAccelerations.get(joint).set(qDDotMin);
         jointMaximumAccelerations.get(joint).set(qDDotMax);
      }
   }

   private void computePrivilegedJointAccelerations()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointAccelerations(motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DenseMatrix64F rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addRhoTask(rhoPrevious, rhoRateWeight);

      DenseMatrix64F copRegularizationWeight = wrenchMatrixCalculator.getCoPRegularizationWeight();
      DenseMatrix64F copRegularizationJacobian = wrenchMatrixCalculator.getCoPRegularizationJacobian();
      qpSolver.addRhoTask(copRegularizationJacobian, zeroObjective, copRegularizationWeight);

      DenseMatrix64F copRateRegularizationWeight = wrenchMatrixCalculator.getCoPRateRegularizationWeight();
      DenseMatrix64F copRateRegularizationJacobian = wrenchMatrixCalculator.getCoPRateRegularizationJacobian();
      qpSolver.addRhoTask(copRateRegularizationJacobian, zeroObjective, copRateRegularizationWeight);

      // The wrench matrix calculator holds on to the command until all inverse dynamics commands are received since the
      // contact state may yet change and the rho Jacobians need to be computed for these inputs.
      // see also wrenchMatrixCalculator#submitWrenchCommand()
      while (wrenchMatrixCalculator.getContactWrenchInput(rhoQPInput))
      {
         qpSolver.addRhoInput(rhoQPInput);
      }

      while (wrenchMatrixCalculator.getCenterOfPressureInput(rhoQPInput))
      {
         qpSolver.addRhoInput(rhoQPInput);
      }
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DenseMatrix64F centroidalMomentumMatrix = motionQPInputCalculator.getCentroidalMomentumMatrix();
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      DenseMatrix64F convectiveTerm = motionQPInputCalculator.getCentroidalMomentumConvectiveTerm();
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      qpSolver.setupWrenchesEquilibriumConstraint(centroidalMomentumMatrix, rhoJacobian, convectiveTerm, additionalExternalWrench, gravityWrench);
   }

   public void setupTorqueMinimizationCommand()
   {
      qpSolver.addTorqueMinimizationObjective(dynamicsMatrixCalculator.getTorqueMinimizationAccelerationJacobian(),
                                              dynamicsMatrixCalculator.getTorqueMinimizationRhoJacobian(),
                                              dynamicsMatrixCalculator.getTorqueMinimizationObjective());
   }

   public void submitSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitJointspaceAccelerationCommand(JointspaceAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointspaceAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumRateCommand(command, motionQPInput);
      if (success)
         qpSolver.addMotionInput(motionQPInput);
   }

   public void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      motionQPInputCalculator.updatePrivilegedConfiguration(command);
   }

   public void submitPrivilegedAccelerationCommand(PrivilegedJointSpaceCommand command)
   {
      motionQPInputCalculator.submitPrivilegedAccelerations(command);
   }

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      boundCalculator.submitJointLimitReductionCommand(command);
   }

   public void submitJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand command)
   {
      boundCalculator.submitJointLimitEnforcementMethodCommand(command);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);
   }

   public void submitCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      wrenchMatrixCalculator.submitCenterOfPressureCommand(command);
   }

   public void submitExternalWrenchCommand(ExternalWrenchCommand command)
   {
      RigidBody rigidBody = command.getRigidBody();
      Wrench wrench = command.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void submitContactWrenchCommand(ContactWrenchCommand command)
   {
      wrenchMatrixCalculator.submitContactWrenchCommand(command);
   }

   public void submitOptimizationSettingsCommand(OptimizationSettingsCommand command)
   {
      rhoMin.set(command.getRhoMin());
   }
}
