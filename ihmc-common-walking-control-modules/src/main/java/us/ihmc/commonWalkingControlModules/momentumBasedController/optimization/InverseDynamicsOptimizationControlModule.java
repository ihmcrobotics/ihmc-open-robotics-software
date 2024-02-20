package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPSolver.QPInputDomain.MOTION;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPSolver.QPInputDomain.MOTION_AND_RHO;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPSolver.QPInputDomain.RHO;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointTorqueAndPowerConstraintHandler;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.LinearMomentumRateCostCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings.JointPowerLimitEnforcementMethod;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings.JointTorqueLimitEnforcementMethod;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.quadraticProgram.NativeActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseDynamicsOptimizationControlModule implements SCS2YoGraphicHolder
{
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_JOINT_LIMIT_CONSTRAINTS = true;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final BasisVectorVisualizer basisVectorVisualizer;
   private final InverseDynamicsQPSolver qpSolver;
   private final NativeQPInputTypeB directMotionQPInput;
   private final NativeQPInputTypeA motionQPInput;
   private final NativeQPInputTypeA rhoQPInput;
   private final NativeQPInputTypeA motionAndRhoQPInput;
   private final NativeQPVariableSubstitution motionQPVariableSubstitution;
   private final MotionQPInputCalculator motionQPInputCalculator;
   private final WholeBodyControllerBoundCalculator boundCalculator;
   private final ExternalWrenchHandler externalWrenchHandler;

   private final JointBasics[] jointsToOptimizeFor;
   private final List<KinematicLoopFunction> kinematicLoopFunctions;
   private final int numberOfDoFs;
   private final int rhoSize;
   private final boolean hasFloatingBase;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final DMatrixRMaj qDDotMinMatrix, qDDotMaxMatrix;
   private final DMatrixRMaj customQDDotMinMatrix, customQDDotMaxMatrix;
   private final DMatrixRMaj qDDotSolution = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj rhoSolution = new DMatrixRMaj(1, 1);

   private final JointIndexHandler jointIndexHandler;
   private final TIntArrayList inactiveJointIndices = new TIntArrayList();
   private final ArrayList<OneDoFJointBasics> torqueConstrainedJoints = new ArrayList<>();
   private final ArrayList<OneDoFJointBasics> powerConstrainedJoints = new ArrayList<>();
   private final TObjectDoubleHashMap<OneDoFJointBasics> jointPowerLimits = new TObjectDoubleHashMap<OneDoFJointBasics>();
   private final JointTorqueCommand torqueConstraintMinimumCommand = new JointTorqueCommand();
   private final JointTorqueCommand torqueConstraintMaximumCommand = new JointTorqueCommand();
   private final JointTorqueAndPowerConstraintHandler torqueConstraintHandler = new JointTorqueAndPowerConstraintHandler();

   private final YoDouble absoluteMaximumJointAcceleration = new YoDouble("absoluteMaximumJointAcceleration", registry);
   private final Map<OneDoFJointBasics, YoDouble> jointMaximumAccelerations = new HashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointMinimumAccelerations = new HashMap<>();
   private final YoDouble rhoMin = new YoDouble("ControllerCoreRhoMin", registry);
   private final MomentumModuleSolution momentumModuleSolution;

   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final YoBoolean useWarmStart = new YoBoolean("useWarmStartInSolver", registry);
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterationsInSolver", registry);
   private final ExecutionTimer optimizationTimer = new ExecutionTimer("InvDynOptimizationTimer", registry);

   private final ArrayList<QPObjectiveCommand> nullspaceQPObjectiveCommands = new ArrayList<>();
   private final ArrayList<RigidBodyReadOnly> rigidBodiesWithCoPCommands = new ArrayList<>();

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      this(toolbox, null, parentRegistry);
   }

   public InverseDynamicsOptimizationControlModule(WholeBodyControlCoreToolbox toolbox,
                                                   DynamicsMatrixCalculator dynamicsMatrixCalculator,
                                                   YoRegistry parentRegistry)
   {
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      kinematicLoopFunctions = toolbox.getKinematicLoopFunctions();
      this.dynamicsMatrixCalculator = dynamicsMatrixCalculator;

      for (OneDoFJointBasics inactiveJoint : toolbox.getInactiveOneDoFJoints())
         inactiveJointIndices.add(jointIndexHandler.getOneDoFJointIndex(inactiveJoint));

      ReferenceFrame centerOfMassFrame = toolbox.getCenterOfMassFrame();

      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
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

      motionQPInput = new NativeQPInputTypeA(numberOfDoFs);
      directMotionQPInput = new NativeQPInputTypeB(numberOfDoFs);
      rhoQPInput = new NativeQPInputTypeA(rhoSize);
      motionAndRhoQPInput = new NativeQPInputTypeA(numberOfDoFs + rhoSize);
      motionQPVariableSubstitution = new NativeQPVariableSubstitution();
      externalWrenchHandler = new ExternalWrenchHandler(toolbox, gravityZ, centerOfMassFrame, toolbox.getTotalRobotMass(), contactablePlaneBodies);

      motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      boundCalculator = toolbox.getQPBoundCalculator();

      absoluteMaximumJointAcceleration.set(optimizationSettings.getMaximumJointAcceleration());
      qDDotMinMatrix = new DMatrixRMaj(numberOfDoFs, 1);
      qDDotMaxMatrix = new DMatrixRMaj(numberOfDoFs, 1);
      customQDDotMinMatrix = new DMatrixRMaj(numberOfDoFs, 1);
      customQDDotMaxMatrix = new DMatrixRMaj(numberOfDoFs, 1);

      CommonOps_DDRM.fill(qDDotMinMatrix, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(qDDotMaxMatrix, Double.POSITIVE_INFINITY);

      JointTorqueLimitEnforcementMethod jointTorqueEnforcementMethod = optimizationSettings.getJointTorqueLimitEnforcementMethod();
      if (jointTorqueEnforcementMethod == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP
          || jointTorqueEnforcementMethod == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER)
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJointBasics joint = oneDoFJoints[i];
            torqueConstrainedJoints.add(joint);
         }
      }

      JointPowerLimitEnforcementMethod jointPowerEnforcementMethod = optimizationSettings.getJointPowerLimitEnforcementMethod();
      if ((jointPowerEnforcementMethod == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP
           || jointPowerEnforcementMethod == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER)
          && !optimizationSettings.getJointPowerLimits().isEmpty())
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJointBasics joint = oneDoFJoints[i];
            if (optimizationSettings.getJointPowerLimits().containsKey(joint.getName()))
            {
               powerConstrainedJoints.add(joint);
               jointPowerLimits.put(joint, optimizationSettings.getJointPowerLimits().get(joint.getName()));
            }
         }
      }

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         jointMaximumAccelerations.put(joint, new YoDouble("qdd_max_qp_" + joint.getName(), registry));
         jointMinimumAccelerations.put(joint, new YoDouble("qdd_min_qp_" + joint.getName(), registry));
      }

      rhoMin.set(optimizationSettings.getRhoMin());

      momentumModuleSolution = new MomentumModuleSolution();

      hasFloatingBase = toolbox.getRootJoint() != null;
      NativeActiveSetQPSolverWithInactiveVariablesInterface activeSetQPSolver = optimizationSettings.getActiveSetQPSolver();
      double dt = toolbox.getControlDT();
      qpSolver = new InverseDynamicsQPSolver(activeSetQPSolver, numberOfDoFs, rhoSize, hasFloatingBase, dt, registry);
      qpSolver.setAccelerationRegularizationWeight(optimizationSettings.getJointAccelerationWeight());
      qpSolver.setJerkRegularizationWeight(optimizationSettings.getJointJerkWeight());
      qpSolver.setJointTorqueWeight(optimizationSettings.getJointTorqueWeight());
      qpSolver.setUseWarmStart(optimizationSettings.useWarmStartInSolver());
      qpSolver.setMaxNumberOfIterations(optimizationSettings.getMaxNumberOfSolverIterations());

      useWarmStart.set(optimizationSettings.useWarmStartInSolver());
      maximumNumberOfIterations.set(optimizationSettings.getMaxNumberOfSolverIterations());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
      motionQPInputCalculator.initialize();
      inactiveJointIndices.reset();
   }

   public void resetCustomBounds()
   {
      CommonOps_DDRM.fill(customQDDotMaxMatrix, Double.POSITIVE_INFINITY);
      CommonOps_DDRM.fill(customQDDotMinMatrix, Double.NEGATIVE_INFINITY);
   }

   public void resetRateRegularization()
   {
      qpSolver.resetRateRegularization();
   }

   public boolean compute()
   {
      optimizationTimer.startMeasurement();
      wrenchMatrixCalculator.collectRigidBodiesWithCoPCommands(rigidBodiesWithCoPCommands);
      wrenchMatrixCalculator.computeMatrices(rigidBodiesWithCoPCommands);
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());
      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      for (int i = 0; i < inactiveJointIndices.size(); i++)
      {
         qpSolver.setActiveDoF(inactiveJointIndices.get(i), false);
      }

      qpSolver.setMinRho(rhoMin.getDoubleValue());
      qpSolver.setMaxRho(wrenchMatrixCalculator.getRhoMaxMatrix());
      qpSolver.setActiveRhos(wrenchMatrixCalculator.getActiveRhoMatrix());

      setupWrenchesEquilibriumConstraint();

      // The Jacobian for all the primary tasks has been computed, so we should now submit the tasks take place in the nullspace.
      for (int i = 0; i < nullspaceQPObjectiveCommands.size(); i++)
      {
         QPObjectiveCommand command = nullspaceQPObjectiveCommands.get(i);
         submitQPObjectiveCommandNow(command);
      }
      nullspaceQPObjectiveCommands.clear();

      computePrivilegedJointAccelerations();

      if (SETUP_JOINT_LIMIT_CONSTRAINTS)
      {
         computeJointAccelerationLimits();
         qpSolver.setMinJointAccelerations(qDDotMinMatrix);
         qpSolver.setMaxJointAccelerations(qDDotMaxMatrix);
      }

      for (int i = 0; i < kinematicLoopFunctions.size(); i++)
      {
         List<? extends OneDoFJointReadOnly> loopJoints = kinematicLoopFunctions.get(i).getLoopJoints();

         // Check if the kinematic loop has joints. If no joints are returned, do not add a substitution
         if (loopJoints != null && !loopJoints.isEmpty())
         {
            motionQPInputCalculator.convertKinematicLoopFunction(kinematicLoopFunctions.get(i), motionQPVariableSubstitution);
            qpSolver.addAccelerationSubstitution(motionQPVariableSubstitution);
         }

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
            LogTools.warn("The QP has not converged. Only showing this once if it happens repeatedly.");
         }

         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();
      }

      qDDotSolution.set(qpSolver.getJointAccelerations());
      rhoSolution.set(qpSolver.getRhos());

      Map<RigidBodyBasics, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      SpatialForceReadOnly centroidalMomentumRateSolution = motionQPInputCalculator.computeCentroidalMomentumRateFromSolution(qDDotSolution);
      Map<RigidBodyBasics, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBodyBasics> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();

      momentumModuleSolution.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);
      momentumModuleSolution.setExternalWrenchSolution(externalWrenchSolution);
      momentumModuleSolution.setJointAccelerations(qDDotSolution);
      momentumModuleSolution.setRhoSolution(rhoSolution);
      momentumModuleSolution.setJointsToOptimizeFor(jointsToOptimizeFor);
      momentumModuleSolution.setRigidBodiesWithExternalWrench(rigidBodiesWithExternalWrench);

      resetCustomBounds();
      optimizationTimer.stopMeasurement();

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
         OneDoFJointBasics joint = oneDoFJoints[i];

         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         boolean hasCustomMin = Double.isFinite(customQDDotMinMatrix.get(jointIndex, 0));
         boolean hasCustomMax = Double.isFinite(customQDDotMaxMatrix.get(jointIndex, 0));
         double qDDotMin = qDDotMinMatrix.get(jointIndex, 0);
         double qDDotMax = qDDotMaxMatrix.get(jointIndex, 0);
         double customQDDotMin = MathTools.clamp(customQDDotMinMatrix.get(jointIndex, 0), absoluteMaximumJointAcceleration.getDoubleValue() - 5.0);
         double customQDDotMax = MathTools.clamp(customQDDotMaxMatrix.get(jointIndex, 0), absoluteMaximumJointAcceleration.getDoubleValue() - 5.0);

         if (hasCustomMin != hasCustomMax)
         {
            if (hasCustomMin)
            {
               qDDotMin = Math.max(qDDotMin, customQDDotMin);
               qDDotMax = Math.max(qDDotMax, customQDDotMin + 5.0);
            }
            else
            {
               qDDotMin = Math.min(qDDotMin, customQDDotMax - 5.0);
               qDDotMax = Math.min(qDDotMax, customQDDotMax);
            }
         }

         jointMinimumAccelerations.get(joint).set(qDDotMin);
         jointMaximumAccelerations.get(joint).set(qDDotMax);

         qDDotMinMatrix.set(jointIndex, 0, qDDotMin);
         qDDotMaxMatrix.set(jointIndex, 0, qDDotMax);
      }
   }

   private void computePrivilegedJointAccelerations()
   {
      boolean success = motionQPInputCalculator.computePrivilegedJointAccelerations(motionQPInput);
      if (success)
         qpSolver.addQPInput(motionQPInput, MOTION);
   }

   private void setupRhoTasks()
   {
      DMatrixRMaj rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DMatrixRMaj rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addIdentityJacobianTask(rhoPrevious, rhoRateWeight, RHO);

      rhoQPInput.getTaskWeightMatrix().set(wrenchMatrixCalculator.getCoPRegularizationWeight());
      rhoQPInput.getTaskJacobian().set(wrenchMatrixCalculator.getCoPRegularizationJacobian());
      rhoQPInput.getTaskObjective().set(wrenchMatrixCalculator.getCoPRegularizationObjective());
      rhoQPInput.setUseWeightScalar(false);
      qpSolver.addQPInput(rhoQPInput, RHO);

      rhoQPInput.getTaskWeightMatrix().set(wrenchMatrixCalculator.getCoPRateRegularizationWeight());
      rhoQPInput.getTaskJacobian().set(wrenchMatrixCalculator.getCoPRateRegularizationJacobian());
      rhoQPInput.getTaskObjective().set(wrenchMatrixCalculator.getCoPRateRegularizationObjective());
      rhoQPInput.setUseWeightScalar(false);
      qpSolver.addQPInput(rhoQPInput, RHO);

      // The wrench matrix calculator holds on to the command until all inverse dynamics commands are received since the
      // contact state may yet change and the rho Jacobians need to be computed for these inputs.
      // see also wrenchMatrixCalculator#submitWrenchCommand()
      while (wrenchMatrixCalculator.getContactWrenchInput(rhoQPInput))
      {
         qpSolver.addQPInput(rhoQPInput, RHO);
      }

      while (wrenchMatrixCalculator.getNextCenterOfPressureInput(rhoQPInput))
      {
         qpSolver.addQPInput(rhoQPInput, RHO);
      }
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DMatrixRMaj centroidalMomentumMatrix = motionQPInputCalculator.getCentroidalMomentumMatrix();
      DMatrixRMaj rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      DMatrixRMaj convectiveTerm = motionQPInputCalculator.getCentroidalMomentumConvectiveTerm();
      DMatrixRMaj additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DMatrixRMaj gravityWrench = externalWrenchHandler.getGravitationalWrench();
      qpSolver.setupWrenchesEquilibriumConstraint(centroidalMomentumMatrix, rhoJacobian, convectiveTerm, additionalExternalWrench, gravityWrench);
   }

   public void setupTorqueMinimizationCommand()
   {
      qpSolver.addTorqueMinimizationObjective(dynamicsMatrixCalculator.getBodyMassMatrix(),
                                              dynamicsMatrixCalculator.getBodyContactForceJacobianTranspose(),
                                              dynamicsMatrixCalculator.getTorqueMinimizationObjective());
   }

   /*
    * If called, constrains (inside the QP) the torques of the joints specified in
    * torqueConstrainedJoints to respect their effort limits (or some other limit if we send it). Can
    * also handle power constraints (which are converted to torque constraints).
    */
   public void setupTorqueConstraintCommand()
   {
      if (torqueConstrainedJoints.isEmpty() && powerConstrainedJoints.isEmpty())
         return;

      torqueConstraintMinimumCommand.clear();
      torqueConstraintMaximumCommand.clear();

      torqueConstraintMinimumCommand.setConstraintType(ConstraintType.GEQ_INEQUALITY);
      torqueConstraintMaximumCommand.setConstraintType(ConstraintType.LEQ_INEQUALITY);

      /* only torque constraints */
      if (!torqueConstrainedJoints.isEmpty() && powerConstrainedJoints.isEmpty())
      {
         for (int i = 0; i < torqueConstrainedJoints.size(); i++)
         {
            OneDoFJointBasics joint = torqueConstrainedJoints.get(i);
            torqueConstraintMinimumCommand.addJoint(joint, joint.getEffortLimitLower());
            torqueConstraintMaximumCommand.addJoint(joint, joint.getEffortLimitUpper());
         }
      }

      /* only power constraints */
      if (torqueConstrainedJoints.isEmpty() && !powerConstrainedJoints.isEmpty())
      {
         setupPowerConstraintCommand();
      }

      /* handle both torque and power constraints */
      if (!torqueConstrainedJoints.isEmpty() && !powerConstrainedJoints.isEmpty())
      {
         setupTorqueAndPowerConstraintCommand();
      }

      submitJointTorqueCommand(torqueConstraintMinimumCommand);
      submitJointTorqueCommand(torqueConstraintMaximumCommand);
   }

   /*
    * Handles the case when only power constraints are present. Since joint velocity is given, the
    * power constraint is converted into a torque constraint (taking care to watch the sign on qd).
    */
   private void setupPowerConstraintCommand()
   {
      double powerLimitLower = Double.NEGATIVE_INFINITY;
      double powerLimitUpper = Double.POSITIVE_INFINITY;

      for (int i = 0; i < powerConstrainedJoints.size(); i++)
      {
         OneDoFJointBasics joint = powerConstrainedJoints.get(i);
         powerLimitLower = -jointPowerLimits.get(joint);
         powerLimitUpper = jointPowerLimits.get(joint);

         torqueConstraintHandler.computeTorqueConstraints(joint, powerLimitLower, powerLimitUpper, false);
         torqueConstraintMinimumCommand.addJoint(joint, torqueConstraintHandler.getTorqueLimitLower());
         torqueConstraintMaximumCommand.addJoint(joint, torqueConstraintHandler.getTorqueLimitUpper());
      }
   }

   /*
    * Handles the case when torque and power constraints are present. Iterates through the joints and
    * checks whether the torque or power constraint is more restrictive at the current time step,
    * adding only the more restrictive constraint for each joint.
    */
   private void setupTorqueAndPowerConstraintCommand()
   {
      double powerLimitLower = Double.NEGATIVE_INFINITY;
      double powerLimitUpper = Double.POSITIVE_INFINITY;

      double torqueLimitLower = Double.NEGATIVE_INFINITY;
      double torqueLimitUpper = Double.POSITIVE_INFINITY;

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         if (powerConstrainedJoints.contains(joint))
         {
            /* this joint has a power constraint and may have a torque constraint */
            powerLimitLower = -jointPowerLimits.get(joint);
            powerLimitUpper = jointPowerLimits.get(joint);

            torqueConstraintHandler.computeTorqueConstraints(joint, powerLimitLower, powerLimitUpper, torqueConstrainedJoints.contains(joint));
            torqueLimitLower = torqueConstraintHandler.getTorqueLimitLower();
            torqueLimitUpper = torqueConstraintHandler.getTorqueLimitUpper();
         }
         else
         {
            /* this joint has only a torque constraint */
            torqueLimitLower = joint.getEffortLimitLower();
            torqueLimitUpper = joint.getEffortLimitUpper();
         }

         torqueConstraintMinimumCommand.addJoint(joint, torqueLimitLower);
         torqueConstraintMaximumCommand.addJoint(joint, torqueLimitUpper);
      }
   }

   public void submitQPObjectiveCommand(QPObjectiveCommand command)
   {
      if (command.isNullspaceProjected())
      {
         nullspaceQPObjectiveCommands.add(command);
      }
      else
      {
         submitQPObjectiveCommandNow(command);
      }
   }

   private void submitQPObjectiveCommandNow(QPObjectiveCommand command)
   {
      boolean success = motionQPInputCalculator.convertQPObjectiveCommand(command, motionQPInput);
      if (success)
         qpSolver.addQPInput(motionQPInput, MOTION);
   }

   public void submitSpatialAccelerationCommand(SpatialAccelerationCommand command)
   {
      boolean success = motionQPInputCalculator.convertSpatialAccelerationCommand(command, motionQPInput);
      if (success)
         qpSolver.addQPInput(motionQPInput, MOTION);
   }

   public void submitJointspaceAccelerationCommand(JointspaceAccelerationCommand command)
   {
      if (command.getConstraintType() == ConstraintType.OBJECTIVE || command.getConstraintType() == ConstraintType.EQUALITY)
      {
         boolean success = motionQPInputCalculator.convertJointspaceAccelerationCommand(command, motionQPInput);
         if (success)
            qpSolver.addQPInput(motionQPInput, MOTION);
      }
      else
      {
         DMatrixRMaj matToMod;
         if (command.getConstraintType() == ConstraintType.GEQ_INEQUALITY)
            matToMod = customQDDotMinMatrix;
         else
            matToMod = customQDDotMaxMatrix;

         for (int jointIdx = 0; jointIdx < command.getNumberOfJoints(); jointIdx++)
         {
            JointBasics joint = command.getJoint(jointIdx);
            if (joint instanceof OneDoFJointBasics)
            {
               matToMod.set(jointIndexHandler.getOneDoFJointIndex((OneDoFJointBasics) joint), 0, command.getDesiredAcceleration(jointIdx).get(0, 0));
            }
         }
      }
   }

   public void submitJointTorqueCommand(JointTorqueCommand command)
   {
      boolean success = motionQPInputCalculator.convertJointTorqueCommand(command,
                                                                          hasFloatingBase,
                                                                          motionAndRhoQPInput,
                                                                          dynamicsMatrixCalculator.getBodyMassMatrix(),
                                                                          dynamicsMatrixCalculator.getBodyContactForceJacobianTranspose(),
                                                                          dynamicsMatrixCalculator.getBodyGravityCoriolisMatrix());
      if (success)
         qpSolver.addQPInput(motionAndRhoQPInput, MOTION_AND_RHO);
   }

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      boolean success = motionQPInputCalculator.convertMomentumRateCommand(command, motionQPInput);
      if (success)
         qpSolver.addQPInput(motionQPInput, MOTION);
   }

   public void submitLinearMomentumRateCostCommand(LinearMomentumRateCostCommand command)
   {
      boolean success = motionQPInputCalculator.convertLinearMomentumRateCostCommand(command, directMotionQPInput);
      if (success)
         qpSolver.addQPInput(directMotionQPInput, MOTION);
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
      RigidBodyBasics rigidBody = command.getRigidBody();
      WrenchReadOnly wrench = command.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void submitContactWrenchCommand(ContactWrenchCommand command)
   {
      wrenchMatrixCalculator.submitContactWrenchCommand(command);
   }

   public void submitOptimizationSettingsCommand(InverseDynamicsOptimizationSettingsCommand command)
   {
      if (command.hasRhoMin())
         rhoMin.set(command.getRhoMin());
      if (command.hasJointAccelerationMax())
         absoluteMaximumJointAcceleration.set(command.getJointAccelerationMax());
      if (command.hasRhoWeight())
         wrenchMatrixCalculator.setRhoWeight(command.getRhoWeight());
      if (command.hasRhoRateWeight())
         wrenchMatrixCalculator.setRhoRateWeight(command.getRhoRateWeight());
      if (command.hasCenterOfPressureWeight())
         wrenchMatrixCalculator.setDesiredCoPWeight(command.getCenterOfPressureWeight());
      if (command.hasCenterOfPressureRateWeight())
         wrenchMatrixCalculator.setCoPRateWeight(command.getCenterOfPressureRateWeight());
      if (command.hasJointAccelerationWeight())
         qpSolver.setAccelerationRegularizationWeight(command.getJointAccelerationWeight());
      if (command.hasJointJerkWeight())
         qpSolver.setJerkRegularizationWeight(command.getJointJerkWeight());
      if (command.hasJointTorqueWeight())
         qpSolver.setJointTorqueWeight(command.getJointTorqueWeight());
      for (int i = 0; i < command.getJointsToActivate().size(); i++)
      {
         OneDoFJointBasics joint = command.getJointsToActivate().get(i);
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         inactiveJointIndices.remove(jointIndex);
      }
      for (int i = 0; i < command.getJointsToDeactivate().size(); i++)
      {
         OneDoFJointBasics joint = command.getJointsToDeactivate().get(i);
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         // Prevent duplicates in the list.
         if (!inactiveJointIndices.contains(jointIndex))
            inactiveJointIndices.add(jointIndex);
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      if (VISUALIZE_RHO_BASIS_VECTORS)
         group.addChild(basisVectorVisualizer.getSCS2YoGraphics());
      return group.isEmpty() ? null : group;
   }
}
