package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
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
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings.JointPowerLimitEnforcementMethod;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings.JointTorqueLimitEnforcementMethod;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class WholeBodyInverseDynamicsSolver implements SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final InverseDynamicsOptimizationControlModule optimizationControlModule;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final FloatingJointBasics rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJointBasics, YoDouble> jointAccelerationsSolution = new HashMap<>();

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;
   private final JointAccelerationIntegrationCalculator jointAccelerationIntegrationCalculator;

   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;
   private final List<KinematicLoopFunction> kinematicLoopFunctions;
   private final JointTorqueAndPowerConstraintHandler torqueConstraintHandler = new JointTorqueAndPowerConstraintHandler();

   private final YoFrameVector3D yoDesiredMomentumRateLinear;
   private final YoFrameVector3D yoDesiredMomentumRateAngular;
   // TODO It seems that the achieved CMP (computed from this guy) can be off sometimes.
   // Need to review the computation of the achieved linear momentum rate or of the achieved CMP. (Sylvain)
   private final YoFrameVector3D yoAchievedMomentumRateLinear;
   private final YoFrameVector3D yoAchievedMomentumRateAngular;
   private final FrameVector3D achievedMomentumRateLinear = new FrameVector3D();
   private final FrameVector3D achievedMomentumRateAngular = new FrameVector3D();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector3D residualRootJointForce = new FrameVector3D();
   private final FrameVector3D residualRootJointTorque = new FrameVector3D();

   private final YoFrameVector3D yoResidualRootJointForce;
   private final YoFrameVector3D yoResidualRootJointTorque;

   /**
    * Whether to assemble the objective for minimizing the joint torques. May be computationally
    * intensive, needs benchmark.
    */
   private final YoBoolean minimizeJointTorques;
   /**
    * Whether the desired joint torques should be constrained to each joint min and max torque and
    * where it should happen (in the QP or after the QP inside the controller).
    */
   private final YoEnum<JointTorqueLimitEnforcementMethod> jointTorqueLimitEnforcementMethod;
   /**
    * Whether the desired joint power should be constrained and where it should happen (in the QP or
    * after the QP inside the controller).
    */
   private final YoEnum<JointPowerLimitEnforcementMethod> jointPowerLimitEnforcementMethod;
   private final TObjectDoubleHashMap<String> powerConstrainedJointLimits = new TObjectDoubleHashMap<String>();
   /**
    * Whether {@link #dynamicsMatrixCalculator} is used for inverse dynamics.
    */
   private final YoBoolean useDynamicMatrixCalculatorForInverseDynamics;
   /**
    * Whether {@link #dynamicsMatrixCalculator} is updated.
    */
   private final YoBoolean updateDynamicMatrixCalculator;

   private final double controlDT;
   private final ExecutionTimer setupTimer;
   private final ExecutionTimer outputTimer;

   public WholeBodyInverseDynamicsSolver(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      controlDT = toolbox.getControlDT();
      rootJoint = toolbox.getRootJoint();
      inverseDynamicsCalculator = toolbox.getInverseDynamicsCalculator();
      dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox);
      optimizationControlModule = new InverseDynamicsOptimizationControlModule(toolbox, dynamicsMatrixCalculator, registry);

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      kinematicLoopFunctions = toolbox.getKinematicLoopFunctions();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[i];
         YoDouble jointAccelerationSolution = new YoDouble("qdd_qp_" + joint.getName(), registry);
         jointAccelerationsSolution.put(joint, jointAccelerationSolution);
      }

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();
      wrenchVisualizer = toolbox.getWrenchVisualizer();

      jointAccelerationIntegrationCalculator = new JointAccelerationIntegrationCalculator(toolbox.getRootBody(), controlDT, registry);

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();
      yoDesiredMomentumRateAngular = toolbox.getYoDesiredMomentumRateAngular();
      yoAchievedMomentumRateAngular = toolbox.getYoAchievedMomentumRateAngular();

      yoResidualRootJointForce = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      TObjectDoubleHashMap<String> jointPowerLimits = toolbox.getOptimizationSettings().getJointPowerLimits();
      if (jointPowerLimits != null)
      {
         jointPowerLimits.forEachKey(jointName ->
         {
            double powerLimit = jointPowerLimits.get(jointName);
            YoDouble powerConstraint = new YoDouble("powerLimit_" + jointName, registry);
            powerConstraint.set(powerLimit);
            powerConstrainedJointLimits.put(jointName, powerLimit);
            return true;
         });
      }

      minimizeJointTorques = new YoBoolean("minimizeJointTorques", registry);
      minimizeJointTorques.set(toolbox.getOptimizationSettings().areJointTorquesMinimized());
      jointTorqueLimitEnforcementMethod = new YoEnum<>("jointTorqueLimitEnforcementMethod", registry, JointTorqueLimitEnforcementMethod.class);
      jointTorqueLimitEnforcementMethod.set(toolbox.getOptimizationSettings().getJointTorqueLimitEnforcementMethod());
      jointPowerLimitEnforcementMethod = new YoEnum<>("jointPowerLimitEnforcementMethod", registry, JointPowerLimitEnforcementMethod.class);
      jointPowerLimitEnforcementMethod.set(toolbox.getOptimizationSettings().getJointPowerLimitEnforcementMethod());
      useDynamicMatrixCalculatorForInverseDynamics = new YoBoolean("useDynamicMatrixCalculator", registry);
      useDynamicMatrixCalculatorForInverseDynamics.set(toolbox.getOptimizationSettings().useDynamicMatrixCalculatorForInverseDynamics());
      updateDynamicMatrixCalculator = new YoBoolean("updateDynamicMatrixCalculator", registry);
      updateDynamicMatrixCalculator.set(toolbox.getOptimizationSettings().updateDynamicMatrixCalculator());

      if (useDynamicMatrixCalculatorForInverseDynamics.getValue() && !updateDynamicMatrixCalculator.getValue())
         throw new RuntimeException("Invalid optimization configuration. Dynamic matrix calculator needs to be updated in order to use for inverse dynamics");

      setupTimer = new ExecutionTimer("inverseDynamicsSetupTimer", registry);
      outputTimer = new ExecutionTimer("inverseDynamicsOutputTimer", registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
      jointAccelerationIntegrationCalculator.resetJointParameters();

      if (updateDynamicMatrixCalculator.getValue())
      {
         dynamicsMatrixCalculator.reset();
      }
      if (!useDynamicMatrixCalculatorForInverseDynamics.getValue())
      {
         inverseDynamicsCalculator.setExternalWrenchesToZero();
      }
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      optimizationControlModule.initialize();
      planeContactWrenchProcessor.initialize();

      if (updateDynamicMatrixCalculator.getValue())
      {
         dynamicsMatrixCalculator.reset();
      }
      if (!useDynamicMatrixCalculatorForInverseDynamics.getValue())
      {
         inverseDynamicsCalculator.compute();
      }

      optimizationControlModule.resetRateRegularization();
      for (int i = 0; i < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(i).clear();
   }

   public void compute()
   {
      setupTimer.startMeasurement();

      if (minimizeJointTorques.getValue())
      {
         optimizationControlModule.setupTorqueMinimizationCommand();
      }
      if (jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP
          || jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP
          || jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER
          || jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER)
      {
         optimizationControlModule.setupTorqueConstraintCommand();
      }

      setupTimer.stopMeasurement();

      if (!optimizationControlModule.compute())
      {
         // TODO:
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
      }
      outputTimer.startMeasurement();
      MomentumModuleSolution momentumModuleSolution = optimizationControlModule.getMomentumModuleSolution();

      DMatrixRMaj jointAccelerations = momentumModuleSolution.getJointAccelerations();
      DMatrixRMaj rhoSolution = momentumModuleSolution.getRhoSolution();
      Map<RigidBodyBasics, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      List<RigidBodyBasics> rigidBodiesWithExternalWrench = momentumModuleSolution.getRigidBodiesWithExternalWrench();
      SpatialForceReadOnly centroidalMomentumRateSolution = momentumModuleSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.setMatchingFrame(centroidalMomentumRateSolution.getLinearPart());
      achievedMomentumRateLinear.setIncludingFrame(yoAchievedMomentumRateLinear);

      yoAchievedMomentumRateAngular.setMatchingFrame(centroidalMomentumRateSolution.getAngularPart());
      achievedMomentumRateAngular.setIncludingFrame(yoAchievedMomentumRateAngular);

      if (useDynamicMatrixCalculatorForInverseDynamics.getValue())
      {
         // TODO Since switch to Mecano: need to update the inverse dynamics to update the rigid-body accelerations, kinda dumb.
         //         rigidBodyAccelerationProvider.compute();
         inverseDynamicsCalculator.compute(jointAccelerations);

         dynamicsMatrixCalculator.compute();
         DMatrixRMaj tauSolution = dynamicsMatrixCalculator.computeJointTorques(jointAccelerations, rhoSolution);

         for (int jointIndex = 0; jointIndex < controlledOneDoFJoints.length; jointIndex++)
         {
            OneDoFJointBasics joint = controlledOneDoFJoints[jointIndex];
            int jointAccelerationIndex = inverseDynamicsCalculator.getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
            JointDesiredOutputBasics jointDesiredOutput = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            jointDesiredOutput.setDesiredAcceleration(jointAccelerations.get(jointAccelerationIndex, 0));
            double tau = tauSolution.get(jointIndex, 0);
            boolean hasTorqueLimits = (jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_CONTROLLER
                                       || jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER);
            boolean hasPowerLimits = (jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_CONTROLLER
                                      || jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER);

            if (hasTorqueLimits || hasPowerLimits)
               jointDesiredOutput.setDesiredTorque(getClampedDesiredTorque(tau, joint, jointDesiredOutput, hasTorqueLimits, hasPowerLimits));
            else
               jointDesiredOutput.setDesiredTorque(tau);
         }

         if (!kinematicLoopFunctions.isEmpty())
            throw new UnsupportedOperationException("The use of the dynamic matrix calculator in the presence of kinematic loop(s) has not been implemented nor tested.");
      }
      else
      {
         for (int i = 0; i < rigidBodiesWithExternalWrench.size(); i++)
         {
            RigidBodyBasics rigidBody = rigidBodiesWithExternalWrench.get(i);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenchSolution.get(rigidBody));
         }

         inverseDynamicsCalculator.compute(jointAccelerations);

         for (OneDoFJointBasics joint : controlledOneDoFJoints)
         {
            int jointIndex = inverseDynamicsCalculator.getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
            JointDesiredOutputBasics jointDesiredOutput = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            jointDesiredOutput.setDesiredAcceleration(jointAccelerations.get(jointIndex, 0));
            double tau = inverseDynamicsCalculator.getComputedJointTau(joint).get(0);
            boolean hasTorqueLimits = (jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_CONTROLLER
                                       || jointTorqueLimitEnforcementMethod.getValue() == JointTorqueLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER);
            boolean hasPowerLimits = (jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_CONTROLLER
                                      || jointPowerLimitEnforcementMethod.getValue() == JointPowerLimitEnforcementMethod.CONSTRAINTS_IN_QP_AND_CONTROLLER);

            if (hasTorqueLimits || hasPowerLimits)
               jointDesiredOutput.setDesiredTorque(getClampedDesiredTorque(tau, joint, jointDesiredOutput, hasTorqueLimits, hasPowerLimits));
            else
               jointDesiredOutput.setDesiredTorque(tau);
         }

         updateKinematicLoopJointEfforts();
      }

      if (rootJoint != null)
         rootJointDesiredConfiguration.setDesiredAcceleration(jointAccelerations, 0);

      jointAccelerationIntegrationCalculator.computeAndUpdateDataHolder(lowLevelOneDoFJointDesiredDataHolder);

      if (rootJoint != null)
      {
         residualRootJointWrench.setIncludingFrame(rootJoint.getJointWrench());
         residualRootJointTorque.setIncludingFrame(residualRootJointWrench.getAngularPart());
         residualRootJointForce.setIncludingFrame(residualRootJointWrench.getLinearPart());
         yoResidualRootJointForce.setMatchingFrame(residualRootJointForce);
         yoResidualRootJointTorque.setMatchingFrame(residualRootJointTorque);
      }

      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJointReadOnly joint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         jointAccelerationsSolution.get(joint).set(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(jointIndex));
      }

      planeContactWrenchProcessor.compute(externalWrenchSolution);
      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenchSolution);

      outputTimer.stopMeasurement();
   }

   private final DMatrixRMaj kinematicLoopJointTau = new DMatrixRMaj(4, 1);

   private void updateKinematicLoopJointEfforts()
   {
      for (int i = 0; i < kinematicLoopFunctions.size(); i++)
      {
         KinematicLoopFunction kinematicLoopFunction = kinematicLoopFunctions.get(i);
         List<? extends OneDoFJointReadOnly> loopJoints = kinematicLoopFunction.getLoopJoints();

         if (loopJoints != null && !loopJoints.isEmpty())
         {
            kinematicLoopJointTau.reshape(loopJoints.size(), 1);

            for (int j = 0; j < loopJoints.size(); j++)
            {
               OneDoFJointReadOnly loopJoint = loopJoints.get(j);
               double tau = lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque((OneDoFJointBasics) loopJoint);
               kinematicLoopJointTau.set(j, tau);
            }

            kinematicLoopFunction.adjustTau(kinematicLoopJointTau);

            for (int j = 0; j < loopJoints.size(); j++)
            {
               OneDoFJointReadOnly loopJoint = loopJoints.get(j);
               // TODO The following cast is ugly and does not seem like it should be needed.
               lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque((OneDoFJointBasics) loopJoint, kinematicLoopJointTau.get(j));
            }
         }
      }
   }

   public void submitResetIntegratorRequests(JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      for (int i = 0; i < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly joint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(i);
         if (jointDesiredOutputList.hasDataForJoint(joint))
         {
            JointDesiredOutputReadOnly jointDesiredOutputOther = jointDesiredOutputList.getJointDesiredOutput(joint);
            lowLevelOneDoFJointDesiredDataHolder.setResetJointIntegrators(joint, jointDesiredOutputOther.peekResetIntegratorsRequest());
         }
      }
   }

   public void submitInverseDynamicsCommandList(InverseDynamicsCommandList inverseDynamicsCommandList)
   {
      if (updateDynamicMatrixCalculator.getValue())
      {
         dynamicsMatrixCalculator.compute();
      }

      for (int i = 0; i < inverseDynamicsCommandList.getNumberOfCommands(); i++)
      {
         InverseDynamicsCommand<?> command = inverseDynamicsCommandList.getCommand(i);

         switch (command.getCommandType())
         {
            case QP_INPUT:
               optimizationControlModule.submitQPObjectiveCommand((QPObjectiveCommand) command);
               break;
            case TASKSPACE:
               optimizationControlModule.submitSpatialAccelerationCommand((SpatialAccelerationCommand) command);
               break;
            case JOINTSPACE:
               if (command instanceof JointspaceAccelerationCommand accelerationCommand)
               {
                  optimizationControlModule.submitJointspaceAccelerationCommand(accelerationCommand);
               }
               else if (command instanceof JointTorqueCommand jointTorqueCommand)
               {
                  if (!updateDynamicMatrixCalculator.getValue())
                     throw new RuntimeException("Dynamic matrix calculator must be enabled in order to consume a JointTorqueCommand");
                  optimizationControlModule.submitJointTorqueCommand(jointTorqueCommand);
               }
               else
               {
                  throw new RuntimeException("The command type: (type=" + command.getCommandType() + ")-(class=" + command.getClass().getSimpleName()
                                             + ") is not handled.");
               }
               break;
            case MOMENTUM:
               optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
               recordMomentumRate((MomentumRateCommand) command);
               break;
            case MOMENTUM_COST:
               optimizationControlModule.submitLinearMomentumRateCostCommand((LinearMomentumRateCostCommand) command);
               break;
            case PRIVILEGED_CONFIGURATION:
               optimizationControlModule.submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
               break;
            case PRIVILEGED_JOINTSPACE_COMMAND:
               optimizationControlModule.submitPrivilegedAccelerationCommand((PrivilegedJointSpaceCommand) command);
               break;
            case LIMIT_REDUCTION:
               optimizationControlModule.submitJointLimitReductionCommand((JointLimitReductionCommand) command);
               break;
            case JOINT_LIMIT_ENFORCEMENT:
               optimizationControlModule.submitJointLimitEnforcementMethodCommand((JointLimitEnforcementMethodCommand) command);
               break;
            case EXTERNAL_WRENCH:
               optimizationControlModule.submitExternalWrenchCommand((ExternalWrenchCommand) command);
               if (useDynamicMatrixCalculatorForInverseDynamics.getValue())
                  dynamicsMatrixCalculator.setExternalWrench(((ExternalWrenchCommand) command).getRigidBody(),
                                                             ((ExternalWrenchCommand) command).getExternalWrench());
               break;
            case CONTACT_WRENCH:
               optimizationControlModule.submitContactWrenchCommand((ContactWrenchCommand) command);
               break;
            case PLANE_CONTACT_STATE:
               optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
               break;
            case CENTER_OF_PRESSURE:
               optimizationControlModule.submitCenterOfPressureCommand((CenterOfPressureCommand) command);
               break;
            case JOINT_ACCELERATION_INTEGRATION:
               jointAccelerationIntegrationCalculator.submitJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) command);
               break;
            case COMMAND_LIST:
               submitInverseDynamicsCommandList((InverseDynamicsCommandList) command);
               break;
            case OPTIMIZATION_SETTINGS:
               submitOptimizationSettingsCommand((InverseDynamicsOptimizationSettingsCommand) command);
               break;
            default:
               throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }

      inverseDynamicsCommandList.clear();
   }

   private void submitOptimizationSettingsCommand(InverseDynamicsOptimizationSettingsCommand command)
   {
      if (Double.isFinite(command.getJointTorqueWeight()) && !minimizeJointTorques.getValue())
         minimizeJointTorques.set(true);

      optimizationControlModule.submitOptimizationSettingsCommand(command);
   }

   // FIXME this assumes there is only one momentum rate command
   private void recordMomentumRate(MomentumRateCommand command)
   {
      DMatrixRMaj momentumRate = command.getMomentumRate();
      yoDesiredMomentumRateAngular.set(0, momentumRate);
      yoDesiredMomentumRateLinear.set(3, momentumRate);
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfiguration;
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return planeContactWrenchProcessor.getDesiredCenterOfPressureDataHolder();
   }

   public FrameVector3DReadOnly getAchievedMomentumRateLinear()
   {
      return achievedMomentumRateLinear;
   }

   public FrameVector3DReadOnly getAchievedMomentumRateAngular()
   {
      return achievedMomentumRateAngular;
   }

   public JointBasics[] getJointsToOptimizeFors()
   {
      return jointsToOptimizeFor;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(optimizationControlModule.getSCS2YoGraphics());
      return group;
   }

   private double getClampedDesiredTorque(double tau,
                                          OneDoFJointBasics joint,
                                          JointDesiredOutputBasics jointDesiredOutput,
                                          boolean hasTorqueLimits,
                                          boolean hasPowerLimits)
   {
      double torqueLimitLower = Double.NEGATIVE_INFINITY;
      double torqueLimitUpper = Double.POSITIVE_INFINITY;
      double powerLimitLower = Double.NEGATIVE_INFINITY;
      double powerLimitUpper = Double.POSITIVE_INFINITY;

      if (hasPowerLimits)
      {
         if (powerConstrainedJointLimits.containsKey(joint.getName()))
         {
            powerLimitLower = -powerConstrainedJointLimits.get(joint.getName());
            powerLimitUpper = powerConstrainedJointLimits.get(joint.getName());

            torqueConstraintHandler.computeTorqueConstraints(joint, powerLimitLower, powerLimitUpper, hasTorqueLimits);
            torqueLimitLower = torqueConstraintHandler.getTorqueLimitLower();
            torqueLimitUpper = torqueConstraintHandler.getTorqueLimitUpper();

         }
         else if (hasTorqueLimits)
         {
            torqueLimitLower = joint.getEffortLimitLower();
            torqueLimitUpper = joint.getEffortLimitUpper();
         }
      }
      return MathTools.clamp(tau, torqueLimitLower, torqueLimitUpper);
   }
}
