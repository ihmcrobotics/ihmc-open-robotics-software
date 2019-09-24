package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationException;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class WholeBodyInverseKinematicsSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseKinematicsOptimizationControlModule optimizationControlModule;
   private final RobotJointVelocityAccelerationIntegrator integrator;

   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJointBasics, YoDouble> jointVelocitiesSolution = new HashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointPositionsSolution = new HashMap<>();

   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;
   private final JointIndexHandler jointIndexHandler;

   private final YoFrameVector3D yoDesiredMomentumLinear;
   private final YoFrameVector3D yoDesiredMomentumAngular;
   private final YoFrameVector3D yoAchievedMomentumLinear;
   private final YoFrameVector3D yoAchievedMomentumAngular;

   public WholeBodyInverseKinematicsSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      rootJoint = toolbox.getRootJoint();
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      optimizationControlModule = new InverseKinematicsOptimizationControlModule(toolbox, registry);
      integrator = new RobotJointVelocityAccelerationIntegrator(toolbox.getControlDT());

      yoDesiredMomentumLinear = toolbox.getYoDesiredMomentumLinear();
      yoDesiredMomentumAngular = toolbox.getYoDesiredMomentumAngular();
      yoAchievedMomentumLinear = toolbox.getYoAchievedMomentumLinear();
      yoAchievedMomentumAngular = toolbox.getYoAchievedMomentumAngular();

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[i];
         YoDouble jointVelocitySolution = new YoDouble("qd_qp_" + joint.getName(), registry);
         YoDouble jointPositionSolution = new YoDouble("q_qp_" + joint.getName(), registry);
         jointVelocitySolution.set(Double.NaN);
         jointPositionSolution.set(Double.NaN);
         jointVelocitiesSolution.put(joint, jointVelocitySolution);
         jointPositionsSolution.put(joint, jointPositionSolution);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
   }

   public void compute()
   {
      InverseKinematicsSolution inverseKinematicsSolution;

      try
      {
         inverseKinematicsSolution = optimizationControlModule.compute();
      }
      catch (InverseKinematicsOptimizationException inverseKinematicsOptimizationException)
      {
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         inverseKinematicsSolution = inverseKinematicsOptimizationException.getSolution();
      }

      DenseMatrix64F jointVelocities = inverseKinematicsSolution.getJointVelocities();

      integrator.integrateJointVelocities(jointsToOptimizeFor, jointVelocities);

      MomentumReadOnly centroidalMomentumSolution = inverseKinematicsSolution.getCentroidalMomentumSolution();
      yoAchievedMomentumLinear.setMatchingFrame(centroidalMomentumSolution.getLinearPart());
      yoAchievedMomentumAngular.setMatchingFrame(centroidalMomentumSolution.getAngularPart());

      DenseMatrix64F jointConfigurations = integrator.getJointConfigurations();
      jointVelocities = integrator.getJointVelocities();

      if (rootJoint != null)
      {
         int[] rootJointIndices = jointIndexHandler.getJointIndices(rootJoint);
         rootJointDesiredConfiguration.setDesiredConfiguration(jointConfigurations, rootJointIndices[0]);
         rootJointDesiredConfiguration.setDesiredVelocity(jointVelocities, rootJointIndices[0]);
      }

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[i];
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double desiredVelocity = jointVelocities.get(jointIndex, 0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, desiredVelocity);
         jointVelocitiesSolution.get(joint).set(desiredVelocity);

         if (rootJoint != null)
            jointIndex++; // Because of quaternion :/
         double desiredPosition = jointConfigurations.get(jointIndex, 0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(joint, desiredPosition);
         jointPositionsSolution.get(joint).set(desiredPosition);
      }
   }

   public void submitInverseKinematicsCommandList(InverseKinematicsCommandList inverseKinematicsCommandList)
   {
      while (inverseKinematicsCommandList.getNumberOfCommands() > 0)
      {
         InverseKinematicsCommand<?> command = inverseKinematicsCommandList.pollCommand();

         switch (command.getCommandType())
         {
         case TASKSPACE:
            optimizationControlModule.submitSpatialVelocityCommand((SpatialVelocityCommand) command);
            break;
         case JOINTSPACE:
            optimizationControlModule.submitJointspaceVelocityCommand((JointspaceVelocityCommand) command);
            break;
         case MOMENTUM:
            optimizationControlModule.submitMomentumCommand((MomentumCommand) command);
            recordMomentumRate((MomentumCommand) command);
            break;
         case MOMENTUM_CONVEX_CONSTRAINT:
            optimizationControlModule.submitLinearMomentumConvexConstraint2DCommand((LinearMomentumConvexConstraint2DCommand) command);
            break;
         case PRIVILEGED_CONFIGURATION:
            optimizationControlModule.submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
            break;
         case PRIVILEGED_JOINTSPACE_COMMAND:
            optimizationControlModule.submitPrivilegedVelocityCommand((PrivilegedJointSpaceCommand) command);
            break;
         case LIMIT_REDUCTION:
            optimizationControlModule.submitJointLimitReductionCommand((JointLimitReductionCommand) command);
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            optimizationControlModule.submitJointLimitEnforcementMethodCommand((JointLimitEnforcementMethodCommand) command);
            break;
         case COMMAND_LIST:
            submitInverseKinematicsCommandList((InverseKinematicsCommandList) command);
            break;
         case OPTIMIZATION_SETTINGS:
            optimizationControlModule.submitOptimizationSettingsCommand((InverseKinematicsOptimizationSettingsCommand) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   private void recordMomentumRate(MomentumCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentum();
      yoDesiredMomentumAngular.set(0, momentumRate);
      yoDesiredMomentumLinear.set(3, momentumRate);
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfiguration;
   }
}
