package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationException;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class WholeBodyInverseKinematicsSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseKinematicsOptimizationControlModule optimizationControlModule;
   private final RobotJointVelocityAccelerationIntegrator integrator;

   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJoint, YoDouble> jointVelocitiesSolution = new HashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositionsSolution = new HashMap<>();

   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] controlledOneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final JointIndexHandler jointIndexHandler;

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

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
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
         OneDoFJoint joint = controlledOneDoFJoints[i];
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
         case COMMAND_LIST:
            submitInverseKinematicsCommandList((InverseKinematicsCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
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
