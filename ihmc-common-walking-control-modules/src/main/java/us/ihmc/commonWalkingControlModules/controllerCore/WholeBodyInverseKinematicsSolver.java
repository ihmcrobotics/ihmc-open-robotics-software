package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationException;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointVelocityIntegrationCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class WholeBodyInverseKinematicsSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseKinematicsOptimizationControlModule optimizationControlModule;

   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJoint, YoDouble> jointVelocitiesSolution = new HashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositionsSolution = new HashMap<>();

   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] controlledOneDoFJoints;
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final JointIndexHandler jointIndexHandler;

   private final JointVelocityIntegrationCalculator jointVelocityIntegrationCalculator;

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

      yoDesiredMomentumLinear = toolbox.getYoDesiredMomentumLinear();
      yoDesiredMomentumAngular = toolbox.getYoDesiredMomentumAngular();
      yoAchievedMomentumLinear = toolbox.getYoAchievedMomentumLinear();
      yoAchievedMomentumAngular = toolbox.getYoAchievedMomentumAngular();

      jointVelocityIntegrationCalculator = new JointVelocityIntegrationCalculator(toolbox.getControlDT(), registry);

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

      SpatialForceVector centroidalMomentumSolution = inverseKinematicsSolution.getCentroidalMomentumSolution();
      yoAchievedMomentumLinear.set(centroidalMomentumSolution.getLinearPart());
      yoAchievedMomentumAngular.set(centroidalMomentumSolution.getAngularPart());

      updateLowLevelData(jointVelocities);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
         jointPositionsSolution.get(joint).set(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointPosition(joint));
      }

   }

   private void updateLowLevelData(DenseMatrix64F jointVelocities)
   {
      if (rootJoint != null)
      {
         int[] rootJointIndices = jointIndexHandler.getJointIndices(rootJoint);
         rootJointDesiredConfiguration.setDesiredVelocity(jointVelocities, rootJointIndices[0]);
      }

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double desiredVelocity = jointVelocities.get(jointIndex, 0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, desiredVelocity);
         jointVelocitiesSolution.get(joint).set(desiredVelocity);
      }

      jointVelocityIntegrationCalculator.computeAndUpdateDataHolder(lowLevelOneDoFJointDesiredDataHolder);
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
         case PRIVILEGED_CONFIGURATION:
            optimizationControlModule.submitPrivilegedConfigurationCommand((PrivilegedConfigurationCommand) command);
            break;
         case PRIVILEGED_JOINTSPACE_COMMAND:
            optimizationControlModule.submitPrivilegedVelocityCommand((PrivilegedJointSpaceCommand) command);
            break;
         case LIMIT_REDUCTION:
            optimizationControlModule.submitJointLimitReductionCommand((JointLimitReductionCommand) command);
            break;
         case JOINT_VELOCITY_INTEGRATION:
            jointVelocityIntegrationCalculator.submitJointVelocityIntegrationCommand((JointVelocityIntegrationCommand) command);
            break;
         case COMMAND_LIST:
            submitInverseKinematicsCommandList((InverseKinematicsCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   private void recordMomentumRate(MomentumCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentum();
      MatrixTools.extractFixedFrameTupleFromEJMLVector(yoDesiredMomentumAngular, momentumRate, 0);
      MatrixTools.extractFixedFrameTupleFromEJMLVector(yoDesiredMomentumLinear, momentumRate, 3);
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
