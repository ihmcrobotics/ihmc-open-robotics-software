package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

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
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyInverseKinematicsSolver implements SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final InverseKinematicsOptimizationControlModule optimizationControlModule;
   private final RobotJointVelocityAccelerationIntegrator integrator;

   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final Map<OneDoFJointBasics, YoDouble> jointVelocitiesSolution = new HashMap<>();
   private final Map<OneDoFJointBasics, YoDouble> jointPositionsSolution = new HashMap<>();

   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final JointBasics[] jointsToOptimizeFor;
   private final List<KinematicLoopFunction> kinematicLoopFunctions;
   private final JointIndexHandler jointIndexHandler;

   private final YoFrameVector3D yoDesiredMomentumLinear;
   private final YoFrameVector3D yoDesiredMomentumAngular;
   private final YoFrameVector3D yoAchievedMomentumLinear;
   private final YoFrameVector3D yoAchievedMomentumAngular;

   public WholeBodyInverseKinematicsSolver(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      rootJoint = toolbox.getRootJoint();
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      kinematicLoopFunctions = toolbox.getKinematicLoopFunctions();
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

      DMatrixRMaj jointVelocities = inverseKinematicsSolution.getJointVelocities();
      DMatrixRMaj jointTorques = inverseKinematicsSolution.getJointTorques();

      integrator.integrateJointVelocities(jointsToOptimizeFor, jointVelocities);

      MomentumReadOnly centroidalMomentumSolution = inverseKinematicsSolution.getCentroidalMomentumSolution();
      yoAchievedMomentumLinear.setMatchingFrame(centroidalMomentumSolution.getLinearPart());
      yoAchievedMomentumAngular.setMatchingFrame(centroidalMomentumSolution.getAngularPart());

      DMatrixRMaj jointConfigurations = integrator.getJointConfigurations();
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
         JointDesiredOutputBasics output = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         output.setDesiredVelocity(desiredVelocity);

         if (jointTorques != null)
         {
            double desiredTorque = jointTorques.get(jointIndex, 0);
            output.setDesiredTorque(desiredTorque);
         }

         jointVelocitiesSolution.get(joint).set(desiredVelocity);

         if (rootJoint != null)
            jointIndex++; // Because of quaternion :/
         double desiredPosition = jointConfigurations.get(jointIndex, 0);
         output.setDesiredPosition(desiredPosition);
         jointPositionsSolution.get(joint).set(desiredPosition);
      }

      updateKinematicLoopJointConfigurations();
   }

   private final DMatrixRMaj kinematicLoopJointConfiguration = new DMatrixRMaj(4, 1);

   private void updateKinematicLoopJointConfigurations()
   {
      for (int i = 0; i < kinematicLoopFunctions.size(); i++)
      {
         KinematicLoopFunction kinematicLoopFunction = kinematicLoopFunctions.get(i);
         List<? extends OneDoFJointReadOnly> loopJoints = kinematicLoopFunction.getLoopJoints();
         kinematicLoopJointConfiguration.reshape(loopJoints.size(), 1);

         for (int j = 0; j < loopJoints.size(); j++)
         {
            OneDoFJointReadOnly loopJoint = loopJoints.get(j);
            double position = lowLevelOneDoFJointDesiredDataHolder.getDesiredJointPosition((OneDoFJointBasics) loopJoint);
            kinematicLoopJointConfiguration.set(j, position);
         }

         kinematicLoopFunction.adjustConfiguration(kinematicLoopJointConfiguration);

         for (int j = 0; j < loopJoints.size(); j++)
         {
            OneDoFJointReadOnly loopJoint = loopJoints.get(j);
            // TODO The following cast is ugly and does not seem like it should be needed.
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition((OneDoFJointBasics) loopJoint, kinematicLoopJointConfiguration.get(j));
            jointPositionsSolution.get(loopJoint).set(kinematicLoopJointConfiguration.get(j));
         }
      }
   }

   public void submitInverseKinematicsCommandList(InverseKinematicsCommandList inverseKinematicsCommandList)
   {
      for (int commandIndex = 0; commandIndex < inverseKinematicsCommandList.getNumberOfCommands(); commandIndex++)
      {
         InverseKinematicsCommand<?> command = inverseKinematicsCommandList.getCommand(commandIndex);

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
      inverseKinematicsCommandList.clear();
   }

   private void recordMomentumRate(MomentumCommand command)
   {
      DMatrixRMaj momentumRate = command.getMomentum();
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(optimizationControlModule.getSCS2YoGraphics());
      if (group.isEmpty())
         return null;
      return group;
   }
}
