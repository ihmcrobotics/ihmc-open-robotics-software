package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.AtlasHumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.SensorDataContext;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

/**
 * The objective of this class is to help the passing commands between two instances of the same
 * robot.
 * <p>
 * The main use-case is for passing commands from one thread to another. In such context, each
 * thread has its own instance of the robot and the corresponding reference frame tree.
 * </p>
 * <p>
 * The main challenge when passing commands is to retrieve the joints, rigid-bodies, and reference
 * frames properly.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class CrossRobotCommandResolver
{
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final JointHashCodeResolver jointHashCodeResolver;

   public CrossRobotCommandResolver(FullHumanoidRobotModel fullRobotModel)
   {
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();
      referenceFrameHashCodeResolver.putAllChildren(fullRobotModel.getRootJoint().getFrameAfterJoint());
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(fullRobotModel);
      jointHashCodeResolver = new JointHashCodeResolver(fullRobotModel);
   }

   public CrossRobotCommandResolver(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                                    JointHashCodeResolver jointHashCodeResolver)
   {
      this.referenceFrameHashCodeResolver = referenceFrameHashCodeResolver;
      this.rigidBodyHashCodeResolver = rigidBodyHashCodeResolver;
      this.jointHashCodeResolver = jointHashCodeResolver;
   }

   public void resolveControllerCoreCommand(ControllerCoreCommandInterface in, ControllerCoreCommandBuffer out)
   {
      out.clear();
      out.setControllerCoreMode(in.getControllerCoreMode());
      if (in.isReinitializationRequested())
         out.requestReinitialization();
      resolveInverseDynamicsCommandList(in.getInverseDynamicsCommandList(), out.getInverseDynamicsCommandList());
      resolveInverseKinematicsCommandList(in.getInverseKinematicsCommandList(), out.getInverseKinematicsCommandList());
      resolveVirtualModelControlCommandList(in.getVirtualModelControlCommandList(), out.getVirtualModelControlCommandList());
      resolveFeedbackControlCommandList(in.getFeedbackControlCommandList(), out.getFeedbackControlCommandList());
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getLowLevelOneDoFJointDesiredDataHolder(), out.getLowLevelOneDoFJointDesiredDataHolder());
   }

   public void resolveControllerCoreOutput(ControllerCoreOutput in, ControllerCoreOutput out)
   {
      resolveFrameTuple3D(in.getLinearMomentumRate(), out.getLinearMomentumRate());
      resolveCenterOfPressureDataHolder(in.getCenterOfPressureData(), out.getCenterOfPressureData());
      out.setRootJointDesiredConfigurationData(in.getRootJointDesiredConfigurationData());
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getLowLevelOneDoFJointDesiredDataHolderPreferred(), out.getLowLevelOneDoFJointDesiredDataHolderPreferred());
   }

   public void resolveCenterOfPressureDataHolder(CenterOfPressureDataHolder in, CenterOfPressureDataHolder out)
   {
      out.clear();
      for (int i = 0; i < in.getNumberOfBodiesWithCenterOfPressure(); i++)
      {
         out.registerRigidBody(resolveRigidBody(in.getRigidBody(i)));
         resolveFrameTuple2D(in.getCenterOfPressure(i), out.getCenterOfPressure(i));
      }
   }

   public void resolveForceSensorDataHolder(ForceSensorDataHolder in, ForceSensorDataHolder out)
   {
      out.clear();
      for (int i = 0; i < in.getNumberOfForceSensors(); i++)
      {
         ForceSensorDefinition inDefinition = in.getForceSensorDefinitions().get(i);
         ForceSensorData inData = in.get(inDefinition);
         out.registerForceSensor(inDefinition);
         ForceSensorDefinition outDefinition = out.getForceSensorDefinitions().get(i);
         ForceSensorData outData = out.get(outDefinition);

         resolveForceSensorDefinition(inDefinition, outDefinition);
         resolveForceSensorData(inData, outData);
      }
   }

   public void resolveForceSensorData(ForceSensorData in, ForceSensorData out)
   {
      out.set(in);
      out.setFrameAndBody(resolveReferenceFrame(in.getMeasurementFrame()), resolveRigidBody(in.getMeasurementLink()));
   }

   public void resolveForceSensorDefinition(ForceSensorDefinition in, ForceSensorDefinition out)
   {
      out.set(in.getSensorName(), resolveRigidBody(in.getRigidBody()), resolveReferenceFrame(in.getSensorFrame()));
   }

   public void resolveHumanoidRobotContextData(HumanoidRobotContextData in, HumanoidRobotContextData out)
   {
      resolveHumanoidRobotContextDataScheduler(in, out);
      resolveHumanoidRobotContextDataController(in, out);
      resolveHumanoidRobotContextDataEstimator(in, out);
   }

   public void resolveHumanoidRobotContextDataScheduler(HumanoidRobotContextData in, HumanoidRobotContextData out)
   {
      resolveSensorDataContext(in.getSensorDataContext(), out.getSensorDataContext());
   }

   public void resolveHumanoidRobotContextDataController(HumanoidRobotContextData in, HumanoidRobotContextData out)
   {
      resolveCenterOfPressureDataHolder(in.getCenterOfPressureDataHolder(), out.getCenterOfPressureDataHolder());
      resolveRobotMotionStatusHolder(in.getRobotMotionStatusHolder(), out.getRobotMotionStatusHolder());
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getJointDesiredOutputList(), out.getJointDesiredOutputList());
      out.setControllerRan(in.getControllerRan());
   }

   public void resolveHumanoidRobotContextDataEstimator(HumanoidRobotContextData in, HumanoidRobotContextData out)
   {
      resolveHumanoidRobotContextJointData(in.getProcessedJointData(), out.getProcessedJointData());
      resolveForceSensorDataHolder(in.getForceSensorDataHolder(), out.getForceSensorDataHolder());
      out.setTimestamp(in.getTimestamp());
      out.setEstimatorRan(in.getEstimatorRan());
   }

   public void resolveAtlasHumanoidRobotContextData(AtlasHumanoidRobotContextData in, AtlasHumanoidRobotContextData out)
   {
      resolveRawJointSensorDataHolderMap(in.getRawJointSensorDataHolderMap(), out.getRawJointSensorDataHolderMap());
      resolveHumanoidRobotContextData(in, out);
   }

   public void resolveSensorDataContext(SensorDataContext in, SensorDataContext out)
   {
      out.set(in);
   }

   public void resolveImuData(ImuData in, ImuData out)
   {
      out.set(in);
   }

   public void resolveRawJointSensorDataHolderMap(RawJointSensorDataHolderMap in, RawJointSensorDataHolderMap out)
   {
      out.clear();
      for (int i = 0; i < in.getNumberOfJoints(); i++)
      {
         out.registerJoint(resolveJoint(in.getJoint(i)));
         resolveRawJointSensorDataHolder(in.get(i), out.get(i));
      }
   }

   public void resolveRawJointSensorDataHolder(RawJointSensorDataHolder in, RawJointSensorDataHolder out)
   {
      out.set(in);
   }

   public void resolveInverseDynamicsCommandList(InverseDynamicsCommandList in, InverseDynamicsCommandBuffer out)
   {
      out.clear();
      resolveInverseDynamicsCommandListInternal(in, out);
   }

   public void resolveInverseKinematicsCommandList(InverseKinematicsCommandList in, InverseKinematicsCommandBuffer out)
   {
      out.clear();
      resolveInverseKinematicsCommandListInternal(in, out);
   }

   public void resolveVirtualModelControlCommandList(VirtualModelControlCommandList in, VirtualModelControlCommandBuffer out)
   {
      out.clear();
      resolveVirtualModelControlCommandListInternal(in, out);
   }

   public void resolveFeedbackControlCommandList(FeedbackControlCommandList in, FeedbackControlCommandBuffer out)
   {
      out.clear();
      resolveFeedbackControlCommandListInternal(in, out);
   }

   private void resolveInverseDynamicsCommandListInternal(InverseDynamicsCommandList in, InverseDynamicsCommandBuffer out)
   {
      for (int commandIndex = 0; commandIndex < in.getNumberOfCommands(); commandIndex++)
      {
         InverseDynamicsCommand<?> commandToResolve = in.getCommand(commandIndex);
         switch (commandToResolve.getCommandType())
         {
         case CENTER_OF_PRESSURE:
            resolveCenterOfPressureCommand((CenterOfPressureCommand) commandToResolve, out.addCenterOfPressureCommand());
            break;
         case CONTACT_WRENCH:
            resolveContactWrenchCommand((ContactWrenchCommand) commandToResolve, out.addContactWrenchCommand());
            break;
         case EXTERNAL_WRENCH:
            resolveExternalWrenchCommand((ExternalWrenchCommand) commandToResolve, out.addExternalWrenchCommand());
            break;
         case OPTIMIZATION_SETTINGS:
            resolveInverseDynamicsOptimizationSettingsCommand((InverseDynamicsOptimizationSettingsCommand) commandToResolve,
                                                              out.addInverseDynamicsOptimizationSettingsCommand());
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            resolveJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) commandToResolve, out.addJointAccelerationIntegrationCommand());
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            resolveJointLimitEnforcementMethodCommand((JointLimitEnforcementMethodCommand) commandToResolve, out.addJointLimitEnforcementMethodCommand());
            break;
         case JOINTSPACE:
            resolveJointspaceAccelerationCommand((JointspaceAccelerationCommand) commandToResolve, out.addJointspaceAccelerationCommand());
            break;
         case MOMENTUM:
            resolveMomentumRateCommand((MomentumRateCommand) commandToResolve, out.addMomentumRateCommand());
            break;
         case PLANE_CONTACT_STATE:
            resolvePlaneContactStateCommand((PlaneContactStateCommand) commandToResolve, out.addPlaneContactStateCommand());
            break;
         case TASKSPACE:
            resolveSpatialAccelerationCommand((SpatialAccelerationCommand) commandToResolve, out.addSpatialAccelerationCommand());
            break;
         case COMMAND_LIST:
            resolveInverseDynamicsCommandListInternal((InverseDynamicsCommandList) commandToResolve, out);
            break;
         case PRIVILEGED_CONFIGURATION:
            resolvePrivilegedConfigurationCommand((PrivilegedConfigurationCommand) commandToResolve, out.addPrivilegedConfigurationCommand());
            break;
         case PRIVILEGED_JOINTSPACE_COMMAND:
            resolvePrivilegedJointSpaceCommand((PrivilegedJointSpaceCommand) commandToResolve, out.addPrivilegedJointSpaceCommand());
            break;
         case LIMIT_REDUCTION:
            resolveJointLimitReductionCommand((JointLimitReductionCommand) commandToResolve, out.addJointLimitReductionCommand());
            break;
         default:
            throw new RuntimeException("The command type: " + commandToResolve.getCommandType() + " is not handled.");
         }
      }
   }

   private void resolveInverseKinematicsCommandListInternal(InverseKinematicsCommandList in, InverseKinematicsCommandBuffer out)
   {
      for (int commandIndex = 0; commandIndex < in.getNumberOfCommands(); commandIndex++)
      {
         InverseKinematicsCommand<?> commandToResolve = in.getCommand(commandIndex);
         switch (commandToResolve.getCommandType())
         {
         case OPTIMIZATION_SETTINGS:
            resolveInverseKinematicsOptimizationSettingsCommand((InverseKinematicsOptimizationSettingsCommand) commandToResolve,
                                                                out.addInverseKinematicsOptimizationSettingsCommand());
            break;
         case LIMIT_REDUCTION:
            resolveJointLimitReductionCommand((JointLimitReductionCommand) commandToResolve, out.addJointLimitReductionCommand());
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            resolveJointLimitEnforcementMethodCommand((JointLimitEnforcementMethodCommand) commandToResolve, out.addJointLimitEnforcementMethodCommand());
            break;
         case JOINTSPACE:
            resolveJointspaceVelocityCommand((JointspaceVelocityCommand) commandToResolve, out.addJointspaceVelocityCommand());
            break;
         case MOMENTUM:
            resolveMomentumCommand((MomentumCommand) commandToResolve, out.addMomentumCommand());
            break;
         case PRIVILEGED_CONFIGURATION:
            resolvePrivilegedConfigurationCommand((PrivilegedConfigurationCommand) commandToResolve, out.addPrivilegedConfigurationCommand());
            break;
         case PRIVILEGED_JOINTSPACE_COMMAND:
            resolvePrivilegedJointSpaceCommand((PrivilegedJointSpaceCommand) commandToResolve, out.addPrivilegedJointSpaceCommand());
            break;
         case TASKSPACE:
            resolveSpatialVelocityCommand((SpatialVelocityCommand) commandToResolve, out.addSpatialVelocityCommand());
            break;
         case COMMAND_LIST:
            resolveInverseKinematicsCommandListInternal((InverseKinematicsCommandList) commandToResolve, out);
            break;
         default:
            throw new RuntimeException("The command type: " + commandToResolve.getCommandType() + " is not handled.");
         }
      }
   }

   private void resolveVirtualModelControlCommandListInternal(VirtualModelControlCommandList in, VirtualModelControlCommandBuffer out)
   {
      for (int commandIndex = 0; commandIndex < in.getNumberOfCommands(); commandIndex++)
      {
         VirtualModelControlCommand<?> commandToResolve = in.getCommand(commandIndex);
         switch (commandToResolve.getCommandType())
         {
         case CENTER_OF_PRESSURE:
            resolveCenterOfPressureCommand((CenterOfPressureCommand) commandToResolve, out.addCenterOfPressureCommand());
            break;
         case CONTACT_WRENCH:
            resolveContactWrenchCommand((ContactWrenchCommand) commandToResolve, out.addContactWrenchCommand());
            break;
         case EXTERNAL_WRENCH:
            resolveExternalWrenchCommand((ExternalWrenchCommand) commandToResolve, out.addExternalWrenchCommand());
            break;
         case OPTIMIZATION_SETTINGS:
            resolveVirtualModelControlOptimizationSettingsCommand((VirtualModelControlOptimizationSettingsCommand) commandToResolve,
                                                                  out.addVirtualModelControlOptimizationSettingsCommand());
            break;
         case JOINT_ACCELERATION_INTEGRATION:
            resolveJointAccelerationIntegrationCommand((JointAccelerationIntegrationCommand) commandToResolve, out.addJointAccelerationIntegrationCommand());
            break;
         case JOINT_LIMIT_ENFORCEMENT:
            resolveJointLimitEnforcementCommand((JointLimitEnforcementCommand) commandToResolve, out.addJointLimitEnforcementCommand());
            break;
         case JOINTSPACE:
            resolveJointTorqueCommand((JointTorqueCommand) commandToResolve, out.addJointTorqueCommand());
            break;
         case MOMENTUM:
            resolveMomentumRateCommand((MomentumRateCommand) commandToResolve, out.addMomentumRateCommand());
            break;
         case PLANE_CONTACT_STATE:
            resolvePlaneContactStateCommand((PlaneContactStateCommand) commandToResolve, out.addPlaneContactStateCommand());
            break;
         case COMMAND_LIST:
            resolveVirtualModelControlCommandListInternal((VirtualModelControlCommandList) commandToResolve, out);
            break;
         case VIRTUAL_FORCE:
            resolveVirtualForceCommand((VirtualForceCommand) commandToResolve, out.addVirtualForceCommand());
            break;
         case VIRTUAL_TORQUE:
            resolveVirtualTorqueCommand((VirtualTorqueCommand) commandToResolve, out.addVirtualTorqueCommand());
            break;
         case VIRTUAL_WRENCH:
            resolveVirtualWrenchCommand((VirtualWrenchCommand) commandToResolve, out.addVirtualWrenchCommand());
            break;
         default:
            throw new RuntimeException("The command type: " + commandToResolve.getCommandType() + " is not handled.");
         }
      }
   }

   private void resolveFeedbackControlCommandListInternal(FeedbackControlCommandList in, FeedbackControlCommandBuffer out)
   {
      for (int commandIndex = 0; commandIndex < in.getNumberOfCommands(); commandIndex++)
      {
         FeedbackControlCommand<?> commandToResolve = in.getCommand(commandIndex);
         switch (commandToResolve.getCommandType())
         {
         case JOINTSPACE:
            resolveOneDoFJointFeedbackControlCommand((OneDoFJointFeedbackControlCommand) commandToResolve, out.addOneDoFJointFeedbackControlCommand());
            break;
         case ORIENTATION:
            resolveOrientationFeedbackControlCommand((OrientationFeedbackControlCommand) commandToResolve, out.addOrientationFeedbackControlCommand());
            break;
         case POINT:
            resolvePointFeedbackControlCommand((PointFeedbackControlCommand) commandToResolve, out.addPointFeedbackControlCommand());
            break;
         case TASKSPACE:
            resolveSpatialFeedbackControlCommand((SpatialFeedbackControlCommand) commandToResolve, out.addSpatialFeedbackControlCommand());
            break;
         case MOMENTUM:
            resolveCenterOfMassFeedbackControlCommand((CenterOfMassFeedbackControlCommand) commandToResolve, out.addCenterOfMassFeedbackControlCommand());
            break;
         case COMMAND_LIST:
            resolveFeedbackControlCommandListInternal((FeedbackControlCommandList) commandToResolve, out);
            break;
         default:
            throw new RuntimeException("The command type: " + commandToResolve.getCommandType() + " is not handled.");
         }
      }
   }

   public void resolveLowLevelOneDoFJointDesiredDataHolder(JointDesiredOutputListReadOnly in, LowLevelOneDoFJointDesiredDataHolder out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         out.registerJointWithEmptyData(resolveJoint(in.getOneDoFJoint(jointIndex))).set(in.getJointDesiredOutput(jointIndex));
      }
   }

   public void resolveCenterOfPressureCommand(CenterOfPressureCommand in, CenterOfPressureCommand out)
   {
      out.setConstraintType(in.getConstraintType());
      out.setContactingRigidBody(resolveRigidBody(in.getContactingRigidBody()));
      resolveFrameTuple2D(in.getWeight(), out.getWeight());
      resolveFrameTuple2D(in.getDesiredCoP(), out.getDesiredCoP());
   }

   public void resolveContactWrenchCommand(ContactWrenchCommand in, ContactWrenchCommand out)
   {
      out.setConstraintType(in.getConstraintType());
      out.setRigidBody(resolveRigidBody(in.getRigidBody()));
      resolveWrench(in.getWrench(), out.getWrench());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolveExternalWrenchCommand(ExternalWrenchCommand in, ExternalWrenchCommand out)
   {
      out.setRigidBody(resolveRigidBody(in.getRigidBody()));
      resolveWrench(in.getExternalWrench(), out.getExternalWrench());
   }

   public void resolveInverseDynamicsOptimizationSettingsCommand(InverseDynamicsOptimizationSettingsCommand in, InverseDynamicsOptimizationSettingsCommand out)
   {
      // There is no robot sensitive information in this command, so the output can directly be set to the input.
      out.set(in);
   }

   public void resolveJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand in, JointAccelerationIntegrationCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJointsToComputeDesiredPositionFor(); jointIndex++)
      {
         out.addJointToComputeDesiredPositionFor(resolveJoint(in.getJointToComputeDesiredPositionFor(jointIndex)));
         // There is no robot sensitive information in this command, so the output can directly be set to the input.
         out.setJointParameters(jointIndex, in.getJointParameters(jointIndex));
      }
   }

   public void resolveJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand in, JointLimitEnforcementMethodCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointBasics joint = resolveJoint(in.getJoint(jointIndex));
         JointLimitParameters parameters = in.getJointLimitParameters(jointIndex);
         JointLimitEnforcement method = in.getJointLimitReductionFactor(jointIndex);
         out.addLimitEnforcementMethod(joint, method, parameters);
      }
   }

   public void resolveJointspaceAccelerationCommand(JointspaceAccelerationCommand in, JointspaceAccelerationCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getDesiredAcceleration(jointIndex), in.getWeight(jointIndex));
      }
   }

   public void resolveMomentumRateCommand(MomentumRateCommand in, MomentumRateCommand out)

   {
      out.setMomentumRate(in.getMomentumRate());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolvePlaneContactStateCommand(PlaneContactStateCommand in, PlaneContactStateCommand out)
   {
      out.clearContactPoints();
      out.setContactingRigidBody(resolveRigidBody(in.getContactingRigidBody()));
      out.setCoefficientOfFriction(in.getCoefficientOfFriction());
      out.setUseHighCoPDamping(in.isUseHighCoPDamping());
      out.setHasContactStateChanged(in.getHasContactStateChanged());
      resolveFrameTuple3D(in.getContactNormal(), out.getContactNormal());
      out.getContactFramePoseInBodyFixedFrame().set(in.getContactFramePoseInBodyFixedFrame());

      for (int contactPointIndex = 0; contactPointIndex < in.getNumberOfContactPoints(); contactPointIndex++)
      {
         resolveFrameTuple3D(in.getContactPoint(contactPointIndex), out.addPointInContact());
         out.setMaxContactPointNormalForce(contactPointIndex, in.getMaxContactPointNormalForce(contactPointIndex));
         out.setRhoWeight(contactPointIndex, in.getRhoWeight(contactPointIndex));
      }
   }

   public void resolveSpatialAccelerationCommand(SpatialAccelerationCommand in, SpatialAccelerationCommand out)
   {
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearAcceleration().set(in.getDesiredLinearAcceleration());
      out.getDesiredAngularAcceleration().set(in.getDesiredAngularAcceleration());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      out.setPrimaryBase(resolveRigidBody(in.getPrimaryBase()));
      out.setScaleSecondaryTaskJointWeight(in.scaleSecondaryTaskJointWeight(), in.getSecondaryTaskJointWeightScale());
   }

   public void resolveInverseKinematicsOptimizationSettingsCommand(InverseKinematicsOptimizationSettingsCommand in,
                                                                   InverseKinematicsOptimizationSettingsCommand out)
   {
      // There is no robot sensitive information in this command, so the output can directly be set to the input.
      out.set(in);
   }

   public void resolveJointLimitReductionCommand(JointLimitReductionCommand in, JointLimitReductionCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addReductionFactor(resolveJoint(in.getJoint(jointIndex)), in.getJointLimitReductionFactor(jointIndex));
      }
   }

   public void resolveJointspaceVelocityCommand(JointspaceVelocityCommand in, JointspaceVelocityCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getDesiredVelocity(jointIndex), in.getWeight(jointIndex));
      }
   }

   public void resolveMomentumCommand(MomentumCommand in, MomentumCommand out)
   {
      out.setMomentum(in.getMomentum());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolvePrivilegedConfigurationCommand(PrivilegedConfigurationCommand in, PrivilegedConfigurationCommand out)
   {
      out.clear();
      out.setDefaultParameters(in.getDefaultParameters());

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getJointSpecificParameters(jointIndex));
      }

      if (in.isEnabled())
         out.enable();
      else
         out.disable();
   }

   public void resolvePrivilegedJointSpaceCommand(PrivilegedJointSpaceCommand in, PrivilegedJointSpaceCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getPrivilegedCommand(jointIndex));
         out.setWeight(jointIndex, in.getWeight(jointIndex));
      }

      if (in.isEnabled())
         out.enable();
      else
         out.disable();
   }

   public void resolveJointLimitEnforcementCommand(JointLimitEnforcementCommand in, JointLimitEnforcementCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getJointLimitData(jointIndex));
      }
   }

   public void resolveSpatialVelocityCommand(SpatialVelocityCommand in, SpatialVelocityCommand out)
   {
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearVelocity().set(in.getDesiredLinearVelocity());
      out.getDesiredAngularVelocity().set(in.getDesiredAngularVelocity());
      out.setConstraintType(in.getConstraintType());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      out.setPrimaryBase(resolveRigidBody(in.getPrimaryBase()));
      out.setScaleSecondaryTaskJointWeight(in.scaleSecondaryTaskJointWeight(), in.getSecondaryTaskJointWeightScale());
   }

   public void resolveJointTorqueCommand(JointTorqueCommand in, JointTorqueCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getDesiredTorque(jointIndex));
      }
   }

   public void resolveVirtualForceCommand(VirtualForceCommand in, VirtualForceCommand out)
   {
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearForce().set(in.getDesiredLinearForce());
      resolveSelectionMatrix3D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolveVirtualModelControlOptimizationSettingsCommand(VirtualModelControlOptimizationSettingsCommand in,
                                                                     VirtualModelControlOptimizationSettingsCommand out)
   {
      // There is no robot sensitive information in this command, so the output can directly be set to the input.
      out.set(in);
   }

   public void resolveVirtualTorqueCommand(VirtualTorqueCommand in, VirtualTorqueCommand out)
   {
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredAngularTorque().set(in.getDesiredAngularTorque());
      resolveSelectionMatrix3D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolveVirtualWrenchCommand(VirtualWrenchCommand in, VirtualWrenchCommand out)
   {
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearForce().set(in.getDesiredLinearForce());
      out.getDesiredAngularTorque().set(in.getDesiredAngularTorque());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolveCenterOfMassFeedbackControlCommand(CenterOfMassFeedbackControlCommand in, CenterOfMassFeedbackControlCommand out)
   {
      out.setControlMode(in.getControlMode());
      out.getReferencePosition().set(in.getReferencePosition());
      out.getReferenceLinearVelocity().set(in.getReferenceLinearVelocity());
      out.getReferenceLinearAcceleration().set(in.getReferenceLinearAcceleration());
      out.setGains(in.getGains());
      resolveMomentumRateCommand(in.getMomentumRateCommand(), out.getMomentumRateCommand());
   }

   public void resolveJointspaceFeedbackControlCommand(JointspaceFeedbackControlCommand in, JointspaceFeedbackControlCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         resolveOneDoFJointFeedbackControlCommand(in.getJointCommand(jointIndex), out.addEmptyCommand());
      }
   }

   public void resolveOneDoFJointFeedbackControlCommand(OneDoFJointFeedbackControlCommand in, OneDoFJointFeedbackControlCommand out)
   {
      out.set(in);
      out.setJoint(resolveJoint(in.getJoint()));
   }

   public void resolveOrientationFeedbackControlCommand(OrientationFeedbackControlCommand in, OrientationFeedbackControlCommand out)
   {
      resolveSpatialAccelerationCommand(in.getSpatialAccelerationCommand(), out.getSpatialAccelerationCommand());
      out.setControlMode(in.getControlMode());
      resolveFrameQuaternion(in.getBodyFixedOrientationToControl(), out.getBodyFixedOrientationToControl());
      resolveFrameQuaternion(in.getReferenceOrientation(), out.getReferenceOrientation());
      resolveFrameTuple3D(in.getReferenceAngularVelocity(), out.getReferenceAngularVelocity());
      resolveFrameTuple3D(in.getReferenceAngularAcceleration(), out.getReferenceAngularAcceleration());
      resolveFrameTuple3D(in.getReferenceTorque(), out.getReferenceTorque());
      out.getGains().set(in.getGains());
      out.setGainsFrame(resolveReferenceFrame(in.getAngularGainsFrame()));
      out.setControlBaseFrame(resolveReferenceFrame(in.getControlBaseFrame()));
   }

   public void resolvePointFeedbackControlCommand(PointFeedbackControlCommand in, PointFeedbackControlCommand out)
   {
      resolveSpatialAccelerationCommand(in.getSpatialAccelerationCommand(), out.getSpatialAccelerationCommand());
      out.setControlMode(in.getControlMode());
      resolveFrameTuple3D(in.getBodyFixedPointToControl(), out.getBodyFixedPointToControl());
      resolveFrameTuple3D(in.getReferencePosition(), out.getReferencePosition());
      resolveFrameTuple3D(in.getReferenceLinearVelocity(), out.getReferenceLinearVelocity());
      resolveFrameTuple3D(in.getReferenceLinearAcceleration(), out.getReferenceLinearAcceleration());
      resolveFrameTuple3D(in.getReferenceForce(), out.getReferenceForce());
      out.getGains().set(in.getGains());
      out.setGainsFrame(resolveReferenceFrame(in.getLinearGainsFrame()));
      out.setControlBaseFrame(resolveReferenceFrame(in.getControlBaseFrame()));
   }

   public void resolveSpatialFeedbackControlCommand(SpatialFeedbackControlCommand in, SpatialFeedbackControlCommand out)
   {
      resolveSpatialAccelerationCommand(in.getSpatialAccelerationCommand(), out.getSpatialAccelerationCommand());
      out.setControlMode(in.getControlMode());
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      resolveFrameTuple3D(in.getReferencePosition(), out.getReferencePosition());
      resolveFrameQuaternion(in.getReferenceOrientation(), out.getReferenceOrientation());
      resolveFrameTuple3D(in.getReferenceLinearVelocity(), out.getReferenceLinearVelocity());
      resolveFrameTuple3D(in.getReferenceAngularVelocity(), out.getReferenceAngularVelocity());
      resolveFrameTuple3D(in.getReferenceLinearAcceleration(), out.getReferenceLinearAcceleration());
      resolveFrameTuple3D(in.getReferenceAngularAcceleration(), out.getReferenceAngularAcceleration());
      resolveFrameTuple3D(in.getReferenceForce(), out.getReferenceForce());
      resolveFrameTuple3D(in.getReferenceTorque(), out.getReferenceTorque());
      out.getGains().set(in.getGains());
      out.setGainsFrames(resolveReferenceFrame(in.getAngularGainsFrame()), resolveReferenceFrame(in.getLinearGainsFrame()));
      out.setControlBaseFrame(resolveReferenceFrame(in.getControlBaseFrame()));
   }

   public void resolveLinearMomentumRateControlModuleInput(LinearMomentumRateControlModuleInput in, LinearMomentumRateControlModuleInput out)
   {
      out.setOmega0(in.getOmega0());
      resolveFrameTuple2D(in.getDesiredCapturePoint(), out.getDesiredCapturePoint());
      resolveFrameTuple2D(in.getDesiredCapturePointVelocity(), out.getDesiredCapturePointVelocity());
      resolveFrameTuple2D(in.getPerfectCMP(), out.getPerfectCMP());
      resolveFrameTuple2D(in.getPerfectCoP(), out.getPerfectCoP());
      out.setControlHeightWithMomentum(in.getControlHeightWithMomentum());
      out.setDesiredCenterOfMassHeightAcceleration(in.getDesiredCoMHeightAcceleration());
      out.setSupportSide(in.getSupportSide());
      out.setTransferToSide(in.getTransferToSide());
      out.setInitializeForStanding(in.getInitializeForStanding());
      out.setInitializeForSingleSupport(in.getInitializeForSingleSupport());
      out.setInitializeForTransfer(in.getInitializeForTransfer());
      out.setKeepCoPInsideSupportPolygon(in.getKeepCoPInsideSupportPolygon());
      out.setMinimizeAngularMomentumRateZ(in.getMinimizeAngularMomentumRateZ());
      RecyclingArrayList<SimpleAdjustableFootstep> outFootsteps = out.getFootsteps();
      RecyclingArrayList<SimpleAdjustableFootstep> inFootsteps = in.getFootsteps();
      outFootsteps.clear();
      for (int i = 0; i < inFootsteps.size(); i++)
         resolveSimpleAdjustableFootstep(inFootsteps.get(i), outFootsteps.add());
      out.setSwingDurations(in.getSwingDurations());
      out.setTransferDurations(in.getTransferDurations());
      out.setFinalTransferDuration(in.getFinalTransferDuration());
      out.setRemainingTimeInSwingUnderDisturbance(in.getRemainingTimeInSwingUnderDisturbance());
      for (RobotSide robotSide : RobotSide.values)
         resolvePlaneContactStateCommand(in.getContactStateCommands().get(robotSide), out.getContactStateCommands().get(robotSide));
   }

   public void resolveLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput in, LinearMomentumRateControlModuleOutput out)
   {
      resolveFrameTuple2D(in.getDesiredCMP(), out.getDesiredCMP());
      resolveFrameTuple3D(in.getEffectiveICPAdjustment(), out.getEffectiveICPAdjustment());
      out.setUsingStepAdjustment(in.getUsingStepAdjustment());
      out.setFootstepWasAdjusted(in.getFootstepWasAdjusted());
      resolveFramePose3D(in.getFootstepSolution(), out.getFootstepSolution());
   }

   public void resolveSimpleAdjustableFootstep(SimpleAdjustableFootstep in, SimpleAdjustableFootstep out)
   {
      out.setIsAdjustable(in.getIsAdjustable());
      out.setRobotSide(in.getRobotSide());
      resolveFramePose3D(in.getSoleFramePose(), out.getSoleFramePose());
      out.setFoothold(in.getFoothold());
   }

   public void resolveHumanoidRobotContextJointData(HumanoidRobotContextJointData in, HumanoidRobotContextJointData out)
   {
      out.set(in);
   }

   public void resolveRobotMotionStatusHolder(RobotMotionStatusHolder in, RobotMotionStatusHolder out)
   {
      out.set(in);
   }

   public void resolveWrench(WrenchReadOnly in, WrenchBasics out)
   {
      out.setIncludingFrame(in);
      out.setReferenceFrame(resolveReferenceFrame(in.getReferenceFrame()));
      out.setBodyFrame(resolveReferenceFrame(in.getBodyFrame()));
   }

   public void resolveSelectionMatrix3D(SelectionMatrix3D in, SelectionMatrix3D out)
   {
      out.set(in);
      out.setSelectionFrame(resolveReferenceFrame(in.getSelectionFrame()));
   }

   public void resolveSelectionMatrix6D(SelectionMatrix6D in, SelectionMatrix6D out)
   {
      resolveSelectionMatrix3D(in.getAngularPart(), out.getAngularPart());
      resolveSelectionMatrix3D(in.getLinearPart(), out.getLinearPart());
   }

   public void resolveWeightMatrix3D(WeightMatrix3D in, WeightMatrix3D out)
   {
      out.set(in);
      out.setWeightFrame(resolveReferenceFrame(in.getWeightFrame()));
   }

   public void resolveWeightMatrix6D(WeightMatrix6D in, WeightMatrix6D out)
   {
      resolveWeightMatrix3D(in.getAngularPart(), out.getAngularPart());
      resolveWeightMatrix3D(in.getLinearPart(), out.getLinearPart());
   }

   public void resolveFrameTuple2D(FrameTuple2DReadOnly in, FrameTuple2DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   public void resolveFrameTuple3D(FrameTuple3DReadOnly in, FrameTuple3DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   public void resolveFrameQuaternion(FrameQuaternionReadOnly in, FrameQuaternionBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   public void resolveFramePose3D(FramePose3DReadOnly in, FramePose3DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   private ReferenceFrame resolveReferenceFrame(ReferenceFrame in)
   {
      if (in == null)
         return null;
      else
         return referenceFrameHashCodeResolver.getReferenceFrame(in.hashCode());
   }

   private <B extends RigidBodyReadOnly> B resolveRigidBody(B in)
   {
      if (in == null)
         return null;
      else
         return rigidBodyHashCodeResolver.castAndGetRigidBody(in.hashCode());
   }

   private <J extends JointReadOnly> J resolveJoint(J in)
   {
      if (in == null)
         return null;
      else
         return jointHashCodeResolver.castAndGetJoint(in.hashCode());
   }
}
