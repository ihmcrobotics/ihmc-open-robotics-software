package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WholeBodyBimanipulationActionExecutor extends ActionNodeExecutor<WholeBodyBimanipulationActionState, WholeBodyBimanipulationActionDefinition>
{
   private final WholeBodyBimanipulationActionState state;
   private final WholeBodyBimanipulationActionDefinition definition;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();

   private final HumanoidKinematicsSolver wholeBodyIKSolver;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final OneDoFJointBasics[] desiredOneDoFJointsExcludingHands;
   private final SideDependentList<KinematicsToolboxRigidBodyCommand> handRigidBodyCommands = new SideDependentList<>();
   private volatile boolean isSolutionGood = false;

   public WholeBodyBimanipulationActionExecutor(long id,
                                                CRDTInfo crdtInfo,
                                                WorkspaceResourceDirectory saveFileDirectory,
                                                ROS2ControllerHelper ros2ControllerHelper,
                                                ReferenceFrameLibrary referenceFrameLibrary,
                                                DRCRobotModel robotModel,
                                                ROS2SyncedRobotModel syncedRobot)
   {
      super(new WholeBodyBimanipulationActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;

      this.desiredFullRobotModel = robotModel.createFullRobotModel();
      desiredOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      wholeBodyIKSolver = new HumanoidKinematicsSolver(robotModel, yoGraphicsListRegistry, new YoRegistry(getClass().getSimpleName()));

      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getJointMap().getHandName(side) != null) // Handle one handed configurations
         {
            KinematicsToolboxRigidBodyCommand rigidBodyCommand = new KinematicsToolboxRigidBodyCommand();
            rigidBodyCommand.setEndEffector(wholeBodyIKSolver.getDesiredFullRobotModel().getHand(side));
            rigidBodyCommand.getControlFramePose().setToZero(wholeBodyIKSolver.getDesiredFullRobotModel().getHandControlFrame(side));
            rigidBodyCommand.getControlFramePose().changeFrame(wholeBodyIKSolver.getDesiredFullRobotModel().getHand(side).getBodyFixedFrame());
            // TODO: Use default values from somewhere else
            rigidBodyCommand.getWeightMatrix().setLinearWeights(5.0, 5.0, 5.0);
            rigidBodyCommand.getWeightMatrix().setAngularWeights(1.0, 1.0, 5.0);
            rigidBodyCommand.getSelectionMatrix().getAngularPart().setAxisSelection(false, false, true);
            handRigidBodyCommands.put(side, rigidBodyCommand);
         }
      }
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (state.getHandFrame(RobotSide.LEFT).isChildOfWorld())
      {
         wholeBodyIKSolver.setInitialConfiguration(syncedRobot.getLatestRobotConfigurationData());
//         wholeBodyIKSolver.setCapturabilityBasedStatus(controllerStatusTracker.getLatestCapturabilityBasedStatus());
         wholeBodyIKSolver.initialize();

         for (RobotSide side : handRigidBodyCommands.sides())
         {
            KinematicsToolboxRigidBodyCommand rigidBodyCommand = handRigidBodyCommands.get(side);
            rigidBodyCommand.getDesiredPose().setFromReferenceFrame(state.getHandFrame(side).getReferenceFrame());
            wholeBodyIKSolver.submit(rigidBodyCommand);
            wholeBodyIKSolver.setAsDoubleSupport();
         }

         // We solve on a thread because the solver can take some milliseconds
         MissingThreadTools.startAThread(getClass().getSimpleName(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
         {
            try
            {
               isSolutionGood = wholeBodyIKSolver.solve();
            }
            finally
            {
               desiredFullRobotModel.getRootJoint()
                           .setJointConfiguration(wholeBodyIKSolver.getSolution().getDesiredRootOrientation(),
                                                  wholeBodyIKSolver.getSolution().getDesiredRootPosition());
               MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(wholeBodyIKSolver.getDesiredOneDoFJoints(), desiredOneDoFJointsExcludingHands);
               desiredFullRobotModel.updateFrames();
               //               state.setJointAngles(Arrays.stream(desiredOneDoFJointsExcludingHands).toArray());
               state.setSolutionQuality(wholeBodyIKSolver.getSolution().getSolutionQuality());

               WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

               populateArmTrajectoryMessage(wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage(), RobotSide.LEFT);
               populateArmTrajectoryMessage(wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage(), RobotSide.RIGHT);

               populateLegTrajectoryMessage(wholeBodyTrajectoryMessage.getLeftLegTrajectoryMessage(), RobotSide.LEFT);
               populateLegTrajectoryMessage(wholeBodyTrajectoryMessage.getRightLegTrajectoryMessage(), RobotSide.RIGHT);

               SpineTrajectoryMessage spineTrajectoryMessage = wholeBodyTrajectoryMessage.getSpineTrajectoryMessage();
               spineTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
               IDLSequence.Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = spineTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
               for (SpineJointName spineJointName : robotModel.getJointMap().getSpineJointNames())
               {
                  OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
                  oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
                  oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

                  TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
                  trajectoryPoint1DMessage.setTime(definition.getTrajectoryDuration());
                  trajectoryPoint1DMessage.setPosition(desiredFullRobotModel.getSpineJoint(spineJointName).getQ());
                  trajectoryPoint1DMessage.setVelocity(0.0);
               }

               long trajectoryReferenceFrameID = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
               FramePose3D desiredPelvisPose = new FramePose3D(desiredFullRobotModel.getPelvis().getBodyFixedFrame());
               desiredPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

               PelvisTrajectoryMessage pelvisTrajectoryMessage = wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage();
               SE3TrajectoryMessage se3TrajectoryMessage = pelvisTrajectoryMessage.getSe3Trajectory();
               se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
               // Select all axes and use default weights
               SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
               se3TrajectoryPointMessage.setTime(definition.getTrajectoryDuration());
               se3TrajectoryPointMessage.getPosition().set(desiredPelvisPose.getPosition());
               se3TrajectoryPointMessage.getOrientation().set(desiredPelvisPose.getOrientation());
               se3TrajectoryPointMessage.getLinearVelocity().setToZero();
               se3TrajectoryPointMessage.getAngularVelocity().setToZero();
               se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);

               LogTools.info("Publishing Wholebody Bimanipulation trajectory");
               ros2ControllerHelper.publishToController(wholeBodyTrajectoryMessage);
            }
         });

         trackingCalculator.reset();
         state.setNominalExecutionDuration(definition.getTrajectoryDuration());
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getTimeIsUp())
      {
         state.setIsExecuting(false);
      }
   }

   private void populateArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage, RobotSide side)
   {
      armTrajectoryMessage.setRobotSide(side.toByte());
      armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      IDLSequence.Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (ArmJointName armJointName : robotModel.getJointMap().getArmJointNames(side))
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
         oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(definition.getTrajectoryDuration());
         trajectoryPoint1DMessage.setPosition(desiredFullRobotModel.getArmJoint(side, armJointName).getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }
   }

   private void populateLegTrajectoryMessage(LegTrajectoryMessage legTrajectoryMessage, RobotSide side)
   {
      legTrajectoryMessage.setRobotSide(side.toByte());
      legTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      IDLSequence.Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = legTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (LegJointName legJointName : robotModel.getJointMap().getLegJointNames())
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
         oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(definition.getTrajectoryDuration());
         trajectoryPoint1DMessage.setPosition(desiredFullRobotModel.getLegJoint(side, legJointName).getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }
   }
}
