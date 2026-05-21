package us.ihmc.rdx.ui.vr;

import toolbox_msgs.FootstepStreamingToolboxInputMessage;
import toolbox_msgs.FootstepStreamingToolboxOutputStatus;
import toolbox_msgs.FootstepStreamingToolboxSideMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxModule;
import us.ihmc.behaviors.tools.walkingController.SwingFootTracker;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Class responsible for streaming footstep placements based on VR tracker data.
 * It monitors the ankle tracker positions to predict and place footsteps as the user steps.
 */
public class RDXVRFootstepStreamingProcessor
{
   private final ROS2Helper ros2Helper;
   private final FootstepStreamingToolboxModule footstepStreamingToolbox;
   private final ROS2Input<FootstepStreamingToolboxOutputStatus> status;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXVRFootstepPlacement footstepPlacer;
   private final SwingFootTracker swingFootTracker;
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final SideDependentList<SpatialVector> ankleTrackerVelocities = new SideDependentList<>();
   private final SideDependentList<Long> ankleTrackerTimestamps = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> previousAdjustment = new SideDependentList<>();
   private final Notification readyToStep = new Notification();
   private boolean wasEnabled = false;

   /**
    * Constructor for the footstep streaming class.
    *
    * @param syncedRobot the synchronized robot model
    * @param footstepPlacer the footstep placer for manual footstep placement
    */
   public RDXVRFootstepStreamingProcessor(ROS2SyncedRobotModel syncedRobot,
                                          ROS2Helper ros2Helper,
                                          RDXVRFootstepPlacement footstepPlacer,
                                          SwingFootTracker swingFootTracker,
                                          boolean enableYoVariableServer)
   {
      this.syncedRobot = syncedRobot;
      this.footstepPlacer = footstepPlacer;
      this.ros2Helper = ros2Helper;
      this.swingFootTracker = swingFootTracker;

      footstepStreamingToolbox = new FootstepStreamingToolboxModule(syncedRobot.getRobotModel(), enableYoVariableServer);
      status = ros2Helper.subscribe(FootstepStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));
   }

   public void processVRInput(boolean enabled)
   {
      if (enabled)
      {
         if (!wasEnabled)
         {
            footstepStreamingToolbox.wakeUp();
            wasEnabled = true;
         }
         FootstepStreamingToolboxInputMessage toolboxInputMessage = new FootstepStreamingToolboxInputMessage();
         toolboxInputMessage.setRobotStepDuration(footstepPlacer.getStepDuration());
         toolboxInputMessage.setRobotSwingDuration(footstepPlacer.getSwingDuration());
         toolboxInputMessage.setRobotStepElapsedTime(footstepPlacer.getTimeElapsedAfterStep());
         toolboxInputMessage.setRobotSwingSide(swingFootTracker.getSide().toByte());
         toolboxInputMessage.setIsRobotSwingFootLanding(swingFootTracker.isLanding());

         for (RobotSide side : RobotSide.values)
         {
            if (ankleTrackerFrames.get(side) != null)
            {
               FootstepStreamingToolboxSideMessage footstepMessage = new FootstepStreamingToolboxSideMessage();
               footstepMessage.setTimestamp(ankleTrackerTimestamps.get(side));
               footstepMessage.setSide(side.toByte());
               RigidBodyTransform currentRobotFootTransformInWorld = new RigidBodyTransform(syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToWorldFrame());
               footstepMessage.getRobotFootPositionInWorld().set(currentRobotFootTransformInWorld.getTranslation());
               footstepMessage.getRobotFootOrientationInWorld().set(currentRobotFootTransformInWorld.getRotation());

               RigidBodyTransform currentTrackerTransform = new RigidBodyTransform();
               ankleTrackerFrames.get(side).getTransformToWorldFrame().transform(currentTrackerTransform);
               footstepMessage.getCurrentPositionInWorld().set(currentTrackerTransform.getTranslation());
               footstepMessage.getCurrentOrientationInWorld().set(currentTrackerTransform.getRotation());

               footstepMessage.setHasCurrentVelocity(true);
               footstepMessage.getCurrentLinearVelocityInWorld().set(ankleTrackerVelocities.get(side).getLinearPart());
               footstepMessage.getCurrentAngularVelocityInWorld().set(ankleTrackerVelocities.get(side).getAngularPart());

               toolboxInputMessage.getSide().add().set(footstepMessage);
            }
         }
         if (toolboxInputMessage.getSide().size() == 2) // Do not publish if we have only the info of one side
            ros2Helper.publish(FootstepStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
      }
      else
      {
         if (wasEnabled)
            internalReset();
      }
   }

   public void processToolboxOutput()
   {
      if (status.getMessageNotification().poll())
      {
         FootstepStreamingToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         RobotSide side = RobotSide.fromByte(latestStatus.getRobotSide());
         if (side != null)
         {
            if (!latestStatus.getAdjustmentFootstep()) // First value estimate for a footstep
            {
               // Place and send footstep
               footstepPlacer.createNewFootstep(side);
               footstepPlacer.setFootstepPose(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                              latestStatus.getDesiredFootPosition().getPoint(),
                                                              latestStatus.getDesiredFootOrientation().getQuaternion()));
               // We can't trigger stepping here. We have to notify the KST and stop streaming
               readyToStep.clear();
               readyToStep.set();
            }
            else if (latestStatus.getAdjustmentFootstep() && !latestStatus.getLastAdjustment()) // Later values of updated estimate
            {
               if (footstepPlacer.setFootstepPose(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                  latestStatus.getDesiredFootPosition().getPoint(),
                                                                  latestStatus.getDesiredFootOrientation().getQuaternion())))
               {
                  step(true);
                  previousAdjustment.put(side, new RigidBodyTransform(latestStatus.getDesiredFootOrientation().getQuaternion(), latestStatus.getDesiredFootPosition().getPoint()));
               }
               else
               {
                  LogTools.error("Could not place step. Please do not release grip on controllers when streaming footsteps");
               }
            }
            else if (latestStatus.getLastAdjustment()) // Last estimate
            {
               LogTools.error("Received last estimate footstep");
               footstepPlacer.reset();
            }
         }
         else
         {
            LogTools.error("Received null footstep streaming output status");
         }
      }
   }

   public Notification getReadyToStepNotification()
   {
      return readyToStep;
   }

   public void step(boolean activeAdjustment)
   {
      footstepPlacer.sendStep(activeAdjustment);
   }

   /**
    * Sets the reference frame for the tracker of a given side.
    *
    * @param side the side (left or right) of the robot
    * @param trackerReferenceFrame the reference frame of the tracker
    */
   public void setTrackerReference(RobotSide side, ReferenceFrame trackerReferenceFrame)
   {
      ankleTrackerFrames.put(side, trackerReferenceFrame);
   }

   public void setTrackerVelocity(RobotSide side, SpatialVector velocity)
   {
      ankleTrackerVelocities.put(side, velocity);
   }

   public void setTrackerTimestamp(RobotSide side, long time)
   {
      ankleTrackerTimestamps.put(side, time);
   }

   public void reset()
   {
      for (RobotSide side : RobotSide.values())
      {
         ankleTrackerFrames.put(side, null);
      }
      internalReset();
   }

   private void internalReset()
   {
      footstepStreamingToolbox.sleep();
      wasEnabled = false;
      readyToStep.clear();
      footstepPlacer.reset();
      footstepPlacer.resetTimer();
   }

   public void destroy()
   {
      footstepStreamingToolbox.destroy();
   }
}