package us.ihmc.rdx.ui.vr;

import toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxModule;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;

import java.awt.*;

/**
 * Class responsible for streaming footstep placements based on VR tracker data.
 * It monitors the ankle tracker positions to predict and place footsteps as the user walks.
 */
public class RDXVRFootstepStreaming
{
   private final ROS2Helper ros2Helper;
   private final FootstepStreamingToolboxModule footstepStreamingToolbox;
   private final ROS2Input<FootstepStreamingToolboxOutputStatus> status;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXManualFootstepPlacement footstepPlacer;
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final Notification readyToStep = new Notification();
   private boolean wasEnabled = false;

   /**
    * Constructor for the footstep streaming class.
    *
    * @param syncedRobot the synchronized robot model
    * @param footstepPlacer the footstep placer for manual footstep placement
    */
   public RDXVRFootstepStreaming(ROS2SyncedRobotModel syncedRobot, ROS2Helper ros2Helper, RDXManualFootstepPlacement footstepPlacer)
   {
      this.syncedRobot = syncedRobot;
      this.footstepPlacer = footstepPlacer;
      this.ros2Helper = ros2Helper;

      footstepStreamingToolbox = new FootstepStreamingToolboxModule(syncedRobot.getRobotModel(), true);
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
         for (RobotSide side : RobotSide.values)
         {
            if (ankleTrackerFrames.get(side) != null)
            {
               FootstepStreamingToolboxInputMessage toolboxInputMessage = new FootstepStreamingToolboxInputMessage();
               toolboxInputMessage.setTimestamp();
               toolboxInputMessage.setSide(side.toByte());
               RigidBodyTransform currentRobotFootTransformInWorld = new RigidBodyTransform(syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToWorldFrame());
               toolboxInputMessage.getRobotFootPositionInWorld().set(currentRobotFootTransformInWorld.getTranslation());
               toolboxInputMessage.getRobotFootOrientationInWorld().set(currentRobotFootTransformInWorld.getRotation());

               RigidBodyTransform currentTrackerTransform = new RigidBodyTransform();
               ankleTrackerFrames.get(side).getTransformToWorldFrame().transform(currentTrackerTransform);
               toolboxInputMessage.getCurrentPositionInWorld().set(currentTrackerTransform.getTranslation());
               toolboxInputMessage.getCurrentOrientationInWorld().set(currentTrackerTransform.getRotation());
               toolboxInputMessage.setHasCurrentVelocity(true);

               ros2Helper.publish(FootstepStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), toolboxInputMessage);
            }
         }
      }
      else
      {
         footstepStreamingToolbox.sleep();
         wasEnabled = false;
      }
   }

   public void processToolboxOutput()
   {
      // Place and send footstep
      footstepPlacer.createNewFootstep(side);
      footstepPlacer.setFootstepPose(new FramePose3D(ReferenceFrame.getWorldFrame(), footstepTransformInWorld));
      if(footstepPlacer.checkAndPlaceFootstep())
      {
         footstepPlacer.exitPlacement();
         isUserStepping.put(side, true);
         readyToStep.clear();
         readyToStep.set();
      }
      else
      {
         footstepPlacer.exitPlacement();
      }
   }

   public Notification getReadyToStepNotification()
   {
      return readyToStep;
   }

   public void step()
   {
      footstepPlacer.walkFromSteps();
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

   public void reset()
   {
      for (RobotSide side : RobotSide.values())
      {
         ankleTrackerFrames.put(side, null);
      }
      footstepStreamingToolbox.sleep();
      wasEnabled = false;
   }
}
