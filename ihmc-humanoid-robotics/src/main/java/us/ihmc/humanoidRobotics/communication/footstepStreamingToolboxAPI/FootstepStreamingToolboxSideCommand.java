package us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI;

import toolbox_msgs.FootstepStreamingToolboxSideMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepStreamingToolboxSideCommand implements Command<FootstepStreamingToolboxSideCommand, FootstepStreamingToolboxSideMessage>
{
   private long sequenceId;
   private long timestamp;
   private RobotSide side;
   private final FramePose3D robotFootPose = new FramePose3D();
   private final FramePose3D currentPose = new FramePose3D();
   private boolean hasCurrentVelocity;
   private final SpatialVector currentVelocity = new SpatialVector();

   @Override
   public void clear()
   {
      sequenceId = 0;
      timestamp = 0;
      hasCurrentVelocity = false;
      robotFootPose.setToNaN(ReferenceFrame.getWorldFrame());
      currentPose.setToNaN(ReferenceFrame.getWorldFrame());
      currentVelocity.setToNaN(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void set(FootstepStreamingToolboxSideCommand other)
   {
      sequenceId = other.sequenceId;
      timestamp = other.timestamp;
      side = other.side;
      robotFootPose.setIncludingFrame(other.robotFootPose);
      currentPose.setIncludingFrame(other.currentPose);
      hasCurrentVelocity = other.hasCurrentVelocity;
      currentVelocity.setIncludingFrame(other.currentVelocity);
   }

   @Override
   public void setFromMessage(FootstepStreamingToolboxSideMessage message)
   {
      sequenceId = message.getSequenceId();
      timestamp = message.getTimestamp();
      side = RobotSide.fromByte(message.getSide());
      robotFootPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getRobotFootPositionInWorld().getPoint(), message.getRobotFootOrientationInWorld().getQuaternion());
      currentPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getCurrentPositionInWorld().getPoint(), message.getCurrentOrientationInWorld().getQuaternion());
      hasCurrentVelocity = message.getHasCurrentVelocity();
      currentVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getCurrentAngularVelocityInWorld().getVector(), message.getCurrentLinearVelocityInWorld().getVector());
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   public void setHasCurrentVelocity(boolean hasCurrentVelocity)
   {
      this.hasCurrentVelocity = hasCurrentVelocity;
   }

   public FramePose3D getCurrentPose()
   {
      return currentPose;
   }

   public SpatialVector getCurrentVelocity()
   {
      return currentVelocity;
   }

   public FramePose3D getRobotFootPose()
   {
      return robotFootPose;
   }

   public boolean getHasCurrentVelocity()
   {
      return hasCurrentVelocity;
   }

   @Override
   public Class<FootstepStreamingToolboxSideMessage> getMessageClass()
   {
      return FootstepStreamingToolboxSideMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (currentPose.containsNaN())
         return false;
      if (hasCurrentVelocity && this.currentVelocity.containsNaN())
         return false;
      if (robotFootPose.containsNaN())
         return false;

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
