package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.robotics.robotSide.RobotSide;

public class HeightMapBasedFootstepAdjustment implements FootstepAdjustment
{
   private final YawPitchRoll yawPitchRoll = new YawPitchRoll();
   private final HeightMap heightMap;

   public HeightMapBasedFootstepAdjustment(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   @Override
   public boolean adjustFootstep(FramePose3DReadOnly supportFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedPose)
   {
      adjustedPose.getLocation().set(footstepPose.getPosition());
      adjustedPose.getLocation().setZ(heightMap.heightAt(footstepPose.getX(), footstepPose.getY(), 0.0));

      yawPitchRoll.set(supportFootPose.getOrientation());
      yawPitchRoll.setYaw(footstepPose.getYaw());
      adjustedPose.getOrientation().set(yawPitchRoll);
      return true;
   }
}