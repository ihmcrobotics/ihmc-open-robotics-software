package us.ihmc.humanoidRobotics.footstep;

public class SimpleAdjustableFootstep extends SimpleFootstep
{
   private boolean isAdjustable;

   public void setIsAdjustable(boolean isAdjustable)
   {
      this.isAdjustable = isAdjustable;
   }

   public boolean getIsAdjustable()
   {
      return isAdjustable;
   }

   public void set(SimpleAdjustableFootstep other)
   {
      isAdjustable = other.isAdjustable;
      super.set(other);
   }

   public void set(Footstep other)
   {
      setIsAdjustable(other.getIsAdjustable());
      setRobotSide(other.getRobotSide());
      setSoleFramePose(other.getFootstepPose());
      setFoothold(other.getPredictedContactPoints());
   }
}
