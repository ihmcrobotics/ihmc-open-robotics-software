package us.ihmc.footstepPlanning;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleFootstep
{
   private RobotSide robotSide;
   private final FramePose soleFramePose = new FramePose();

   public SimpleFootstep(RobotSide robotSide, FramePose soleFramePose)
   {
      this.robotSide = robotSide;
      this.soleFramePose.setIncludingFrame(soleFramePose);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void getSoleFramePose(FramePose soleFramePoseToPack)
   {
      soleFramePoseToPack.setIncludingFrame(soleFramePose);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setSoleFramePose(FramePose soleFramePose)
   {
      this.soleFramePose.setIncludingFrame(soleFramePose);
   }
}
