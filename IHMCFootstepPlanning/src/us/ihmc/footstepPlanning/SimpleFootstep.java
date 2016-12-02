package us.ihmc.footstepPlanning;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleFootstep
{
   private RobotSide robotSide;
   private final FramePose soleFramePose = new FramePose();
   private final ConvexPolygon2d foothold = new ConvexPolygon2d();

   public SimpleFootstep(RobotSide robotSide, FramePose soleFramePose)
   {
      this.robotSide = robotSide;
      this.soleFramePose.setIncludingFrame(soleFramePose);
      foothold.clear();
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

   public void setFoothold(ConvexPolygon2d foothold)
   {
      this.foothold.setAndUpdate(foothold);
   }

   public boolean hasFoothold()
   {
      if (this.foothold.isEmpty())
         return false;
      return true;
   }

   public void getFoothold(ConvexPolygon2d footholdToPack)
   {
      if (!hasFoothold())
         footholdToPack.setToNaN();
      else
         footholdToPack.setAndUpdate(foothold);
   }

   public boolean epsilonEquals(SimpleFootstep otherFootstep, double epsilon)
   {
      this.foothold.update();
      otherFootstep.foothold.update();

      if (!this.robotSide.equals(otherFootstep.robotSide))
         return false;
      if (!this.soleFramePose.epsilonEquals(otherFootstep.soleFramePose, epsilon))
         return false;
      if (!this.foothold.epsilonEquals(otherFootstep.foothold, epsilon))
         return false;
      return true;
   }
}
