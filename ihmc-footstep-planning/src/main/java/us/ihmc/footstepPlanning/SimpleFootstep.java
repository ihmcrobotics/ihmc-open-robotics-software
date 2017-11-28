package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleFootstep
{
   private RobotSide robotSide;
   private final FramePose soleFramePose = new FramePose();
   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   public SimpleFootstep(RobotSide robotSide, FramePose soleFramePose)
   {
      this.robotSide = robotSide;
      this.soleFramePose.setIncludingFrame(soleFramePose);
      foothold.clear();
   }

   public void set(SimpleFootstep other)
   {
      this.robotSide = other.robotSide;
      this.soleFramePose.setIncludingFrame(other.soleFramePose);
      this.foothold.setAndUpdate(other.foothold);
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

   public void setFoothold(ConvexPolygon2D foothold)
   {
      this.foothold.setAndUpdate(foothold);
   }

   public boolean hasFoothold()
   {
      if (this.foothold.isEmpty())
         return false;
      return true;
   }

   public void getFoothold(ConvexPolygon2D footholdToPack)
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
