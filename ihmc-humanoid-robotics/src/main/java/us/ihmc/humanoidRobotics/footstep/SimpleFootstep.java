package us.ihmc.humanoidRobotics.footstep;

import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleFootstep implements Settable<SimpleFootstep>
{
   private RobotSide robotSide;
   private final FramePose3D soleFramePose = new FramePose3D();
   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   public SimpleFootstep()
   {
   }

   public SimpleFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      this.robotSide = robotSide;
      this.soleFramePose.setIncludingFrame(soleFramePose);
      foothold.clear();
   }

   public SimpleFootstep(SimpleFootstep other)
   {
      set(other);
   }

   @Override
   public void set(SimpleFootstep other)
   {
      this.robotSide = other.robotSide;
      this.soleFramePose.setIncludingFrame(other.soleFramePose);
      this.foothold.set(other.foothold);
   }

   public void set(Footstep other)
   {
      setRobotSide(other.getRobotSide());
      setSoleFramePose(other.getFootstepPose());
      setFoothold(other.getPredictedContactPoints());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void getSoleFramePose(FramePose3D soleFramePoseToPack)
   {
      soleFramePoseToPack.setIncludingFrame(soleFramePose);
   }

   public FramePose3D getSoleFramePose()
   {
      return soleFramePose;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setSoleFramePose(FramePose3D soleFramePose)
   {
      this.soleFramePose.setIncludingFrame(soleFramePose);
   }

   public void setFoothold(ConvexPolygon2DReadOnly foothold)
   {
      this.foothold.set(foothold);
   }

   public void setFoothold(List<Point2D> contactPoints)
   {
      foothold.clear();
      if (contactPoints != null)
      {
         for (int i = 0; i < contactPoints.size(); i++)
         {
            foothold.addVertex(contactPoints.get(i));
         }
      }
      foothold.update();
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
         footholdToPack.set(foothold);
   }

   public ConvexPolygon2DReadOnly getFoothold()
   {
      return foothold;
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

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof SimpleFootstep)
      {
         SimpleFootstep other = (SimpleFootstep) obj;
         if (robotSide != other.robotSide)
            return false;
         if (!soleFramePose.equals(other.soleFramePose))
            return false;
         if (!foothold.equals(other.foothold))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String message = "Robot side = " + robotSide;
      message += "\nSole frame pose = " + soleFramePose;
      message += "\nFoothold = " + foothold;

      return message;
   }
}
