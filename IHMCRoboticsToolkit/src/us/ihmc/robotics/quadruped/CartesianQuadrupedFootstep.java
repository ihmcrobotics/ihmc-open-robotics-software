package us.ihmc.robotics.quadruped;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Serializable;
import java.util.StringTokenizer;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;
@SuppressWarnings("all")
/**
 * CartesianPositionFootstep is a general utility class to represent the position
 * of a foot. It contains the position and the leg name to
 * specify the foot based on static constants in LittleDogRobot.
 *
 * CartesianPositionFootsteps are immutable. Nothing should return internal data that is immutable, but instead copies of it.
 *
 * @author Learning Locomotion Team
 */
public class CartesianQuadrupedFootstep implements Footstep, Serializable
{
   private final FramePoint footPosition;
   private final RobotQuadrant legName;

   public CartesianQuadrupedFootstep(CartesianQuadrupedFootstep footstep)
   {
      this.footPosition = new FramePoint(footstep.footPosition);
      this.legName = footstep.legName;
   }

   public CartesianQuadrupedFootstep(FramePoint position, RobotQuadrant legName)
   {
      this.footPosition = new FramePoint(position);
      this.legName = legName;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return footPosition.getReferenceFrame();
   }

   public FramePoint getPositionFramePointCopy()
   {
      return new FramePoint(footPosition);
   }

   public double getX()
   {
      return footPosition.getX();
   }

   public double getY()
   {
      return footPosition.getY();
   }

   public double getZ()
   {
      return footPosition.getZ();
   }

   public RobotQuadrant getLegName()
   {
      return this.legName;
   }

   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      this.footPosition.checkReferenceFrameMatch(frame);
   }

   /**
    * toString
    *
    * @return String Represents "id:(x,y,z)"
    */
   public String toString()
   {
      return legName + ":" + footPosition.toString();
   }

   public boolean equals(Object object)
   {
      return (this == object);
   }

   public boolean isEpsilonEqualTo(Footstep footstep, double epsilon)
   {
      return (footPosition.epsilonEquals(footstep.getPositionFramePointCopy(), epsilon) && (legName == footstep.getLegName()));
   }

   /**
    * changeFrameCopy
    *
    * @param desiredFrame Frame
    * @return Footstep
    */
   public CartesianQuadrupedFootstep changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePoint newPosition = new FramePoint(footPosition);
      newPosition.changeFrame(desiredFrame);

      return new CartesianQuadrupedFootstep(newPosition, legName);
   }

   public CartesianQuadrupedFootstep yawAboutPointCopy(FramePoint pointToYawAbout, double yaw)
   {
      return new CartesianQuadrupedFootstep(this.footPosition.yawAboutPoint(pointToYawAbout, yaw), this.legName);
   }

   public CartesianQuadrupedFootstep applyTransformCopy(RigidBodyTransform transform3D)
   {
      FramePoint temp = new FramePoint(footPosition);
      temp.applyTransform(transform3D);
      return new CartesianQuadrupedFootstep(temp, legName);
   }

   public double distanceToFootstep(Footstep footstepToCheck)
   {
      return this.footPosition.distance(footstepToCheck.getPositionFramePointCopy());
   }

   public double distanceToFootstepInXY(Footstep footstepToCheck)
   {
      Point2d position1 = new Point2d(this.footPosition.getX(), this.footPosition.getY());
      Point2d position2 = new Point2d(footstepToCheck.getX(), footstepToCheck.getY());

      return position1.distance(position2);
   }


   public double distanceSquaredToFootstep(Footstep footstepToCheck)
   {
      return this.footPosition.distanceSquared(footstepToCheck.getPositionFramePointCopy());
   }

   public Footstep morphCopy(Footstep footstep, double alpha)
   {
      return morph(this, (CartesianQuadrupedFootstep) footstep, alpha);
   }


   public static CartesianQuadrupedFootstep morph(CartesianQuadrupedFootstep footstep1, CartesianQuadrupedFootstep footstep2, double alpha)
   {
      RobotQuadrant legName = footstep1.getLegName();
      if (legName != footstep2.getLegName())
      {
         throw new RuntimeException("CartesianPositionFootstep.morph(). Footsteps must have same leg name!");
      }

      footstep1.checkReferenceFrameMatch(footstep2.getReferenceFrame());

      FramePoint framePoint1 = footstep1.getPositionFramePointCopy();
      FramePoint framePoint2 = footstep2.getPositionFramePointCopy();

      framePoint1.scale(1.0 - alpha);
      framePoint2.scale(alpha);
      framePoint1.add(framePoint2);

      return new CartesianQuadrupedFootstep(framePoint1, legName);
   }

   /**
    * load
    *
    * reads footstep from file
    * @todo read in the Frame also
    * @param bufferedReader BufferedReader
    * @return Footstep
    */
   public static CartesianQuadrupedFootstep load(BufferedReader bufferedReader, ReferenceFrame referenceFrame)
   {
      try
      {
         StringTokenizer s = new StringTokenizer(bufferedReader.readLine(), " ");

         // we want to skip the first token because it is the name of the foot step type
         String footstepTpye = s.nextToken();

         FramePoint position = new FramePoint(referenceFrame, Double.parseDouble(s.nextToken()), Double.parseDouble(s.nextToken()),
                                  Double.parseDouble(s.nextToken()));
         String legNameString = s.nextToken();
         RobotQuadrant legName;

         if (legNameString.startsWith("null"))
            legName = null;
         else
            legName = RobotQuadrant.getQuadrantName(legNameString);

         return new CartesianQuadrupedFootstep(position, legName);
      }
      catch (IOException ex)
      {
         System.err.println(ex);
      }

      return null;
   }

   /**
    * save
    *
    * write footstep to file
    * @todo Save out the frame also
    * @param printWriter PrintWriter
    */
   public void save(PrintWriter printWriter)
   {
      printWriter.println(footPosition.getX() + " " + footPosition.getY() + " " + footPosition.getZ() + " " + legName);
   }

   // Serializable support to handle the fact that wedoobject matching for reference frame matching
   public Object readResolve()
   {
      CartesianQuadrupedFootstep footstep = this;
      if (this.getReferenceFrame().getName().equals(ReferenceFrame.getWorldFrame().getName()))
      {
         FramePoint position = new FramePoint(ReferenceFrame.getWorldFrame(), this.getX(), this.getY(), this.getZ());
         footstep = new CartesianQuadrupedFootstep(position, this.legName);
      }
      else
      {
         System.err.println("WARNING: can not serialize reference frameacross VMs: " + this.getReferenceFrame());
      }


      return footstep;
   }

}
