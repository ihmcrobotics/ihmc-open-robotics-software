package us.ihmc.communication.packets.walking;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class CapturabilityBasedStatus extends Packet<CapturabilityBasedStatus>
{
   public static final int MAXIMUM_NUMBER_OF_VERTICES = 8;
   
   public Point2d capturePoint = new Point2d();
   public Point2d desiredCapturePoint = new Point2d();

   public Point3d centerOfMass = new Point3d();
   
   public int leftFootSupportPolygonLength;
   public Point2d[] leftFootSupportPolygonStore = new Point2d[MAXIMUM_NUMBER_OF_VERTICES];
   public int rightFootSupportPolygonLength;
   public Point2d[] rightFootSupportPolygonStore = new Point2d[MAXIMUM_NUMBER_OF_VERTICES];

   public CapturabilityBasedStatus(Random random)
   {
      double max = Double.MAX_VALUE / 2;
      capturePoint = RandomTools.generateRandomPoint2d(random, max, max);
      desiredCapturePoint = RandomTools.generateRandomPoint2d(random, max, max);
      centerOfMass = RandomTools.generateRandomPoint(random, max, max, max);
      
      leftFootSupportPolygonLength = Math.abs(random.nextInt(MAXIMUM_NUMBER_OF_VERTICES));
      for (int i = 0; i < leftFootSupportPolygonLength; i++)
      {
         leftFootSupportPolygonStore[i] = RandomTools.generateRandomPoint2d(random, max, max);
      }
      rightFootSupportPolygonLength = Math.abs(random.nextInt(MAXIMUM_NUMBER_OF_VERTICES));
      for (int i = 0; i < rightFootSupportPolygonLength; i++)
      {
         rightFootSupportPolygonStore[i] = RandomTools.generateRandomPoint2d(random, max, max);
      }
   }

   public CapturabilityBasedStatus()
   {
      // Empty constructor for serialization
      for(int i = 0; i < MAXIMUM_NUMBER_OF_VERTICES; i++)
      {
         leftFootSupportPolygonStore[i] = new Point2d();
         rightFootSupportPolygonStore[i] = new Point2d();
         
      }
   }
   
   public void setSupportPolygon(RobotSide robotSide, FrameConvexPolygon2d footPolygon)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();
      
      if(numberOfVertices > MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = MAXIMUM_NUMBER_OF_VERTICES;
      }
      
      if(robotSide == RobotSide.LEFT)
      {
         leftFootSupportPolygonLength = numberOfVertices;
      }
      else
      {
         rightFootSupportPolygonLength = numberOfVertices;
      }
      
      for(int i = 0; i < numberOfVertices; i++)
      {
         if(robotSide == RobotSide.LEFT)
         {
            footPolygon.getFrameVertex(i).get(leftFootSupportPolygonStore[i]);
         }
         else
         {
            footPolygon.getFrameVertex(i).get(rightFootSupportPolygonStore[i]);
         }
      }
   }


   public FramePoint2d getCapturePoint()
   {
      return new FramePoint2d(ReferenceFrame.getWorldFrame(), capturePoint);
   }

   public FramePoint2d getDesiredCapturePoint()
   {
      return new FramePoint2d(ReferenceFrame.getWorldFrame(), desiredCapturePoint);
   }

   public FrameConvexPolygon2d getFootSupportPolygon(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && leftFootSupportPolygonLength != 0)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), Arrays.copyOf(leftFootSupportPolygonStore, leftFootSupportPolygonLength));
      else if (rightFootSupportPolygonStore != null)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), Arrays.copyOf(rightFootSupportPolygonStore, rightFootSupportPolygonLength));
      else
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame());
   }

   public boolean isInDoubleSupport()
   {
      return leftFootSupportPolygonLength != 0 & rightFootSupportPolygonLength != 0;
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {

      boolean ret = this.capturePoint.epsilonEquals(other.capturePoint, epsilon);
      ret &= this.desiredCapturePoint.epsilonEquals(other.desiredCapturePoint, epsilon);

      ret &= this.centerOfMass.epsilonEquals(other.centerOfMass, epsilon);

      ret &= this.leftFootSupportPolygonLength == other.leftFootSupportPolygonLength;
      ret &= this.rightFootSupportPolygonLength == other.rightFootSupportPolygonLength;
      
      if (leftFootSupportPolygonStore == null || other.leftFootSupportPolygonStore == null)
      {
         ret &= leftFootSupportPolygonStore == null && other.leftFootSupportPolygonStore == null;
      }
      else
      {
         for (int i = 0; i < leftFootSupportPolygonStore.length; i++)
         {
            ret &= this.leftFootSupportPolygonStore[i].epsilonEquals(other.leftFootSupportPolygonStore[i], epsilon);
         }
      }

      if (rightFootSupportPolygonStore == null || other.rightFootSupportPolygonStore == null)
      {
         ret &= rightFootSupportPolygonStore == null && other.rightFootSupportPolygonStore == null;
      }
      else
      {
         for (int i = 0; i < rightFootSupportPolygonStore.length; i++)
         {
            ret &= this.rightFootSupportPolygonStore[i].epsilonEquals(other.rightFootSupportPolygonStore[i], epsilon);
         }
      }

      return ret;
   }
}
