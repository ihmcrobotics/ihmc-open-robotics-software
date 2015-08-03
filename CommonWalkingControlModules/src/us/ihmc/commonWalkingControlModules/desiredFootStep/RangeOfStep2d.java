package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.geom.Ellipse2D;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RangeOfStep2d extends Ellipse2D.Double
{
   private static final boolean DEBUG = false;
   private static final long serialVersionUID = -2194197143315735789L;
   private static final RigidBodyTransform IDENTITY = new RigidBodyTransform();

   private final RobotSide robotSide;
   private final RigidBody rigidBody;
   private ReferenceFrame referenceFrame;


   public RangeOfStep2d(RigidBody rigidBody, RobotSide robotSide, double forwardLength, double sideLength, double offset)
   {
      super(-0.5 * sideLength, -0.5 * forwardLength + robotSide.negateIfRightSide(offset), forwardLength, sideLength);

      this.robotSide = robotSide;
      this.rigidBody = rigidBody;
      setReferenceFrame(rigidBody.getBodyFixedFrame());

      if (DEBUG)
      {
         System.out.println("RangeOfStep2d: Coords of ellipse center: " + this.getCenterX() + ", " + this.getCenterY());
         System.out.println(rigidBody.getBodyFixedFrame());
      }
   }

   public boolean contains(FramePoint point)
   {
      updateReferenceFrame(rigidBody.getBodyFixedFrame());
      if (DEBUG)
         System.out.println("RangeOfStep2d: Before frame change: " + point);

      point.changeFrame(referenceFrame);
      if (DEBUG)
         System.out.println("RangeOfStep2d: After frame change: " + point);

      if ((robotSide == RobotSide.LEFT) && (point.getY() < getCenterY()))
         return false;

      if ((robotSide == RobotSide.RIGHT) && (point.getY() > getCenterY()))
         return false;

      return contains(point.getX(), point.getY());
   }

   private void updateReferenceFrame(ReferenceFrame referenceFrame)
   {
      RigidBodyTransform transform = this.referenceFrame.getTransformToDesiredFrame(referenceFrame);

      if (!transform.equals(IDENTITY))
         setReferenceFrame(referenceFrame);
   }

   private void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      RigidBodyTransform transform = referenceFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      this.referenceFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("translation", ReferenceFrame.getWorldFrame(), transform);
   }
}
