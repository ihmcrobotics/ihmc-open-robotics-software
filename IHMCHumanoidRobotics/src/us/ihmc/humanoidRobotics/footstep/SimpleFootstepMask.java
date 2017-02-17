package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.InclusionFunction;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleFootstepMask implements InclusionFunction<Point3D>
{
   private final static boolean DEBUG = false;
   private ReferenceFrame yawFrame2d;
   private double safetyBuffer;
   private FrameConvexPolygon2d footPolygon;

   public SimpleFootstepMask(ReferenceFrame yawFrame2D, ContactablePlaneBody foot, double footKernelMaskSafetyBuffer)
   {
      this.yawFrame2d = yawFrame2D;
      this.safetyBuffer = footKernelMaskSafetyBuffer;
      ArrayList<FramePoint2d> contactPoints = new ArrayList<FramePoint2d>();
      for (FramePoint2d point : foot.getContactPoints2d())
      {
         contactPoints.add(new FramePoint2d(yawFrame2D, inflate(point.getX()), inflate(point.getY())));
      }

      footPolygon = new FrameConvexPolygon2d(contactPoints);
      if (DEBUG)
         System.out.println("SimpleFootstepMask: footPolygon is \n" + footPolygon + " \nand yawFrame2d = \n"
                            + yawFrame2d.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
   }

   private double inflate(double x)
   {
      return x + Math.signum(x) * safetyBuffer;
   }

   public boolean isIncluded(Point3D testInput)
   {
      FramePoint test = new FramePoint(ReferenceFrame.getWorldFrame(),testInput);
      if (DEBUG)
         System.out.printf("SimpleFootstepMask: testing point input is <%.2f,%.2f> in frame %s", test.getX(), test.getY(), test.getReferenceFrame());
      test.changeFrame(yawFrame2d);
      boolean isPointInside = footPolygon.isPointInside(test.toFramePoint2d());
      if (DEBUG)
         System.out.printf("SimpleFootstepMask: testing point <%.2f,%.2f> in frame %s. Point is in is %s.  ", test.getX(), test.getY(), test.getReferenceFrame(),isPointInside ? "true":"false");

      return isPointInside;
   }

}
