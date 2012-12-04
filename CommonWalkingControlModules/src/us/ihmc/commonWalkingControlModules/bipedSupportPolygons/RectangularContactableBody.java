package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class RectangularContactableBody implements ContactablePlaneBody
{
   public static final int SIZE = 4;
   private final RigidBody rigidBody;
   private final ReferenceFrame soleFrame;
   private final List<Point2d> contactPoints = new ArrayList<Point2d>(SIZE);

   public RectangularContactableBody(RigidBody rigidBody, ReferenceFrame soleFrame, double forward, double back, double left, double right)
   {
      MathTools.checkIfInRange(forward, back, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(left, right, Double.POSITIVE_INFINITY);

      this.rigidBody = rigidBody;
      this.soleFrame = soleFrame;
      contactPoints.add(new Point2d(forward, left));
      contactPoints.add(new Point2d(back, left));
      contactPoints.add(new Point2d(back, right));
      contactPoints.add(new Point2d(forward, right));
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public List<FramePoint> getContactPoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (Point2d point : contactPoints)
      {
         ret.add(new FramePoint(soleFrame, point.getX(), point.getY(), 0.0));
      }

      return ret;
   }

   public ReferenceFrame getBodyFrame()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public FrameConvexPolygon2d getContactPolygon()
   {
      return new FrameConvexPolygon2d(soleFrame, contactPoints);
   }
}
