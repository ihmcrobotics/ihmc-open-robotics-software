package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RectangularContactableBody extends ListOfPointsContactablePlaneBody
{
   public RectangularContactableBody(RigidBody rigidBody, ReferenceFrame soleFrame, double forward, double back, double left, double right)
   {
      super(rigidBody, soleFrame, createContactPoints(forward, back, left, right));
      MathTools.checkIntervalContains(forward, back, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(left, right, Double.POSITIVE_INFINITY);
   }

   private static List<Point2D> createContactPoints(double forward, double back, double left, double right)
   {
      ArrayList<Point2D> contactPoints = new ArrayList<Point2D>();
      contactPoints.add(new Point2D(forward, left));
      contactPoints.add(new Point2D(back, left));
      contactPoints.add(new Point2D(back, right));
      contactPoints.add(new Point2D(forward, right));
      return contactPoints;
   }
}
