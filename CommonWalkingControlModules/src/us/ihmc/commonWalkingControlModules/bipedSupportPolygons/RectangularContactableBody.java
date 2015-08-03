package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;


import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RectangularContactableBody extends ListOfPointsContactablePlaneBody
{
   public RectangularContactableBody(RigidBody rigidBody, ReferenceFrame soleFrame, double forward, double back, double left, double right)
   {
      super(rigidBody, soleFrame, createContactPoints(forward, back, left, right));
      MathTools.checkIfInRange(forward, back, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(left, right, Double.POSITIVE_INFINITY);
   }

   private static List<Point2d> createContactPoints(double forward, double back, double left, double right)
   {
      ArrayList<Point2d> contactPoints = new ArrayList<Point2d>();
      contactPoints.add(new Point2d(forward, left));
      contactPoints.add(new Point2d(back, left));
      contactPoints.add(new Point2d(back, right));
      contactPoints.add(new Point2d(forward, right));
      return contactPoints;
   }
}
