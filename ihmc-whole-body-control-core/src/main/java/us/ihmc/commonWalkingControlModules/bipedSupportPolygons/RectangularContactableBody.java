package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.commons.MathTools;

public class RectangularContactableBody extends ListOfPointsContactablePlaneBody
{
   public RectangularContactableBody(RigidBodyBasics rigidBody, ReferenceFrame soleFrame, double forward, double back, double left, double right)
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
