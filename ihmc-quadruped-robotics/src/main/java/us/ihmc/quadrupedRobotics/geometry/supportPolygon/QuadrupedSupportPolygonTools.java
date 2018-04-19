package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class QuadrupedSupportPolygonTools
{
   /**
    *  If the two polygons differ only in that one footstep has moved, return that quadrant.
    *
    * @param next polygon
    * @return quadrant that has moved
    */
   public static RobotQuadrant getWhichFootstepHasMoved(QuadrupedSupportPolygon startingPolygon, QuadrupedSupportPolygon nextPolygon)
   {
      if (!startingPolygon.containsSameQuadrants(nextPolygon))
      {
         throw new IllegalArgumentException("Polygons contain different quadrants");
      }

      RobotQuadrant swingLeg = null;

      for (RobotQuadrant robotQuadrant : startingPolygon.getSupportingQuadrantsInOrder())
      {
         if (!startingPolygon.getFootstep(robotQuadrant).epsilonEquals(nextPolygon.getFootstep(robotQuadrant), 1e-5))
         {
            if (swingLeg == null)
            {
               swingLeg = robotQuadrant;
            }
            else // make sure only one foot differs
            {
               throw new IllegalArgumentException("More than one foot differs");
            }
         }
      }

      if (swingLeg == null)
      {
         throw new IllegalArgumentException("No feet were different");
      }

      return swingLeg;
   }


   /**
    * Returns true if any of the legs are crossing.
    * By this what we mean is that as you trace out a path from FL to FR to HR to HL,
    * considering only the x,y coordinates, that the path will only take right turns
    * and no left turns.
    * @return true if any of the legs are crossing. Otherwise false.
    */
   public static boolean areLegsCrossing(QuadrupedSupportPolygon polygon)
   {
      // JEP: This is hardcoded like this instead of using points and vectors because it needs
      // to be really fast I think. I could be wrong though.

      double xFL, yFL, xFR, yFR, xHR, yHR, xHL, yHL;
      if (polygon.getNumberOfVertices() == 4)
      {
         xFL = polygon.getFootstep(RobotQuadrant.FRONT_LEFT).getX();
         yFL = polygon.getFootstep(RobotQuadrant.FRONT_LEFT).getY();

         xFR = polygon.getFootstep(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = polygon.getFootstep(RobotQuadrant.FRONT_RIGHT).getY();

         xHR = polygon.getFootstep(RobotQuadrant.HIND_RIGHT).getX();
         yHR = polygon.getFootstep(RobotQuadrant.HIND_RIGHT).getY();

         xHL = polygon.getFootstep(RobotQuadrant.HIND_LEFT).getX();
         yHL = polygon.getFootstep(RobotQuadrant.HIND_LEFT).getY();
      }
      else
      {
         xFL = (polygon.getFootstep(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.FRONT_LEFT).getX();
         yFL = (polygon.getFootstep(RobotQuadrant.FRONT_LEFT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.FRONT_LEFT).getY();

         xFR = (polygon.getFootstep(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.FRONT_RIGHT).getX();
         yFR = (polygon.getFootstep(RobotQuadrant.FRONT_RIGHT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.FRONT_RIGHT).getY();

         xHR = (polygon.getFootstep(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.HIND_RIGHT).getX();
         yHR = (polygon.getFootstep(RobotQuadrant.HIND_RIGHT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.HIND_RIGHT).getY();

         xHL = (polygon.getFootstep(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.HIND_LEFT).getX();
         yHL = (polygon.getFootstep(RobotQuadrant.HIND_LEFT) == null) ? Double.NaN : polygon.getFootstep(RobotQuadrant.HIND_LEFT).getY();
      }

      double xFLtoFR = xFR - xFL;
      double yFLtoFR = yFR - yFL;

      double xFRtoHR = xHR - xFR;
      double yFRtoHR = yHR - yFR;

      double xHRtoHL = xHL - xHR;
      double yHRtoHL = yHL - yHR;

      double xHLtoFL = xFL - xHL;
      double yHLtoFL = yFL - yHL;

      if (xFLtoFR * yFRtoHR - yFLtoFR * xFRtoHR > 0.0)
         return true;
      if (xFRtoHR * yHRtoHL - yFRtoHR * xHRtoHL > 0.0)
         return true;
      if (xHRtoHL * yHLtoFL - yHRtoHL * xHLtoFL > 0.0)
         return true;
      if (xHLtoFL * yFLtoFR - yHLtoFL * xFLtoFR > 0.0)
         return true;

      return false;

   }


   /**
    * isValidTrotPolygon:
    * TRUE if there are exactly two legs in the polygon and they are diagonally opposite
    * FALSE if not true.
    *
    * @return boolean
    */
   public static boolean isValidTrotPolygon(QuadrupedSupportPolygon polygon)
   {
      // check that there are two legs in the polygon
      if (polygon.getNumberOfVertices() != 2)
         return false;

      RobotQuadrant firstLeg = polygon.getFirstSupportingQuadrant();
      RobotQuadrant diagonalLeg = firstLeg.getDiagonalOppositeQuadrant();

      if (polygon.getFootstep(diagonalLeg) == null)
         return false;

      return true;
   }

   /**
    *  getStanceWidthFrontLegs
    *
    *  @return double
    */
   public static double getStanceLength(QuadrupedSupportPolygon supportPolygon, RobotSide robotSide)
   {
      if (supportPolygon.containsFootstep(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide))
            && supportPolygon.containsFootstep(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide)))
      {
         FramePoint3D frontFootstep = supportPolygon.getFootstep(RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide));
         FramePoint3D hindFootstep = supportPolygon.getFootstep(RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide));

         return frontFootstep.distance(hindFootstep);
      }
      else
      {
         throw new UndefinedOperationException("Polygon must contain both legs on this side: " + robotSide);
      }
   }
}
