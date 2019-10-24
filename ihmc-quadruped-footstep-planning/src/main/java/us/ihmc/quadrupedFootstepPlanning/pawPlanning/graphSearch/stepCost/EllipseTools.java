package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class EllipseTools
{
   static double computeMagnitudeOnEllipseInDirection(double ellipseXRadius, double ellipseYRadius, double xDirection, double yDirection)
   {
      double magnitude = EuclidCoreTools.norm(xDirection, yDirection);
      if (magnitude < 1e-3)
         return 0.0;

      magnitude *= getProjectionMultiplierOntoEllipse(ellipseXRadius, ellipseYRadius, xDirection, yDirection);

      return magnitude;
   }

   static void projectPointOntoEllipse(double ellipseXRadius, double ellipseYRadius, Point2DReadOnly pointToProject, Point2DBasics projectedPointToPack)
   {
      projectedPointToPack.set(pointToProject);
      projectedPointToPack.scale(getProjectionMultiplierOntoEllipse(ellipseXRadius, ellipseYRadius, pointToProject));
   }

   static double getProjectionMultiplierOntoEllipse(double ellipseXRadius, double ellipseYRadius, Point2DReadOnly position)
   {
      return getProjectionMultiplierOntoEllipse(ellipseXRadius, ellipseYRadius, position.getX(), position.getY());
   }

   /**
    * There exists an ellipse frame defined such that the x and y axes align with the major and minor axes of the ellipse, with magnitudes given by
    * {@param maxX} and {@param maxY}. This method returns the multiplier to project the point defined by {@param xPosition} and {@param yPosition} onto the
    * ellipse. The point being projected is defined in the ellipse frame.
    *
    * @param ellipseXRadius radius of ellipse along x axis
    * @param ellipseYRadius radius of ellipse along y axis
    * @param xPosition x position of the point in the ellipse frame
    * @param yPosition y position of the point in the ellipse frame
    *
    * @return scale multiplier to project the x and y positions onto the ellipse
    */
   static double getProjectionMultiplierOntoEllipse(double ellipseXRadius, double ellipseYRadius, double xPosition, double yPosition)
   {
      if (ellipseXRadius < 1e-3 && ellipseXRadius < 1e-3)
         return 0.0;

      return ellipseXRadius * ellipseYRadius / Math.sqrt(MathTools.square(ellipseXRadius * yPosition) + MathTools.square(ellipseYRadius * xPosition));
   }

   static boolean isPointInsideEllipse(double ellipseXRadius, double ellipseYRadius, Point2DReadOnly point)
   {
      return isPointInsideEllipse(ellipseXRadius, ellipseYRadius, point.getX(), point.getY());
   }

   static boolean isPointInsideEllipse(double ellipseXRadius, double ellipseYRadius, double xPosition, double yPosition)
   {
      return MathTools.square(xPosition / ellipseXRadius) + MathTools.square(yPosition / ellipseYRadius) < 1.0;
   }


   static double getDistanceFromPointToEllipse(double ellipseXRadius, double ellipseYRadius, Point2DReadOnly point)
   {
      return getDistanceFromPointToEllipse(ellipseXRadius, ellipseYRadius, point.getX(), point.getY());
   }

   static double getDistanceFromPointToEllipse(double ellipseXRadius, double ellipseYRadius, double xPosition, double yPosition)
   {
      double multiplier = getProjectionMultiplierOntoEllipse(ellipseXRadius, ellipseYRadius, xPosition, yPosition);
      return (1.0 - multiplier) * EuclidCoreTools.norm(xPosition, yPosition);
   }
}
