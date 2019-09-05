package us.ihmc.pathPlanning.bodyPathPlanner;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;

import java.util.List;

public class WaypointDefinedBodyPathPlanHolder implements BodyPathPlanHolder
{
   private double[] maxAlphas;
   private double[] segmentLengths;
   private final BodyPathPlan bodyPathPlan = new BodyPathPlan();

   @Override
   public void setWaypoints(List<? extends Point3DReadOnly> waypointPositions, List<MutableDouble> waypointHeadings)
   {
      if (waypointPositions.size() < 2)
         throw new RuntimeException("Must have at least two waypoint Positions!");
      if (waypointHeadings.size() != waypointPositions.size())
         throw new RuntimeException("The number of waypoint positions and waypoint headings must be equal.");

      bodyPathPlan.clear();

      for (int i = 0; i < waypointPositions.size(); i++)
      {
         Pose3D waypointPose = new Pose3D();
         waypointPose.getPosition().set(waypointPositions.get(i));
         waypointPose.getOrientation().setYawPitchRoll(waypointHeadings.get(i).getValue(), 0.0, 0.0);
         bodyPathPlan.addWaypoint(waypointPose);
      }
      this.maxAlphas = new double[waypointPositions.size() - 1];
      this.segmentLengths = new double[waypointPositions.size() - 1];

      double totalPathLength = 0.0;

      for (int i = 0; i < segmentLengths.length; i++)
      {
         Point3DReadOnly segmentStart = waypointPositions.get(i);
         Point3DReadOnly segmentEnd = waypointPositions.get(i + 1);
         segmentLengths[i] = segmentEnd.distanceXY(segmentStart);
         totalPathLength = totalPathLength + segmentLengths[i];
      }

      for (int i = 0; i < segmentLengths.length; i++)
      {
         double previousMaxAlpha = (i == 0) ? 0.0 : maxAlphas[i - 1];
         maxAlphas[i] = previousMaxAlpha + segmentLengths[i] / totalPathLength;
      }

      int startIndex = 0;
      int endIndex = waypointPositions.size() - 1;
      Point3DReadOnly startPosition = waypointPositions.get(startIndex);
      Point3DReadOnly endPosition = waypointPositions.get(endIndex);

      bodyPathPlan.setStartPose(startPosition.getX(), startPosition.getY(), waypointHeadings.get(startIndex).getValue());
      bodyPathPlan.setGoalPose(endPosition.getX(), endPosition.getY(), waypointHeadings.get(endIndex).getValue());
   }

   @Override
   public BodyPathPlan getPlan()
   {
      return bodyPathPlan;
   }

   @Override
   public void getPointAlongPath(double alpha, Pose3DBasics poseToPack)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      Pose3DReadOnly firstPoint = bodyPathPlan.getWaypoint(segmentIndex);
      Pose3DReadOnly secondPoint = bodyPathPlan.getWaypoint(segmentIndex + 1);

      double alphaInSegment = getPercentInSegment(segmentIndex, alpha);

      poseToPack.interpolate(firstPoint, secondPoint, alphaInSegment);
   }

   @Override
   public double getClosestPoint(Point2DReadOnly point, Pose3DBasics poseToPack)
   {
      double closestPointDistance = Double.POSITIVE_INFINITY;
      double alpha = Double.NaN;
      Point2D tempClosestPoint = new Point2D();

      for (int i = 0; i < segmentLengths.length; i++)
      {
         Point3DReadOnly segmentStart = bodyPathPlan.getWaypoint(i).getPosition();
         Point3DReadOnly segmentEnd = bodyPathPlan.getWaypoint(i + 1).getPosition();
         EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point, segmentStart.getX(), segmentStart.getY(), segmentEnd.getX(), segmentEnd.getY(), tempClosestPoint);

         double distance = tempClosestPoint.distance(point);
         if (distance < closestPointDistance)
         {
            double distanceToSegmentStart = tempClosestPoint.distanceXY(segmentStart);
            double alphaInSegment = distanceToSegmentStart / segmentLengths[i];

            boolean firstSegment = i == 0;
            double alphaSegmentStart = firstSegment ? 0.0 : maxAlphas[i - 1];
            double alphaSegmentEnd = maxAlphas[i];
            alpha = alphaSegmentStart + alphaInSegment * (alphaSegmentEnd - alphaSegmentStart);

            closestPointDistance = distance;
         }
      }

      getPointAlongPath(alpha, poseToPack);
      return alpha;
   }

   @Override
   public double computePathLength(double alpha)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      double alphaInSegment = getPercentInSegment(segmentIndex, alpha);

      double segmentLength = (1.0 - alphaInSegment) * segmentLengths[segmentIndex];
      for (int i = segmentIndex + 1; i < segmentLengths.length; i++)
      {
         segmentLength = segmentLength + segmentLengths[i];
      }

      return segmentLength;
   }

   private double getPercentInSegment(int segment, double alpha)
   {
      boolean firstSegment = segment == 0;
      double alphaSegmentStart = firstSegment ? 0.0 : maxAlphas[segment - 1];
      double alphaSegmentEnd = maxAlphas[segment];
      return (alpha - alphaSegmentStart) / (alphaSegmentEnd - alphaSegmentStart);
   }

   private int getRegionIndexFromAlpha(double alpha)
   {
      if (alpha > maxAlphas[maxAlphas.length - 1])
      {
         return maxAlphas.length - 1;
      }

      for (int i = 0; i < maxAlphas.length; i++)
      {
         if (maxAlphas[i] >= alpha)
         {
            return i;
         }
      }

      throw new RuntimeException("Alpha = " + alpha + "\nalpha must be between [0,1] and maxAlphas highest value must be 1.0.");
   }
}
