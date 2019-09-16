package us.ihmc.pathPlanning.bodyPathPlanner;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.AngleTools;

public class WaypointDefinedBodyPathPlanHolder implements BodyPathPlanHolder
{
   private List<Point2DReadOnly> waypointPositions;
   private List<MutableDouble> waypointHeadings;
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

      this.waypointPositions = new ArrayList<>();
      this.waypointHeadings = new ArrayList<>();
      this.waypointHeadings.addAll(waypointHeadings);
      for (int i = 0; i < waypointPositions.size(); i++)
         this.waypointPositions.add(new Point2D(waypointPositions.get(i)));
      this.maxAlphas = new double[waypointPositions.size() - 1];
      this.segmentLengths = new double[waypointPositions.size() - 1];

      double totalPathLength = 0.0;

      for (int i = 0; i < segmentLengths.length; i++)
      {
         Point2DReadOnly segmentStart = this.waypointPositions.get(i);
         Point2DReadOnly segmentEnd = this.waypointPositions.get(i + 1);
         segmentLengths[i] = segmentEnd.distance(segmentStart);
         totalPathLength = totalPathLength + segmentLengths[i];
      }

      for (int i = 0; i < segmentLengths.length; i++)
      {
         double previousMaxAlpha = (i == 0) ? 0.0 : maxAlphas[i - 1];
         maxAlphas[i] = previousMaxAlpha + segmentLengths[i] / totalPathLength;
      }

      int startIndex = 0;
      int endIndex = this.waypointPositions.size() - 1;
      bodyPathPlan.clear();
      bodyPathPlan.setStartPose(this.waypointPositions.get(startIndex), this.waypointHeadings.get(startIndex).getValue());
      bodyPathPlan.setGoalPose(this.waypointPositions.get(endIndex), this.waypointHeadings.get(endIndex).getValue());
      bodyPathPlan.addWaypoints(waypointPositions);
   }

   @Override
   public BodyPathPlan getPlan()
   {
      return bodyPathPlan;
   }

   @Override
   public void getPointAlongPath(double alpha, Pose2D poseToPack)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      Point2DReadOnly firstPoint = waypointPositions.get(segmentIndex);
      Point2DReadOnly secondPoint = waypointPositions.get(segmentIndex + 1);

      double alphaInSegment = getPercentInSegment(segmentIndex, alpha);

      poseToPack.getPosition().interpolate(firstPoint, secondPoint, alphaInSegment);
      double desiredYaw = AngleTools.interpolateAngle(waypointHeadings.get(segmentIndex).getValue(), waypointHeadings.get(segmentIndex + 1).getValue(), alphaInSegment);
      poseToPack.setYaw(desiredYaw);
   }

   @Override
   public double getClosestPoint(Point2DReadOnly point, Pose2D poseToPack)
   {
      double closestPointDistance = Double.POSITIVE_INFINITY;
      double alpha = Double.NaN;
      Point2D tempClosestPoint = new Point2D();

      for (int i = 0; i < segmentLengths.length; i++)
      {
         Point2DReadOnly segmentStart = waypointPositions.get(i);
         Point2DReadOnly segmentEnd = waypointPositions.get(i + 1);
         EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point, segmentStart, segmentEnd, tempClosestPoint);

         double distance = tempClosestPoint.distance(point);
         if (distance < closestPointDistance)
         {
            double distanceToSegmentStart = tempClosestPoint.distance(segmentStart);
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
