package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.HashMap;

public class HeightMapSnapWiggler
{
   private static final double minSearchRadius = 0.02;
   private static final int searchPoints = 12;

   private final static double wiggleAreaWeight = 2.0; // prioritize weighting the area over the RMS value.
   private final static double gradientGain = 0.5;
   private final static double maxWiggle = 0.03;
   private final static double maxTotalWiggle = 0.07;
   private final static int maxIterations = 5;

   private final WiggleParameters wiggleParameters;
   private final HeightMapPolygonSnapper heightMapSnapper;
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;

   private final double[] wiggleAreas = new double[searchPoints];
   private final double[] wiggleRMSErrors = new double[searchPoints];
   private final Vector2D[] offsets = new Vector2D[searchPoints];
   private final double[] gradientMagnitudes = new double[searchPoints];
   private final double[] offsetCosts = new double[searchPoints];

   public HeightMapSnapWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                               WiggleParameters wiggleParameters)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.heightMapSnapper = new HeightMapPolygonSnapper();
      this.wiggleParameters = wiggleParameters;
      // we want to use a finer resolution for this wiggle than the standard search
      this.heightMapSnapper.setSnapAreaResolution(0.05);
   }

   public void computeWiggleTransform(DiscreteFootstep footstepToWiggle,
                                      HeightMapData heightMapData,
                                      FootstepSnapData snapData,
                                      double snapHeightThreshold,
                                      double minSurfaceInclineRadians)
   {
      RobotSide robotSide = footstepToWiggle.getRobotSide();
      double maxArea = footPolygonsInSoleFrame.get(robotSide).getArea();

      Point2D currentPosition = new Point2D(footstepToWiggle.getX(), footstepToWiggle.getY());
      Point2D originalPosition = new Point2D(footstepToWiggle.getX(), footstepToWiggle.getY());

      computeWiggleOffsets(footstepToWiggle.getYaw());

      for (int iteration = 0; iteration < maxIterations; iteration++)
      {
         for (int wiggleIndex = 0; wiggleIndex < searchPoints; wiggleIndex++)
         {
            Point2D offsetPosition = new Point2D(currentPosition);
            offsetPosition.add(offsets[wiggleIndex]);

            FootstepSnapData footstepSnapData = computeSnapData(offsetPosition, footstepToWiggle.getYaw(), robotSide, heightMapData, snapHeightThreshold, minSurfaceInclineRadians);

            wiggleAreas[wiggleIndex] = Double.isNaN(footstepSnapData.getHeightMapArea()) ? 0.0 : Math.min(footstepSnapData.getHeightMapArea() / maxArea, 1.0);
            wiggleRMSErrors[wiggleIndex] = Double.isNaN(footstepSnapData.getRMSErrorHeightMap()) ? 1.0 : footstepSnapData.getRMSErrorHeightMap();
         }

         FootstepSnapData currentSnapData = computeSnapData(currentPosition, footstepToWiggle.getYaw(), robotSide, heightMapData, snapHeightThreshold, minSurfaceInclineRadians);

         double normalizedArea = Math.min(currentSnapData.getHeightMapArea() / maxArea, 1.0);
         computeGradientMagnitudes(normalizedArea, currentSnapData.getRMSErrorHeightMap());

         Vector2D wiggleGradient = computeWiggleGradient();
         if (wiggleGradient.normSquared() < MathTools.square(0.01))
            break;

         Vector2D wiggleAdjustment = new Vector2D(wiggleGradient);
         wiggleAdjustment.scale(gradientGain);
         wiggleAdjustment.clipToMaxNorm(maxWiggle);

         currentPosition.add(wiggleAdjustment);

         // check the total adjustment
         wiggleAdjustment.sub(currentPosition, originalPosition);
         if (wiggleAdjustment.normSquared() > maxTotalWiggle * maxTotalWiggle)
         {
            wiggleAdjustment.clipToMaxNorm(maxTotalWiggle);
            currentPosition.add(wiggleAdjustment, originalPosition);
            break;
         }
      }

      FootstepSnapData wiggledSnapData = computeSnapData(currentPosition, footstepToWiggle.getYaw(), robotSide, heightMapData, snapHeightThreshold, minSurfaceInclineRadians);

      FramePose3D snappedPose = new FramePose3D();
      snappedPose.getPosition().set(originalPosition);
      snappedPose.applyTransform(snapData.getSnapTransform());

      FixedReferenceFrame originalFrame = new FixedReferenceFrame("originalFrame", ReferenceFrame.getWorldFrame(), snappedPose);

      FramePose3D wiggledPose = new FramePose3D();
      wiggledPose.getPosition().set(currentPosition);
      wiggledPose.applyTransform(wiggledSnapData.getSnapTransform());

      wiggledPose.changeFrame(originalFrame);

      // TODO need to do a wiggle rotation
      snapData.getWiggleTransformInWorld().getTranslation().set(wiggledPose.getPosition());
      snapData.getWiggleTransformInWorld().getRotation().setToZero();

      originalFrame.remove();
   }

   private void computeWiggleOffsets(double stepYaw)
   {
      double offsetDistance = Math.max(minSearchRadius, Math.abs(wiggleParameters.deltaInside));
      for (int wiggleIndex = 0; wiggleIndex < searchPoints; wiggleIndex++)
      {
         double yaw = stepYaw + ((double) wiggleIndex / searchPoints) * 2.0 * Math.PI;
         Vector2D offset = new Vector2D(offsetDistance, 0.0);
         new Orientation2D(yaw).transform(offset);
         offsets[wiggleIndex] = offset;
      }
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private FootstepSnapData computeSnapData(Point2DReadOnly position,
                                            double yaw,
                                            RobotSide robotSide,
                                            HeightMapData heightMapData,
                                            double snapHeightThreshold,
                                            double minSurfaceInclineRadians)
   {
      DiscreteFootstepTools.getFootPolygon(position.getX(), position.getY(), yaw, footPolygonsInSoleFrame.get(robotSide), footPolygon);

      RigidBodyTransform snapTransform = heightMapSnapper.snapPolygonToHeightMap(footPolygon, heightMapData, snapHeightThreshold, minSurfaceInclineRadians);

      if (snapTransform == null)
      {
         return FootstepSnapData.emptyData();
      }
      else
      {
         FootstepSnapData snapData = new FootstepSnapData(snapTransform);

         if (heightMapData != null)
         {
            snapData.setRMSErrorHeightMap(heightMapSnapper.getNormalizedRMSError());
            snapData.setHeightMapArea(heightMapSnapper.getArea());
         }

         return snapData;
      }
   }

   private Vector2D computeWiggleGradient()
   {
      double maxMagnitude = 1e-2; // This is the minimum gradient we care about, if it's less than this, do nothing
      int bestIndex = -1;
      for (int index = 0; index < searchPoints; index++)
      {
         double gradientMagnitude = Math.abs(gradientMagnitudes[index]);

         if (gradientMagnitude > maxMagnitude)
         {
            maxMagnitude = gradientMagnitude;
            bestIndex = index;
         }
      }

      if (bestIndex == -1)
         return new Vector2D();

      Vector2D wiggleGradient = new Vector2D(offsets[bestIndex]);
      wiggleGradient.normalize();
      wiggleGradient.scale(gradientMagnitudes[bestIndex]);

      double negativeDiscount = 1.0;
      if (gradientMagnitudes[bestIndex] < 0.0)
         wiggleGradient.scale(1.0 / negativeDiscount);

      return wiggleGradient;
   }

   private void computeGradientMagnitudes(double normalizedCurrentArea, double normalzedCurrentRMSError)
   {
      double originCost = computeCost(normalizedCurrentArea, normalzedCurrentRMSError);

      for (int index = 0; index < searchPoints; index++)
      {
         double offsetCost = computeCost(wiggleAreas[index], wiggleRMSErrors[index]);
         double gradientMagnitude = (originCost - offsetCost) / offsets[index].norm();
         gradientMagnitudes[index] = gradientMagnitude;
         offsetCosts[index] = offsetCost;
      }

      // Here, we want to zero out the unstable gradients, which are gradients that are pointing in the opposite direction but both negative
      int oppositeOffset = searchPoints / 2;
      for (int index = 0; index < searchPoints; index++)
      {
         int oppositeIndex = moveIndexInRange(index + oppositeOffset, searchPoints);
         if (gradientMagnitudes[index] < 0.0 && gradientMagnitudes[oppositeIndex] < 0.0)
         {
            if (gradientMagnitudes[index] < gradientMagnitudes[oppositeIndex] - 1e-3)
            {
               gradientMagnitudes[index] -= gradientMagnitudes[oppositeIndex];
               gradientMagnitudes[oppositeIndex] = 0.0;
            }
            else if (gradientMagnitudes[index] > gradientMagnitudes[oppositeIndex] + 1e-3)
            {
               gradientMagnitudes[oppositeIndex] -= gradientMagnitudes[index];
               gradientMagnitudes[index] = 0.0;
            }
            else
            {
               // trying to push away, so let's zero this out if the opposite direction is bad
               gradientMagnitudes[index] = 0.0;
               gradientMagnitudes[oppositeIndex] = 0.0;
            }
         }
      }
   }

   private static double computeCost(double areaFraction, double rmsError)
   {
      return (wiggleAreaWeight * MathTools.square(areaFraction - 1.0) + MathTools.square(rmsError)) / (wiggleAreaWeight + 1.0);
   }

   private static int moveIndexInRange(int index, int range)
   {
      while (index < 0)
         index += range;
      while (index >= range)
         index -= range;

      return index;
   }
}
