package us.ihmc.footstepPlanning.polygonSnapping;

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
   private final HeightMapPolygonSnapper heightMapSnapper;
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;

   private static final double searchRadius = 0.10;
   private static final int searchPoints = 20;

   private final double[] wiggleAreas = new double[searchPoints];
   private final double[] wiggleRMSErrors = new double[searchPoints];

   private final double[] blurredWiggleAreas = new double[searchPoints];
   private final double[] blurredRMSErrors = new double[searchPoints];
   private final Vector2D[] offsets = new Vector2D[searchPoints];
   private final double[] gradientMagnitudes = new double[searchPoints];

   private final static double[] blurWeight = new double[] {0.0, 1.0, 7.5, 1.0, 0.0};
   private final static int[] blurOffsets = new int[] {-2, -1, 0, 1, 2};

   private final static double wiggleAreaWeight = 2.0;
   private final static double gradientGain = 0.75;
   private final static double maxWiggle = 0.06;
   private final static double maxTotalWiggle = 0.07;
   private final static int maxIterations = 5;

   public HeightMapSnapWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, HeightMapPolygonSnapper heightMapSnapper)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.heightMapSnapper = heightMapSnapper;
   }

   public void computeWiggleTransform(DiscreteFootstep footstepToWiggle, HeightMapData heightMapData, FootstepSnapData snapData, double snapHeightThreshold)
   {
      RobotSide robotSide = footstepToWiggle.getRobotSide();
      double maxArea = footPolygonsInSoleFrame.get(robotSide).getArea();
      double maxRMSError = MathTools.square(snapHeightThreshold / heightMapData.getGridResolutionXY()) * maxArea;

      Point2D currentPosition = new Point2D(footstepToWiggle.getX(), footstepToWiggle.getY());
      Point2D originalPosition = new Point2D(footstepToWiggle.getX(), footstepToWiggle.getY());

      computeWiggleOffsets();

      for (int iteration = 0; iteration < maxIterations; iteration++)
      {
         for (int wiggleIndex = 0; wiggleIndex < searchPoints; wiggleIndex++)
         {
            Point2D offsetPosition = new Point2D(currentPosition);
            offsetPosition.add(offsets[wiggleIndex]);

            FootstepSnapData footstepSnapData = computeSnapData(offsetPosition,
                                                                footstepToWiggle.getYaw(),
                                                                robotSide,
                                                                heightMapData,
                                                                snapHeightThreshold);

            wiggleAreas[wiggleIndex] = Double.isNaN(footstepSnapData.getHeightMapArea()) ? 0.0 : Math.min(footstepSnapData.getHeightMapArea() / maxArea, 1.0);
            wiggleRMSErrors[wiggleIndex] = Double.isNaN(footstepSnapData.getRMSErrorHeightMap()) ? maxRMSError : footstepSnapData.getRMSErrorHeightMap() / maxRMSError;
         }

         blurValuesWithNeighbors();

         FootstepSnapData currentSnapData = computeSnapData(currentPosition,
                                                            footstepToWiggle.getYaw(),
                                                            robotSide,
                                                            heightMapData,
                                                            snapHeightThreshold);

         double normalizedArea = Math.min(currentSnapData.getHeightMapArea() / maxArea, 1.0);
         double normalizedError = currentSnapData.getRMSErrorHeightMap() / maxRMSError;
         computeGradientMagnitudes(normalizedArea, normalizedError);

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

      FootstepSnapData wiggledSnapData = computeSnapData(currentPosition,
                                                         footstepToWiggle.getYaw(),
                                                         robotSide,
                                                         heightMapData,
                                                         snapHeightThreshold);

      RigidBodyTransform wiggledTransform = new RigidBodyTransform(wiggledSnapData.getSnapTransform());
      wiggledTransform.prependTranslation((currentPosition.getX() - originalPosition.getX()), (currentPosition.getY() - originalPosition.getY()), 0.0);

      FixedReferenceFrame originalFrame = new FixedReferenceFrame("originalFrame", ReferenceFrame.getWorldFrame(), snapData.getSnapTransform());
      FixedReferenceFrame wiggledFrame = new FixedReferenceFrame("wiggledFrame", ReferenceFrame.getWorldFrame(), wiggledTransform);

      FramePose3D wigglePose = new FramePose3D(wiggledFrame);
      wigglePose.changeFrame(originalFrame);

      snapData.getWiggleTransformInWorld().set(wigglePose);

      originalFrame.remove();
      wiggledFrame.remove();
   }

   private void computeWiggleOffsets()
   {
      for (int wiggleIndex = 0; wiggleIndex < searchPoints; wiggleIndex++)
      {
         double yaw = ((double) wiggleIndex / searchPoints) * 2.0 * Math.PI;
         Vector2D offset = new Vector2D(searchRadius, 0.0);
         new Orientation2D(yaw).transform(offset);
         offsets[wiggleIndex] = offset;
      }
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private FootstepSnapData computeSnapData(Point2DReadOnly position, double yaw, RobotSide robotSide, HeightMapData heightMapData, double snapHeightThreshold)
   {
      DiscreteFootstepTools.getFootPolygon(position.getX(), position.getY(), yaw, footPolygonsInSoleFrame.get(robotSide), footPolygon);

      RigidBodyTransform snapTransform = heightMapSnapper.snapPolygonToHeightMap(footPolygon, heightMapData, snapHeightThreshold);

      if (snapTransform == null)
      {
         return FootstepSnapData.emptyData();
      }
      else
      {
         FootstepSnapData snapData = new FootstepSnapData(snapTransform);

         if (heightMapData != null)
         {
            snapData.setRMSErrorHeightMap(heightMapSnapper.getRMSError());
            snapData.setHeightMapArea(heightMapSnapper.getArea());
         }

         return snapData;
      }
   }

   private void blurValuesWithNeighbors()
   {
      double totalWeight = 0.0;
      for (int offset = 0; offset < blurOffsets.length; offset++)
      {
         totalWeight += blurWeight[offset];
      }
      for (int index = 0; index < searchPoints; index++)
      {
         double blurredArea = 0.0;
         double blurredRMS = 0.0;
         for (int offset = 0; offset < blurOffsets.length; offset++)
         {
            int offsetIndex = blurOffsets[offset] + index;
            offsetIndex = moveIndexInRange(offsetIndex, searchPoints);

            blurredArea += blurWeight[offset] * wiggleAreas[offsetIndex];
            blurredRMS += blurWeight[offset] * wiggleRMSErrors[offsetIndex];
         }

         blurredWiggleAreas[index] = blurredArea / totalWeight;
         blurredRMSErrors[index] = blurredRMS / totalWeight;
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
         double offsetCost = computeCost(blurredWiggleAreas[index], blurredRMSErrors[index]);
         double gradientMagnitude = (originCost - offsetCost) / searchRadius;
         gradientMagnitudes[index] = gradientMagnitude;
      }

      // Here, we want to zero out the unstable gradients
      int oppositeOffset = searchPoints / 2;
      for (int index = 0; index < searchPoints; index++)
      {
         int oppositeIndex = moveIndexInRange(index + oppositeOffset, searchPoints);
         if (gradientMagnitudes[index] < 0.0 && gradientMagnitudes[oppositeIndex] < 0.0)
         {
            // trying to push away, so let's zero this out if the opposite direction is bad
            gradientMagnitudes[index] = 0.0;
            gradientMagnitudes[oppositeIndex] = 0.0;
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
