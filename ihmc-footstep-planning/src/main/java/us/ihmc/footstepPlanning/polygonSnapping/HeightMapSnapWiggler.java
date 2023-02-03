package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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
   private final HashMap<DiscreteFootstep, FootstepSnapData> snapDataHolder;
   private final HeightMapPolygonSnapper heightMapSnapper;
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;

   private static final double searchRadius = 0.06;
   private static final int searchPoints = 20;

   private final double[] wiggleAreas = new double[searchPoints];
   private final double[] wiggleRMSErrors = new double[searchPoints];

   private final double[] blurredWiggleAreas = new double[searchPoints];
   private final double[] blurredRMSErrors = new double[searchPoints];
   private final Vector2D[] offsets = new Vector2D[searchPoints];
   private final double[] gradientMagnitudes = new double[searchPoints];

   private final static double[] blurWeight = new double[] {0.5, 1.5, 5.0, 1.5, 0.5};
   private final static int[] blurOffsets = new int[] {-2, -1, 0, 1, 2};

   private final static double wiggleAreaWeight = 0.2;
   private final static double gradientGain = 0.2;
   private final static double maxWiggle = 0.03;
   private final static double maxTotalWiggle = 0.07;
   private final static int maxIterations = 5;

   public HeightMapSnapWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                               HashMap<DiscreteFootstep, FootstepSnapData> snapDataHolder, HeightMapPolygonSnapper heightMapSnapper)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.snapDataHolder = snapDataHolder;
      this.heightMapSnapper = heightMapSnapper;
   }

   public void computeWiggleTransform(DiscreteFootstep footstepToWiggle, HeightMapData heightMapData, FootstepSnapData snapData, double snapHeightThreshold)
   {
      int yawIndex = footstepToWiggle.getYawIndex();
      RobotSide robotSide = footstepToWiggle.getRobotSide();
      for (int wiggleIndex = 0; wiggleIndex < searchPoints; wiggleIndex++)
      {
         double yaw = footstepToWiggle.getYaw() + ((double) wiggleIndex / searchPoints) * 2.0 * Math.PI;
         Vector2D offset = new Vector2D(searchRadius, 0.0);
         new Orientation2D(yaw).transform(offset);

         Point2D offsetPosition = new Point2D(footstepToWiggle.getX(), footstepToWiggle.getY());
         offsetPosition.add(offset);

         DiscreteFootstep offsetStep = new DiscreteFootstep(LatticePoint.computePositionIndex(offsetPosition.getX()),
                                                            LatticePoint.computePositionIndex(offsetPosition.getY()),
                                                            yawIndex,
                                                            robotSide);

         FootstepSnapData footstepSnapData = snapDataHolder.get(offsetStep);
         if (footstepSnapData == null)
         {
            footstepSnapData = computeSnapData(offsetStep, heightMapData, snapHeightThreshold);
            snapDataHolder.put(offsetStep, footstepSnapData);
         }

         wiggleAreas[wiggleIndex] = Double.isNaN(footstepSnapData.getHeightMapArea()) ? 0.0 : footstepSnapData.getHeightMapArea();
         // TODO need to better parameterize what the worst RMS error is.
         wiggleRMSErrors[wiggleIndex] = Double.isNaN(footstepSnapData.getRMSErrorHeightMap()) ? 1000.0 : footstepSnapData.getRMSErrorHeightMap();
         offsets[wiggleIndex] = offset;
      }

      blurValuesWithNeighbors();

      Vector2D wiggleGradient = computeWiggleGradient(snapData);
      Vector2D wiggleAdjustment = new Vector2D(wiggleGradient);
      wiggleAdjustment.scale(gradientGain);
      wiggleAdjustment.clipToMinMax(-maxWiggle, maxWiggle);

      snapData.getWiggleTransformInWorld().getTranslation().set(wiggleAdjustment.getX(), wiggleAdjustment.getY(), 0.0);
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private FootstepSnapData computeSnapData(DiscreteFootstep footstep, HeightMapData heightMapData, double snapHeightThreshold)
   {
      // TODO don't use the discrete footstep, use the x-yaw location, such that the search radius can be smaller.
      DiscreteFootstepTools.getFootPolygon(footstep, footPolygonsInSoleFrame.get(footstep.getRobotSide()), footPolygon);

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

   private Vector2D computeWiggleGradient(FootstepSnapData snapData)
   {
      double maxMagnitude = Double.NEGATIVE_INFINITY;
      int bestIndex = -1;
      double originCost = computeCost(snapData.getHeightMapArea(), snapData.getRMSErrorHeightMap());
      for (int index = 0; index < searchPoints; index++)
      {
         double offsetCost = computeCost(blurredWiggleAreas[index], blurredRMSErrors[index]);
         double gradientMagnitude = (originCost - offsetCost) / searchRadius;
         gradientMagnitudes[index] = gradientMagnitude;

         if (Math.abs(gradientMagnitude) > maxMagnitude)
         {
            maxMagnitude = gradientMagnitude;
            bestIndex = index;
         }
      }

      Vector2D wiggleGradient = new Vector2D(offsets[bestIndex]);
      wiggleGradient.normalize();
      wiggleGradient.scale(gradientMagnitudes[bestIndex]);

      return wiggleGradient;
   }

   private static double computeCost(double area, double rmsError)
   {
      return wiggleAreaWeight * MathTools.square(area) + MathTools.square(rmsError);
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
