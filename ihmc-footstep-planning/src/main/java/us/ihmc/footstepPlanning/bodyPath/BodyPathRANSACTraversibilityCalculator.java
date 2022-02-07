package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.function.ToDoubleFunction;

import static us.ihmc.footstepPlanning.bodyPath.BodyPathCollisionDetector.computeCollisionOffsetX;
import static us.ihmc.footstepPlanning.bodyPath.BodyPathCollisionDetector.computeCollisionOffsetY;

public class BodyPathRANSACTraversibilityCalculator
{
   static final double sampleSizeX = 0.3;
   static final double sampleSizeY = 0.4;
   static final double halfStanceWidth = 0.22;
   static final double heightWindow = 0.18;
   static final double nonGroundDiscount = 0.7;
   static final double inclineWeight = 0.1;

   private static final double minPercent = 0.2;
   private static final double minNormalToPenalize = Math.toRadians(45.0);
   private static final double maxNormalToPenalize = Math.toRadians(70.0);

   private BodyPathLatticePoint startNode;
   private final ToDoubleFunction<BodyPathLatticePoint> gridHeightMap;
   private final HeightMapRANSACNormalCalculator surfaceNormalCalculator;
   private HeightMapData heightMapData;
   private final Pose2D stepPose = new Pose2D();

   private final SideDependentList<YoDouble> stanceScore;
   private final SideDependentList<YoDouble> stepScores;
   private final SideDependentList<YoInteger> traversibileCells;

   private final TIntArrayList zeroDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList zeroDegCollisionOffsetsY = new TIntArrayList();

   private final TIntArrayList fourtyFiveDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList fourtyFiveDegCollisionOffsetsY = new TIntArrayList();

   private final TIntArrayList twentyTwoDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList twentyTwoDegCollisionOffsetsY = new TIntArrayList();

   public BodyPathRANSACTraversibilityCalculator(ToDoubleFunction<BodyPathLatticePoint> gridHeightMap,
                                                 HeightMapRANSACNormalCalculator surfaceNormalCalculator,
                                                 YoRegistry registry)
   {
      this.gridHeightMap = gridHeightMap;
      this.stanceScore = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StanceScore", registry));
      this.stepScores = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StepScore", registry));
      this.traversibileCells = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "TraversibleCells", registry));
      this.surfaceNormalCalculator = surfaceNormalCalculator;
   }

   void setHeightMap(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), zeroDegCollisionOffsetsX, zeroDegCollisionOffsetsY, sampleSizeX, sampleSizeY, 0.0);
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), fourtyFiveDegCollisionOffsetsX, fourtyFiveDegCollisionOffsetsY, sampleSizeX, sampleSizeY, Math.toRadians(45.0));
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), twentyTwoDegCollisionOffsetsX, twentyTwoDegCollisionOffsetsY, sampleSizeX, sampleSizeY, Math.toRadians(22.5));
   }

   void initialize(BodyPathLatticePoint startNode)
   {
      this.startNode = startNode;
   }

   void clearVariables()
   {
      for (RobotSide side : RobotSide.values)
      {
         stanceScore.get(side).set(0);
         traversibileCells.get(side).set(0);
      }
   }

   double computeTraversibility(BodyPathLatticePoint node, BodyPathLatticePoint parentNode, int yawIndex)
   {
      double parentHeight = gridHeightMap.applyAsDouble(parentNode);
      double nodeHeight = gridHeightMap.applyAsDouble(node);

      double leftTraversibility = compute(RobotSide.LEFT, node, yawIndex, parentHeight, true);
      double rightTraversibility = compute(RobotSide.RIGHT, node, yawIndex, parentHeight, true);

      stanceScore.get(RobotSide.LEFT).set(leftTraversibility);
      stanceScore.get(RobotSide.RIGHT).set(rightTraversibility);

      double alphaStance = 0.6;
      double alphaStep = 0.5;

      double previousLeftTraversibility = 1.0;
      double previousRightTraversibility = 1.0;
      if (!startNode.equals(parentNode))
      {
         previousLeftTraversibility = compute(RobotSide.LEFT, parentNode, yawIndex, nodeHeight, false);
         previousRightTraversibility = compute(RobotSide.RIGHT, parentNode, yawIndex, nodeHeight, false);
      }

      double stanceTraversibility = Math.max(leftTraversibility, rightTraversibility);
      double leftStepScore = Math.sqrt(leftTraversibility * previousRightTraversibility);
      double rightStepScore = Math.sqrt(rightTraversibility * previousLeftTraversibility);

      stepScores.get(RobotSide.LEFT).set(leftStepScore);
      stepScores.get(RobotSide.RIGHT).set(rightStepScore);
      double stepTraversibility = Math.max(leftStepScore, rightStepScore);

      return alphaStance * (1.0 - stanceTraversibility) + alphaStep * (1.0 - stepTraversibility);
   }

   boolean isTraversible()
   {
      return stanceScore.get(RobotSide.LEFT).getValue() >= minPercent || stanceScore.get(RobotSide.RIGHT).getValue() >= minPercent;
   }

   private double compute(RobotSide side, BodyPathLatticePoint node, int yawIndex, double nominalHeight, boolean updateYoVariables)
   {
      stepPose.set(node.getX(), node.getY(), getYaw(yawIndex));
      stepPose.appendTranslation(0.0, side.negateIfRightSide(halfStanceWidth));

      int xIndex = HeightMapTools.coordinateToIndex(stepPose.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(stepPose.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());

      TIntArrayList xOffsets = getXOffsets(yawIndex);
      TIntArrayList yOffsets = getYOffsets(yawIndex);

      int numberOfSampledCells = 0;
      int numberOfTraversibleCells = 0;

      double traversibilityScoreNumerator = 0.0;
      double minHeight = nominalHeight - heightWindow;
      double maxHeight = nominalHeight + heightWindow;

      for (int i = 0; i < xOffsets.size(); i++)
      {
         int xQuery = xIndex + computeCollisionOffsetX(i, xOffsets.get(i), yOffsets.get(i));
         int yQuery = yIndex + computeCollisionOffsetY(i, xOffsets.get(i), yOffsets.get(i));
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);

         if (xQuery < 0 || yQuery < 0 || xQuery >= heightMapData.getCellsPerAxis() || yQuery >= heightMapData.getCellsPerAxis())
         {
            continue;
         }

         numberOfSampledCells++;
         if (heightQuery > minHeight && heightQuery < maxHeight)
         {
            numberOfTraversibleCells++;

            if (heightMapData.isCellAtGroundPlane(xQuery, yQuery))
            {
               traversibilityScoreNumerator += 1.0;
            }
            else
            {
               double nonGroundAlpha = heightQuery - heightMapData.getEstimatedGroundHeight() < heightWindow ? nonGroundDiscount : 1.0;

               UnitVector3DBasics normal = surfaceNormalCalculator.getSurfaceNormal(xQuery, yQuery);
               double incline = Math.acos(normal.getZ());
               double inclineAlpha = MathTools.clamp(EuclidCoreTools.interpolate(0.0, 1.0, (incline - minNormalToPenalize) / (maxNormalToPenalize - minNormalToPenalize)), 0.0, 1.0);
               traversibilityScoreNumerator += ((1.0 - inclineWeight) * nonGroundAlpha + inclineWeight * inclineAlpha);
            }
         }
      }

      if (updateYoVariables)
      {
         traversibileCells.get(side).set(numberOfTraversibleCells);
      }

      if (numberOfSampledCells < 10)
      {
         return 0.0;
      }
      else
      {
         return traversibilityScoreNumerator / numberOfSampledCells;
      }
   }

   private TIntArrayList getXOffsets(int yawIndex)
   {
      switch (yawIndex)
      {
         case 0:
         case 2:
         case 4:
         case 6:
            return zeroDegCollisionOffsetsX;
         case 1:
         case 3:
         case 5:
         case 7:
            return fourtyFiveDegCollisionOffsetsX;
         case 8:
         case 9:
         case 10:
         case 11:
         case 12:
         case 13:
         case 14:
         case 15:
            return twentyTwoDegCollisionOffsetsX;
      }

      throw new RuntimeException("Yaw index out of range: " + yawIndex);
   }

   private TIntArrayList getYOffsets(int yawIndex)
   {
      switch (yawIndex)
      {
         case 0:
         case 2:
         case 4:
         case 6:
            return zeroDegCollisionOffsetsY;
         case 1:
         case 3:
         case 5:
         case 7:
            return fourtyFiveDegCollisionOffsetsY;
         case 8:
         case 9:
         case 10:
         case 11:
         case 12:
         case 13:
         case 14:
         case 15:
            return twentyTwoDegCollisionOffsetsY;
      }

      throw new RuntimeException("Yaw index out of range: " + yawIndex);
   }

   private static double getYaw(int yawIndex)
   {
      return yawIndex * Math.PI / 4.0;
   }
}
