package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
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
   private BodyPathLatticePoint startNode;
   private final ToDoubleFunction<BodyPathLatticePoint> gridHeightMap;
   private final NormalProvider surfaceNormalCalculator;
   private HeightMapData heightMapData;
   private final Pose2D stepPose = new Pose2D();

   private final SideDependentList<YoDouble> stanceScore;
   private final SideDependentList<YoDouble> stepScores;
   private final YoDouble stanceTraversibility;
   private final SideDependentList<YoInteger> traversibileCells;

   private final TIntArrayList zeroDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList zeroDegCollisionOffsetsY = new TIntArrayList();

   private final TIntArrayList fourtyFiveDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList fourtyFiveDegCollisionOffsetsY = new TIntArrayList();

   private final TIntArrayList twentyTwoDegCollisionOffsetsX = new TIntArrayList();
   private final TIntArrayList twentyTwoDegCollisionOffsetsY = new TIntArrayList();

   private final AStarBodyPathPlannerParametersReadOnly parameters;

   public BodyPathRANSACTraversibilityCalculator(AStarBodyPathPlannerParametersReadOnly traversibilityCalculatorParameters,
                                                 ToDoubleFunction<BodyPathLatticePoint> gridHeightMap,
                                                 NormalProvider surfaceNormalCalculator,
                                                 YoRegistry registry)
   {
      this.parameters = traversibilityCalculatorParameters;
      this.gridHeightMap = gridHeightMap;
      this.stanceScore = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StanceScore", registry));
      this.stepScores = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StepScore", registry));
      this.traversibileCells = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "TraversibleCells", registry));
      this.stanceTraversibility = new YoDouble("stanceTraversibility", registry);
      this.surfaceNormalCalculator = surfaceNormalCalculator;
   }

   void setHeightMap(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
      double size = parameters.getTraversibilitySearchWidth() / 2.0;
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), zeroDegCollisionOffsetsX, zeroDegCollisionOffsetsY, size, size, 0.0);
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), fourtyFiveDegCollisionOffsetsX, fourtyFiveDegCollisionOffsetsY, size, size, Math.toRadians(45.0));
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), twentyTwoDegCollisionOffsetsX, twentyTwoDegCollisionOffsetsY, size, size, Math.toRadians(22.5));
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

      double leftTraversibility = compute(RobotSide.LEFT, node, yawIndex, parentHeight, nodeHeight, true);
      double rightTraversibility = compute(RobotSide.RIGHT, node, yawIndex, parentHeight, nodeHeight,true);

      stanceScore.get(RobotSide.LEFT).set(leftTraversibility);
      stanceScore.get(RobotSide.RIGHT).set(rightTraversibility);

      double previousLeftTraversibility = 1.0;
      double previousRightTraversibility = 1.0;
      if (!startNode.equals(parentNode))
      {
         previousLeftTraversibility = compute(RobotSide.LEFT, parentNode, yawIndex, nodeHeight, parentHeight, false);
         previousRightTraversibility = compute(RobotSide.RIGHT, parentNode, yawIndex, nodeHeight, parentHeight, false);
      }

      stanceTraversibility.set(Math.max(leftTraversibility, rightTraversibility));
      double leftStepScore = Math.sqrt(leftTraversibility * previousRightTraversibility);
      double rightStepScore = Math.sqrt(rightTraversibility * previousLeftTraversibility);

      stepScores.get(RobotSide.LEFT).set(leftStepScore);
      stepScores.get(RobotSide.RIGHT).set(rightStepScore);

      return getTraversability();
   }

   boolean isTraversible()
   {
      return stanceScore.get(RobotSide.LEFT).getValue() >= parameters.getMinTraversibilityScore() || stanceScore.get(RobotSide.RIGHT).getValue() >= parameters.getMinTraversibilityScore();
   }

   double getTraversability()
   {
      double leftStepScore = stepScores.get(RobotSide.LEFT).getDoubleValue();
      double rightStepScore = stepScores.get(RobotSide.RIGHT).getDoubleValue();
      double stepTraversibility = Math.max(leftStepScore, rightStepScore);

      return parameters.getTraversibilityStanceWeight() * (1.0 - stanceTraversibility.getValue())
             + parameters.getTraversibilityStepWeight() * (1.0 - stepTraversibility);
   }

   private double compute(RobotSide side, BodyPathLatticePoint node, int yawIndex, double oppositeHeight, double nominalHeight, boolean updateYoVariables)
   {
      stepPose.set(node.getX(), node.getY(), getYaw(yawIndex));
      stepPose.appendTranslation(0.0, side.negateIfRightSide(parameters.getHalfStanceWidth()));

      int xIndex = HeightMapTools.coordinateToIndex(stepPose.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(stepPose.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());

      TIntArrayList xOffsets = getXOffsets(yawIndex);
      TIntArrayList yOffsets = getYOffsets(yawIndex);

      int numberOfSampledCells = 0;
      int numberOfTraversibleCells = 0;

      double traversibilityScoreNumerator = 0.0;
      double minHeight = Math.max(oppositeHeight, nominalHeight) - parameters.getTraversibilityHeightWindowWidth();
      double maxHeight = Math.min(oppositeHeight, nominalHeight) + parameters.getTraversibilityHeightWindowWidth();
      double averageHeight = 0.5 * (nominalHeight + oppositeHeight);
      double windowWidth = (maxHeight - minHeight) / 2.0;

      double heightAboveGround = Math.abs(averageHeight - heightMapData.getEstimatedGroundHeight());
      boolean isWalkingOnGround = false;
      double discountForNonGroundPointsWhenWalkingOnGround = 1.0;
      if (heightAboveGround < parameters.getHeightProximityForSayingWalkingOnGround())
      {
         isWalkingOnGround = true;
         double lowestNonGroundAlpha = parameters.getTraversibilityNonGroundDiscountWhenWalkingOnGround();

         discountForNonGroundPointsWhenWalkingOnGround = lowestNonGroundAlpha + (1.0 - lowestNonGroundAlpha) * heightAboveGround / parameters.getHeightProximityForSayingWalkingOnGround();
      }

      if (minHeight > maxHeight - 1e-3)
      {
         traversibileCells.get(side).set(0);
         return 0.0;
      }

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

            double deltaHeight = Math.max(0.0, Math.abs(averageHeight - heightQuery) - parameters.getTraversibilityHeightWindowDeadband());
            double cellPercentage = 1.0 - deltaHeight / windowWidth;
            double nonGroundDiscount = 1.0;

            if (isWalkingOnGround && !heightMapData.isCellAtGroundPlane(xQuery, yQuery))
            {
               nonGroundDiscount = discountForNonGroundPointsWhenWalkingOnGround;
            }

            double minNormalToPenalize = Math.toRadians(parameters.getMinNormalAngleToPenalizeForTraversibility());
            double maxNormalToPenalize = Math.toRadians(parameters.getMaxNormalAngleToPenalizeForTraversibility());
            double inclineWeight = parameters.getTraversibilityInclineWeight();

            UnitVector3DReadOnly normal = surfaceNormalCalculator.getSurfaceNormal(xQuery, yQuery);
            double incline = Math.max(0.0, Math.acos(normal.getZ()) - minNormalToPenalize);
            double inclineAlpha = MathTools.clamp((maxNormalToPenalize - incline) / (maxNormalToPenalize - minNormalToPenalize), 0.0, 1.0);
            traversibilityScoreNumerator += nonGroundDiscount * ((1.0 - inclineWeight) * cellPercentage + inclineWeight * inclineAlpha);
         }
      }

      if (updateYoVariables)
      {
         traversibileCells.get(side).set(numberOfTraversibleCells);
      }

      if (numberOfSampledCells < parameters.getMinOccupiedNeighborsForTraversibility())
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
      return yawIndex * Math.PI / 8.0;
   }
}
