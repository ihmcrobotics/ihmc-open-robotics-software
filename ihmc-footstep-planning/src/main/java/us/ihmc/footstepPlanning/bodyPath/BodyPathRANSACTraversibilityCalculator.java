package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.HashMap;
import java.util.function.ToDoubleFunction;

import static us.ihmc.footstepPlanning.bodyPath.BodyPathCollisionDetector.computeCollisionOffsetX;
import static us.ihmc.footstepPlanning.bodyPath.BodyPathCollisionDetector.computeCollisionOffsetY;

public class BodyPathRANSACTraversibilityCalculator
{
   private static final double sampleSizeX = 0.3;
   private static final double sampleSizeY = 0.3;
   private static final double halfStanceWidth = 0.15;
   private static final double heightWindow = 0.2;

   private static final double minPercent = 0.2;

   private final ToDoubleFunction<BodyPathLatticePoint> gridHeightMap;
   private final HeightMapRANSACNormalCalculator surfaceNormalCalculator;
   private HeightMapData heightMapData;
   private final Pose2D stepPose = new Pose2D();

   private final SideDependentList<YoDouble> traversibilityScores;
   private final SideDependentList<YoInteger> traversibleCells;
   private final SideDependentList<YoInteger> groundCells;
   private final SideDependentList<YoInteger> sampledCells;

   private final HashMap<BodyPathLatticePoint, Double> leftTraversibilityMap = new HashMap<>();
   private final HashMap<BodyPathLatticePoint, Double> rightTraversibilityMap = new HashMap<>();

   private final TIntArrayList squaredUpOffsetsX = new TIntArrayList();
   private final TIntArrayList squaredUpOffsetsY = new TIntArrayList();

   private final TIntArrayList diagonalOffsetsX = new TIntArrayList();
   private final TIntArrayList diagonalOffsetsY = new TIntArrayList();

   public BodyPathRANSACTraversibilityCalculator(ToDoubleFunction<BodyPathLatticePoint> gridHeightMap,
                                                 HeightMapRANSACNormalCalculator surfaceNormalCalculator,
                                                 YoRegistry registry)
   {
      this.gridHeightMap = gridHeightMap;

      traversibilityScores = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "TraversibleScores", registry));
      traversibleCells = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "TraversibleCells", registry));
      groundCells = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "GroundCells", registry));
      sampledCells = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "SampledCells", registry));

      this.surfaceNormalCalculator = surfaceNormalCalculator;
   }

   void setHeightMap(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), squaredUpOffsetsX, squaredUpOffsetsY, sampleSizeX, sampleSizeY, 0.0);
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(), diagonalOffsetsX, diagonalOffsetsY, sampleSizeX, sampleSizeY, Math.toRadians(45.0));

      leftTraversibilityMap.clear();
      rightTraversibilityMap.clear();
   }

   void initialize(BodyPathLatticePoint startNode)
   {
      leftTraversibilityMap.put(startNode, 1.0);
      rightTraversibilityMap.put(startNode, 1.0);
   }

   double computeTraversibility(BodyPathLatticePoint node, BodyPathLatticePoint parentNode, int yawIndex)
   {
      compute(RobotSide.LEFT, node, yawIndex);
      compute(RobotSide.RIGHT, node, yawIndex);

      double leftTraversibility = traversibilityScores.get(RobotSide.LEFT).getValue();
      double rightTraversibility = traversibilityScores.get(RobotSide.RIGHT).getValue();

      leftTraversibilityMap.put(node, leftTraversibility);
      rightTraversibilityMap.put(node, rightTraversibility);

      double alphaNode0 = 0.2;
      double alphaEdge = 0.5;

      double previousLeftTraversibility = leftTraversibilityMap.get(parentNode);
      double previousRightTraversibility = rightTraversibilityMap.get(parentNode);

      return alphaNode0 * (1.0 - Math.max(leftTraversibility, rightTraversibility)) +
             alphaEdge * (1.0 - Math.sqrt(Math.max(leftTraversibility * previousRightTraversibility, rightTraversibility * previousLeftTraversibility)));
   }

   boolean isTraversible()
   {
      return traversibilityScores.get(RobotSide.LEFT).getValue() >= minPercent || traversibilityScores.get(RobotSide.RIGHT).getValue() >= minPercent;
   }

   private void compute(RobotSide side, BodyPathLatticePoint node, int yawIndex)
   {
      stepPose.set(node.getX(), node.getY(), getYaw(yawIndex));
      stepPose.appendTranslation(0.0, side.negateIfLeftSide(halfStanceWidth));

      int xIndex = HeightMapTools.coordinateToIndex(stepPose.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(stepPose.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());

      TIntArrayList xOffsets = yawIndex % 2 == 0 ? squaredUpOffsetsX : diagonalOffsetsX;
      TIntArrayList yOffsets = yawIndex % 2 == 0 ? squaredUpOffsetsY : diagonalOffsetsY;

      YoInteger numberOfTraversibleCells = traversibleCells.get(side);
      YoInteger numberOfGroundCells = groundCells.get(side);
      YoInteger numberOfSampledCells = sampledCells.get(side);

      numberOfTraversibleCells.set(0);
      numberOfGroundCells.set(0);
      numberOfSampledCells.set(0);

      double nominalHeight = gridHeightMap.applyAsDouble(node);
      double minHeight = nominalHeight - heightWindow;
      double maxHeight = nominalHeight + heightWindow;

      for (int i = 0; i < xOffsets.size(); i++)
      {
         int xQuery = xIndex + computeCollisionOffsetX(i, xOffsets.get(i), yOffsets.get(i));
         int yQuery = yIndex + computeCollisionOffsetY(i, yOffsets.get(i), yOffsets.get(i));
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);

         if (xQuery < 0 || yQuery < 0 || xQuery >= heightMapData.getCellsPerAxis() || yQuery >= heightMapData.getCellsPerAxis())
         {
            continue;
         }

         numberOfSampledCells.increment();

         if (heightQuery > minHeight && heightQuery < maxHeight)
         {
            numberOfTraversibleCells.increment();

            if (heightMapData.isCellAtGroundPlane(xQuery, yQuery))
            {
               numberOfGroundCells.increment();
            }
         }
      }

      if (numberOfSampledCells.getValue() < 10)
      {
         traversibilityScores.get(side).set(0.0);
      }
      else
      {
         traversibilityScores.get(side).set(((double) numberOfGroundCells.getValue() + 1.0 * (numberOfTraversibleCells.getValue() - numberOfGroundCells.getValue())) / numberOfSampledCells.getValue());
      }
   }

   private static double getYaw(int yawIndex)
   {
      return yawIndex * Math.PI / 4.0;
   }
}
