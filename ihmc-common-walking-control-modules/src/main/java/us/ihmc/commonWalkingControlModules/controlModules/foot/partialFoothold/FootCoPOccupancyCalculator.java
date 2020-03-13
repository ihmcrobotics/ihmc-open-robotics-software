package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridCell;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.List;

public class FootCoPOccupancyCalculator
{
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 0.02;

   private final ReferenceFrame soleFrame;

   private final YoBoolean leftSideIsOccupied;
   private final YoBoolean rightSideIsOccupied;
   private final YoInteger numberOfOccupiedCellsOnLeft;
   private final YoInteger numberOfOccupiedCellsOnRight;
   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   private final OccupancyGrid occupancyGrid;
   private final OccupancyGridVisualizer visualizer;

   private final FrameLine2D shiftedLine = new FrameLine2D();
   private final FrameVector2D shiftingVector = new FrameVector2D();
   private final FramePoint2D cellCenter = new FramePoint2D();

   private final YoEnum<RobotSide> sideOfFootToCrop;

   public FootCoPOccupancyCalculator(String namePrefix,
                                     ReferenceFrame soleFrame,
                                     double lengthResolution,
                                     double widthResoultion,
                                     FootholdRotationParameters explorationParameters,
                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;

      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);
      sideOfFootToCrop = new YoEnum<>(namePrefix + "OccupancySideOfFootToCrop", registry, RobotSide.class, true);
      this.occupancyGrid = new OccupancyGrid(namePrefix + "CoPOccupancy", soleFrame, registry);

      occupancyGrid.setCellXSize(lengthResolution);
      occupancyGrid.setCellYSize(widthResoultion);
      occupancyGrid.setThresholdForCellOccupancy(defaultThresholdForCellActivation);
      occupancyGrid.setOccupancyDecayRate(defaultDecayRate);

      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();

      numberOfOccupiedCellsOnRight = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnRightSideOfLine", registry);
      numberOfOccupiedCellsOnLeft = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnLeftSideOfLine", registry);

      leftSideIsOccupied = new YoBoolean(namePrefix + "FootLeftSideOfLineIsOccupied", registry);
      rightSideIsOccupied = new YoBoolean(namePrefix + "FootRightSideOfLineIsOccupied", registry);

      if (yoGraphicsListRegistry != null)
         visualizer = new OccupancyGridVisualizer(namePrefix + "Occupancy", occupancyGrid, 50, YoAppearance.Red(), registry, yoGraphicsListRegistry);
      else
         visualizer = null;

      parentRegistry.addChild(registry);
   }

   public void registerCenterOfPressureLocation(FramePoint2DReadOnly copToRegister)
   {
      occupancyGrid.registerPoint(copToRegister);
   }

   public RobotSide computeSideOfFootholdToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      numberOfOccupiedCellsOnLeft.set(computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation,
                                                                               RobotSide.LEFT,
                                                                               distanceFromLineOfRotationToComputeCoPOccupancy.getValue()));
      numberOfOccupiedCellsOnRight.set(computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation,
                                                                                RobotSide.RIGHT,
                                                                                distanceFromLineOfRotationToComputeCoPOccupancy.getValue()));

      leftSideIsOccupied.set(numberOfOccupiedCellsOnLeft.getIntegerValue() >= thresholdForCoPRegionOccupancy.getValue());
      rightSideIsOccupied.set(numberOfOccupiedCellsOnRight.getIntegerValue() >= thresholdForCoPRegionOccupancy.getValue());

      if (leftSideIsOccupied.getBooleanValue() && rightSideIsOccupied.getBooleanValue())
         sideOfFootToCrop.set(null);
      else if (leftSideIsOccupied.getBooleanValue())
         sideOfFootToCrop.set(RobotSide.RIGHT);
      else if (rightSideIsOccupied.getBooleanValue())
         sideOfFootToCrop.set(RobotSide.LEFT);
      else
         sideOfFootToCrop.set(null);

      return sideOfFootToCrop.getEnumValue();
   }

   private int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2DReadOnly frameLine, RobotSide sideToLookAt, double minDistanceFromLine)
   {
      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.
      frameLine.checkReferenceFrameMatch(soleFrame);
      shiftingVector.setIncludingFrame(frameLine.getDirection());
      shiftedLine.setIncludingFrame(frameLine);

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      EuclidGeometryTools.perpendicularVector2D(shiftingVector);
      if (sideToLookAt == RobotSide.RIGHT)
      {
         shiftingVector.negate();
      }

      double theta = Math.atan2(shiftedLine.getDirection().getY(), shiftedLine.getDirection().getX());

      // It is scaled such that the line is being shifted by one cell or minDistanceFromLine depending on which one is the greatest.
      double cellXSize = occupancyGrid.getCellXSize();
      double cellYSize = occupancyGrid.getCellYSize();
      double distanceToMoveAwayFromLine = Math.max(minDistanceFromLine, Math.abs(cellXSize * Math.cos(theta) + cellYSize * Math.sin(theta)));
      shiftingVector.scale(distanceToMoveAwayFromLine);

      // The point of the shiftedLine is shifted using the shiftingVector.
      shiftedLine.getPoint().add(shiftingVector);

      int numberOfCellsActivatedOnSideToLookAt = 0;
      List<OccupancyGridCell> activeCells = occupancyGrid.getAllActiveCells();
      for (int i = 0; i < activeCells.size(); i++)
      {
         OccupancyGridCell cell = activeCells.get(i);
         cellCenter.setIncludingFrame(soleFrame, occupancyGrid.getXLocation(cell.getXIndex()), occupancyGrid.getYLocation(cell.getYIndex()));
         if (shiftedLine.isPointOnSideOfLine(cellCenter, sideToLookAt == RobotSide.LEFT) && cell.getIsOccupied())
            numberOfCellsActivatedOnSideToLookAt++;
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }

   public void reset()
   {
      leftSideIsOccupied.set(false);
      rightSideIsOccupied.set(false);

      occupancyGrid.reset();
      sideOfFootToCrop.set(null);
      if (visualizer != null)
         visualizer.update();
   }

   public void update()
   {
      occupancyGrid.update();
      if (visualizer != null)
         visualizer.update();
   }
}
