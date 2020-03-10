package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridCell;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class FootCoPOccupancyCropper
{
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 1.0;

   private final YoInteger nLengthSubdivisions;
   private final YoInteger nWidthSubdivisions;

   private final ReferenceFrame soleFrame;

   private final double footLength;
   private final double footWidth;

   private final YoInteger numberOfCellsOccupiedOnRightSideOfLine;
   private final YoInteger numberOfCellsOccupiedOnLeftSideOfLine;
   private final SideDependentList<YoInteger> numberOfOccupiedCells;
   private final YoInteger thresholdForCoPRegionOccupancy;
   private final YoDouble distanceFromLineOfRotationToComputeCoPOccupancy;

   private final OccupancyGrid occupancyGrid;
   private final OccupancyGridVisualizer visualizer;


   private final FrameLine2D shiftedLine = new FrameLine2D();
   private final FrameVector2D shiftingVector = new FrameVector2D();
   private final FramePoint2D cellCenter = new FramePoint2D();

   public FootCoPOccupancyCropper(String namePrefix, ReferenceFrame soleFrame, int nLengthSubdivisions, int nWidthSubdivisions,
                                  WalkingControllerParameters walkingControllerParameters, ExplorationParameters explorationParameters,
                                  YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      this.footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      this.soleFrame = soleFrame;


      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);

      this.occupancyGrid = new OccupancyGrid(namePrefix, soleFrame, registry);

      this.nLengthSubdivisions = new YoInteger(namePrefix + "NLengthSubdivisions", registry);
      this.nLengthSubdivisions.set(nLengthSubdivisions);
      this.nWidthSubdivisions = new YoInteger(namePrefix + "NWidthSubdivisions", registry);
      this.nWidthSubdivisions.set(nWidthSubdivisions);

      occupancyGrid.setThresholdForCellOccupancy(defaultThresholdForCellActivation);
      occupancyGrid.setOccupancyDecayRate(defaultDecayRate);

      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();

      numberOfCellsOccupiedOnRightSideOfLine = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnRightSideOfLine", registry);
      numberOfCellsOccupiedOnLeftSideOfLine = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnLeftSideOfLine", registry);

      numberOfOccupiedCells = new SideDependentList<>(numberOfCellsOccupiedOnLeftSideOfLine, numberOfCellsOccupiedOnRightSideOfLine);

      setupChangedGridParameterListeners();

      if (yoGraphicsListRegistry != null)
         visualizer = new OccupancyGridVisualizer(namePrefix, occupancyGrid, 100, 100, registry, yoGraphicsListRegistry);
      else
         visualizer = null;

      parentRegistry.addChild(registry);
   }

   private void setupChangedGridParameterListeners()
   {
      VariableChangedListener changedGridSizeListener = (v) -> {
         occupancyGrid.setCellXSize(footLength / nLengthSubdivisions.getIntegerValue());
         occupancyGrid.setCellYSize(footWidth / nWidthSubdivisions.getIntegerValue());
      };
      nLengthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      nWidthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      changedGridSizeListener.notifyOfVariableChange(null);
   }

   public void registerCenterOfPressureLocation(FramePoint2DReadOnly copToRegister)
   {
      occupancyGrid.registerPoint(copToRegister);
   }

   public RobotSide computeSideOfFootholdToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         numberOfOccupiedCells.get(robotSide)
                              .set(computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation,
                                                                                                 robotSide,
                                                                                                 distanceFromLineOfRotationToComputeCoPOccupancy.getDoubleValue()));
      }

      boolean leftOccupied = numberOfOccupiedCells.get(RobotSide.LEFT).getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();
      boolean rightOccupied = numberOfOccupiedCells.get(RobotSide.RIGHT).getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();

      if (leftOccupied && rightOccupied)
         throw new RuntimeException("Error: both can't be occupied.");

      if (leftOccupied)
         return RobotSide.RIGHT;
      else if (rightOccupied)
         return RobotSide.LEFT;

      return null;
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
      occupancyGrid.reset();
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
