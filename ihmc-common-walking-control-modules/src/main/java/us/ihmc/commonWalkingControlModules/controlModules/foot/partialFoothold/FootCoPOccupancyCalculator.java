package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootCoPOccupancyCalculator
{
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 0.0;

   private final YoBoolean leftSideIsOccupied;
   private final YoBoolean rightSideIsOccupied;
   private final YoInteger numberOfOccupiedCellsOnLeft;
   private final YoInteger numberOfOccupiedCellsOnRight;
   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   private final OccupancyGrid occupancyGrid;
   private final OccupancyGridVisualizer visualizer;

   private final YoEnum<RobotSide> sideOfFootToCrop;

   public FootCoPOccupancyCalculator(String namePrefix,
                                     ReferenceFrame soleFrame,
                                     double lengthResolution,
                                     double widthResoultion,
                                     FootholdRotationParameters explorationParameters,
                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoVariableRegistry parentRegistry)
   {
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
      numberOfOccupiedCellsOnLeft.set(OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid,
                                                                                                  lineOfRotation,
                                                                                                  RobotSide.LEFT,
                                                                                                  distanceFromLineOfRotationToComputeCoPOccupancy.getValue()));
      numberOfOccupiedCellsOnRight.set(OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid,
                                                                                                   lineOfRotation,
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
