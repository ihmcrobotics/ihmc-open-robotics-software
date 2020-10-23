package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootCoPOccupancyCalculator
{
   private final YoBoolean leftSideIsOccupied;
   private final YoBoolean rightSideIsOccupied;
   private final YoInteger numberOfOccupiedCellsOnLeft;
   private final YoInteger numberOfOccupiedCellsOnRight;
   private final IntegerProvider thresholdForCoPRegionOccupancy;
   private final DoubleProvider distanceFromLineOfRotationToComputeCoPOccupancy;

   private final OccupancyGrid occupancyGrid;

   private final YoEnum<RobotSide> sideOfFootToCrop;

   public FootCoPOccupancyCalculator(String namePrefix,
                                     OccupancyGrid occupancyGrid,
                                     FootholdRotationParameters explorationParameters,
                                     YoRegistry parentRegistry)
   {
      this.occupancyGrid = occupancyGrid;
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(namePrefix + name);
      sideOfFootToCrop = new YoEnum<>(namePrefix + "OccupancySideOfFootToCrop", registry, RobotSide.class, true);

      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();

      numberOfOccupiedCellsOnRight = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnRightSideOfLine", registry);
      numberOfOccupiedCellsOnLeft = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnLeftSideOfLine", registry);

      leftSideIsOccupied = new YoBoolean(namePrefix + "FootLeftSideOfLineIsOccupied", registry);
      rightSideIsOccupied = new YoBoolean(namePrefix + "FootRightSideOfLineIsOccupied", registry);

      reset();

      parentRegistry.addChild(registry);
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

      numberOfOccupiedCellsOnLeft.set(-1);
      numberOfOccupiedCellsOnRight.set(-1);

      sideOfFootToCrop.set(null);
   }


}
