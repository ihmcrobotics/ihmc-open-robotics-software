package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CropVerifier
{
   private final OccupancyGrid occupancyGrid;
   private final YoDouble perpendicularCoPError;

   private final IntegerProvider numberOfCellsThreshold;
   private final DoubleProvider perpendicularCopErrorThreshold;
   private final DoubleProvider distanceFromLineToComputeDesiredCoPOccupancy;
   private final YoInteger numberOfCellsOccupiedOnCropSide;
   private final YoBoolean desiredCopOnCorrectSide;
   private final YoBoolean perpendicularCopErrorAboveThreshold;
   private final YoBoolean enoughDesiredCopOnCropSide;

   public CropVerifier(String namePrefix, OccupancyGrid occupancyGrid, FootholdRotationParameters explorationParameters, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.occupancyGrid = occupancyGrid;

      perpendicularCoPError = new YoDouble(namePrefix + "PerpendicularCopError", registry);
      perpendicularCopErrorThreshold = explorationParameters.getPerpendicularCoPErrorThreshold();
      distanceFromLineToComputeDesiredCoPOccupancy = explorationParameters.getDistanceFromLineToComputeDesiredCoPOccupancy();
      perpendicularCopErrorAboveThreshold = new YoBoolean(namePrefix + "PerpendicularCopErrorAboveThreshold", registry);
      enoughDesiredCopOnCropSide = new YoBoolean(namePrefix + "EnoughDesiredCopOnCropSide", registry);
      numberOfCellsThreshold = explorationParameters.getNumberOfDesiredCopsOnCropSide();

      numberOfCellsOccupiedOnCropSide = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnCropSide", registry);

      desiredCopOnCorrectSide = new YoBoolean(namePrefix + "DesiredCopOnCorrectSide", registry);

      parentRegistry.addChild(registry);
   }

   public boolean verifyFootholdCrop(FramePoint2DReadOnly desiredCoP, RobotSide sideToCrop, FrameLine2DReadOnly lineOfRotation)
   {
      perpendicularCoPError.set(lineOfRotation.distance(desiredCoP));
      perpendicularCopErrorAboveThreshold.set(perpendicularCoPError.getDoubleValue() > perpendicularCopErrorThreshold.getValue());

      desiredCopOnCorrectSide.set(lineOfRotation.isPointOnSideOfLine(desiredCoP, sideToCrop == RobotSide.LEFT));
      numberOfCellsOccupiedOnCropSide.set(OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid,
                                                                                                      lineOfRotation,
                                                                                                      sideToCrop,
                                                                                                      distanceFromLineToComputeDesiredCoPOccupancy.getValue()));
      enoughDesiredCopOnCropSide.set(numberOfCellsOccupiedOnCropSide.getValue() > numberOfCellsThreshold.getValue());

      return perpendicularCopErrorAboveThreshold.getBooleanValue() && desiredCopOnCorrectSide.getBooleanValue() && enoughDesiredCopOnCropSide.getBooleanValue();
   }
}
