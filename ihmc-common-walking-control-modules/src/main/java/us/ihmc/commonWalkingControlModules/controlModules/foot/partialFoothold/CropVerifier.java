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

   private final OccupancyGridVisualizer visualizer;

   public CropVerifier(String namePrefix,
                       ReferenceFrame soleFrame,
                       double resolution,
                       FootholdRotationParameters explorationParameters,
                       YoVariableRegistry parentRegistry,
                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      occupancyGrid = new OccupancyGrid(namePrefix + "DesiredCoPOccupancy", soleFrame, registry);
      occupancyGrid.setCellSize(resolution);

      perpendicularCoPError = new YoDouble(namePrefix + "PerpendicularCopError", registry);
      perpendicularCopErrorThreshold = explorationParameters.getPerpendicularCoPErrorThreshold();
      distanceFromLineToComputeDesiredCoPOccupancy = explorationParameters.getDistanceFromLineToComputeDesiredCoPOccupancy();
      perpendicularCopErrorAboveThreshold = new YoBoolean(namePrefix + "PerpendicularCopErrorAboveThreshold", registry);
      enoughDesiredCopOnCropSide = new YoBoolean(namePrefix + "EnoughDesiredCopOnCropSide", registry);
      numberOfCellsThreshold = explorationParameters.getNumberOfDesiredCopsOnCropSide();

      numberOfCellsOccupiedOnCropSide = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnCropSide", registry);

      desiredCopOnCorrectSide = new YoBoolean(namePrefix + "DesiredCopOnCorrectSide", registry);

      if (yoGraphicsListRegistry != null)
      {
         visualizer = new OccupancyGridVisualizer(namePrefix + "DesiredCoP", occupancyGrid, 50, YoAppearance.Blue(), registry, yoGraphicsListRegistry);
      }
      else
      {
         visualizer = null;
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      occupancyGrid.reset();

      if (visualizer != null)
         visualizer.update();
   }

   public void update(FramePoint2DReadOnly desiredCoP)
   {
      if (!desiredCoP.containsNaN())
         occupancyGrid.registerPoint(desiredCoP);

      if (visualizer != null)
         visualizer.update();
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
