package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class CropVerifier
{
   private final OccupancyGrid occupancyGrid;
   private final YoDouble perpendicularCoPError;

   private final BooleanProvider useCoPOccupancyGridForCropping;
   private final IntegerProvider numberOfCellsThreshold;
   private final DoubleProvider perpendicularCopErrorThreshold;
   private final DoubleProvider distanceFromLineToComputeDesiredCoPOccupancy;
   private final YoInteger numberOfCellsOccupiedOnCropSide;
   private final YoBoolean desiredCopOnCorrectSide;
   private final YoBoolean perpendicularCopErrorAboveThreshold;
   private final YoBoolean enoughDesiredCopOnCropSide;

   private final OccupancyGridVisualizer visualizer;
   private final YoEnum<RobotSide> previousSideToCrop;

   private final YoDouble maxFootPitch;
   private final YoDouble minFootPitch;

   private final YoDouble maxFootRoll;
   private final YoDouble minFootRoll;

   private final ReferenceFrame soleFrame;

   public CropVerifier(String namePrefix,
                       ReferenceFrame soleFrame,
                       double resolution,
                       FootholdRotationParameters rotationParameters,
                       YoRegistry parentRegistry,
                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.soleFrame = soleFrame;

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      occupancyGrid = new OccupancyGrid(namePrefix + "DesiredCoPOccupancy", soleFrame, registry);
      occupancyGrid.setCellSize(resolution);

      perpendicularCoPError = new YoDouble(namePrefix + "PerpendicularCopError", registry);
      perpendicularCopErrorThreshold = rotationParameters.getPerpendicularCoPErrorThreshold();
      distanceFromLineToComputeDesiredCoPOccupancy = rotationParameters.getDistanceFromLineToComputeDesiredCoPOccupancy();
      useCoPOccupancyGridForCropping = rotationParameters.getUseCoPOccupancyGridForCropping();
      perpendicularCopErrorAboveThreshold = new YoBoolean(namePrefix + "PerpendicularCopErrorAboveThreshold", registry);
      enoughDesiredCopOnCropSide = new YoBoolean(namePrefix + "EnoughDesiredCopOnCropSide", registry);
      numberOfCellsThreshold = rotationParameters.getNumberOfDesiredCopsOnCropSide();

      numberOfCellsOccupiedOnCropSide = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnCropSide", registry);
      previousSideToCrop = new YoEnum<>(namePrefix + "PreviousSideToCrop", registry, RobotSide.class, true);

      desiredCopOnCorrectSide = new YoBoolean(namePrefix + "DesiredCopOnCorrectSide", registry);

      maxFootPitch = new YoDouble(namePrefix + "MaxFootPitch", registry);
      minFootPitch = new YoDouble(namePrefix + "MinFootPitch", registry);
      maxFootRoll = new YoDouble(namePrefix + "MaxFootRoll", registry);
      minFootRoll = new YoDouble(namePrefix + "MinFootRoll", registry);

      if (yoGraphicsListRegistry != null)
      {
         visualizer = new OccupancyGridVisualizer(namePrefix + "DesiredCoP", occupancyGrid, 50, YoAppearance.Blue(), registry, yoGraphicsListRegistry);
      }
      else
      {
         visualizer = null;
      }

      reset();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      occupancyGrid.reset();
      previousSideToCrop.set(null);

      maxFootPitch.set(0.0);
      minFootPitch.set(0.0);
      maxFootRoll.set(0.0);
      minFootRoll.set(0.0);

      if (visualizer != null)
         visualizer.update();
   }

   private final FrameQuaternion footOrientation = new FrameQuaternion();

   public void initialize()
   {
      footOrientation.setToZero(soleFrame);
      footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      maxFootPitch.set(footOrientation.getPitch());
      minFootPitch.set(footOrientation.getPitch());
      maxFootPitch.set(footOrientation.getRoll());
      minFootPitch.set(footOrientation.getRoll());
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

      boolean shouldCrop = (enoughDesiredCopOnCropSide.getBooleanValue() || !useCoPOccupancyGridForCropping.getValue());// && desiredCopOnCorrectSide.getBooleanValue();

      if (perpendicularCopErrorAboveThreshold.getBooleanValue() && shouldCrop)
      {
         previousSideToCrop.set(sideToCrop);
         return true;
      }
      else
      {
         return false;
      }
   }
}
