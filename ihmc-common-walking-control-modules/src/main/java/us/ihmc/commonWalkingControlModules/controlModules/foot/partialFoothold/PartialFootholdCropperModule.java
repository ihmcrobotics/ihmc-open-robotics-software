package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class PartialFootholdCropperModule
{
   private enum RotationDetectorType
   {
      GEOMETRIC, KINEMATIC, VELOCITY, ANY, KINEMATIC_AND_VELOCITY;

      static final RotationDetectorType[] values = {GEOMETRIC, KINEMATIC, VELOCITY};
   }

   private enum EdgeCalculatorType
   {
      VELOCITY, COP_HISTORY, GEOMETRIC, VELOCITY_AND_COP
   }

   private final YoEnum<EdgeCalculatorType> edgeCalculatorType;
   private final YoEnum<RotationDetectorType> rotationDetectorType;
   private final EnumMap<EdgeCalculatorType, RotationEdgeCalculator> edgeCalculators = new EnumMap<>(EdgeCalculatorType.class);
   private final EnumMap<RotationDetectorType, FootRotationDetector> rotationDetectors = new EnumMap<>(RotationDetectorType.class);
   private final FootholdCropper croppedFootholdCalculator;

   private final EdgeCalculatorType[] edgeCalculatorTypes = EdgeCalculatorType.values();

   private final YoBoolean isRotating;
   private final YoBoolean isEdgeStable;

   public PartialFootholdCropperModule(RobotSide side,
                                       MovingReferenceFrame soleFrame,
                                       ContactableFoot contactableFoot,
                                       FootholdRotationParameters rotationParameters,
                                       double dt,
                                       YoVariableRegistry parentRegistry,
                                       YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);
      isEdgeStable = new YoBoolean(side.getLowerCaseName() + "IsEdgeStable", registry);

      RotationEdgeCalculator velocityEdgeCalculator = new VelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, null);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side,
                                                                                             soleFrame,
                                                                                             rotationParameters,
                                                                                             dt,
                                                                                             registry,
                                                                                             null);
      RotationEdgeCalculator geometricEdgeCalculator = new GeometricRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, null);
      RotationEdgeCalculator copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side,
                                                                                                     soleFrame,
                                                                                                     velocityEdgeCalculator,
                                                                                                     rotationParameters,
                                                                                                     dt,
                                                                                                     registry,
                                                                                                     graphicsRegistry);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY, velocityEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.COP_HISTORY, copHistoryEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.GEOMETRIC, geometricEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY_AND_COP, copAndVelocityEdgeCalculator);

      FootRotationDetector geometricRotationDetector = new GeometricRotationDetector(side, soleFrame, rotationParameters, registry);
      FootRotationDetector velocityRotationDetector = new VelocityFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);
      FootRotationDetector kinematicRotationDetector = new KinematicFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);
      rotationDetectors.put(RotationDetectorType.GEOMETRIC, geometricRotationDetector);
      rotationDetectors.put(RotationDetectorType.KINEMATIC, kinematicRotationDetector);
      rotationDetectors.put(RotationDetectorType.VELOCITY, velocityRotationDetector);

      edgeCalculatorType = YoEnum.create(side.getCamelCaseName() + "EdgeCalculatorType", EdgeCalculatorType.class, registry);
      rotationDetectorType = YoEnum.create(side.getCamelCaseName() + "RotationDetectorType", RotationDetectorType.class, registry);

      rotationDetectorType.set(RotationDetectorType.KINEMATIC_AND_VELOCITY);
      edgeCalculatorType.set(EdgeCalculatorType.VELOCITY_AND_COP);

      croppedFootholdCalculator = new FootholdCropper(side.getLowerCaseName(),
                                                      contactableFoot,
                                                      rotationParameters,
                                                      registry,
                                                      graphicsRegistry);

      reset();
   }

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      isRotating.set(computeIsRotating());
      croppedFootholdCalculator.update(measuredCoP);

      if (!isRotating.getBooleanValue())
      {
         resetEdgeCalculators();
         isEdgeStable.set(false);
         return;
      }

      RotationEdgeCalculator edgeCalculator = edgeCalculators.get(edgeCalculatorType.getEnumValue());
      edgeCalculator.compute(measuredCoP);
      isEdgeStable.set(edgeCalculator.isRotationEdgeTrusted());
      if (!isEdgeStable.getBooleanValue())
         return;

      croppedFootholdCalculator.computeShrunkenFoothold(edgeCalculator.getLineOfRotation());
   }

   public boolean isRotating()
   {
      return isRotating.getBooleanValue();
   }

   public boolean shouldShrinkFoothold()
   {
      return croppedFootholdCalculator.shouldShrinkFoothold();
   }

   public boolean applyShrunkenFoothold(YoPlaneContactState contactStateToModify)
   {
      return croppedFootholdCalculator.applyShrunkenFoothold(contactStateToModify);
   }

   private boolean computeIsRotating()
   {
      if (rotationDetectorType.getEnumValue() == RotationDetectorType.ANY)
      {
         boolean rotationDetected = false;
         for (RotationDetectorType type : RotationDetectorType.values)
         {
            if (rotationDetectors.get(type).compute())
               rotationDetected = true;
         }

         return rotationDetected;
      }
      else if (rotationDetectorType.getEnumValue() == RotationDetectorType.KINEMATIC_AND_VELOCITY)
      {
         boolean rotationDetected = rotationDetectors.get(RotationDetectorType.KINEMATIC).compute();
         rotationDetected |= rotationDetectors.get(RotationDetectorType.VELOCITY).compute();
         return rotationDetected;
      }
      else
      {
         return rotationDetectors.get(rotationDetectorType.getEnumValue()).compute();
      }
   }

   private void resetRotationDetectors()
   {
      for (RotationDetectorType type : RotationDetectorType.values)
         rotationDetectors.get(type).reset();
   }

   private void resetEdgeCalculators()
   {
      for (EdgeCalculatorType type : edgeCalculatorTypes)
         edgeCalculators.get(type).reset();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return edgeCalculators.get(edgeCalculatorType.getEnumValue()).getLineOfRotation();
   }

   public void reset()
   {
      isRotating.set(false);
      isEdgeStable.set(false);
      resetRotationDetectors();
      resetEdgeCalculators();
      croppedFootholdCalculator.reset();
   }
}
