package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import com.sun.prism.ReadbackGraphics;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class FootRotationCalculationModule
{
   private enum RotationDetectorType
   {
      GEOMETRIC, KINEMATIC, VELOCITY, ANY;

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

   private final EdgeCalculatorType[] edgeCalculatorTypes = EdgeCalculatorType.values();

   private final YoBoolean isRotating;

   public FootRotationCalculationModule(RobotSide side, MovingReferenceFrame soleFrame, FootholdRotationParameters rotationParameters, double dt,
                                        YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);

      RotationEdgeCalculator velocityEdgeCalculator = new VelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry,
                                                                                         graphicsRegistry);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry,
                                                                                             graphicsRegistry);
      RotationEdgeCalculator geometricEdgeCalculator = new GeometricRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry,
                                                                                           graphicsRegistry);
      RotationEdgeCalculator copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side, soleFrame, copHistoryEdgeCalculator,
                                                                                                     velocityEdgeCalculator, rotationParameters, dt,
                                                                                                     registry, graphicsRegistry);
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

      rotationDetectorType.set(RotationDetectorType.ANY);
      edgeCalculatorType.set(EdgeCalculatorType.VELOCITY_AND_COP);

      reset();
   }

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      isRotating.set(computeIsRotating());
      if (isRotating.getBooleanValue())
      {
         edgeCalculators.get(edgeCalculatorType.getEnumValue()).compute(measuredCoP);
      }
      else
      {
         resetEdgeCalculators();
      }
   }

   private boolean computeIsRotating()
   {
      if (rotationDetectorType.getEnumValue() == RotationDetectorType.ANY)
      {
         for (RotationDetectorType type : RotationDetectorType.values)
         {
            if (rotationDetectors.get(type).compute())
               return true;
         }

         return false;
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
      resetRotationDetectors();
      resetEdgeCalculators();
   }
}
