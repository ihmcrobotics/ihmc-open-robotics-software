package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.RotationVerificator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.EnumMap;

public class NewPartialFootholdControlModule
{
   private enum RotationDetectorType
   {
      GEOMETRIC, KINEMATIC, VELOCITY
   }
   private enum EdgeCalculatorType
   {
      VELOCITY, COP_HISTORY, BOTH
   }

   private final YoEnum<EdgeCalculatorType> edgeCalculatorType;
   private final YoEnum<RotationDetectorType> rotationDetectorType;
   private final EnumMap<EdgeCalculatorType, RotationEdgeCalculator> edgeCalculators = new EnumMap<>(EdgeCalculatorType.class);
   private final EnumMap<RotationDetectorType, FootRotationDetector> rotationDetectors = new EnumMap<>(RotationDetectorType.class);

   private final RotationDetectorType[] rotationDetectorTypes = RotationDetectorType.values();

   public NewPartialFootholdControlModule(RobotSide side, MovingReferenceFrame soleFrame, ExplorationParameters explorationParameters,
                                          double dt, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      String namePrefix = soleFrame.getName();

      RotationEdgeCalculator velocityEdgeCalculator = new VelocityRotationEdgeCalculator(side, soleFrame, dt, registry, graphicsRegistry);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, registry, graphicsRegistry);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY, velocityEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.COP_HISTORY, copHistoryEdgeCalculator);

      FootRotationDetector geometricRotationDetector = new GeometricRotationDetector(namePrefix, explorationParameters, registry);
      FootRotationDetector velocityRotationDetector = new VelocityFootRotationDetector(side, soleFrame, dt, registry);
      FootRotationDetector kinematicRotationDetector = new KinematicFootRotationDetector(namePrefix, soleFrame, explorationParameters, dt, registry);
      rotationDetectors.put(RotationDetectorType.GEOMETRIC, geometricRotationDetector);
      rotationDetectors.put(RotationDetectorType.KINEMATIC, kinematicRotationDetector);
      rotationDetectors.put(RotationDetectorType.VELOCITY, velocityRotationDetector);

      edgeCalculatorType = YoEnum.create(side.getCamelCaseName() + "EdgeCalculatorType", EdgeCalculatorType.class, registry);
      rotationDetectorType = YoEnum.create(side.getCamelCaseName() + "RotationDetectorType", RotationDetectorType.class, registry);

      reset();

      parentRegistry.addChild(registry);
   }

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      boolean isRotating = computeIsRotating();
      if (isRotating)
      {
         computeEdgeCalculator(measuredCoP);
      }
      else
      {
         resetEdgeCalculators();
      }
   }

   private boolean computeIsRotating()
   {
      return rotationDetectors.get(rotationDetectorType.getEnumValue()).compute();
   }

   private void computeEdgeCalculator(FramePoint2DReadOnly measuredCoP)
   {
      switch (edgeCalculatorType.getEnumValue())
      {
      case VELOCITY:
         edgeCalculators.get(EdgeCalculatorType.VELOCITY).compute(measuredCoP);
         break;
      case COP_HISTORY:
         edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).compute(measuredCoP);
         break;
      case BOTH:
         edgeCalculators.get(EdgeCalculatorType.VELOCITY).compute(measuredCoP);
         edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).compute(measuredCoP);
         break;
      default:
         throw new IllegalArgumentException("Can't happen");
      }
   }

   private void resetRotationDetectors()
   {
      for (RotationDetectorType type : rotationDetectorTypes)
         rotationDetectors.get(type).reset();
   }

   private void resetEdgeCalculators()
   {
      edgeCalculators.get(EdgeCalculatorType.VELOCITY).reset();
      edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).reset();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      // TODO figure out a way to combine all this stuff
      return edgeCalculators.get(EdgeCalculatorType.VELOCITY).getLineOfRotation();
   }


   public void reset()
   {
      resetRotationDetectors();
      resetEdgeCalculators();
   }
}
