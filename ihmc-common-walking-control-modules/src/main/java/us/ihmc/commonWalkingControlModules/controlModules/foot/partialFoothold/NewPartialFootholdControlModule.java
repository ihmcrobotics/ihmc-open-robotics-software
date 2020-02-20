package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.NewKinematicFootRotationDetector;
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
   private final DoubleProvider omegaThresholdForEstimation;

   private final NewKinematicFootRotationDetector rotationDetector;

   private enum EdgeCalculatorType
   {VELOCITY, COP_HISTORY, BOTH}

   private final YoEnum<EdgeCalculatorType> edgeCalculatorType;
   private final EnumMap<EdgeCalculatorType, RotationEdgeCalculator> edgeCalculators = new EnumMap<>(EdgeCalculatorType.class);

   public NewPartialFootholdControlModule(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                                          YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      rotationDetector = new NewKinematicFootRotationDetector(side, soleFrame, dt, registry, graphicsRegistry);
      RotationEdgeCalculator velocityEdgeCalculator = new VelocityRotationEdgeCalculator(side, soleFrame, dt, registry, graphicsRegistry);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, registry, graphicsRegistry);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY, velocityEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.COP_HISTORY, copHistoryEdgeCalculator);

      edgeCalculatorType = YoEnum.create(side.getCamelCaseName() + "EdgeCalculatorType", EdgeCalculatorType.class, registry);
      parentRegistry.addChild(registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      omegaThresholdForEstimation = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "omegaThresholdForEstimation", registry, 3.0);

      reset();
   }

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      rotationDetector.compute();
      if (rotationDetector.getAbsoluteFootOmega() > omegaThresholdForEstimation.getValue())
      {
         computeEdgeCalculator(measuredCoP);
      }
      else if (!rotationDetector.isRotating())
      {
         resetEdgeCalculators();
      }
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
      rotationDetector.reset();
      resetEdgeCalculators();
   }
}
