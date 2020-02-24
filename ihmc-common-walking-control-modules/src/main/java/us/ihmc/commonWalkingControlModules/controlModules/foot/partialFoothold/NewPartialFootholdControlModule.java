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
      VELOCITY, COP_HISTORY, GEOMETRIC, VELOCITY_AND_COP
   }

   private final YoEnum<EdgeCalculatorType> edgeCalculatorType;
   private final YoEnum<RotationDetectorType> rotationDetectorType;
   private final EnumMap<EdgeCalculatorType, RotationEdgeCalculator> edgeCalculators = new EnumMap<>(EdgeCalculatorType.class);
   private final EnumMap<RotationDetectorType, FootRotationDetector> rotationDetectors = new EnumMap<>(RotationDetectorType.class);

   private final RotationDetectorType[] rotationDetectorTypes = RotationDetectorType.values();
   private final EdgeCalculatorType[] edgeCalculatorTypes = EdgeCalculatorType.values();

   public NewPartialFootholdControlModule(RobotSide side, MovingReferenceFrame soleFrame, ExplorationParameters explorationParameters, double dt,
                                          YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      String namePrefix = soleFrame.getName();

      RotationEdgeCalculator velocityEdgeCalculator = new VelocityRotationEdgeCalculator(side, soleFrame, explorationParameters, dt, registry,
                                                                                         graphicsRegistry);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, explorationParameters, dt, registry,
                                                                                             graphicsRegistry);
      RotationEdgeCalculator geometricEdgeCalculator = new GeometricRotationEdgeCalculator(side, soleFrame, explorationParameters, dt, registry,
                                                                                           graphicsRegistry);
      RotationEdgeCalculator copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side, soleFrame, copHistoryEdgeCalculator,
                                                                                                     velocityEdgeCalculator, explorationParameters, dt,
                                                                                                     registry, graphicsRegistry);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY, velocityEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.COP_HISTORY, copHistoryEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.GEOMETRIC, geometricEdgeCalculator);
      edgeCalculators.put(EdgeCalculatorType.VELOCITY_AND_COP, copAndVelocityEdgeCalculator);

      FootRotationDetector geometricRotationDetector = new GeometricRotationDetector(namePrefix, explorationParameters, registry);
      FootRotationDetector velocityRotationDetector = new VelocityFootRotationDetector(side, soleFrame, dt, registry);
      FootRotationDetector kinematicRotationDetector = new KinematicFootRotationDetector(side, soleFrame, explorationParameters, dt, registry);
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
         edgeCalculators.get(edgeCalculatorType.getEnumValue()).compute(measuredCoP);
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

   private void resetRotationDetectors()
   {
      for (RotationDetectorType type : rotationDetectorTypes)
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
      resetRotationDetectors();
      resetEdgeCalculators();
   }
}
