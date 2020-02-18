package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.statistics.Line2DStatisticsCalculator;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.EnumMap;

public class NewPartialFootholdControlModule
{
   private final DoubleProvider omegaThresholdForEstimation;

   private final NewKinematicFootRotationDetector rotationDetector;

   private enum EdgeCalculatorType
   {KINEMATICS, COP_HISTORY, BOTH}

   private final YoEnum<EdgeCalculatorType> edgeCalculatorType;
   private final EnumMap<EdgeCalculatorType, RotationEdgeCalculator> edgeCalculators = new EnumMap<>(EdgeCalculatorType.class);

   public NewPartialFootholdControlModule(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                                          YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      rotationDetector = new NewKinematicFootRotationDetector(side, soleFrame, dt, registry, graphicsRegistry);
      RotationEdgeCalculator kinematicEdgeCalculator = new KinematicsRotationEdgeCalculator(side, soleFrame, dt, registry, graphicsRegistry);
      RotationEdgeCalculator copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, registry, graphicsRegistry);
      edgeCalculators.put(EdgeCalculatorType.KINEMATICS, kinematicEdgeCalculator);
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
      case KINEMATICS:
         edgeCalculators.get(EdgeCalculatorType.KINEMATICS).compute(measuredCoP);
         break;
      case COP_HISTORY:
         edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).compute(measuredCoP);
         break;
      case BOTH:
         edgeCalculators.get(EdgeCalculatorType.KINEMATICS).compute(measuredCoP);
         edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).compute(measuredCoP);
         break;
      default:
         throw new IllegalArgumentException("Can't happen");
      }
   }

   private void resetEdgeCalculators()
   {
      edgeCalculators.get(EdgeCalculatorType.KINEMATICS).reset();
      edgeCalculators.get(EdgeCalculatorType.COP_HISTORY).reset();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      // TODO figure out a way to combine all this stuff
      return edgeCalculators.get(EdgeCalculatorType.KINEMATICS).getLineOfRotation();
   }


   public void reset()
   {
      rotationDetector.reset();
      resetEdgeCalculators();
   }
}
