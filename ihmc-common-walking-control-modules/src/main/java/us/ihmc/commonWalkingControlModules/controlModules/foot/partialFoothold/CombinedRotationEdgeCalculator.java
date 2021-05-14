package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;

public class CombinedRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final RotationEdgeCalculator copHistoryEdgeCalculator;
   private final CoPAndVelocityRotationEdgeCalculator copAndVelocityEdgeCalculator;
   private final EdgeVisualizer edgeVisualizer;

   private final YoBoolean isEdgeStable;

   public CombinedRotationEdgeCalculator(RobotSide side,
                                         MovingReferenceFrame soleFrame,
                                         FootholdRotationParameters rotationParameters,
                                         double dt,
                                         YoRegistry registry,
                                         YoGraphicsListRegistry graphicsRegistry)
   {
      copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, Color.BLUE, graphicsRegistry);
      copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, Color.GRAY, graphicsRegistry);

      isEdgeStable = new YoBoolean(side.getLowerCaseName() + "IsEdgeStable", registry);

      //      if (graphicsRegistry != null)
      //         edgeVisualizer = new EdgeVisualizer(side.getLowerCaseName(), Color.RED, registry, graphicsRegistry);
      //      else
      edgeVisualizer = null;
   }

   @Override
   public void reset()
   {
      isEdgeStable.set(false);
      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(false);
         edgeVisualizer.reset();
      }

      copAndVelocityEdgeCalculator.reset();
      copHistoryEdgeCalculator.reset();
   }

   @Override
   public boolean compute(FramePoint2DReadOnly measuredCoP)
   {
      FrameLine2DReadOnly lineOfRotation = computeLineOfRotation(measuredCoP);
      isEdgeStable.set(lineOfRotation != null);
      if (!isEdgeStable.getBooleanValue())
         return false;

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotation);
      }

      return isRotationEdgeTrusted();
   }

   @Override
   public FrameLine2DReadOnly getLineOfRotation()
   {
      if (copHistoryEdgeCalculator.isRotationEdgeTrusted())
         return copHistoryEdgeCalculator.getLineOfRotation();

      return copAndVelocityEdgeCalculator.getLineOfRotation();
   }

   @Override
   public boolean isRotationEdgeTrusted()
   {
      return isEdgeStable.getBooleanValue();
   }

   private FrameLine2DReadOnly computeLineOfRotation(FramePoint2DReadOnly measuredCoP)
   {
      copHistoryEdgeCalculator.compute(measuredCoP);
      copAndVelocityEdgeCalculator.compute(measuredCoP);

      return getLineOfRotation();
   }

}
