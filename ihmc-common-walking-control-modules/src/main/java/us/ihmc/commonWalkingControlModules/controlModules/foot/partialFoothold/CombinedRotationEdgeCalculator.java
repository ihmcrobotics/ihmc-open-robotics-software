package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;

public class CombinedRotationEdgeCalculator implements RotationEdgeCalculator, SCS2YoGraphicHolder
{
   private final RotationEdgeCalculator copHistoryEdgeCalculator;
   private final CoPAndVelocityRotationEdgeCalculator copAndVelocityEdgeCalculator;
   private final EdgeVisualizer edgeVisualizer;

   private final YoBoolean isEdgeStable;

   public CombinedRotationEdgeCalculator(RobotSide side,
                                         MovingReferenceFrame soleFrame,
                                         YoPartialFootholdModuleParameters rotationParameters,
                                         double dt,
                                         YoRegistry registry)
   {
      copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, ColorDefinitions.Blue());
      copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, ColorDefinitions.Gray());

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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(copHistoryEdgeCalculator.getSCS2YoGraphics());
      group.addChild(copAndVelocityEdgeCalculator.getSCS2YoGraphics());

      return group;
   }

}
