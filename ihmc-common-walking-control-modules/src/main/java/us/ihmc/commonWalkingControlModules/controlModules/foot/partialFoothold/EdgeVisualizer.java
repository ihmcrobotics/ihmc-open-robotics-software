package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class EdgeVisualizer implements SCS2YoGraphicHolder
{
   private static final double LineVizWidth = 0.1;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameLine3DBasics tempLineOfRotationInWorld = new FrameLine3D();

   private final YoBoolean visualize;
   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   private final String prefix;
   private final ColorDefinition color;

   public EdgeVisualizer(String prefix, ColorDefinition color, YoRegistry registry)
   {
      this.prefix = prefix;
      this.color = color;

      visualize = new YoBoolean(prefix + "_Visualize", registry);
      linePointA = new YoFramePoint2D(prefix + "_FootRotationPointA", worldFrame, registry);
      linePointB = new YoFramePoint2D(prefix + "_FootRotationPointB", worldFrame, registry);
   }

   public void visualize(boolean visualize)
   {
      this.visualize.set(visualize);
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();
   }

   public void updateGraphics(FrameLine2DReadOnly lineOfRotation)
   {
      if (visualize.getBooleanValue())
      {
         tempLineOfRotationInWorld.setToZero(lineOfRotation.getReferenceFrame());
         tempLineOfRotationInWorld.set(lineOfRotation);
         tempLineOfRotationInWorld.changeFrame(ReferenceFrame.getWorldFrame());

         linePointA.set(tempLineOfRotationInWorld.getDirection());
         linePointA.scale(-0.5 * LineVizWidth);
         linePointA.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());

         linePointB.set(tempLineOfRotationInWorld.getDirection());
         linePointB.scale(0.5 * LineVizWidth);
         linePointB.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());
      }
      else
      {
         reset();
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (color == null)
         return null;

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(prefix + "_LineOfRotation", linePointA, linePointB, color));

      return group;
   }
}
