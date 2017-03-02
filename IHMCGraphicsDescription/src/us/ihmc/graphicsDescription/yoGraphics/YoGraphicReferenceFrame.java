package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicReferenceFrame extends YoGraphicCoordinateSystem
{
   private final ReferenceFrame referenceFrame;

   public YoGraphicReferenceFrame(ReferenceFrame referenceFrame, YoVariableRegistry registry, double scale)
   {
      this(referenceFrame, registry, scale, YoAppearance.Gray());
   }

   public YoGraphicReferenceFrame(ReferenceFrame referenceFrame, YoVariableRegistry registry, double scale, AppearanceDefinition arrowColor)
   {
      super(referenceFrame.getName(), "", registry, scale, arrowColor);

      this.referenceFrame = referenceFrame;
   }
   
   public YoGraphicReferenceFrame(String prefix, ReferenceFrame referenceFrame, YoVariableRegistry registry, double scale, AppearanceDefinition arrowColor)
   {
      super(prefix + referenceFrame.getName(), "", registry, scale, arrowColor);

      this.referenceFrame = referenceFrame;
   }

   @Override
   public void update()
   {
      this.setToReferenceFrame(referenceFrame);
   }
}
