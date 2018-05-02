package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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

   YoGraphicReferenceFrame(String name, YoDouble x, YoDouble y, YoDouble z, YoDouble yaw, YoDouble pitch, YoDouble roll, double[] constants)
   {
      super(name, x, y, z, yaw, pitch, roll, constants);
      referenceFrame = null;
   }

   @Override
   public void update()
   {
      if (referenceFrame != null)
         this.setToReferenceFrame(referenceFrame);
   }
}
