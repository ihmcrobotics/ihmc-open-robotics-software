package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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

   public void update()
   {
      this.setToReferenceFrame(referenceFrame);
   }
}
