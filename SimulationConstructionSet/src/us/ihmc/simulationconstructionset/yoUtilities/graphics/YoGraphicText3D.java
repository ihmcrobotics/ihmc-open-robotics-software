package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicText3D extends YoGraphicAbstractShape
{
   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddExtrusionInstruction instruction;

   public YoGraphicText3D(String name, String text, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, AppearanceDefinition appearance)
   {
      super(name, framePoint, orientation, scale);
      this.graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      instruction = graphics3dObject.addText(text, 20.0, appearance);
   }

   public YoGraphicText3D(String name, String text, String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale,
         AppearanceDefinition appearance)
   {
      this(name, text, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), new YoFrameOrientation(namePrefix, nameSuffix,
            ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public void setAppearance(AppearanceDefinition appearance)
   {
      instruction.setAppearance(appearance);
   }

   public void setText(String text)
   {
      instruction.setText(text);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }
}
