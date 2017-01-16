package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicShape extends YoGraphicAbstractShape
{
   private final Graphics3DObject linkGraphics;

   public YoGraphicShape(String name, Graphics3DObject linkGraphics, YoFramePose framePose, double scale)
   {
      this(name, linkGraphics, framePose.getPosition(), framePose.getOrientation(), scale);
   }

   public YoGraphicShape(String name, Graphics3DObject linkGraphics, YoFramePoint framePoint, YoFrameOrientation frameOrientation, double scale)
   {
      super(name, framePoint, frameOrientation, scale);

      this.linkGraphics = linkGraphics;
   }

   public YoGraphicShape(String name, Graphics3DObject linkGraphics, String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale,
         AppearanceDefinition appearance)
   {
      this(name, linkGraphics, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), new YoFrameOrientation(namePrefix,
            nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }
}
