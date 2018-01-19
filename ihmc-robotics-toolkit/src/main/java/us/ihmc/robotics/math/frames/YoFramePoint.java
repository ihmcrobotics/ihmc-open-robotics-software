package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoint extends YoFrameTuple implements FixedFramePoint3DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FramePoint3D framePoint3D = new FramePoint3D();

   public YoFramePoint(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   public YoFramePoint(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   public YoFramePoint(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      framePoint3D.setIncludingFrame(frameTuple3DReadOnly);
      framePoint3D.changeFrame(getReferenceFrame());
      set(framePoint3D);
   }
}
