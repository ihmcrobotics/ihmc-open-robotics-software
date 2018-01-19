package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoint2d extends YoFrameTuple2d implements FixedFramePoint2DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FramePoint2D framePoint2D = new FramePoint2D();

   public YoFramePoint2d(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, referenceFrame);
   }

   public YoFramePoint2d(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   public YoFramePoint2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      framePoint2D.setIncludingFrame(frameTuple2DReadOnly);
      framePoint2D.changeFrame(getReferenceFrame());
      set(framePoint2D);
   }
}
