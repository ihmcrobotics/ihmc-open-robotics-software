package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameVector2d extends YoFrameTuple2d implements FixedFrameVector2DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FrameVector2D frameVector2D = new FrameVector2D();

   public YoFrameVector2d(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, referenceFrame);
   }

   public YoFrameVector2d(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameVector2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      frameVector2D.setIncludingFrame(frameTuple2DReadOnly);
      frameVector2D.changeFrame(getReferenceFrame());
      set(frameVector2D);
   }
}
