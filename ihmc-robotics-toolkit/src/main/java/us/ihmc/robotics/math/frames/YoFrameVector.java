package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameVector extends YoFrameTuple implements FixedFrameVector3DBasics
{
   /** Only for some garbage-free operations and reducing number of operations on the YoDoubles. */
   private final FrameVector3D frameVector3D = new FrameVector3D();

   public YoFrameVector(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   public YoFrameVector(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameVector(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   @Override
   public void setAndMatchFrame(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      frameVector3D.setIncludingFrame(frameTuple3DReadOnly);
      frameVector3D.changeFrame(getReferenceFrame());
      set(frameVector3D);
   }
}
