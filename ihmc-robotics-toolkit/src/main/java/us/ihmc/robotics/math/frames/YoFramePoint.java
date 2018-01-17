package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFramePoint extends YoFrameTuple<YoFramePoint, FramePoint3D> implements FramePoint3DReadOnly
{
   public YoFramePoint(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }
   
   public YoFramePoint(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFramePoint(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, zVariable, frame);
   }

   protected FramePoint3D createEmptyFrameTuple()
   {
      return new FramePoint3D();
   }

   /**
    * Sets this point to the location of the origin of passed in referenceFrame.
    */
   @Override
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      super.setFromReferenceFrame(referenceFrame);
   }
}
