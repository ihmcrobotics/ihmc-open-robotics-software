package us.ihmc.robotics.math.frames;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFramePoint2d extends YoFrameTuple2d<FramePoint2d>
{
   public YoFramePoint2d(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }

   public YoFramePoint2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFramePoint2d(DoubleYoVariable xVariable, DoubleYoVariable yVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, frame);
   }

   protected FramePoint2d createEmptyFrameTuple2d()
   {
      return new FramePoint2d();
   }
   
   public double distance(FramePoint2d framePoint2d)
   {
      return getFrameTuple2d().distance(framePoint2d);
   }
}
