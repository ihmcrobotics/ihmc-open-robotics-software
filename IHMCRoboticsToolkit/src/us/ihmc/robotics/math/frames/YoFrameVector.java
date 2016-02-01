package us.ihmc.robotics.math.frames;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameVector extends YoFrameTuple<FrameVector>
{
   public YoFrameVector(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }

   public YoFrameVector(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFrameVector(DoubleYoVariable xVariable, DoubleYoVariable yVariable, DoubleYoVariable zVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, zVariable, frame);
   }

   protected FrameVector createEmptyFrameTuple()
   {
      return new FrameVector();
   }

   public double length()
   {
      return getFrameTuple().length();
   }

   public double lengthSquared()
   {
      return getFrameTuple().lengthSquared();
   }

   public double dot(FrameVector vector)
   {
      return this.getFrameTuple().dot(vector);
   }

   public double dot(YoFrameVector yoFrameVector)
   {
      return dot(yoFrameVector.getFrameTuple());
   }

   public void normalize()
   {
      getFrameTuple().normalize();
      getYoValuesFromFrameTuple();
   }
}
