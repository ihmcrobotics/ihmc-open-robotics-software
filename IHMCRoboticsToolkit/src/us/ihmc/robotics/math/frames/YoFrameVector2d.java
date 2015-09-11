package us.ihmc.robotics.math.frames;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameVector2d extends YoFrameTuple2d<FrameVector2d>
{
   public YoFrameVector2d(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }

   public YoFrameVector2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFrameVector2d(String namePrefix, String nameSuffix, String description, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, description, frame, registry);
   }

   public YoFrameVector2d(DoubleYoVariable xVariable, DoubleYoVariable yVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, frame);
   }

   protected FrameVector2d createEmptyFrameTuple2d()
   {
      return new FrameVector2d();
   }

   public double length()
   {
      return getFrameTuple2d().length();
   }

   public double lengthSquared()
   {
      return getFrameTuple2d().lengthSquared();
   }

   public double dot(FrameVector2d vector)
   {
      return getFrameTuple2d().dot(vector);
   }

   public double dot(YoFrameVector2d yoFrameVector)
   {
      return dot(yoFrameVector.getFrameTuple2d());
   }

   public double cross(FrameVector2d frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return getFrameTuple2d().cross(frameVector);
   }

   public double cross(YoFrameVector2d yoFrameVector)
   {
      checkReferenceFrameMatch(yoFrameVector);

      return cross(yoFrameVector.getFrameTuple2d());
   }

   public void normalize()
   {
      getFrameTuple2d().normalize();
      getYoValuesFromFrameTuple2d();
   }
}
