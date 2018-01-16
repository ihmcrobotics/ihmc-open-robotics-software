package us.ihmc.robotics.math.frames;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameVector2d extends YoFrameTuple2d<YoFrameVector2d, FrameVector2D> implements FrameVector2DReadOnly
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

   public YoFrameVector2d(YoDouble xVariable, YoDouble yVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, frame);
   }

   protected FrameVector2D createEmptyFrameTuple2d()
   {
      return new FrameVector2D();
   }

   public double length()
   {
      return getFrameTuple2d().length();
   }

   public double lengthSquared()
   {
      return getFrameTuple2d().lengthSquared();
   }

   public double dot(FrameVector2D vector)
   {
      return getFrameTuple2d().dot(vector);
   }

   public double dot(YoFrameVector2d yoFrameVector)
   {
      return dot(yoFrameVector.getFrameTuple2d());
   }

   public double cross(FrameVector2D frameVector)
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
