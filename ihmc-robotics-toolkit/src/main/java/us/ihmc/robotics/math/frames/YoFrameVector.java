package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly
// since they contain YoVariables.
public class YoFrameVector extends YoFrameTuple<YoFrameVector, FrameVector3D> implements FrameVector3DReadOnly
{
   public YoFrameVector(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }

   public YoFrameVector(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFrameVector(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, zVariable, frame);
   }

   protected FrameVector3D createEmptyFrameTuple()
   {
      return new FrameVector3D();
   }

   public void cross(FrameVector3D vector1, FrameVector3D vector2)
   {
      getFrameTuple().cross(vector1.getVector(), vector2.getVector());
      getYoValuesFromFrameTuple();
   }

   public void cross(YoFrameVector yoFrameVector1, YoFrameVector yoFrameVector2)
   {
      getFrameTuple().cross(yoFrameVector1.getFrameTuple().getVector(), yoFrameVector2.getFrameTuple().getVector());
      getYoValuesFromFrameTuple();
   }

   public void normalize()
   {
      getFrameTuple().normalize();
      getYoValuesFromFrameTuple();
   }

   private final Vector3D tempVector = new Vector3D();

   public void setAsRotationVector(QuaternionReadOnly quaternion)
   {
      quaternion.get(tempVector);
      set(tempVector);
   }

   public void setAsRotationVector(FrameQuaternion frameOrientation)
   {
      frameOrientation.get(getFrameTuple());
      getYoValuesFromFrameTuple();
   }

   public void setAsRotationVector(YoFrameQuaternion yoFrameQuaternion)
   {
      yoFrameQuaternion.getRotationVector(getFrameTuple());
      getYoValuesFromFrameTuple();
   }
}
