package us.ihmc.sensorProcessing.outputData;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class ImuData implements Settable<ImuData>
{
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   private final Quaternion orientation = new Quaternion();

   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   public void setLinearAcceleration(Vector3DReadOnly linearAcceleration)
   {
      this.linearAcceleration.set(linearAcceleration);
   }

   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   public Vector3DReadOnly getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public QuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   @Override
   public void set(ImuData other)
   {
      this.angularVelocity.set(other.angularVelocity);
      this.linearAcceleration.set(other.linearAcceleration);
      this.orientation.set(other.orientation);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
      {
         return true;
      }
      else if (obj instanceof ImuData)
      {
         ImuData other = (ImuData) obj;
         if (!angularVelocity.equals(other.angularVelocity))
            return false;
         if (!linearAcceleration.equals(other.linearAcceleration))
            return false;
         if (!orientation.equals(other.orientation))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

}
