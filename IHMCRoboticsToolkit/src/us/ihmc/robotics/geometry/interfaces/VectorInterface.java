package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.tuple3D.Vector3D;

public interface VectorInterface
{
   public abstract void getVector(Vector3D vectorToPack);

   public abstract void setVector(VectorInterface vectorInterface);

   public abstract void setVector(Vector3D vector);
}
