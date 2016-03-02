package us.ihmc.robotics.geometry.interfaces;

import javax.vecmath.Vector3d;

public interface VectorInterface
{
   public abstract void getVector(Vector3d vectorToPack);

   public abstract void setVector(VectorInterface vectorInterface);

   public abstract void setVector(Vector3d vector);
}
