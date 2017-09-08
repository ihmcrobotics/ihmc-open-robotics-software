package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.interfaces.GeometryObject;

public interface GeometryTrajectoryObject<P extends GeometryObject<P>, V extends GeometryObject<V>>
{
   public abstract void getPosition(P positionToPack);
   public abstract void getVelocity(V velocityToPack);
   
   public abstract void setPosition(P position);
   public abstract void setVelocity(V velocity);
   
   public abstract double getTime();
}
