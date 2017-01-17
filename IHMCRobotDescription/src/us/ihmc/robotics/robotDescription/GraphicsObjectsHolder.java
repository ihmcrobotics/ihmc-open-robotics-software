package us.ihmc.robotics.robotDescription;


import us.ihmc.graphicsDescription.Graphics3DObject;

public interface GraphicsObjectsHolder
{
   public abstract CollisionMeshDescription getCollisionObject(String name);
   public abstract Graphics3DObject getGraphicsObject(String name);
}

