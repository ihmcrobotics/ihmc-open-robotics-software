package us.ihmc.robotics.robotDescription;


import java.util.ArrayList;

import us.ihmc.graphicsDescription.Graphics3DObject;

public interface GraphicsObjectsHolder
{
   public abstract ArrayList<CollisionMeshDescription> getCollisionObjects(String name);
   public abstract Graphics3DObject getGraphicsObject(String name);
}

