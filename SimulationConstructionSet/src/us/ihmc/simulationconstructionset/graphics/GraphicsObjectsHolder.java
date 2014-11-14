package us.ihmc.simulationconstructionset.graphics;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

public interface GraphicsObjectsHolder
{
   public abstract Graphics3DObject getCollisionObject(String name);

   public abstract Graphics3DObject getGraphicsObject(String name);
}
