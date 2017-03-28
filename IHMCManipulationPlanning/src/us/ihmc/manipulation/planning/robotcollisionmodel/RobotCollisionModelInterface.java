package us.ihmc.manipulation.planning.robotcollisionmodel;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.physics.CollisionShape;

public interface RobotCollisionModelInterface
{
   public abstract void create();
   public abstract void update();
   public abstract CollisionShape getCollisionShape();
   public abstract Graphics3DObject getGraphicObject();
}
