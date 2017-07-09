package us.ihmc.manipulation.planning.robotcollisionmodel;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public abstract class AbstractCollisionModel
{
   protected SimpleCollisionShapeFactory shapeFactory;
   protected CollisionShape collisionShape;
   protected CollisionShapeDescription<?> collisionShapeDescription;
   protected RigidBodyTransform transform;
   
   public abstract void create();
   public abstract void updateCollisionShape();
   public abstract void updateRighdBodyTransform();
   public abstract CollisionShape getCollisionShape();
   public abstract Graphics3DObject getGraphicObject();
}
