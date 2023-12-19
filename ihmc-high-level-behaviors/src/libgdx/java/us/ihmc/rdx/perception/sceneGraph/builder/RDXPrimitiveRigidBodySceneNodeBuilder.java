package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.rdx.perception.sceneGraph.RDXPrimitiveRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXPrimitiveRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXPrimitiveRigidBodySceneNode>
{
   public RDXPrimitiveRigidBodySceneNodeBuilder(SceneGraph sceneGraph)
   {
      super(sceneGraph);
   }

   @Override
   public RDXPrimitiveRigidBodySceneNode build()
   {
      return build(PrimitiveRigidBodyShape.BOX); // Build a box by default
   }

   public RDXPrimitiveRigidBodySceneNode build(PrimitiveRigidBodyShape shape)
   {
      return build(shape,name.get());
   }

   public RDXPrimitiveRigidBodySceneNode build(PrimitiveRigidBodyShape shape, String name)
   {
      long nextID = sceneGraph.getNextID().getAndIncrement();
      PrimitiveRigidBodySceneNode sceneNode = new PrimitiveRigidBodySceneNode(nextID,
                                                                              name,
                                                                              sceneGraph.getIDToNodeMap(),
                                                                              parent.getID(),
                                                                              new RigidBodyTransform(),
                                                                              shape);
      return new RDXPrimitiveRigidBodySceneNode(sceneNode, RDXBaseUI.getInstance().getPrimary3DPanel());
   }
}
