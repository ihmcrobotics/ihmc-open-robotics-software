package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodyShape;
import us.ihmc.rdx.perception.sceneGraph.RDXPrimitiveRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.List;

import static us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodyShape.getAvailableShapes;

public class RDXPrimitiveRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXPrimitiveRigidBodySceneNode>
{
   private final List<PrimitiveRigidBodyShape> availableShapes;

   public RDXPrimitiveRigidBodySceneNodeBuilder(SceneGraph sceneGraph)
   {
      super(sceneGraph);
      availableShapes = getAvailableShapes();
   }

   @Override
   public RDXPrimitiveRigidBodySceneNode build()
   {
      return build("BOX"); // Build a box by default
   }

   public RDXPrimitiveRigidBodySceneNode build(String selectedShape)
   {
      long nextID = sceneGraph.getNextID().getAndIncrement();

      for (PrimitiveRigidBodyShape shape : availableShapes)
      {
         if (shape.toString() == selectedShape)
         {
            PrimitiveRigidBodySceneNode sceneNode = new PrimitiveRigidBodySceneNode(nextID,
                                                                                    name.get(),
                                                                                    sceneGraph.getIDToNodeMap(),
                                                                                    parent.getID(),
                                                                                    new RigidBodyTransform(),
                                                                                    selectedShape);
            return new RDXPrimitiveRigidBodySceneNode(sceneNode, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
      }
      throw new IllegalStateException("Unexpected value: " + selectedShape);
   }
}
