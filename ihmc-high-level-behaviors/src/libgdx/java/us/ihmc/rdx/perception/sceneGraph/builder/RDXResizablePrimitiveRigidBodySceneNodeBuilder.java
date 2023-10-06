package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodyShape;
import us.ihmc.rdx.perception.sceneGraph.RDXResizablePrimitiveRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.List;

import static us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodyShape.getAvailableShapes;

public class RDXResizablePrimitiveRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXResizablePrimitiveRigidBodySceneNode>
{
   private final List<PrimitiveRigidBodyShape> availableShapes;

   public RDXResizablePrimitiveRigidBodySceneNodeBuilder(SceneGraph sceneGraph)
   {
      super(sceneGraph);
      availableShapes = getAvailableShapes();
   }

   @Override
   public RDXResizablePrimitiveRigidBodySceneNode build()
   {
      return build("BOX"); // Build a box by default
   }

   public RDXResizablePrimitiveRigidBodySceneNode build(String selectedShape)
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
                                                                                    shape);
            return new RDXResizablePrimitiveRigidBodySceneNode(sceneNode, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
      }
      throw new IllegalStateException("Unexpected value: " + selectedShape);
   }
}
