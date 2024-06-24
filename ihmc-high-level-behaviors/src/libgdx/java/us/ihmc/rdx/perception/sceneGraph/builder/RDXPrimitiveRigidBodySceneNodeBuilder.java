package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.rdx.perception.sceneGraph.RDXPrimitiveRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.HashMap;
import java.util.Map;

public class RDXPrimitiveRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXPrimitiveRigidBodySceneNode>
{
   private final Map<PrimitiveRigidBodyShape, Integer> ids = new HashMap<>();

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
      long nextID = sceneGraph.getNextID().getAndIncrement();
      PrimitiveRigidBodySceneNode sceneNode = new PrimitiveRigidBodySceneNode(nextID,
                                                                              getName(shape),
                                                                              sceneGraph.getIDToNodeMap(),
                                                                              parent.getID(),
                                                                              new RigidBodyTransform(),
                                                                              shape,
                                                                              sceneGraph.getCRDTInfo());
      return new RDXPrimitiveRigidBodySceneNode(sceneNode, RDXBaseUI.getInstance().getPrimary3DPanel());
   }

   public RDXPrimitiveRigidBodySceneNode build(PrimitiveRigidBodyShape shape, Vector3D32 lengths, Vector3D32 radii)
   {
      long nextID = sceneGraph.getNextID().getAndIncrement();
      PrimitiveRigidBodySceneNode sceneNode = new PrimitiveRigidBodySceneNode(nextID,
                                                                              getName(shape),
                                                                              sceneGraph.getIDToNodeMap(),
                                                                              parent.getID(),
                                                                              new RigidBodyTransform(),
                                                                              shape,
                                                                              sceneGraph.getCRDTInfo());
      return new RDXPrimitiveRigidBodySceneNode(lengths, radii, sceneNode, RDXBaseUI.getInstance().getPrimary3DPanel());
   }

   private String getName(PrimitiveRigidBodyShape shape)
   {
      if (super.name.isEmpty())
      {
         ids.merge(shape, 1, Integer::sum);
         return shape.getCapitalizedName() + ids.get(shape).toString();
      }
      else
      {
         return super.name.get();
      }
   }
}
