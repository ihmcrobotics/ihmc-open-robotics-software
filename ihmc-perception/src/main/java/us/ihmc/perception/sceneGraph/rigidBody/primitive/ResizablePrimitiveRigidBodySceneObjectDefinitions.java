package us.ihmc.perception.sceneGraph.rigidBody.primitive;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;

public class ResizablePrimitiveRigidBodySceneObjectDefinitions
{
   public static final String RESIZABLE_BOX_NAME = "ResizableBox";
   public static final String RESIZABLE_PRISM_NAME = "ResizablePrism";
   public static final String RESIZABLE_CYLINDER_NAME = "ResizableCylinder";
   public static final String RESIZABLE_ELLIPSOID_NAME = "ResizableEllipsoid";
   public static final String RESIZABLE_CONE_NAME = "ResizableCone";
   public static final RigidBodyTransform RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static void ensureResizableBoxNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = new PrimitiveRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                RESIZABLE_BOX_NAME,
                                                                                                sceneGraph.getIDToNodeMap(),
                                                                                                parentNode.getID(),
                                                                                                RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM,
                                                                                                PrimitiveRigidBodyShape.BOX);
      LogTools.info("Adding Resizable Box Node to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode, parentNode));
   }

   public static void ensureResizablePrismNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = new PrimitiveRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                RESIZABLE_PRISM_NAME,
                                                                                                sceneGraph.getIDToNodeMap(),
                                                                                                parentNode.getID(),
                                                                                                RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM,
                                                                                                PrimitiveRigidBodyShape.PRISM);
      LogTools.info("Adding Resizable Prism Node to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode, parentNode));
   }

   public static void ensureResizableCylinderNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = new PrimitiveRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                RESIZABLE_CYLINDER_NAME,
                                                                                                sceneGraph.getIDToNodeMap(),
                                                                                                parentNode.getID(),
                                                                                                RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM,
                                                                                                PrimitiveRigidBodyShape.CYLINDER);
      LogTools.info("Adding Resizable Cylinder Node to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode, parentNode));
   }

   public static void ensureResizableEllipsoidNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = new PrimitiveRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                RESIZABLE_ELLIPSOID_NAME,
                                                                                                sceneGraph.getIDToNodeMap(),
                                                                                                parentNode.getID(),
                                                                                                RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM,
                                                                                                PrimitiveRigidBodyShape.ELLIPSOID);
      LogTools.info("Adding Resizable Ellipsoid Node to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode, parentNode));
   }

   public static void ensureResizableConeNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode = new PrimitiveRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                                                RESIZABLE_CONE_NAME,
                                                                                                sceneGraph.getIDToNodeMap(),
                                                                                                parentNode.getID(),
                                                                                                RESIZABLE_BOX_TO_NODE_FRAME_TRANSFORM,
                                                                                                PrimitiveRigidBodyShape.CONE);
      LogTools.info("Adding Resizable Cone Node to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(primitiveRigidBodySceneNode, parentNode));
   }
}
