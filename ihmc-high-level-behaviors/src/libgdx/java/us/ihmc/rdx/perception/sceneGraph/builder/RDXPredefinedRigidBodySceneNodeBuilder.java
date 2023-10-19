package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.perception.sceneGraph.RDXPredefinedRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

import static us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions.*;

public class RDXPredefinedRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXPredefinedRigidBodySceneNode>
{
   public RDXPredefinedRigidBodySceneNodeBuilder(SceneGraph sceneGraph)
   {
      super(sceneGraph);
   }

   @Override
   public RDXPredefinedRigidBodySceneNode build()
   {
      return build("Box"); // Build a box by default
   }

   public RDXPredefinedRigidBodySceneNode build(String modelName)
   {
      long nextID = sceneGraph.getNextID().getAndIncrement();

      return switch (modelName)
      {
         case "Box" ->
         {
            PredefinedRigidBodySceneNode box = new PredefinedRigidBodySceneNode(nextID,
                                                                                name.get(),
                                                                                sceneGraph.getIDToNodeMap(),
                                                                                parent.getID(),
                                                                                BOX_TRANSFORM_TO_MARKER,
                                                                                BOX_VISUAL_MODEL_FILE_PATH,
                                                                                BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(box, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "CanOfSoup" ->
         {
            PredefinedRigidBodySceneNode canOfSoup = new PredefinedRigidBodySceneNode(nextID,
                                                                                      name.get(),
                                                                                      sceneGraph.getIDToNodeMap(),
                                                                                      parent.getID(),
                                                                                      CAN_OF_SOUP_TO_MARKER_TRANSFORM,
                                                                                      CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH,
                                                                                      CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(canOfSoup, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "2X4" ->
         {
            PredefinedRigidBodySceneNode twoByFour = new PredefinedRigidBodySceneNode(nextID,
                                                                                      name.get(),
                                                                                      sceneGraph.getIDToNodeMap(),
                                                                                      parent.getID(),
                                                                                      DEBRIS_TRANSFORM_TO_MARKER,
                                                                                      DEBRIS_VISUAL_MODEL_FILE_PATH,
                                                                                      DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(twoByFour, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Target" ->
         {
            PredefinedRigidBodySceneNode twoByFour = new PredefinedRigidBodySceneNode(nextID,
                                                                                      name.get(),
                                                                                      sceneGraph.getIDToNodeMap(),
                                                                                      parent.getID(),
                                                                                      TARGET_TRANSFORM_TO_MARKER,
                                                                                      TARGET_VISUAL_MODEL_FILE_PATH,
                                                                                      TARGET_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(twoByFour, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         default -> throw new IllegalStateException("Unexpected value: " + name);
      };
   }
}
