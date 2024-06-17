package us.ihmc.rdx.perception.sceneGraph.builder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.perception.sceneGraph.RDXPredefinedRigidBodySceneNode;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.HashMap;
import java.util.Map;

import static us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions.*;

public class RDXPredefinedRigidBodySceneNodeBuilder extends RDXSceneNodeBuilder<RDXPredefinedRigidBodySceneNode>
{
   private final Map<String, Integer> ids = new HashMap<>();

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
      String name;

      if (super.name.isEmpty())
      {
         ids.merge(modelName, 1, Integer::sum);
         name = modelName + ids.get(modelName).toString();
      }
      else
      {
         name = super.name.get();
      }

      long nextID = sceneGraph.getNextID().getAndIncrement();
      return switch (modelName)
      {
         case "Box" ->
         {
            PredefinedRigidBodySceneNode box = new PredefinedRigidBodySceneNode(nextID,
                                                                                name,
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
                                                                                      name,
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
                                                                                      name,
                                                                                      sceneGraph.getIDToNodeMap(),
                                                                                      parent.getID(),
                                                                                      DEBRIS_TRANSFORM_TO_MARKER,
                                                                                      DEBRIS_VISUAL_MODEL_FILE_PATH,
                                                                                      DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(twoByFour, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "WorkPlatform" ->
         {
            PredefinedRigidBodySceneNode workPlatform = new PredefinedRigidBodySceneNode(nextID,
                                                                                         name,
                                                                                         sceneGraph.getIDToNodeMap(),
                                                                                         parent.getID(),
                                                                                         PLATFORM_TRANSFORM_TO_MARKER,
                                                                                         PLATFORM_VISUAL_MODEL_FILE_PATH,
                                                                                         PLATFORM_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(workPlatform, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Shoe" ->
         {
            PredefinedRigidBodySceneNode shoe = new PredefinedRigidBodySceneNode(nextID,
                                                                                 name,
                                                                                 sceneGraph.getIDToNodeMap(),
                                                                                 parent.getID(),
                                                                                 new RigidBodyTransform(),
                                                                                 SHOE_VISUAL_MODEL_FILE_PATH,
                                                                                 SHOE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(shoe, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Laptop" ->
         {
            PredefinedRigidBodySceneNode laptop = new PredefinedRigidBodySceneNode(nextID,
                                                                                   name,
                                                                                   sceneGraph.getIDToNodeMap(),
                                                                                   parent.getID(),
                                                                                   new RigidBodyTransform(),
                                                                                   LAPTOP_VISUAL_MODEL_FILE_PATH,
                                                                                   LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(laptop, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Book" ->
         {
            PredefinedRigidBodySceneNode book = new PredefinedRigidBodySceneNode(nextID,
                                                                                 name,
                                                                                 sceneGraph.getIDToNodeMap(),
                                                                                 parent.getID(),
                                                                                 new RigidBodyTransform(),
                                                                                 BOOK_VISUAL_MODEL_FILE_PATH,
                                                                                 BOOK_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(book, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Cereal" ->
         {
            PredefinedRigidBodySceneNode cereal = new PredefinedRigidBodySceneNode(nextID,
                                                                                   name,
                                                                                   sceneGraph.getIDToNodeMap(),
                                                                                   parent.getID(),
                                                                                   new RigidBodyTransform(),
                                                                                   CEREAL_VISUAL_MODEL_FILE_PATH,
                                                                                   CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(cereal, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Mug" ->
         {
            PredefinedRigidBodySceneNode mug = new PredefinedRigidBodySceneNode(nextID,
                                                                                name,
                                                                                sceneGraph.getIDToNodeMap(),
                                                                                parent.getID(),
                                                                                new RigidBodyTransform(),
                                                                                MUG_VISUAL_MODEL_FILE_PATH,
                                                                                MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(mug, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Bike" ->
         {
            PredefinedRigidBodySceneNode bike = new PredefinedRigidBodySceneNode(nextID,
                                                                                 name,
                                                                                 sceneGraph.getIDToNodeMap(),
                                                                                 parent.getID(),
                                                                                 new RigidBodyTransform(),
                                                                                 BIKE_VISUAL_MODEL_FILE_PATH,
                                                                                 BIKE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(bike, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Drill" ->
         {
            PredefinedRigidBodySceneNode drill = new PredefinedRigidBodySceneNode(nextID,
                                                                                 name,
                                                                                 sceneGraph.getIDToNodeMap(),
                                                                                 parent.getID(),
                                                                                 new RigidBodyTransform(),
                                                                                 DRILL_VISUAL_MODEL_FILE_PATH,
                                                                                 DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(drill, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "Couch" ->
         {
            PredefinedRigidBodySceneNode couch = new PredefinedRigidBodySceneNode(nextID,
                                                                                  name,
                                                                                  sceneGraph.getIDToNodeMap(),
                                                                                  parent.getID(),
                                                                                  new RigidBodyTransform(),
                                                                                  COUCH_VISUAL_MODEL_FILE_PATH,
                                                                                  COUCH_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(couch, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         case "TrashCan" ->
         {
            PredefinedRigidBodySceneNode trashCan = new PredefinedRigidBodySceneNode(nextID,
                                                                                     name,
                                                                                     sceneGraph.getIDToNodeMap(),
                                                                                     parent.getID(),
                                                                                     new RigidBodyTransform(),
                                                                                     TRASHCAN_VISUAL_MODEL_FILE_PATH,
                                                                                     TRASHCAN_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
            yield new RDXPredefinedRigidBodySceneNode(trashCan, RDXBaseUI.getInstance().getPrimary3DPanel());
         }
         default -> throw new IllegalStateException("Unexpected value: " + name);
      };
   }
}
