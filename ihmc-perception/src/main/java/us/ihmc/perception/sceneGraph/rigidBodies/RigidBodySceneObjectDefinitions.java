package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class RigidBodySceneObjectDefinitions
{
   /**
    * This is the width of the markers printed with IHMC's large format
    * printer. Send ihmc-perception/src/main/resources/arUcoMarkers/Markers0Through3.pdf
    * to IT to get new ones printed.
    */
   public static final double LARGE_MARKER_WIDTH = 0.1982;

   public static final int BOX_MARKER_ID = 2;
   // The box is a cube
   public static final double BOX_SIZE = 0.35;
   public static final double BOX_MARKER_FROM_BOTTOM_Z = 0.047298;
   public static final double BOX_MARKER_FROM_RIGHT_Y = 0.047298;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(BOX_TRANSFORM_TO_MARKER.getRotation(), 180.0, 0.0, 0.0);
      BOX_TRANSFORM_TO_MARKER.getTranslation().set(0.0, BOX_MARKER_FROM_RIGHT_Y, BOX_MARKER_FROM_BOTTOM_Z);
   }
   public static final String BOX_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/box.g3dj";
   public static final RigidBodyTransform BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   // TODO: Get soup can model from Arghya
   public static final int CAN_OF_SOUP_MARKER_ID = 3;
   public static final double CAN_OF_SOUP_RADIUS = 0.0329375;
   public static final double CAN_OF_SOUP_HEIGHT = 0.082388;
   public static final String CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH = "environmentObjects/canOfSoup/CanOfSoup.g3dj";
   public static final RigidBodyTransform CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(CAN_OF_SOUP_HEIGHT / 2.0);
   }
   public static final double MARKER_TO_CAN_OF_SOUP_X = 0.5;
   public static final RigidBodyTransform MARKER_TO_CAN_OF_SOUP_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(MARKER_TO_CAN_OF_SOUP_TRANSFORM.getRotation(), 0.0, -90.0, 0.0);
      MARKER_TO_CAN_OF_SOUP_TRANSFORM.getTranslation().set(0.0, -MARKER_TO_CAN_OF_SOUP_X, 0.0);
   }

   public static void createBox(SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(SceneGraph.NEXT_ID.getAndIncrement(),
                                                                           "Box",
                                                                           BOX_VISUAL_MODEL_FILE_PATH,
                                                                           BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(BOX_TRANSFORM_TO_MARKER);
      node.setOriginalParentID(parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      node.setOriginalParentFrame(parentNode::getNodeFrame);
      parentNode.getChildren().add(node);
   }

   /**
    * Represents a can of soup detected by a statically nearby placed ArUco marker.
    */
   public static void createCanOfSoup(SceneNode parentNode)
   {
      PredefinedRigidBodySceneNode node = new PredefinedRigidBodySceneNode(SceneGraph.NEXT_ID.getAndIncrement(),
                                                                           "CanOfSoup",
                                                                           CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH,
                                                                           CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      node.getNodeToParentFrameTransform().setAndInvert(MARKER_TO_CAN_OF_SOUP_TRANSFORM);
      node.setOriginalParentID(parentNode.getID());
      node.setOriginalTransformToParent(node.getNodeToParentFrameTransform());
      node.setOriginalParentFrame(parentNode::getNodeFrame);
      parentNode.getChildren().add(node);
   }
}
