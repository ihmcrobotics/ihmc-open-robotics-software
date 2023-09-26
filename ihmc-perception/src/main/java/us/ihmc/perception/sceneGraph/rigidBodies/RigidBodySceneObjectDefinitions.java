package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
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
   public static final double LARGE_MARKER_WIDTH = 0.150;

   public static final int BOX_MARKER_ID = 2;
   public static final double BOX_MARKER_WIDTH = LARGE_MARKER_WIDTH;
   // The box is a cube
   public static final double BOX_SIZE = 0.35;
   public static final double BOX_MARKER_FROM_BOTTOM_Z = 0.154;
   public static final double BOX_MARKER_FROM_RIGHT_X = 0.1625;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(BOX_TRANSFORM_TO_MARKER.getRotation(), 180.0, 0.0, 0.0);
      BOX_TRANSFORM_TO_MARKER.getTranslation().set(-BOX_MARKER_FROM_RIGHT_X, 0.0, BOX_MARKER_FROM_BOTTOM_Z);
   }
   public static final String BOX_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/emptyBox.g3dj";
   public static final RigidBodyTransform BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final int CAN_OF_SOUP_MARKER_ID = 3;
   public static final double CAN_OF_SOUP_MARKER_SIZE = LARGE_MARKER_WIDTH;
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

   public static final int DEBRIS_MARKER_ID = 7;
   public static final double DEBRIS_MARKER_WIDTH = LARGE_MARKER_WIDTH;
   public static final RigidBodyTransform DEBRIS_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   public static final String DEBRIS_VISUAL_MODEL_FILE_PATH = "environmentObjects/debris/2x4.g3dj";
   public static final RigidBodyTransform DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static ArUcoDetectableNode createBox()
   {
      return new ArUcoDetectableNode("Box",
                                     BOX_MARKER_ID,
                                     BOX_MARKER_WIDTH,
                                     BOX_TRANSFORM_TO_MARKER,
                                     BOX_VISUAL_MODEL_FILE_PATH,
                                     BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }

   /**
    * Represents a can of soup detected by a statically nearby placed ArUco
    * marker,
    */
   public static ArUcoDetectableNode createCanOfSoup()
   {
      return new ArUcoDetectableNode("CanOfSoup",
                                     CAN_OF_SOUP_MARKER_ID,
                                     CAN_OF_SOUP_MARKER_SIZE,
                                     MARKER_TO_CAN_OF_SOUP_TRANSFORM,
                                     CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH,
                                     CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }

   /**
    * Represents a 2x4 debris
    */
   public static ArUcoDetectableNode createDebris()
   {
      return new ArUcoDetectableNode("2x4Debris",
                                     DEBRIS_MARKER_ID,
                                     DEBRIS_MARKER_WIDTH,
                                     DEBRIS_TRANSFORM_TO_MARKER,
                                     DEBRIS_VISUAL_MODEL_FILE_PATH,
                                     DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
   }
}
