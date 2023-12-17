package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
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

   public static final String BOX_NAME = "Box";
   public static final int BOX_MARKER_ID = 2;
   public static final double BOX_MARKER_SIZE = 0.150;
   // The box is a cube
   public static final double BOX_WIDTH = 0.394;
   public static final double BOX_DEPTH = 0.31;
   public static final double BOX_HEIGHT = 0.265;
   public static final double BOX_MARKER_FROM_BOTTOM_Z = 0.0795;
   public static final double BOX_MARKER_FROM_RIGHT_Y = 0.0765;
   public static final double BOX_MARKER_FROM_FRONT_X = 0.1685;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(BOX_TRANSFORM_TO_MARKER.getRotation(), 180.0, 0.0, 0.0);
      BOX_TRANSFORM_TO_MARKER.getTranslation().set(-BOX_MARKER_FROM_FRONT_X, -BOX_MARKER_FROM_RIGHT_Y, -BOX_MARKER_FROM_BOTTOM_Z);
   }
   public static final String BOX_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/emptyBox.g3dj";
   public static final RigidBodyTransform BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String CAN_OF_SOUP_NAME = "CanOfSoup";
   public static final int CAN_OF_SOUP_MARKER_ID = 3;
   public static final double CAN_OF_SOUP_RADIUS = 0.0329375;
   public static final double CAN_OF_SOUP_HEIGHT = 0.082388;
   public static final String CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH = "environmentObjects/canOfSoup/CanOfSoup.g3dj";
   public static final RigidBodyTransform CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(CAN_OF_SOUP_HEIGHT / 2.0);
   }
   public static final double MARKER_TO_CAN_OF_SOUP_X_WORLD = 0.5;
   public static final double CAN_OF_SOUP_FROM_TABLE_EDGE = 0.03;
   public static final RigidBodyTransform MARKER_TO_CAN_OF_SOUP_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform CAN_OF_SOUP_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(MARKER_TO_CAN_OF_SOUP_TRANSFORM.getRotation(), 0.0, -90.0, 0.0);
      double canDistanceFromMarker = CAN_OF_SOUP_FROM_TABLE_EDGE - TableModelParameters.TABLE_ARUCO_MARKER_FROM_FRONT_EDGE;
      MARKER_TO_CAN_OF_SOUP_TRANSFORM.getTranslation().set(canDistanceFromMarker,
                                                           -MARKER_TO_CAN_OF_SOUP_X_WORLD,
                                                           0.0);
      CAN_OF_SOUP_TO_MARKER_TRANSFORM.setAndInvert(MARKER_TO_CAN_OF_SOUP_TRANSFORM);
   }

   public static final String DEBRIS_NAME = "2x4Debris";
   public static final int DEBRIS_MARKER_ID = 7;
   public static final double DEBRIS_MARKER_WIDTH = LARGE_MARKER_WIDTH;
   public static final RigidBodyTransform DEBRIS_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   public static final String DEBRIS_VISUAL_MODEL_FILE_PATH = "environmentObjects/debris/2x4.g3dj";
   public static final RigidBodyTransform DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final TIntDoubleMap ARUCO_MARKER_SIZES = new TIntDoubleHashMap();
   static
   {
      ARUCO_MARKER_SIZES.put(BOX_MARKER_ID, BOX_MARKER_SIZE);
      ARUCO_MARKER_SIZES.put(CAN_OF_SOUP_MARKER_ID, RigidBodySceneObjectDefinitions.LARGE_MARKER_WIDTH);
   }

   public static final String SHOE_NAME = "Shoe";
   public static final double SHOE_WIDTH = 0.921;
   public static final double SHOE_DEPTH = 0.123;
   public static final double SHOE_HEIGHT = 0.245;
   public static final String SHOE_VISUAL_MODEL_FILE_PATH = "environmentObjects/shoe/shoe.g3dj";
   public static final RigidBodyTransform SHOE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String THINKPAD_NAME = "ThinkPad";
   public static final double THINKPAD_WIDTH = 0.34;
   public static final double THINKPAD_DEPTH = 0.23;
   public static final double THINKPAD_HEIGHT = 0.05;
   public static final String THINKPAD_VISUAL_MODEL_FILE_PATH = "environmentObjects/thinkpad/thinkpad.g3dj";
   public static final RigidBodyTransform THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 180, 0, 90);
      THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(0.5);
      THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addX(0.5);
      THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.15);
   }

   public static void ensureNodesAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      ArUcoMarkerNode boxArUcoMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(BOX_MARKER_ID);
      if (boxArUcoMarker != null)
      {
         SceneNode box = sceneGraph.getNamesToNodesMap().get(BOX_NAME);
         if (box == null)
         {
            ensureBoxNodeAdded(sceneGraph, modificationQueue, boxArUcoMarker);
         }
      }

      ArUcoMarkerNode canOfSoupMarker = sceneGraph.getArUcoMarkerIDToNodeMap().get(CAN_OF_SOUP_MARKER_ID);
      if (canOfSoupMarker != null)
      {
         SceneNode canOfSoup = sceneGraph.getNamesToNodesMap().get(CAN_OF_SOUP_NAME);
         if (canOfSoup == null)
         {
            ensureCanOfSoupNodeAdded(sceneGraph, modificationQueue, canOfSoupMarker);
         }
      }
   }

   public static void ensureBoxNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode box = sceneGraph.getNamesToNodesMap().get(BOX_NAME);
      if (box == null)
      {
         box = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                BOX_NAME,
                                                sceneGraph.getIDToNodeMap(),
                                                parentNode.getID(),
                                                BOX_TRANSFORM_TO_MARKER,
                                                BOX_VISUAL_MODEL_FILE_PATH,
                                                BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         LogTools.info("Adding Box to scene graph.");
         modificationQueue.accept(new SceneGraphNodeAddition(box, parentNode));
      }
   }

   public static void ensureCanOfSoupNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      // Represents a can of soup detected by a statically nearby placed ArUco marker.
      SceneNode canOfSoup = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                   CAN_OF_SOUP_NAME,
                                                   sceneGraph.getIDToNodeMap(),
                                                   parentNode.getID(),
                                                   CAN_OF_SOUP_TO_MARKER_TRANSFORM,
                                                   CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH,
                                                   CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding CanOfSoup to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(canOfSoup, parentNode));
   }

   /**
    * Represents a 2x4 debris
    */
   public static void ensureDebrisNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode canOfSoup = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                             DEBRIS_NAME,
                                                             sceneGraph.getIDToNodeMap(),
                                                             parentNode.getID(),
                                                             DEBRIS_TRANSFORM_TO_MARKER,
                                                             DEBRIS_VISUAL_MODEL_FILE_PATH,
                                                             DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Debris to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(canOfSoup, parentNode));
   }

   public static void ensureShoeNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode shoe = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                             SHOE_NAME,
                                                             sceneGraph.getIDToNodeMap(),
                                                             parentNode.getID(),
                                                             new RigidBodyTransform(),
                                                             SHOE_VISUAL_MODEL_FILE_PATH,
                                                             SHOE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Shoe to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(shoe, parentNode));
   }

   public static void ensureThinkPadNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode thinkpad = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                        THINKPAD_NAME,
                                                        sceneGraph.getIDToNodeMap(),
                                                        parentNode.getID(),
                                                        new RigidBodyTransform(), THINKPAD_VISUAL_MODEL_FILE_PATH,
                                                        THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding ThinkPad to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(thinkpad, parentNode));
   }
}
