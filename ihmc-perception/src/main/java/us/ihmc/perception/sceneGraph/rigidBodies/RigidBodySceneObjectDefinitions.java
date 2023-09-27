package us.ihmc.perception.sceneGraph.rigidBodies;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorModelParameters;
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
   public static final double MARKER_TO_CAN_OF_SOUP_X = 0.5;
   public static final RigidBodyTransform MARKER_TO_CAN_OF_SOUP_TRANSFORM = new RigidBodyTransform();
   public static final RigidBodyTransform CAN_OF_SOUP_TO_MARKER_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(MARKER_TO_CAN_OF_SOUP_TRANSFORM.getRotation(), 0.0, -90.0, 0.0);
      MARKER_TO_CAN_OF_SOUP_TRANSFORM.getTranslation().set(0.0, -MARKER_TO_CAN_OF_SOUP_X, 0.0);
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
}
