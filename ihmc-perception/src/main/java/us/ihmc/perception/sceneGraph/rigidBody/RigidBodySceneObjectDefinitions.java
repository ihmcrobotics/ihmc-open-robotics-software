package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class RigidBodySceneObjectDefinitions
{

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
   public static final double DEBRIS_MARKER_WIDTH = SceneObjectDefinitions.LARGE_MARKER_WIDTH;
   public static final RigidBodyTransform DEBRIS_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   public static final String DEBRIS_VISUAL_MODEL_FILE_PATH = "environmentObjects/debris/2x4.g3dj";
   public static final RigidBodyTransform DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String PLATFORM_NAME = "WorkPlatform";
   public static final RigidBodyTransform PLATFORM_TRANSFORM_TO_MARKER = new RigidBodyTransform();
   public static final String PLATFORM_VISUAL_MODEL_FILE_PATH = "environmentObjects/workPlatform/workPlatform.g3dj";
   public static final RigidBodyTransform PLATFORM_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final TIntDoubleMap ARUCO_MARKER_SIZES = new TIntDoubleHashMap();
   static
   {
      ARUCO_MARKER_SIZES.put(BOX_MARKER_ID, BOX_MARKER_SIZE);
      ARUCO_MARKER_SIZES.put(CAN_OF_SOUP_MARKER_ID, SceneObjectDefinitions.LARGE_MARKER_WIDTH);
   }

   public static final String SHOE_NAME = "Shoe";
   public static final double SHOE_WIDTH = 0.921;
   public static final double SHOE_DEPTH = 0.123;
   public static final double SHOE_HEIGHT = 0.245;
   public static final String SHOE_VISUAL_MODEL_FILE_PATH = "environmentObjects/shoe/shoe.g3dj";
   public static final RigidBodyTransform SHOE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String LAPTOP_NAME = "Laptop";
   public static final String LAPTOP_VISUAL_MODEL_FILE_PATH = "environmentObjects/laptop/thinkpad.g3dj";
   public static final RigidBodyTransform LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 180, 0, 90);
      LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(0.5);
      LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addX(0.5);
      LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.15);
   }

   public static final String BOOK_NAME = "Book";
   public static final String BOOK_VISUAL_MODEL_FILE_PATH = "environmentObjects/book/book.g3dj";
   public static final RigidBodyTransform BOOK_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(BOOK_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 90, 90, 0);
   }

   public static final String CEREAL_NAME = "Cereal";
   public static final String CEREAL_VISUAL_MODEL_FILE_PATH = "environmentObjects/cereal/cereal.g3dj";
   public static final RigidBodyTransform CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 90, 90, 0);
      CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.2);
      CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(-0.05);
   }

   public static final String MUG_NAME = "Mug";
   public static final String MUG_VISUAL_MODEL_FILE_PATH = "environmentObjects/mug/mug.g3dj";
   public static final RigidBodyTransform MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 0, -90, -90);
      MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.1);
   }

   public static final String BIKE_NAME = "Bike";
   public static final String BIKE_VISUAL_MODEL_FILE_PATH = "environmentObjects/bike/bike.g3dj";
   public static final RigidBodyTransform BIKE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
//      EuclidCoreMissingTools.setYawPitchRollDegrees(CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 90, 90, 0);
//      CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.2);
//      CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addZ(-0.05);
   }

   public static final String DRILL_NAME = "Drill";
   public static final String DRILL_VISUAL_MODEL_FILE_PATH = "environmentObjects/drill/drill.g3dj";
   public static final RigidBodyTransform DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getRotation(), 0, 90, -90);
      DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM.getTranslation().addY(-0.05);
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

   public static void ensureLaptopNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode laptop = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                        LAPTOP_NAME,
                                                        sceneGraph.getIDToNodeMap(),
                                                        parentNode.getID(),
                                                        new RigidBodyTransform(), LAPTOP_VISUAL_MODEL_FILE_PATH,
                                                        LAPTOP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Laptop to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(laptop, parentNode));
   }

   public static void ensureBookNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode book = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                            BOOK_NAME,
                                                            sceneGraph.getIDToNodeMap(),
                                                            parentNode.getID(),
                                                            new RigidBodyTransform(), BOOK_VISUAL_MODEL_FILE_PATH,
                                                            BOOK_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Book to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(book, parentNode));
   }

   public static void ensureCerealNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode cereal = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                        CEREAL_NAME,
                                                        sceneGraph.getIDToNodeMap(),
                                                        parentNode.getID(),
                                                        new RigidBodyTransform(), CEREAL_VISUAL_MODEL_FILE_PATH,
                                                        CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Cereal to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(cereal, parentNode));
   }

   public static void ensureMugNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode mug = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                          MUG_NAME,
                                                          sceneGraph.getIDToNodeMap(),
                                                          parentNode.getID(),
                                                          new RigidBodyTransform(), MUG_VISUAL_MODEL_FILE_PATH,
                                                          MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Mug to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(mug, parentNode));
   }

   public static void ensureBikeNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode bike = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                       BIKE_NAME,
                                                       sceneGraph.getIDToNodeMap(),
                                                       parentNode.getID(),
                                                       new RigidBodyTransform(), BIKE_VISUAL_MODEL_FILE_PATH,
                                                       BIKE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Bike to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(bike, parentNode));
   }

   public static void ensureDrillNodeAdded(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, SceneNode parentNode)
   {
      SceneNode drill = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                        DRILL_NAME,
                                                        sceneGraph.getIDToNodeMap(),
                                                        parentNode.getID(),
                                                        new RigidBodyTransform(), DRILL_VISUAL_MODEL_FILE_PATH,
                                                        DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      LogTools.info("Adding Drill to scene graph.");
      modificationQueue.accept(new SceneGraphNodeAddition(drill, parentNode));
   }
}
