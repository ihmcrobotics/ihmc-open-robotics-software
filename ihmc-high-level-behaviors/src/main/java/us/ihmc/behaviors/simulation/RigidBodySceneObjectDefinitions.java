package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class RigidBodySceneObjectDefinitions
{
   public static final String BOX_NAME = "Box";
   // The box is a cube
   public static final double BOX_WIDTH = 0.394;
   public static final double BOX_DEPTH = 0.31;
   public static final double BOX_HEIGHT = 0.265;

   public static final String BOX_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/emptyBox.g3dj";
   public static final RigidBodyTransform BOX_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String CAN_OF_SOUP_NAME = "CanOfSoup";
   public static final double CAN_OF_SOUP_RADIUS = 0.0329375;
   public static final double CAN_OF_SOUP_HEIGHT = 0.082388;
   public static final String CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH = "environmentObjects/canOfSoup/CanOfSoup.g3dj";
   public static final RigidBodyTransform CAN_OF_SOUP_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final double CAN_OF_SOUP_FROM_TABLE_EDGE = 0.03;

   public static final String DEBRIS_NAME = "2x4Debris";
   public static final String DEBRIS_VISUAL_MODEL_FILE_PATH = "environmentObjects/debris/2x4.g3dj";
   public static final RigidBodyTransform DEBRIS_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String PLATFORM_NAME = "WorkPlatform";
   public static final String PLATFORM_VISUAL_MODEL_FILE_PATH = "environmentObjects/workPlatform/workPlatform.g3dj";
   public static final RigidBodyTransform PLATFORM_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

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

   public static final String CEREAL_NAME = "Cereal";
   public static final String CEREAL_VISUAL_MODEL_FILE_PATH = "environmentObjects/cereal/cereal.g3dj";
   public static final RigidBodyTransform CEREAL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String MUG_NAME = "Mug";
   public static final String MUG_VISUAL_MODEL_FILE_PATH = "environmentObjects/mug/mug.g3dj";
   public static final RigidBodyTransform MUG_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String BIKE_NAME = "Bike";
   public static final String BIKE_VISUAL_MODEL_FILE_PATH = "environmentObjects/bike/bike.g3dj";
   public static final RigidBodyTransform BIKE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String DRILL_NAME = "Drill";
   public static final String DRILL_VISUAL_MODEL_FILE_PATH = "environmentObjects/drill/drill.g3dj";
   public static final RigidBodyTransform DRILL_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String COUCH_NAME = "Couch";
   public static final String COUCH_VISUAL_MODEL_FILE_PATH = "environmentObjects/couch/Couch.g3dj";
   public static final RigidBodyTransform COUCH_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String TRASHCAN_NAME = "TrashCan";
   public static final String TRASHCAN_VISUAL_MODEL_FILE_PATH = "environmentObjects/trashCan/TrashCan.g3dj";
   public static final RigidBodyTransform TRASHCAN_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String CHARGE_NAME = "Charge";
   public static final String CHARGE_VISUAL_MODEL_FILE_PATH = "environmentObjects/charge/charge.g3dj";
   public static final RigidBodyTransform CHARGE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String PERSON_NAME = "Person";
   public static final String PERSON_VISUAL_MODEL_FILE_PATH = "environmentObjects/person/person.glb";
   public static final RigidBodyTransform PERSON_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String BARRIER_NAME = "Barrier";
   public static final String BARRIER_VISUAL_MODEL_FILE_PATH = "environmentObjects/barrier/barrier.g3dj";
   public static final RigidBodyTransform BARRIER_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   /** Matches the {@code traffic_barrier} YOLO class and the FoundationPose mesh of the same name. */
   public static final String TRAFFIC_BARRIER_NAME = "Traffic Barrier";
   public static final String TRAFFIC_BARRIER_VISUAL_MODEL_FILE_PATH = "environmentObjects/traffic_barrier/traffic_barrier.glb";
   public static final RigidBodyTransform TRAFFIC_BARRIER_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();

   public static final String TABLE_NAME = "Table";
   public static final String TABLE_VISUAL_MODEL_FILE_PATH = "environmentObjects/table/Table.g3dj";
   public static final RigidBodyTransform TABLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM = new RigidBodyTransform();
}
