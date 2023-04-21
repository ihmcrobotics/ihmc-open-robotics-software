package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableObject;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class BasicSceneObjects
{
   public static final double MARKER_WIDTH = 0.1680;

   public static final int BOX_MARKER_ID = 2;
   public static final double BOX_MARKER_WIDTH = 0.210101;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), Math.toRadians(0.0), Math.toRadians(-90.0)),
         new Point3D(0.07, 0.15, 0.17)
   );
   public static final String BOX_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/box.g3dj";

   // TODO: Get soup can model from Arghya
   public static final String CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH = "environmentObjects/box/box.g3dj";

   public static ArUcoDetectableObject createBox()
   {
      return new ArUcoDetectableObject("Box", BOX_MARKER_ID, BOX_MARKER_WIDTH, BOX_TRANSFORM_TO_MARKER, BOX_VISUAL_MODEL_FILE_PATH);
   }

   /**
    * Represents a can of soup detected by a statically nearby placed ArUco
    * marker,
    */
   public static ArUcoDetectableObject createCanOfSoup()
   {
      return new ArUcoDetectableObject("CanOfSoup", 4, MARKER_WIDTH, new RigidBodyTransform(), CAN_OF_SOUP_VISUAL_MODEL_FILE_PATH);
   }
}
