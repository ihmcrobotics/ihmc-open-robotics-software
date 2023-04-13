package us.ihmc.perception.objects;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.scene.ArUcoDetectableObject;

/**
 * Static methods to create boxes, cylinders, etc.
 */
public class BasicSceneObjects
{
   public static ArUcoDetectableObject createBox()
   {
      return new ArUcoDetectableObject("Box", 3, 0.2, new RigidBodyTransform());
   }

   /**
    * Represents a can of soup detected by a statically nearby placed ArUco
    * marker,
    */
   public static ArUcoDetectableObject createCanOfSoup()
   {
      return new ArUcoDetectableObject("CanOfSoup", 3, 0.2, new RigidBodyTransform());
   }
}
