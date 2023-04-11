package us.ihmc.perception.arUco;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.arUco.ArUcoMarkerInfo.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ArUcoMarkerInfoReadOnly extends StoredPropertySetReadOnly
{
   /**
    * ArUco marker ID in the ArUco dictionary
    */
   default double getMarkerID()
   {
      return get(markerID);
   }

   /**
    * ArUco marker side length size of the black outer part
    */
   default double getMarkerSize()
   {
      return get(markerSize);
   }

   /**
    * ArUco marker origin X translation to parent
    */
   default double getMarkerXTranslationToParent()
   {
      return get(markerXTranslationToParent);
   }

   /**
    * ArUco marker origin Y translation to parent
    */
   default double getMarkerYTranslationToParent()
   {
      return get(markerYTranslationToParent);
   }

   /**
    * ArUco marker origin Z translation to parent
    */
   default double getMarkerZTranslationToParent()
   {
      return get(markerZTranslationToParent);
   }

   /**
    * ArUco marker origin yaw rotation to parent
    */
   default double getMarkerYawRotationToParent()
   {
      return get(markerYawRotationToParent);
   }

   /**
    * ArUco marker origin pitch rotation to parent
    */
   default double getMarkerPitchRotationToParent()
   {
      return get(markerPitchRotationToParent);
   }

   /**
    * ArUco marker origin roll rotation to parent
    */
   default double getMarkerRollRotationToParent()
   {
      return get(markerRollRotationToParent);
   }
}
