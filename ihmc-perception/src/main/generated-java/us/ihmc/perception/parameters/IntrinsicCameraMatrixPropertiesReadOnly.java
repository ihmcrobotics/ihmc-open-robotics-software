package us.ihmc.perception.parameters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IntrinsicCameraMatrixPropertiesReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Focal length X
    */
   default double getFocalLengthX()
   {
      return get(focalLengthX);
   }

   /**
    * Focal length Y
    */
   default double getFocalLengthY()
   {
      return get(focalLengthY);
   }

   /**
    * Principle point X
    */
   default double getPrinciplePointX()
   {
      return get(principlePointX);
   }

   /**
    * Principle point Y
    */
   default double getPrinciplePointY()
   {
      return get(principlePointY);
   }
}
