package us.ihmc.perception.parameters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IntrinsicCameraMatrixPropertiesBasics extends IntrinsicCameraMatrixPropertiesReadOnly, StoredPropertySetBasics
{
   /**
    * Focal length X
    */
   default void setFocalLengthX(double focalLengthX)
   {
      set(IntrinsicCameraMatrixProperties.focalLengthX, focalLengthX);
   }

   /**
    * Focal length Y
    */
   default void setFocalLengthY(double focalLengthY)
   {
      set(IntrinsicCameraMatrixProperties.focalLengthY, focalLengthY);
   }

   /**
    * Principle point X
    */
   default void setPrinciplePointX(double principlePointX)
   {
      set(IntrinsicCameraMatrixProperties.principlePointX, principlePointX);
   }

   /**
    * Principle point Y
    */
   default void setPrinciplePointY(double principlePointY)
   {
      set(IntrinsicCameraMatrixProperties.principlePointY, principlePointY);
   }
}
