package us.ihmc.perception;

import perception_msgs.msg.dds.ImageMessage;

/**
 * Used with the ImageMessage camera model field.
 */
public enum CameraModel
{
   /**
    * Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model)
    */
   PINHOLE,
   /**
    * If the camera model is the Ouster camera model.
    * With the Ouster camera model, the vertical and horizontal fields of view
    * are used instead of the focal length and principal point parameters.
    */
   OUSTER,
   /**
    * Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
    */
   EQUIDISTANT_FISHEYE,
   /**
    * Used for height maps.
    */
   ORTHOGRAPHIC;

   public static final CameraModel[] values = values();

   public void packMessageFormat(ImageMessage imageMessage)
   {
      imageMessage.setCameraModel((byte) ordinal());
   }

   public static CameraModel getCameraModel(ImageMessage imageMessage)
   {
      for (CameraModel cameraModel : values)
      {
         if (imageMessage.getCameraModel() == cameraModel.ordinal())
         {
            return cameraModel;
         }
      }

      throw new RuntimeException("Missing format " + imageMessage.getFormat());
   }
}
