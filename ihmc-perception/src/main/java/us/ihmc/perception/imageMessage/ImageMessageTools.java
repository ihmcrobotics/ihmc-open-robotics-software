package us.ihmc.perception.imageMessage;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.PixelFormat;
import us.ihmc.perception.RawImage;

public final class ImageMessageTools
{
   public static void setCameraModel(CameraModel cameraModel, ImageMessage messageToPack)
   {
      messageToPack.setCameraModel((byte) cameraModel.ordinal());
   }

   public static CameraModel getCameraModel(ImageMessage imageMessage)
   {
      for (CameraModel cameraModel : CameraModel.values)
      {
         if (imageMessage.getCameraModel() == cameraModel.ordinal())
         {
            return cameraModel;
         }
      }

      throw new RuntimeException("Missing format " + imageMessage.getCameraModel());
   }

   public static PixelFormat getPixelFormat(ImageMessage imageMessage)
   {
      return PixelFormat.fromByte(imageMessage.getPixelFormat());
   }

   public static void packImageMessageMetaData(RawImage rawImage, ImageMessage messageToPack)
   {
      messageToPack.setPixelFormat(rawImage.getPixelFormat().toByte());
      messageToPack.setImageWidth(rawImage.getWidth());
      messageToPack.setImageHeight(rawImage.getHeight());
      messageToPack.setFocalLengthXPixels(rawImage.getFocalLengthX());
      messageToPack.setFocalLengthYPixels(rawImage.getFocalLengthY());
      messageToPack.setPrincipalPointXPixels(rawImage.getPrincipalPointX());
      messageToPack.setPrincipalPointYPixels(rawImage.getPrincipalPointY());
      messageToPack.setCameraModel(getCameraModel(messageToPack).toByte());
      messageToPack.setDepthDiscretization(rawImage.getDepthDiscretization());
      messageToPack.setSequenceNumber(rawImage.getSequenceNumber());
      MessageTools.toMessage(rawImage.getAcquisitionTime(), messageToPack.getAcquisitionTime());
      messageToPack.getPosition().set(rawImage.getPosition());
      messageToPack.getOrientation().set(rawImage.getOrientation());
   }
}
