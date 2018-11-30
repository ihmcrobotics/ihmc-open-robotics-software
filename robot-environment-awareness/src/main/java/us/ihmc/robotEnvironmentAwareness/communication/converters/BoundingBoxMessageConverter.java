package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.util.Scanner;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;

public class BoundingBoxMessageConverter
{

   public static BoxMessage convertToMessage(OcTreeBoundingBoxInterface boundingBox)
   {
      if (boundingBox == null)
         return BoxMessage.emptyBox();

      if (boundingBox instanceof OcTreeBoundingBoxWithCenterAndYaw)
         return convertToMessage((OcTreeBoundingBoxWithCenterAndYaw) boundingBox);
      else if (boundingBox instanceof OcTreeSimpleBoundingBox)
         return convertToMessage((OcTreeSimpleBoundingBox) boundingBox);
      else
         throw new RuntimeException("No conversion implemented for this bounding box: " + boundingBox.getClass().getSimpleName());
   }

   public static BoxMessage convertToMessage(OcTreeSimpleBoundingBox boundingBox)
   {
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      Point3D center = new Point3D();
      Vector3D size = new Vector3D();
      boundingBox.getSize(size);
      boundingBox.getCenterCoordinate(center);

      BoxMessage boxMessage = new BoxMessage();
      boxMessage.setCenter(center);
      boxMessage.setOrientation(orientation);
      boxMessage.setSize(size);
      return boxMessage;
   }

   public static BoxMessage convertToMessage(OcTreeBoundingBoxWithCenterAndYaw boundingBox)
   {
      Point3D center = new Point3D();
      Vector3D size = new Vector3D();
      boundingBox.getLocalSize(size);
      boundingBox.getCenterCoordinate(center);
      Quaternion orientation = new Quaternion(0.0, 0.0, Math.sin(0.5 * boundingBox.getYaw()), Math.cos(0.5 * boundingBox.getYaw()));

      BoxMessage boxMessage = new BoxMessage();
      boxMessage.setCenter(center);
      boxMessage.setOrientation(orientation);
      boxMessage.setSize(size);
      return boxMessage;
   }

   public static BoundingBoxParametersMessage createBoundingBoxParametersMessage(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
   {
      BoundingBoxParametersMessage message = new BoundingBoxParametersMessage();

      message.minX = minX;
      message.minY = minY;
      message.minZ = minZ;
      message.maxX = maxX;
      message.maxY = maxY;
      message.maxZ = maxZ;

      return message;
   }

   public static BoundingBoxParametersMessage parse(String boundingBoxAsString)
   {
      boundingBoxAsString = boundingBoxAsString.replace("(", "").replace(")", " ").replace(",", "");
      Scanner scanner = new Scanner(boundingBoxAsString);
      scanner.next();
      float minX = scanner.nextFloat();
      float minY = scanner.nextFloat();
      float minZ = scanner.nextFloat();
      scanner.next();
      float maxX = scanner.nextFloat();
      float maxY = scanner.nextFloat();
      float maxZ = scanner.nextFloat();
      scanner.close();
      return createBoundingBoxParametersMessage(minX, minY, minZ, maxX, maxY, maxZ);
   }
}
