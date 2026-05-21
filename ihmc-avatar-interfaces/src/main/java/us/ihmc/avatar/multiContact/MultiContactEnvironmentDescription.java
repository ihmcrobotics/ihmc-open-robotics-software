package us.ihmc.avatar.multiContact;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import ihmc_common_msgs.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.jros2.ROS2Message;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MultiContactEnvironmentDescription
{
   public static final String ENVIRONMENT_JSON = "environment";

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());

   public static JsonNode toJSON(FrameShape3DReadOnly environmentShape)
   {
      try
      {
         if (environmentShape instanceof Box3DReadOnly)
         {
            return messageToJSON(Box3DMessage.class.getSimpleName(), MessageTools.createBox3DMessage((Box3DReadOnly) environmentShape));
         }
         else if (environmentShape instanceof Capsule3DReadOnly)
         {
            return messageToJSON(Capsule3DMessage.class.getSimpleName(), MessageTools.createCapsule3DMessage((Capsule3DReadOnly) environmentShape));
         }
         else if (environmentShape instanceof ConvexPolytope3DReadOnly)
         {
            return messageToJSON(ConvexPolytope3DMessage.class.getSimpleName(),
                                 MessageTools.createConvexPolytope3DMessage((ConvexPolytope3DReadOnly) environmentShape));
         }
         else
         {
            throw new RuntimeException("Shape type not supported: " + environmentShape.getClass().getSimpleName());
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static FrameShape3DBasics fromJSON(JsonNode jsonNode)
   {
      try
      {
         ObjectNode environmentShape = (ObjectNode) jsonNode;
         String messageClassName = getMessageClassName(environmentShape);

         if (messageClassName.equals(Box3DMessage.class.getSimpleName()))
         {
            FrameBox3D box = new FrameBox3D(ReferenceFrame.getWorldFrame());
            Box3DMessage message = new Box3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackBox3DMessage(message, box);
            return box;
         }
         else if (messageClassName.equals(Capsule3DMessage.class.getSimpleName()))
         {
            FrameCapsule3D capsule = new FrameCapsule3D(ReferenceFrame.getWorldFrame());
            Capsule3DMessage message = new Capsule3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackCapsule3DMessage(message, capsule);
            return capsule;
         }
         else if (messageClassName.equals(ConvexPolytope3DMessage.class.getSimpleName()))
         {
            FrameConvexPolytope3D polytope = new FrameConvexPolytope3D(ReferenceFrame.getWorldFrame());
            ConvexPolytope3DMessage message = new ConvexPolytope3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackConvexPolytope3DMessage(message, polytope);
            return polytope;
         }
         else if (messageClassName.equals(Cylinder3DMessage.class.getSimpleName()))
         {
            FrameCylinder3D cylinder = new FrameCylinder3D(ReferenceFrame.getWorldFrame());
            Cylinder3DMessage message = new Cylinder3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackCylinder3DMessage(message, cylinder);
            return cylinder;
         }
         else if (messageClassName.equals(Ellipsoid3DMessage.class.getSimpleName()))
         {
            FrameEllipsoid3D ellipsoid = new FrameEllipsoid3D(ReferenceFrame.getWorldFrame());
            Ellipsoid3DMessage message = new Ellipsoid3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackEllipsoid3DMessage(message, ellipsoid);
            return ellipsoid;
         }
         else if (messageClassName.equals(Ramp3DMessage.class.getSimpleName()))
         {
            FrameRamp3D ramp = new FrameRamp3D(ReferenceFrame.getWorldFrame());
            Ramp3DMessage message = new Ramp3DMessage();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(environmentShape.get(messageClassName), message);
            MessageTools.unpackRamp3DMessage(message, ramp);
            return ramp;
         }
         else
         {
            throw new RuntimeException("Unknown shape message: " + messageClassName);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static <T extends ROS2Message<T>> JsonNode messageToJSON(String messageTypeName, T message) throws IOException
   {
      ObjectNode node = objectMapper.createObjectNode();
      node.set(messageTypeName, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, message));
      return node;
   }

   private static String getMessageClassName(ObjectNode jsonNode)
   {
      String field = jsonNode.fieldNames().next();
      if (!field.contains("::"))
         return field;

      String[] messagePath = field.split("::");
      String messageNameWithUnderbar = messagePath[messagePath.length - 1];
      return messageNameWithUnderbar.substring(0, messageNameWithUnderbar.length() - 1);
   }

   public static String serializeEnvironmentData(List<FrameShape3DReadOnly> environmentShapes)
   {
      ArrayNode arrayNode = objectMapper.createArrayNode();
      for (int i = 0; i < environmentShapes.size(); i++)
      {
         arrayNode.add(toJSON(environmentShapes.get(i)));
      }
      return arrayNode.toString();
   }

   public static List<FrameShape3DBasics> deserializeEnvironmentData(String serializedEnvironmentData)
   {
      try
      {
         ArrayNode arrayNode = (ArrayNode) objectMapper.readTree(serializedEnvironmentData);
         List<FrameShape3DBasics> environmentData = new ArrayList<>();
         for (int i = 0; i < arrayNode.size(); i++)
         {
            environmentData.add(fromJSON(arrayNode.get(i)));
         }

         return environmentData;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
