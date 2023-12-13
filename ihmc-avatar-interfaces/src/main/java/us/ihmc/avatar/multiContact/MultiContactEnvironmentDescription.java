package us.ihmc.avatar.multiContact;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import ihmc_common_msgs.msg.dds.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.robotics.geometry.Capsule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MultiContactEnvironmentDescription
{
   public static final String ENVIRONMENT_JSON = "environment";

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<Box3DMessage> boxSerializer = new JSONSerializer<>(new Box3DMessagePubSubType());
   private static final JSONSerializer<Capsule3DMessage> capsuleSerializer = new JSONSerializer<>(new Capsule3DMessagePubSubType());
   private static final JSONSerializer<ConvexPolytope3DMessage> polytopeSerializer = new JSONSerializer<>(new ConvexPolytope3DMessagePubSubType());
   private static final JSONSerializer<Cylinder3DMessage> cylinderSerializer = new JSONSerializer<>(new Cylinder3DMessagePubSubType());
   private static final JSONSerializer<Ellipsoid3DMessage> ellipsoidSerializer = new JSONSerializer<>(new Ellipsoid3DMessagePubSubType());
   private static final JSONSerializer<Ramp3DMessage> rampSerializer = new JSONSerializer<>(new Ramp3DMessagePubSubType());

   public static JsonNode toJSON(FrameShape3DReadOnly environmentShape)
   {
      try
      {
         if (environmentShape instanceof Box3DReadOnly)
         {
            return messageToJSON(boxSerializer, MessageTools.createBox3DMessage((Box3DReadOnly) environmentShape));
         }
         else if (environmentShape instanceof Capsule3DMessage)
         {
            return messageToJSON(capsuleSerializer, MessageTools.createCapsule3DMessage((Capsule3DReadOnly) environmentShape));
         }
         else if (environmentShape instanceof ConvexPolytope3DReadOnly)
         {
            return messageToJSON(polytopeSerializer, MessageTools.createConvexPolytope3DMessage((ConvexPolytope3DReadOnly) environmentShape));
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
            MessageTools.unpackBox3DMessage(boxSerializer.deserialize(environmentShape.toString()), box);
            return box;
         }
         else if (messageClassName.equals(Capsule3DMessage.class.getSimpleName()))
         {
            FrameCapsule3D capsule = new FrameCapsule3D(ReferenceFrame.getWorldFrame());
            MessageTools.unpackCapsule3DMessage(capsuleSerializer.deserialize(environmentShape.toString()), capsule);
            return capsule;
         }
         else if (messageClassName.equals(ConvexPolytope3DMessage.class.getSimpleName()))
         {
            FrameConvexPolytope3D polytope = new FrameConvexPolytope3D(ReferenceFrame.getWorldFrame());
            MessageTools.unpackConvexPolytope3DMessage(polytopeSerializer.deserialize(environmentShape.toString()), polytope);
            return polytope;
         }
         else if (messageClassName.equals(Cylinder3DMessage.class.getSimpleName()))
         {
            FrameCylinder3D cylinder = new FrameCylinder3D(ReferenceFrame.getWorldFrame());
            MessageTools.unpackCylinder3DMessage(cylinderSerializer.deserialize(environmentShape.toString()), cylinder);
            return cylinder;
         }
         else if (messageClassName.equals(Ellipsoid3DMessage.class.getSimpleName()))
         {
            FrameEllipsoid3D ellipsoid = new FrameEllipsoid3D(ReferenceFrame.getWorldFrame());
            MessageTools.unpackEllipsoid3DMessage(ellipsoidSerializer.deserialize(environmentShape.toString()), ellipsoid);
            return ellipsoid;
         }
         else if (messageClassName.equals(Ramp3DMessage.class.getSimpleName()))
         {
            FrameRamp3D ramp = new FrameRamp3D(ReferenceFrame.getWorldFrame());
            MessageTools.unpackRamp3DMessage(rampSerializer.deserialize(environmentShape.toString()), ramp);
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

   private static <T> JsonNode messageToJSON(JSONSerializer<T> serializer, T message) throws IOException
   {
      return objectMapper.readTree(serializer.serializeToString(message));
   }

   private static FrameShape3DReadOnly stringToShape(String serializedMessage) throws IOException
   {
      JsonNode jsonNode = objectMapper.readTree(serializedMessage);
      return fromJSON(jsonNode);
   }

   private static String getMessageClassName(ObjectNode jsonNode)
   {
      String field = jsonNode.fieldNames().next();
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
