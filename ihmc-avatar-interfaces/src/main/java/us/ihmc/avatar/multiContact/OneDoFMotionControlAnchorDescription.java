package us.ihmc.avatar.multiContact;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;

public class OneDoFMotionControlAnchorDescription
{
   public static final String ANCHOR_JSON = OneDoFMotionControlAnchorDescription.class.getSimpleName();
   public static final String JOINT_NAME_JSON = "jointName";
   public static final String IS_TRACKING_CONTROLLER_JSON = "isTrackingController";
   public static final String IK_SOLVER_MESSAGE_JSON = SixDoFMotionControlAnchorDescription.IK_SOLVER_MESSAGE_JSON;

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<KinematicsToolboxOneDoFJointMessage> messageSerializer = new JSONSerializer<>(new KinematicsToolboxOneDoFJointMessagePubSubType());

   public String jointName;
   private boolean isTrackingController;
   public KinematicsToolboxOneDoFJointMessage inputMessage;

   public OneDoFMotionControlAnchorDescription()
   {
   }

   public OneDoFMotionControlAnchorDescription(OneDoFMotionControlAnchorDescription other)
   {
      jointName = other.jointName;
      isTrackingController = other.isTrackingController;
      inputMessage = new KinematicsToolboxOneDoFJointMessage(other.inputMessage);
   }

   public static OneDoFMotionControlAnchorDescription fromJSON(JsonNode node)
   {
      JsonNode anchorNode = node.get(ANCHOR_JSON);

      try
      {
         OneDoFMotionControlAnchorDescription description = new OneDoFMotionControlAnchorDescription();
         description.setJointName(anchorNode.get(JOINT_NAME_JSON).asText());
         description.setTrackingController(anchorNode.get(IS_TRACKING_CONTROLLER_JSON).asBoolean());
         description.setInputMessage(messageSerializer.deserialize(anchorNode.get(IK_SOLVER_MESSAGE_JSON).toString()));
         return description;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public JsonNode toJSON(ObjectMapper objectMapper)
   {
      ObjectNode root = objectMapper.createObjectNode();
      ObjectNode anchorJSON = root.putObject(ANCHOR_JSON);

      anchorJSON.put(JOINT_NAME_JSON, jointName);
      anchorJSON.put(IS_TRACKING_CONTROLLER_JSON, isTrackingController);

      try
      {
         anchorJSON.set(IK_SOLVER_MESSAGE_JSON, messageToJSON(messageSerializer, inputMessage));

         return root;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private static <T> JsonNode messageToJSON(JSONSerializer<T> serializer, T message) throws IOException
   {
      return objectMapper.readTree(serializer.serializeToString(message));
   }

   public String getJointName()
   {
      return jointName;
   }

   public boolean isTrackingController()
   {
      return isTrackingController;
   }

   public KinematicsToolboxOneDoFJointMessage getInputMessage()
   {
      return inputMessage;
   }

   public void setJointName(String jointName)
   {
      this.jointName = jointName;
   }

   public void setTrackingController(boolean isTrackingController)
   {
      this.isTrackingController = isTrackingController;
   }

   public void setInputMessage(KinematicsToolboxOneDoFJointMessage inputMessage)
   {
      this.inputMessage = inputMessage;
   }

   @Override
   public String toString()
   {
      return String.format("[%s=%s, %s=%s]", JOINT_NAME_JSON, jointName, IK_SOLVER_MESSAGE_JSON, inputMessage);
   }
}
