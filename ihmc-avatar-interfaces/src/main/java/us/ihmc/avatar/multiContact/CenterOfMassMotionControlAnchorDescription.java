package us.ihmc.avatar.multiContact;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.idl.serializers.extra.JSONSerializer;

public class CenterOfMassMotionControlAnchorDescription
{
   public static final String ANCHOR_JSON = CenterOfMassMotionControlAnchorDescription.class.getSimpleName();
   public static final String IS_TRACKING_CONTROLLER_JSON = "isTrackingController";
   public static final String IK_SOLVER_MESSAGE_JSON = "ikSolverMessage";

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<KinematicsToolboxCenterOfMassMessage> messageSerializer = new JSONSerializer<>(new KinematicsToolboxCenterOfMassMessagePubSubType());

   private boolean isTrackingController;
   private KinematicsToolboxCenterOfMassMessage inputMessage;

   public CenterOfMassMotionControlAnchorDescription()
   {
      inputMessage = new KinematicsToolboxCenterOfMassMessage();
      inputMessage.getSelectionMatrix().setXSelected(false);
      inputMessage.getSelectionMatrix().setYSelected(false);
      inputMessage.getSelectionMatrix().setZSelected(false);
   }

   public CenterOfMassMotionControlAnchorDescription(CenterOfMassMotionControlAnchorDescription other)
   {
      isTrackingController = other.isTrackingController;
      inputMessage = new KinematicsToolboxCenterOfMassMessage(other.inputMessage);
   }

   public static CenterOfMassMotionControlAnchorDescription fromJSON(JsonNode node)
   {
      if (node == null)
         return new CenterOfMassMotionControlAnchorDescription();

      JsonNode anchorNode = node.get(ANCHOR_JSON);

      try
      {
         CenterOfMassMotionControlAnchorDescription description = new CenterOfMassMotionControlAnchorDescription();
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

   public boolean isTrackingController()
   {
      return isTrackingController;
   }

   public KinematicsToolboxCenterOfMassMessage getInputMessage()
   {
      return inputMessage;
   }

   public void setTrackingController(boolean isTrackingController)
   {
      this.isTrackingController = isTrackingController;
   }

   public void setInputMessage(KinematicsToolboxCenterOfMassMessage inputMessage)
   {
      this.inputMessage = inputMessage;
   }

   public void applyTransform(Transform transform)
   {
      inputMessage.getDesiredPositionInWorld().applyTransform(transform);
   }

   @Override
   public String toString()
   {
      return String.format("[%s=%s, %s=%s]", IS_TRACKING_CONTROLLER_JSON, isTrackingController, IK_SOLVER_MESSAGE_JSON, inputMessage);
   }
}
