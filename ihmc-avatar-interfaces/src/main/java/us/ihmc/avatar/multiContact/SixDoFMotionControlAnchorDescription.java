package us.ihmc.avatar.multiContact;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.idl.serializers.extra.JSONSerializer;

public class SixDoFMotionControlAnchorDescription
{
   public static final String ANCHOR_JSON = SixDoFMotionControlAnchorDescription.class.getSimpleName();
   public static final String RIGID_BODY_NAME_JSON = "rigidBodyName";
   public static final String IS_CONTACT_STATE_JSON = "isContactState";
   public static final String IS_TRACKING_CONTROLLER_JSON = "isTrackingController";
   public static final String ANCHOR_ID = "anchorId";
   public static final String IK_SOLVER_MESSAGE_JSON = "ikSolverMessage";

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<KinematicsToolboxRigidBodyMessage> messageSerializer = new JSONSerializer<>(new KinematicsToolboxRigidBodyMessagePubSubType());

   private String rigidBodyName;
   private boolean isContactState;
   private boolean isTrackingController;
   private int anchorId = -1;
   private KinematicsToolboxRigidBodyMessage inputMessage;

   public SixDoFMotionControlAnchorDescription()
   {
   }

   public SixDoFMotionControlAnchorDescription(SixDoFMotionControlAnchorDescription other)
   {
      rigidBodyName = other.rigidBodyName;
      isContactState = other.isContactState;
      isTrackingController = other.isTrackingController;
      anchorId = other.anchorId;
      inputMessage = new KinematicsToolboxRigidBodyMessage(other.inputMessage);
   }

   public static SixDoFMotionControlAnchorDescription fromJSON(JsonNode node)
   {
      JsonNode anchorNode = node.get(ANCHOR_JSON);

      try
      {
         SixDoFMotionControlAnchorDescription description = new SixDoFMotionControlAnchorDescription();
         description.setRigidBodyName(anchorNode.get(RIGID_BODY_NAME_JSON).asText());
         description.setContactState(anchorNode.get(IS_CONTACT_STATE_JSON).asBoolean());
         description.setTrackingController(anchorNode.get(IS_TRACKING_CONTROLLER_JSON).asBoolean());
         description.setInputMessage(messageSerializer.deserialize(anchorNode.get(IK_SOLVER_MESSAGE_JSON).toString()));

         if (anchorNode.has(ANCHOR_ID))
            description.setAnchorId(anchorNode.get(ANCHOR_ID).asInt());

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

      anchorJSON.put(RIGID_BODY_NAME_JSON, rigidBodyName);
      anchorJSON.put(IS_CONTACT_STATE_JSON, isContactState);
      anchorJSON.put(IS_TRACKING_CONTROLLER_JSON, isTrackingController);
      anchorJSON.put(ANCHOR_ID, anchorId);

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

   public String getRigidBodyName()
   {
      return rigidBodyName;
   }

   public boolean isContactState()
   {
      return isContactState;
   }

   public boolean isTrackingController()
   {
      return isTrackingController;
   }

   public int getAnchorId()
   {
      return anchorId;
   }

   public KinematicsToolboxRigidBodyMessage getInputMessage()
   {
      return inputMessage;
   }

   public void setRigidBodyName(String rigidBodyName)
   {
      this.rigidBodyName = rigidBodyName;
   }

   public void setContactState(boolean isContactState)
   {
      this.isContactState = isContactState;
   }

   public void setTrackingController(boolean isTrackingController)
   {
      this.isTrackingController = isTrackingController;
   }

   public void setAnchorId(int anchorId)
   {
      this.anchorId = anchorId;
   }

   public void setInputMessage(KinematicsToolboxRigidBodyMessage inputMessage)
   {
      this.inputMessage = inputMessage;
   }

   public void applyTransform(Transform transform)
   {
      inputMessage.getDesiredPositionInWorld().applyTransform(transform);
      inputMessage.getDesiredOrientationInWorld().applyTransform(transform);
   }

   @Override
   public String toString()
   {
      return String.format("[%s=%s, %s=%s, %s=%s, %s=%s]",
                           RIGID_BODY_NAME_JSON,
                           rigidBodyName,
                           IS_CONTACT_STATE_JSON,
                           isContactState,
                           IS_TRACKING_CONTROLLER_JSON,
                           isTrackingController,
                           IK_SOLVER_MESSAGE_JSON,
                           inputMessage);
   }
}
