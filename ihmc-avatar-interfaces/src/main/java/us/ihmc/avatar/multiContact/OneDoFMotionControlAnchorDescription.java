package us.ihmc.avatar.multiContact;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import toolbox_msgs.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.communication.serialization.Ros2MessageCdrFileTools;
import us.ihmc.jros2.ROS2Message;

public class OneDoFMotionControlAnchorDescription
{
   public static final String ANCHOR_JSON = OneDoFMotionControlAnchorDescription.class.getSimpleName();
   public static final String JOINT_NAME_JSON = "jointName";
   public static final String IS_TRACKING_CONTROLLER_JSON = "isTrackingController";
   public static final String IK_SOLVER_MESSAGE_JSON = SixDoFMotionControlAnchorDescription.IK_SOLVER_MESSAGE_JSON;

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());

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
      inputMessage = ROS2Message.createInstance(KinematicsToolboxOneDoFJointMessage.class);
      inputMessage.set(other.inputMessage);
   }

   public static OneDoFMotionControlAnchorDescription fromJSON(JsonNode node)
   {
      JsonNode anchorNode = node.get(ANCHOR_JSON);

      try
      {
         OneDoFMotionControlAnchorDescription description = new OneDoFMotionControlAnchorDescription();
         description.setJointName(anchorNode.get(JOINT_NAME_JSON).asText());
         description.setTrackingController(anchorNode.get(IS_TRACKING_CONTROLLER_JSON).asBoolean());
         KinematicsToolboxOneDoFJointMessage inputMessage = ROS2Message.createInstance(KinematicsToolboxOneDoFJointMessage.class);
         Ros2MessageCdrFileTools.deserializeFromJsonNode(anchorNode.get(IK_SOLVER_MESSAGE_JSON), inputMessage);
         description.setInputMessage(inputMessage);
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

      anchorJSON.set(IK_SOLVER_MESSAGE_JSON, Ros2MessageCdrFileTools.messageToJsonNode(objectMapper, inputMessage));
      return root;
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
