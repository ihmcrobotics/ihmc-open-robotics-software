package us.ihmc.avatar.multiContact;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import toolbox_msgs.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.euclid.transform.interfaces.Transform;

import java.io.IOException;

public class CenterOfMassMotionControlAnchorDescription
{
   public static final String ANCHOR_JSON = CenterOfMassMotionControlAnchorDescription.class.getSimpleName();
   public static final String IS_TRACKING_CONTROLLER_JSON = "isTrackingController";
   public static final String IK_SOLVER_MESSAGE_JSON = "ikSolverMessage";

   private boolean isTrackingController;
   private KinematicsToolboxCenterOfMassMessage inputMessage;

   public CenterOfMassMotionControlAnchorDescription()
   {
   }

   public CenterOfMassMotionControlAnchorDescription(CenterOfMassMotionControlAnchorDescription other)
   {
      isTrackingController = other.isTrackingController;
      inputMessage = new KinematicsToolboxCenterOfMassMessage(other.inputMessage);
   }

   public static CenterOfMassMotionControlAnchorDescription fromJSON(JsonNode node)
   {
      if (node == null)
      {
         CenterOfMassMotionControlAnchorDescription anchorDescription = new CenterOfMassMotionControlAnchorDescription();
         KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
         comMessage.getSelectionMatrix().setXSelected(false);
         comMessage.getSelectionMatrix().setYSelected(false);
         comMessage.getSelectionMatrix().setZSelected(false);
         comMessage.getWeights().setXWeight(-1.0);
         comMessage.getWeights().setYWeight(-1.0);
         comMessage.getWeights().setZWeight(-1.0);
         anchorDescription.setInputMessage(comMessage);
         return anchorDescription;
      }

      JsonNode anchorNode = node.get(ANCHOR_JSON);

      try
      {
         CenterOfMassMotionControlAnchorDescription description = new CenterOfMassMotionControlAnchorDescription();
         description.setTrackingController(anchorNode.get(IS_TRACKING_CONTROLLER_JSON).asBoolean());
         KinematicsToolboxCenterOfMassMessage inputMessage = new KinematicsToolboxCenterOfMassMessage();
         ROS2MessageCdrFileTools.deserializeFromJsonNode(anchorNode.get(IK_SOLVER_MESSAGE_JSON), inputMessage);
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

      anchorJSON.put(IS_TRACKING_CONTROLLER_JSON, isTrackingController);

      anchorJSON.set(IK_SOLVER_MESSAGE_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, inputMessage));
      return root;
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
      inputMessage.getDesiredPositionInWorld().getPoint().applyTransform(transform);
   }

   @Override
   public String toString()
   {
      return String.format("[%s=%s, %s=%s]", IS_TRACKING_CONTROLLER_JSON, isTrackingController, IK_SOLVER_MESSAGE_JSON, inputMessage);
   }
}
