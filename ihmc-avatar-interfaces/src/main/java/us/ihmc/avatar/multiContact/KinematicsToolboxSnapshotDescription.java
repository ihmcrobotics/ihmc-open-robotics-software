package us.ihmc.avatar.multiContact;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.RobotConfigurationData;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import toolbox_msgs.KinematicsToolboxPrivilegedConfigurationMessage;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.euclid.transform.interfaces.Transform;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class KinematicsToolboxSnapshotDescription
{
   public static final String SCRIPT_JSON = "script";
   public static final String CONFIGURATION_JSON = KinematicsToolboxSnapshotDescription.class.getSimpleName();
   public static final String CONTROLLER_CONFIGURATION_JSON = "controllerConfiguration";
   public static final String IK_SOLUTION_JSON = "ikSolution";
   public static final String IK_PRIVILEGED_CONFIGURATION_JSON = "ikPrivilegedConfiguration";
   public static final String COM_ANCHOR_JSON = "centerOfMassAnchor";
   public static final String SIX_DOF_ANCHORS_JSON = "sixDoFAnchors";
   public static final String ONE_DOF_ANCHORS_JSON = "oneDoFAnchors";
   public static final String EXECUTION_DURATION_JSON = "executionDuration";

   public double executionDuration = Double.NaN;
   public RobotConfigurationData controllerConfiguration;
   public KinematicsToolboxOutputStatus ikSolution;
   public KinematicsToolboxPrivilegedConfigurationMessage ikPrivilegedConfiguration;
   public CenterOfMassMotionControlAnchorDescription centerOfMassAnchor;
   public List<SixDoFMotionControlAnchorDescription> sixDoFAnchors;
   public List<OneDoFMotionControlAnchorDescription> oneDoFAnchors;

   public KinematicsToolboxSnapshotDescription()
   {
   }

   public KinematicsToolboxSnapshotDescription(KinematicsToolboxSnapshotDescription other)
   {
      executionDuration = other.executionDuration;
      controllerConfiguration = copyRobotConfigurationData(other.controllerConfiguration);
      ikSolution = copyIkSolution(other.ikSolution);
      ikPrivilegedConfiguration = copyIkPrivilegedConfiguration(other.ikPrivilegedConfiguration);
      sixDoFAnchors = other.sixDoFAnchors.stream().map(SixDoFMotionControlAnchorDescription::new).collect(Collectors.toList());
      oneDoFAnchors = other.oneDoFAnchors.stream().map(OneDoFMotionControlAnchorDescription::new).collect(Collectors.toList());

      if (other.centerOfMassAnchor == null)
      {
         centerOfMassAnchor = new CenterOfMassMotionControlAnchorDescription();
      }
      else
      {
         centerOfMassAnchor = new CenterOfMassMotionControlAnchorDescription(other.centerOfMassAnchor);
      }
   }

   private static RobotConfigurationData copyRobotConfigurationData(RobotConfigurationData source)
   {
      return source == null ? null : new RobotConfigurationData(source);
   }

   private static KinematicsToolboxOutputStatus copyIkSolution(KinematicsToolboxOutputStatus source)
   {
      return source == null ? null : new KinematicsToolboxOutputStatus(source);
   }

   private static KinematicsToolboxPrivilegedConfigurationMessage copyIkPrivilegedConfiguration(KinematicsToolboxPrivilegedConfigurationMessage source)
   {
      return source == null ? null : new KinematicsToolboxPrivilegedConfigurationMessage(source);
   }

   public static KinematicsToolboxSnapshotDescription fromJSON(JsonNode node)
   {
      JsonNode configurationNode = node.get(CONFIGURATION_JSON);

      try
      {
         KinematicsToolboxSnapshotDescription description = new KinematicsToolboxSnapshotDescription();
         description.setControllerConfiguration(deserializeRobotConfigurationData(configurationNode.get(CONTROLLER_CONFIGURATION_JSON)));
         description.setIkSolution(deserializeIkSolution(configurationNode.get(IK_SOLUTION_JSON)));
         description.setIkPrivilegedConfiguration(deserializeIkPrivilegedConfiguration(configurationNode.get(IK_PRIVILEGED_CONFIGURATION_JSON)));

         description.setCenterOfMassAnchor(CenterOfMassMotionControlAnchorDescription.fromJSON(configurationNode.get(COM_ANCHOR_JSON)));

         JsonNode sixDoFAnchorsNode = configurationNode.get(SIX_DOF_ANCHORS_JSON);
         ArrayList<SixDoFMotionControlAnchorDescription> sixDoFAnchors = new ArrayList<>();
         for (int i = 0; i < sixDoFAnchorsNode.size(); i++)
            sixDoFAnchors.add(SixDoFMotionControlAnchorDescription.fromJSON(sixDoFAnchorsNode.get(i)));
         description.setSixDoFAnchors(sixDoFAnchors);

         JsonNode oneDoFAnchorsNode = configurationNode.get(ONE_DOF_ANCHORS_JSON);
         ArrayList<OneDoFMotionControlAnchorDescription> oneDoFAnchors = new ArrayList<>();
         for (int i = 0; i < oneDoFAnchorsNode.size(); i++)
            oneDoFAnchors.add(OneDoFMotionControlAnchorDescription.fromJSON(oneDoFAnchorsNode.get(i)));
         description.setOneDoFAnchors(oneDoFAnchors);

         JsonNode executionDurationNode = configurationNode.get(EXECUTION_DURATION_JSON);
         if (executionDurationNode != null)
         {
            description.setExecutionDuration(executionDurationNode.asDouble());
         }

         return description;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static RobotConfigurationData deserializeRobotConfigurationData(JsonNode node) throws IOException
   {
      if (node == null || node.isNull())
         return null;
      RobotConfigurationData message = new RobotConfigurationData();
      ROS2MessageCdrFileTools.deserializeFromJsonNode(node, message);
      return message;
   }

   private static KinematicsToolboxOutputStatus deserializeIkSolution(JsonNode node) throws IOException
   {
      if (node == null || node.isNull())
         return null;
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      ROS2MessageCdrFileTools.deserializeFromJsonNode(node, message);
      return message;
   }

   private static KinematicsToolboxPrivilegedConfigurationMessage deserializeIkPrivilegedConfiguration(JsonNode node) throws IOException
   {
      if (node == null || node.isNull())
         return null;
      KinematicsToolboxPrivilegedConfigurationMessage message = new KinematicsToolboxPrivilegedConfigurationMessage();
      ROS2MessageCdrFileTools.deserializeFromJsonNode(node, message);
      return message;
   }

   public JsonNode toJSON(ObjectMapper objectMapper)
   {
      Objects.requireNonNull(controllerConfiguration);
      Objects.requireNonNull(ikSolution);
      Objects.requireNonNull(ikPrivilegedConfiguration);
      Objects.requireNonNull(sixDoFAnchors);
      Objects.requireNonNull(oneDoFAnchors);

      ObjectNode root = objectMapper.createObjectNode();
      ObjectNode configurationJSON = root.putObject(CONFIGURATION_JSON);

      configurationJSON.set(CONTROLLER_CONFIGURATION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, controllerConfiguration));
      configurationJSON.set(IK_SOLUTION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, ikSolution));
      configurationJSON.set(IK_PRIVILEGED_CONFIGURATION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, ikPrivilegedConfiguration));
      if (centerOfMassAnchor != null)
         configurationJSON.set(COM_ANCHOR_JSON, centerOfMassAnchor.toJSON(objectMapper));
      ArrayNode arraySixDoFAnchorNode = configurationJSON.arrayNode(sixDoFAnchors.size());
      sixDoFAnchors.forEach(anchor -> arraySixDoFAnchorNode.add(anchor.toJSON(objectMapper)));
      configurationJSON.set(SIX_DOF_ANCHORS_JSON, arraySixDoFAnchorNode);
      ArrayNode arrayOneDoFAnchorNode = configurationJSON.arrayNode(oneDoFAnchors.size());
      oneDoFAnchors.forEach(anchor -> arrayOneDoFAnchorNode.add(anchor.toJSON(objectMapper)));
      configurationJSON.set(ONE_DOF_ANCHORS_JSON, arrayOneDoFAnchorNode);
      configurationJSON.put(EXECUTION_DURATION_JSON, executionDuration);

      return root;
   }

   public JsonNode toJSONNew(ObjectMapper objectMapper)
   {
      Objects.requireNonNull(controllerConfiguration);
      Objects.requireNonNull(ikSolution);
      Objects.requireNonNull(ikPrivilegedConfiguration);
      Objects.requireNonNull(sixDoFAnchors);
      Objects.requireNonNull(oneDoFAnchors);

      ObjectNode root = objectMapper.createObjectNode();

      root.set(CONTROLLER_CONFIGURATION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, controllerConfiguration));
      root.set(IK_SOLUTION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, ikSolution));
      root.set(IK_PRIVILEGED_CONFIGURATION_JSON, ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, ikPrivilegedConfiguration));
      if (centerOfMassAnchor != null)
         root.set(COM_ANCHOR_JSON, centerOfMassAnchor.toJSON(objectMapper));
      ArrayNode arraySixDoFAnchorNode = root.arrayNode(sixDoFAnchors.size());
      sixDoFAnchors.forEach(anchor -> arraySixDoFAnchorNode.add(anchor.toJSON(objectMapper)));
      root.set(SIX_DOF_ANCHORS_JSON, arraySixDoFAnchorNode);
      ArrayNode arrayOneDoFAnchorNode = root.arrayNode(oneDoFAnchors.size());
      oneDoFAnchors.forEach(anchor -> arrayOneDoFAnchorNode.add(anchor.toJSON(objectMapper)));
      root.set(ONE_DOF_ANCHORS_JSON, arrayOneDoFAnchorNode);
      root.put(EXECUTION_DURATION_JSON, executionDuration);

      return root;
   }

   public RobotConfigurationData getControllerConfiguration()
   {
      return controllerConfiguration;
   }

   public KinematicsToolboxOutputStatus getIkSolution()
   {
      return ikSolution;
   }

   public KinematicsToolboxPrivilegedConfigurationMessage getIkPrivilegedConfiguration()
   {
      return ikPrivilegedConfiguration;
   }

   public CenterOfMassMotionControlAnchorDescription getCenterOfMassAnchor()
   {
      return centerOfMassAnchor;
   }

   public List<SixDoFMotionControlAnchorDescription> getSixDoFAnchors()
   {
      return sixDoFAnchors;
   }

   public List<OneDoFMotionControlAnchorDescription> getOneDoFAnchors()
   {
      return oneDoFAnchors;
   }

   public double getExecutionDuration()
   {
      return executionDuration;
   }

   public void setControllerConfiguration(RobotConfigurationData controllerConfiguration)
   {
      this.controllerConfiguration = controllerConfiguration;
   }

   public void setIkSolution(KinematicsToolboxOutputStatus ikSolution)
   {
      this.ikSolution = ikSolution;
   }

   public void setIkPrivilegedConfiguration(KinematicsToolboxPrivilegedConfigurationMessage ikPrivilegedConfiguration)
   {
      this.ikPrivilegedConfiguration = ikPrivilegedConfiguration;
   }

   public void setCenterOfMassAnchor(CenterOfMassMotionControlAnchorDescription centerOfMassAnchor)
   {
      this.centerOfMassAnchor = centerOfMassAnchor;
   }

   public void setSixDoFAnchors(List<SixDoFMotionControlAnchorDescription> sixDoFAnchors)
   {
      this.sixDoFAnchors = sixDoFAnchors;
   }

   public void setOneDoFAnchors(List<OneDoFMotionControlAnchorDescription> oneDoFAnchors)
   {
      this.oneDoFAnchors = oneDoFAnchors;
   }

   public void setExecutionDuration(double executionDuration)
   {
      this.executionDuration = executionDuration;
   }

   public boolean hasExecutionDuration()
   {
      return !Double.isNaN(executionDuration);
   }

   public void applyTransform(Transform transform)
   {
      controllerConfiguration.getRootPosition().getPoint().applyTransform(transform);
      controllerConfiguration.getRootOrientation().getQuaternion().applyTransform(transform);
      ikSolution.getDesiredRootPosition().getPoint().applyTransform(transform);
      ikSolution.getDesiredRootOrientation().getQuaternion().applyTransform(transform);
      ikPrivilegedConfiguration.getPrivilegedRootJointPosition().getPoint().applyTransform(transform);
      ikPrivilegedConfiguration.getPrivilegedRootJointOrientation().getQuaternion().applyTransform(transform);
      if (centerOfMassAnchor != null)
         centerOfMassAnchor.applyTransform(transform);
      sixDoFAnchors.forEach(anchor -> anchor.applyTransform(transform));
   }

   @Override
   public String toString()
   {
      return String.format("[%s=%s,\n\t%s=%s,\n\t%s=%s,\n\t%s=%s,\n\t%s=%s\n]",
                           CONTROLLER_CONFIGURATION_JSON,
                           controllerConfiguration,
                           IK_SOLUTION_JSON,
                           ikSolution,
                           IK_PRIVILEGED_CONFIGURATION_JSON,
                           ikPrivilegedConfiguration,
                           SIX_DOF_ANCHORS_JSON,
                           sixDoFAnchors,
                           ONE_DOF_ANCHORS_JSON,
                           oneDoFAnchors);
   }
}
