package us.ihmc.avatar.multiContact;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType;
import toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessagePubSubType;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.idl.serializers.extra.JSONSerializer;

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

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<RobotConfigurationData> rcdSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private static final JSONSerializer<KinematicsToolboxOutputStatus> ktosSerializer = new JSONSerializer<>(new KinematicsToolboxOutputStatusPubSubType());
   private static final JSONSerializer<KinematicsToolboxPrivilegedConfigurationMessage> ktpcmSerializer = new JSONSerializer<>(new KinematicsToolboxPrivilegedConfigurationMessagePubSubType());

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
      controllerConfiguration = new RobotConfigurationData(other.controllerConfiguration);
      ikSolution = new KinematicsToolboxOutputStatus(other.ikSolution);
      ikPrivilegedConfiguration = new KinematicsToolboxPrivilegedConfigurationMessage(other.ikPrivilegedConfiguration);
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

   public static KinematicsToolboxSnapshotDescription fromJSON(JsonNode node)
   {
      JsonNode configurationNode = node.get(CONFIGURATION_JSON);

      try
      {
         KinematicsToolboxSnapshotDescription description = new KinematicsToolboxSnapshotDescription();
         description.setControllerConfiguration(rcdSerializer.deserialize(configurationNode.get(CONTROLLER_CONFIGURATION_JSON).toString()));
         description.setIkSolution(ktosSerializer.deserialize(configurationNode.get(IK_SOLUTION_JSON).toString()));
         description.setIkPrivilegedConfiguration(ktpcmSerializer.deserialize(configurationNode.get(IK_PRIVILEGED_CONFIGURATION_JSON).toString()));

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

   public JsonNode toJSON(ObjectMapper objectMapper)
   {
      Objects.requireNonNull(controllerConfiguration);
      Objects.requireNonNull(ikSolution);
      Objects.requireNonNull(ikPrivilegedConfiguration);
      Objects.requireNonNull(sixDoFAnchors);
      Objects.requireNonNull(oneDoFAnchors);

      try
      {
         ObjectNode root = objectMapper.createObjectNode();
         ObjectNode configurationJSON = root.putObject(CONFIGURATION_JSON);

         configurationJSON.set(CONTROLLER_CONFIGURATION_JSON, messageToJSON(rcdSerializer, controllerConfiguration));
         configurationJSON.set(IK_SOLUTION_JSON, messageToJSON(ktosSerializer, ikSolution));
         configurationJSON.set(IK_PRIVILEGED_CONFIGURATION_JSON, messageToJSON(ktpcmSerializer, ikPrivilegedConfiguration));
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
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public JsonNode toJSONNew(ObjectMapper objectMapper)
   {
      Objects.requireNonNull(controllerConfiguration);
      Objects.requireNonNull(ikSolution);
      Objects.requireNonNull(ikPrivilegedConfiguration);
      Objects.requireNonNull(sixDoFAnchors);
      Objects.requireNonNull(oneDoFAnchors);

      try
      {
         ObjectNode root = objectMapper.createObjectNode();

         root.set(CONTROLLER_CONFIGURATION_JSON, messageToJSON(rcdSerializer, controllerConfiguration));
         root.set(IK_SOLUTION_JSON, messageToJSON(ktosSerializer, ikSolution));
         root.set(IK_PRIVILEGED_CONFIGURATION_JSON, messageToJSON(ktpcmSerializer, ikPrivilegedConfiguration));
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
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static <T> JsonNode messageToJSON(JSONSerializer<T> serializer, T message) throws IOException
   {
      return objectMapper.readTree(serializer.serializeToString(message));
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
      controllerConfiguration.getRootPosition().applyTransform(transform);
      controllerConfiguration.getRootOrientation().applyTransform(transform);
      ikSolution.getDesiredRootPosition().applyTransform(transform);
      ikSolution.getDesiredRootOrientation().applyTransform(transform);
      ikPrivilegedConfiguration.getPrivilegedRootJointPosition().applyTransform(transform);
      ikPrivilegedConfiguration.getPrivilegedRootJointOrientation().applyTransform(transform);
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
