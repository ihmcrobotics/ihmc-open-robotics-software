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

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType;
import controller_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessagePubSubType;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;

public class KinematicsToolboxSnapshotDescription
{
   public static final String CONFIGURATION_JSON = KinematicsToolboxSnapshotDescription.class.getSimpleName();
   public static final String CONTROLLER_CONFIGURATION_JSON = "controllerConfiguration";
   public static final String IK_SOLUTION_JSON = "ikSolution";
   public static final String IK_PRIVILEGED_CONFIGURATION_JSON = "ikPrivilegedConfiguration";
   public static final String COM_ANCHOR_JSON = "centerOfMassAnchor";
   public static final String SIX_DOF_ANCHORS_JSON = "sixDoFAnchors";
   public static final String ONE_DOF_ANCHORS_JSON = "oneDoFAnchors";

   private static final ObjectMapper objectMapper = new ObjectMapper(new JsonFactory());
   private static final JSONSerializer<RobotConfigurationData> rcdSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private static final JSONSerializer<KinematicsToolboxOutputStatus> ktosSerializer = new JSONSerializer<>(new KinematicsToolboxOutputStatusPubSubType());
   private static final JSONSerializer<KinematicsToolboxPrivilegedConfigurationMessage> ktpcmSerializer = new JSONSerializer<>(new KinematicsToolboxPrivilegedConfigurationMessagePubSubType());

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

   public void applyTransform(Transform transform)
   {
      Point3D point = new Point3D();
      point.set(controllerConfiguration.getRootTranslation());
      point.applyTransform(transform);
//      controllerConfiguration.getRootTranslation().applyTransform(transform);
      controllerConfiguration.getRootTranslation().set(point);
      controllerConfiguration.getRootOrientation().applyTransform(transform);

      point.set(ikSolution.getDesiredRootTranslation());
      point.applyTransform(transform);
//      ikSolution.getDesiredRootTranslation().applyTransform(transform);
      ikSolution.getDesiredRootTranslation().set(point);
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
