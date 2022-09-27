package us.ihmc.avatar.multiContact;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.List;

public class MultiContactEnvironmentIOTools
{
   public static void writeToJSON(ObjectMapper objectMapper, ObjectNode environmentNode, List<FrameShape3DReadOnly> environmentShapes)
   {
      ArrayNode shapeTypes = environmentNode.putArray(MultiContactScriptReader.ENVIRONMENT_SHAPES_TAG);
      ArrayNode shapeData = environmentNode.putArray(MultiContactScriptReader.ENVIRONMENT_DATA_TAG);

      for (int i = 0; i < environmentShapes.size(); i++)
      {
         FrameShape3DReadOnly environmentShape = environmentShapes.get(i);
         shapeTypes.add(environmentShape.getClass().getSimpleName());
         shapeData.add(createNode(objectMapper, environmentShape));
      }
   }

   private static JsonNode createNode(ObjectMapper objectMapper, FrameShape3DReadOnly shape)
   {
      ObjectNode node = objectMapper.createObjectNode();

      if (shape instanceof Box3DReadOnly boxShape)
      {
         writePosition(node, boxShape.getPose());

         ArrayNode size = node.putArray("size");
         size.add(boxShape.getSizeX());
         size.add(boxShape.getSizeY());
         size.add(boxShape.getSizeZ());
      }
      else
      {
         throw new RuntimeException("Implement me! Shape type: " + shape.getClass().getSimpleName());
      }

      return node;
   }

   private static void writePosition(ObjectNode node, Shape3DPoseReadOnly pose)
   {
      ArrayNode poseNode = node.putArray("pose");
      poseNode.add(pose.getTranslationX());
      poseNode.add(pose.getTranslationY());
      poseNode.add(pose.getTranslationZ());
      poseNode.add(pose.getRotation().getYaw());
      poseNode.add(pose.getRotation().getPitch());
      poseNode.add(pose.getRotation().getRoll());
   }
}
