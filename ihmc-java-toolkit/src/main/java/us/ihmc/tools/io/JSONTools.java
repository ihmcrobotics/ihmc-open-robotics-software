package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class JSONTools
{
   public static void toJSON(ObjectNode jsonNode, RigidBodyTransform rigidBodyTransform)
   {
      jsonNode.put("x", rigidBodyTransform.getTranslation().getX());
      jsonNode.put("y", rigidBodyTransform.getTranslation().getY());
      jsonNode.put("z", rigidBodyTransform.getTranslation().getZ());
      jsonNode.put("m00", rigidBodyTransform.getRotation().getM00());
      jsonNode.put("m01", rigidBodyTransform.getRotation().getM01());
      jsonNode.put("m02", rigidBodyTransform.getRotation().getM02());
      jsonNode.put("m10", rigidBodyTransform.getRotation().getM10());
      jsonNode.put("m11", rigidBodyTransform.getRotation().getM11());
      jsonNode.put("m12", rigidBodyTransform.getRotation().getM12());
      jsonNode.put("m20", rigidBodyTransform.getRotation().getM20());
      jsonNode.put("m21", rigidBodyTransform.getRotation().getM21());
      jsonNode.put("m22", rigidBodyTransform.getRotation().getM22());
   }

   public static void toEuclid(JsonNode jsonNode, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getTranslation().setX(jsonNode.get("x").asDouble());
      rigidBodyTransform.getTranslation().setY(jsonNode.get("y").asDouble());
      rigidBodyTransform.getTranslation().setZ(jsonNode.get("z").asDouble());
      rigidBodyTransform.getRotation().setRotationMatrix(jsonNode.get("m00").asDouble(),
                                                         jsonNode.get("m01").asDouble(),
                                                         jsonNode.get("m02").asDouble(),
                                                         jsonNode.get("m10").asDouble(),
                                                         jsonNode.get("m11").asDouble(),
                                                         jsonNode.get("m12").asDouble(),
                                                         jsonNode.get("m20").asDouble(),
                                                         jsonNode.get("m21").asDouble(),
                                                         jsonNode.get("m22").asDouble());
   }
}
