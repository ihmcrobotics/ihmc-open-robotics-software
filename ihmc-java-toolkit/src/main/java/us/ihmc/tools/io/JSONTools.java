package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.Iterator;
import java.util.function.Consumer;

public class JSONTools
{
   public static void toJSON(ObjectNode jsonNode, RigidBodyTransform rigidBodyTransform)
   {
      // Round to half a millimeter
      jsonNode.put("x", (float) MathTools.roundToPrecision(rigidBodyTransform.getTranslation().getX(), 0.0005));
      jsonNode.put("y", (float) MathTools.roundToPrecision(rigidBodyTransform.getTranslation().getY(), 0.0005));
      jsonNode.put("z", (float) MathTools.roundToPrecision(rigidBodyTransform.getTranslation().getZ(), 0.0005));
      // Round to 1/50th of a degree
      jsonNode.put("rollInDegrees", (float) MathTools.roundToPrecision(Math.toDegrees(rigidBodyTransform.getRotation().getRoll()), 0.02));
      jsonNode.put("pitchInDegrees", (float) MathTools.roundToPrecision(Math.toDegrees(rigidBodyTransform.getRotation().getPitch()), 0.02));
      jsonNode.put("yawInDegrees", (float) MathTools.roundToPrecision(Math.toDegrees(rigidBodyTransform.getRotation().getYaw()), 0.02));
   }

   public static void toEuclid(JsonNode jsonNode, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getTranslation().setX(jsonNode.get("x").asDouble());
      rigidBodyTransform.getTranslation().setY(jsonNode.get("y").asDouble());
      rigidBodyTransform.getTranslation().setZ(jsonNode.get("z").asDouble());
      rigidBodyTransform.getRotation().setYawPitchRoll(Math.toRadians(jsonNode.get("rollInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("pitchInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("yawInDegrees").asDouble()));
   }

   /**
    * Jackson requires us to create and manipulate an Iterator, so let's provide a tool
    * to at least show how to do that.
    */
   public static void forEachArrayElement(JsonNode parentNode, String arrayName, Consumer<JsonNode> nodeConsumer)
   {
      for (Iterator<JsonNode> actionNodeIterator = parentNode.withArray(arrayName).elements(); actionNodeIterator.hasNext();)
      {
         nodeConsumer.accept(actionNodeIterator.next());
      }
   }
}
