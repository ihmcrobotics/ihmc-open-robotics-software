package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.Iterator;
import java.util.function.Consumer;

public class JSONTools
{
   /**
    * When saving we reduce the precision of the numbers so that infintesimal changes
    * do not show up as changes to the actions. We choose half a millimeter as the smallest
    * increment of translation you might care about and 1/50th of a degree for orientations.
    *
    * Additionally, we save the orientations in yaw-pitch-roll degrees so it is human
    * understandable.
    */
   public static void toJSON(ObjectNode jsonNode, RigidBodyTransform rigidBodyTransform)
   {
      // Round to half a millimeter
      // Cast to float, otherwise you get numbers like 0.0200000001 showing up in the JSON
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
      rigidBodyTransform.getRotation().setYawPitchRoll(Math.toRadians(jsonNode.get("yawInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("pitchInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("rollInDegrees").asDouble()));
   }

   public static void toJSONWithRotationMatrix(ObjectNode jsonNode, RigidBodyTransform rigidBodyTransform)
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

   public static void toEuclidWithRotationMatrix(JsonNode jsonNode, RigidBodyTransform rigidBodyTransform)
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
