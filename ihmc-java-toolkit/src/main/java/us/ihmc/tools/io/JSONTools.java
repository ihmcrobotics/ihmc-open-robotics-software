package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.util.Iterator;
import java.util.function.Consumer;

public class JSONTools
{
   /**
    * {@link #toJSON(ObjectNode, RigidBodyTransformReadOnly)} but embedded in a named object.
    */
   public static void toJSON(ObjectNode jsonNode, String name, RigidBodyTransformReadOnly rigidBodyTransform)
   {
      ObjectNode transformObject = jsonNode.putObject(name);
      toJSON(transformObject, rigidBodyTransform);
   }

   /**
    * {@link #toEuclid(JsonNode, RigidBodyTransformBasics)} but embedded in a named object.
    */
   public static void toEuclid(JsonNode jsonNode, String name, RigidBodyTransformBasics rigidBodyTransform)
   {
      ObjectNode transformObject = (ObjectNode) jsonNode.get(name);
      toEuclid(transformObject, rigidBodyTransform);
   }

   /**
    * When saving we reduce the precision of the numbers so that infintesimal changes
    * do not show up as changes to the actions. We choose half a millimeter as the smallest
    * increment of translation you might care about and 1/50th of a degree for orientations.
    *
    * Additionally, we save the orientations in yaw-pitch-roll degrees so it is human
    * understandable.
    */
   public static void toJSON(ObjectNode jsonNode, RigidBodyTransformReadOnly rigidBodyTransform)
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

   public static void toEuclid(JsonNode jsonNode, RigidBodyTransformBasics rigidBodyTransform)
   {
      rigidBodyTransform.getTranslation().setX(jsonNode.get("x").asDouble());
      rigidBodyTransform.getTranslation().setY(jsonNode.get("y").asDouble());
      rigidBodyTransform.getTranslation().setZ(jsonNode.get("z").asDouble());
      rigidBodyTransform.getRotation().setYawPitchRoll(Math.toRadians(jsonNode.get("yawInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("pitchInDegrees").asDouble()),
                                                       Math.toRadians(jsonNode.get("rollInDegrees").asDouble()));
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
