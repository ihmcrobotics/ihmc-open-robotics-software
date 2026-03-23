package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.Iterator;
import java.util.function.Consumer;

public class JSONTools
{
   /** Round to half a millimeter, cast to float, otherwise you get numbers like 0.0200000001 showing up in the JSON */
   public static float toJsonMeters(double meters)
   {
      return (float) MathTools.roundToPrecision(meters, 0.0005);
   }

   /** Convert and round to 1/50th of a degree */
   public static float toJsonRadians(double radians)
   {
      return (float) MathTools.roundToPrecision(Math.toDegrees(radians), 0.02);
   }

   public static void putArrayRound(ObjectNode jsonNode, String name, float[] values)
   {
      ArrayNode arrayNode = jsonNode.putArray(name);
      for (int i = 0; i < values.length; i++)
         arrayNode.add((float) MathTools.roundToPrecision(values[i], 0.02));
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
      jsonNode.put("x", toJsonMeters(rigidBodyTransform.getTranslation().getX()));
      jsonNode.put("y", toJsonMeters(rigidBodyTransform.getTranslation().getY()));
      jsonNode.put("z", toJsonMeters(rigidBodyTransform.getTranslation().getZ()));
      jsonNode.put("rollInDegrees", toJsonRadians(rigidBodyTransform.getRotation().getRoll()));
      jsonNode.put("pitchInDegrees", toJsonRadians(rigidBodyTransform.getRotation().getPitch()));
      jsonNode.put("yawInDegrees", toJsonRadians(rigidBodyTransform.getRotation().getYaw()));
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
    * When saving we reduce the precision of the numbers so that infintesimal changes
    * do not show up as changes to the actions. We choose half a millimeter as the smallest
    * increment of translation you might care about and 1/50th of a degree for orientations.
    */
   public static void toJSON(ObjectNode jsonNode, Tuple3DReadOnly tuple3D)
   {
      jsonNode.put("x", toJsonMeters(tuple3D.getX()));
      jsonNode.put("y", toJsonMeters(tuple3D.getY()));
      jsonNode.put("z", toJsonMeters(tuple3D.getZ()));
   }

   public static void toEuclid(JsonNode jsonNode, Tuple3DBasics tuple3D)
   {
      tuple3D.setX(jsonNode.get("x").asDouble());
      tuple3D.setY(jsonNode.get("y").asDouble());
      tuple3D.setZ(jsonNode.get("z").asDouble());
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
