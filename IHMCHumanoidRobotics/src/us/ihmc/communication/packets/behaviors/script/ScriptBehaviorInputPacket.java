package us.ihmc.communication.packets.behaviors.script;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.RandomStringUtils;

import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ScriptBehaviorInputPacket extends Packet<ScriptBehaviorInputPacket>
{
   public String scriptName;
   public RigidBodyTransform referenceTransform;


   public ScriptBehaviorInputPacket()
   {
   }

   public ScriptBehaviorInputPacket(String scriptName, RigidBodyTransform referenceTransform)
   {
      this.scriptName = scriptName;
      this.referenceTransform = referenceTransform;
   }

   public String getScriptName()
   {
      return scriptName;
   }

   public RigidBodyTransform getReferenceTransform()
   {
      return referenceTransform;
   }

   @Override
   public boolean epsilonEquals(ScriptBehaviorInputPacket other, double epsilon)
   {
      boolean result = scriptName.equals(other.getScriptName());
      result &= referenceTransform.epsilonEquals(other.getReferenceTransform(), epsilon);

      return result;
   }

   public ScriptBehaviorInputPacket(Random random)
   {
      int length = random.nextInt(255);

      double[] XYZ_MAX = { 2.0, 2.0, 2.0 };
      double[] XYZ_MIN = { -2.0, -2.0, -3.0 };

      
      double xMax = 0.90 * Math.min(Math.abs(XYZ_MAX[0]), Math.abs(XYZ_MIN[0]));
      double yMax = 0.90 * Math.min(Math.abs(XYZ_MAX[1]), Math.abs(XYZ_MIN[1]));
      double zMax = 0.90 * Math.min(Math.abs(XYZ_MAX[2]), Math.abs(XYZ_MIN[2]));
      Point3d position = RandomTools.generateRandomPoint(random, xMax, yMax, zMax);
      Vector3d translation = new Vector3d(position);
      Quat4d orientation = new Quat4d();

      orientation.set(RandomTools.generateRandomRotation(random));

      this.scriptName = RandomStringUtils.random(length, true, true);
      this.referenceTransform = new RigidBodyTransform(orientation, translation);
   }
}
