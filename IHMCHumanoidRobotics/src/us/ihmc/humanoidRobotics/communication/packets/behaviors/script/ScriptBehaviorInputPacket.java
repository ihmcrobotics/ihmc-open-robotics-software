package us.ihmc.humanoidRobotics.communication.packets.behaviors.script;

import java.util.Random;

import org.apache.commons.lang3.RandomStringUtils;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomGeometry;

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
      Point3D position = RandomGeometry.nextPoint3D(random, xMax, yMax, zMax);
      Vector3D translation = new Vector3D(position);
      Quaternion orientation = new Quaternion();

      orientation.set(RandomGeometry.nextAxisAngle(random));

      this.scriptName = RandomStringUtils.random(length, true, true);
      this.referenceTransform = new RigidBodyTransform(orientation, translation);
   }
}
