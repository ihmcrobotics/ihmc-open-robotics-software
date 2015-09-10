package us.ihmc.communication.packets.walking;


import java.util.Random;

import javax.vecmath.Quat4d;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.tools.random.RandomTools;

/**
 * User: Matt
 * Date: 2/18/13
 */
@ClassDocumentation("This message sets the orientation of the robot's chest in world coordinates.")
public class ChestOrientationPacket extends IHMCRosApiPacket<ChestOrientationPacket> implements TransformableDataObject<ChestOrientationPacket>, VisualizablePacket
{
   public Quat4d orientation;
   @FieldDocumentation("trajectoryTime specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;

   @FieldDocumentation("toHomePosition can be used to move the chest back to its default starting position")
   public boolean toHomeOrientation;
   
   @IgnoreField
   @Optional(value = "scripting")
   public Quat4d orientationChestInPelvisFrame;
   
   public ChestOrientationPacket()
   {
      // Empty constructor for deserialization
   }

   public ChestOrientationPacket(Quat4d orientation, boolean toHomeOrientation,double trajectoryTime)
   {
      this(orientation, toHomeOrientation, trajectoryTime, null);
   }
   
   public ChestOrientationPacket(Quat4d orientation, boolean toHomeOrientation,double trajectoryTime, Quat4d orientationChestInPelvisFrame)
   {
      this.orientation = orientation;
      this.toHomeOrientation = toHomeOrientation;
      this.trajectoryTime = trajectoryTime;
      this.orientationChestInPelvisFrame = orientationChestInPelvisFrame;
   }
   
   public Quat4d getOrientation()
   {
      return orientation;
   }
   
   public boolean isToHomeOrientation()
   {
      return toHomeOrientation;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public String toString()
   {
      return "Chest Orientation " + orientation + " " + toHomeOrientation + " " + trajectoryTime;
   }
   
   public Quat4d getOrientationChestInPelvisFrame()
   {
      if (orientationChestInPelvisFrame == null)
         return null;
      
      return new Quat4d(orientationChestInPelvisFrame);
   }

   @Override
   public ChestOrientationPacket transform(RigidBodyTransform transform)
   {
      ChestOrientationPacket ret = new ChestOrientationPacket();
      if (this.getOrientation() != null)
         ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      else
         ret.orientation = null;
      
      if (this.isToHomeOrientation())
         ret.toHomeOrientation = true;
      else
         ret.toHomeOrientation = false;
      
      ret.orientationChestInPelvisFrame = this.getOrientationChestInPelvisFrame();
      
      return ret;
   }

   @Override
   public boolean epsilonEquals(ChestOrientationPacket other, double epsilon)
   {
      if (this.isToHomeOrientation() && other.isToHomeOrientation())
      {
         return true;
      }
      else
      {
         boolean ret = RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
         ret &= MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime, epsilon);
         return ret;
      }
   }

   public ChestOrientationPacket(Random random)
   {
      this.orientation = RandomTools.generateRandomQuaternion(random);
      orientation.normalize();
      this.trajectoryTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1, 5);
      this.toHomeOrientation = random.nextBoolean();
   }
}