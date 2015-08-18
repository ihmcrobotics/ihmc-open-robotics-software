package us.ihmc.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;

/**
 * User: Matt
 * Date: 2/18/13
 */
@ClassDocumentation(documentation = "This message gives the desired pelvis pose of the robot in world coordinates.")
public class PelvisPosePacket extends IHMCRosApiPacket<PelvisPosePacket> implements TransformableDataObject<PelvisPosePacket>, VisualizablePacket
{
   /** Desired pelvis orientation. Set it to {@code null} to NOT control it. */
   @FieldDocumentation(documentation = "orientation can be set to null to NOT control it.")
   public Quat4d orientation;
   /** Desired pelvis position. Set it to {@code null} to NOT control it. */
   @FieldDocumentation(documentation = "position can be set to null to NOT control it.")
   public Point3d position;
   /** Desired trajectory time. Can be set to 0sec. */
   @FieldDocumentation(documentation = "trajectoryTime specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;

   @FieldDocumentation(documentation = "toHomePosition can be used to move the pelvis back to its default starting position")
   public boolean toHomePosition;

   public PelvisPosePacket()
   {
      // Empty constructor for deserialization
   }

   public PelvisPosePacket(Quat4d orientation)
   {
      this(null, orientation, false, 3.0);
   }

   public PelvisPosePacket(Point3d point)
   {
      this(point, null, false, 3.0);
   }

   public PelvisPosePacket(Point3d position, Quat4d orientation)
   {
      this(position, orientation, false, 3.0);
   }
   
   public PelvisPosePacket(PelvisPosePacket pelvisPosePacket)
   {
      this(pelvisPosePacket.position, pelvisPosePacket.orientation, pelvisPosePacket.toHomePosition, pelvisPosePacket.trajectoryTime);
   }

   public PelvisPosePacket(Point3d position, Quat4d orientation, boolean toHomePosition, double trajectoryTime)
   {
      this.position = position;
      this.orientation = orientation;
      this.toHomePosition = toHomePosition;
      this.trajectoryTime = trajectoryTime;
   }

   public boolean isToHomePosition()
   {
      return toHomePosition;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public String toString()
   {
      return "Pelvis Pose";
   }

   public PelvisPosePacket transform(RigidBodyTransform transform)
   {
      PelvisPosePacket ret = new PelvisPosePacket();

      if (this.getOrientation() != null)
         ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      else
         ret.orientation = null;

      if (this.getPosition() != null)
         ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);
      else
         ret.position = null;

      if (this.isToHomePosition())
         ret.toHomePosition = true;
      else
         ret.toHomePosition = false;

      ret.trajectoryTime = this.getTrajectoryTime();

      return ret;
   }

   @Override
   public boolean epsilonEquals(PelvisPosePacket other, double epsilon)
   {
      if (this.isToHomePosition() && other.isToHomePosition())
      {
         return true;
      }
      else
      {
         return (RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon) && this.getPosition().epsilonEquals(
               other.getPosition(), epsilon));
      }
   }

   public PelvisPosePacket(Random random)
   {
      Point3d point = new Point3d();
      Quat4d quat = new Quat4d();

      point.set(RandomTools.generateRandomPoint(random, 0.2, 0.2, 0.2)); // magic numbers so point will not exceed XYZ_MIN / MAX in PelvisOrientationPacketSerializer
      quat.set(RandomTools.generateRandomRotation(random));

      this.position = point;
      this.orientation = quat;
      this.toHomePosition = random.nextBoolean();
      this.trajectoryTime = 3.0;
   }
}
