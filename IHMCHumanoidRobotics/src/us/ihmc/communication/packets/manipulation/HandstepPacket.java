package us.ihmc.communication.packets.manipulation;

import us.ihmc.communication.TransformableDataObject;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Random;

public class HandstepPacket extends Packet<HandstepPacket> implements TransformableDataObject<HandstepPacket>
{
   // Must be public for efficient serialization.
   public RobotSide robotSide;
   public Point3d location;
   public Quat4d orientation;
   public Vector3d surfaceNormal;
   public double swingTrajectoryTime;

   public HandstepPacket()
   {
      // Must have null constructor for efficient serialization.
   }

   public HandstepPacket(RobotSide robotSide, Point3d location, Quat4d orientation, Vector3d surfaceNormal, double swingTrajectoryTime)
   {
      this.robotSide = robotSide;
      this.location = location;
      this.orientation = orientation;
      this.surfaceNormal = surfaceNormal;
      this.swingTrajectoryTime = swingTrajectoryTime;
      RotationFunctions.checkUnitLength(this.orientation);
   }

   public HandstepPacket(HandstepPacket handstepPacket)
   {
      this.robotSide = handstepPacket.robotSide;
      this.location = new Point3d(handstepPacket.location);
      this.orientation = new Quat4d(handstepPacket.orientation);
      this.surfaceNormal = new Vector3d(handstepPacket.surfaceNormal);
      RotationFunctions.checkUnitLength(this.orientation);
   }

   public Point3d getLocation()
   {
      return location;
   }

   public void getLocation(Point3d locationToPack)
   {
      locationToPack.set(location);
   }

   public void setLocation(Point3d location)
   {
      this.location.set(location);
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(this.orientation);
   }

   public Vector3d getSurfaceNormal()
   {
      return surfaceNormal;
   }

   public void getSurfaceNormal(Vector3d surfaceNormalToPack)
   {
      surfaceNormalToPack.set(this.surfaceNormal);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
      RotationFunctions.checkUnitLength(this.orientation);
   }

   public void setSurfaceNormal(Vector3d surfaceNormal)
   {
      this.surfaceNormal.set(surfaceNormal);

   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getSwingTrajectoryTime()
   {
      return swingTrajectoryTime;
   }

   public void setSwingTrajectoryTime(double swingTrajectoryTime)
   {
      this.swingTrajectoryTime = swingTrajectoryTime;
   }

   public String toString()
   {
      String ret = "";

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), this.orientation);
      double[] ypr = frameOrientation.getYawPitchRoll();
      ret = location.toString();
      ret += ", YawPitchRoll = " + ArrayTools.arrayToString(ypr) + "\n";

      return ret;
   }

   @Override
   public boolean epsilonEquals(HandstepPacket handstepPacket, double epsilon)
   {
      boolean robotSideEquals = robotSide == handstepPacket.robotSide;
      boolean locationEquals = location.epsilonEquals(handstepPacket.location, epsilon);

      boolean orientationEquals = orientation.epsilonEquals(handstepPacket.orientation, epsilon);
      if (!orientationEquals)
      {
         Quat4d temp = new Quat4d();
         temp.negate(orientation);
         orientationEquals = temp.epsilonEquals(handstepPacket.orientation, epsilon);
      }

      return robotSideEquals && locationEquals && orientationEquals;
   }

   public HandstepPacket transform(RigidBodyTransform transform)
   {
      HandstepPacket ret = new HandstepPacket();

      // String rigidBodyName;
      ret.robotSide = this.getRobotSide();

      // Point3d location;
      ret.location = TransformTools.getTransformedPoint(this.getLocation(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);

      // Vector3d surface normal
      ret.surfaceNormal = TransformTools.getTransformedVector(this.getSurfaceNormal(), transform);

      ret.swingTrajectoryTime = this.getSwingTrajectoryTime();

      return ret;
   }

   public HandstepPacket(Random random)
   {
      double TRAJECTORY_TIME_MIN = 0.5;
      double TRAJECTORY_TIME_MAX = 10;

      this.robotSide = RobotSide.generateRandomRobotSide(random);
      this.location = RandomTools.generateRandomPoint(random, 0.5, 0.5, 0.5);
      this.orientation = RandomTools.generateRandomQuaternion(random, Math.PI / 4.0);
      this.surfaceNormal = RandomTools.generateRandomVector(random, 1.0);
      this.swingTrajectoryTime = RandomTools.generateRandomDouble(random, TRAJECTORY_TIME_MIN, TRAJECTORY_TIME_MAX);
   }
}
