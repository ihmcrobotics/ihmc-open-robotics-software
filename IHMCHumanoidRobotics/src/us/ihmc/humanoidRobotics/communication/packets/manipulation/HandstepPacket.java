package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandstepPacket extends Packet<HandstepPacket> implements TransformableDataObject<HandstepPacket>
{
   // Must be public for efficient serialization.
   public RobotSide robotSide;
   public Point3D location;
   public Quaternion orientation;
   public Vector3D surfaceNormal;
   public double swingTrajectoryTime;

   public HandstepPacket()
   {
      // Must have null constructor for efficient serialization.
   }

   public HandstepPacket(RobotSide robotSide, Point3D location, Quaternion orientation, Vector3D surfaceNormal, double swingTrajectoryTime)
   {
      this.robotSide = robotSide;
      this.location = location;
      this.orientation = orientation;
      this.surfaceNormal = surfaceNormal;
      this.swingTrajectoryTime = swingTrajectoryTime;
      orientation.checkIfUnitary();
   }

   public HandstepPacket(HandstepPacket handstepPacket)
   {
      this.robotSide = handstepPacket.robotSide;
      this.location = new Point3D(handstepPacket.location);
      this.orientation = new Quaternion(handstepPacket.orientation);
      this.surfaceNormal = new Vector3D(handstepPacket.surfaceNormal);
      orientation.checkIfUnitary();
   }

   public Point3D getLocation()
   {
      return location;
   }

   public void getLocation(Point3D locationToPack)
   {
      locationToPack.set(location);
   }

   public void setLocation(Point3D location)
   {
      this.location.set(location);
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public void getOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(this.orientation);
   }

   public Vector3D getSurfaceNormal()
   {
      return surfaceNormal;
   }

   public void getSurfaceNormal(Vector3D surfaceNormalToPack)
   {
      surfaceNormalToPack.set(this.surfaceNormal);
   }

   public void setOrientation(Quaternion orientation)
   {
      this.orientation.set(orientation);
      orientation.checkIfUnitary();
   }

   public void setSurfaceNormal(Vector3D surfaceNormal)
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
      ret += ", YawPitchRoll = " + Arrays.toString(ypr) + "\n";

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
         Quaternion temp = new Quaternion();
         temp.setAndNegate(orientation);
         orientationEquals = temp.epsilonEquals(handstepPacket.orientation, epsilon);
      }

      return robotSideEquals && locationEquals && orientationEquals;
   }

   public HandstepPacket transform(RigidBodyTransform transform)
   {
      HandstepPacket ret = new HandstepPacket();

      // String rigidBodyName;
      ret.robotSide = this.getRobotSide();

      // Point3D location;
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
      this.location = RandomGeometry.nextPoint3D(random, 0.5, 0.5, 0.5);
      this.orientation = RandomGeometry.nextQuaternion(random, Math.PI / 4.0);
      this.surfaceNormal = RandomGeometry.nextVector3D(random, 1.0);
      this.swingTrajectoryTime = RandomNumbers.nextDouble(random, TRAJECTORY_TIME_MIN, TRAJECTORY_TIME_MAX);
   }
}
