package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

@RosMessagePacket(documentation = "This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class FootstepDataMessage extends Packet<FootstepDataMessage> implements TransformableDataObject<FootstepDataMessage>
{
   public enum FootstepOrigin
   {
      @Deprecated
      @RosEnumValueDocumentation(documentation = "The location of the footstep refers to the location of the ankle frame."
            + " The ankle frame is fixed in the foot, centered at the last ankle joint."
            + " The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward."
            + " This option is for backward compatibility only and will be gone in an upcoming release."
            + " This origin is deprecated as it directly depends on the robot structure and is not directly related to the actual foot sole.")
      AT_ANKLE_FRAME,
      @RosEnumValueDocumentation(documentation = "The location of the footstep refers to the location of the sole frame."
            + " The sole frame is fixed in the foot, centered at the center of the sole."
            + " The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward."
            + " This origin is preferred as it directly depends on the actual foot sole and is less dependent on the robot structure.")
      AT_SOLE_FRAME
   }

   @RosExportedField(documentation = "Specifies whether the given location is the location of the ankle or the sole.")
   public FootstepOrigin origin;
   @RosExportedField(documentation = "Specifies which foot will swing to reach the foostep.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies the position of the footstep. It is expressed in world frame.")
   public Point3D location;
   @RosExportedField(documentation = "Specifies the orientation of the footstep. It is expressed in world frame.")
   public Quaternion orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to using the entire foot. Contact points  are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public ArrayList<Point2D> predictedContactPoints;

   @RosExportedField(documentation = "This contains information on what the swing trajectory should be for each step. Recomended is DEFAULT.\n")
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   @RosExportedField(documentation = "In case the trajectory type is set to custom the swing waypoints can be specified here (As of Dec 2016 only two waypoints are supported).\n"
         + "The waypoints specify the sole position in the world frame.")
   public Point3D[] trajectoryWaypoints = new Point3D[0];
   @RosExportedField(documentation = "Contains information on how high the robot should step. This affects trajectory types default and obstacle clearance."
         + "Recommended values are between 0.1 (minimum swing height, default) and 0.25.\n")
   public double swingHeight = 0.0;

   @RosExportedField(documentation = "The swingDuration is the time a foot is not in ground contact during a step."
         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default swingDuration.")
   public double swingDuration = -1.0;
   @RosExportedField(documentation = "The transferDuration is the time spent with the feet in ground contact before a step."
         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default transferDuration.")
   public double transferDuration = -1.0;

   /**
    * Empty constructor for serialization.
    */
   public FootstepDataMessage()
   {
      origin = FootstepOrigin.AT_ANKLE_FRAME;
   }

   public FootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation)
   {
      this(robotSide, new Point3D(location), new Quaternion(orientation), null);
   }

   public FootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      this(robotSide, location, orientation, null);
   }

   public FootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation, ArrayList<Point2D> predictedContactPoints)
   {
      this(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public FootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public FootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation, ArrayList<Point2D> predictedContactPoints,
         TrajectoryType trajectoryType, double swingHeight)
   {
      origin = FootstepOrigin.AT_ANKLE_FRAME;
      this.robotSide = robotSide;
      this.location = location;
      this.orientation = orientation;
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         this.predictedContactPoints = null;
      else
         this.predictedContactPoints = predictedContactPoints;
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
   }

   public FootstepDataMessage(FootstepDataMessage footstepData)
   {
      this.origin = footstepData.origin;
      this.robotSide = footstepData.robotSide;
      this.location = new Point3D(footstepData.location);
      this.orientation = new Quaternion(footstepData.orientation);
      this.orientation.checkIfUnitary();
      if (footstepData.predictedContactPoints == null || footstepData.predictedContactPoints.isEmpty())
      {
         this.predictedContactPoints = null;
      }
      else
      {
         this.predictedContactPoints = new ArrayList<>();
         for (Point2D contactPoint : footstepData.predictedContactPoints)
         {
            this.predictedContactPoints.add(new Point2D(contactPoint));
         }
      }
      this.trajectoryType = footstepData.trajectoryType;
      this.swingHeight = footstepData.swingHeight;

      if (footstepData.trajectoryWaypoints != null)
      {
         this.trajectoryWaypoints = new Point3D[footstepData.trajectoryWaypoints.length];
         for (int i = 0; i < footstepData.trajectoryWaypoints.length; i++)
            trajectoryWaypoints[i] = new Point3D(footstepData.trajectoryWaypoints[i]);
      }

      this.swingDuration = footstepData.swingDuration;
      this.transferDuration = footstepData.transferDuration;
   }

   @Override
   public FootstepDataMessage clone()
   {
      return new FootstepDataMessage(this);
   }

   public FootstepDataMessage(Footstep footstep)
   {
      origin = FootstepOrigin.AT_ANKLE_FRAME;
      robotSide = footstep.getRobotSide();
      location = new Point3D();
      orientation = new Quaternion();
      footstep.getPositionInWorldFrame(location);
      footstep.getOrientationInWorldFrame(orientation);

      List<Point2D> footstepContactPoints = footstep.getPredictedContactPoints();
      if (footstepContactPoints != null)
      {
         if (predictedContactPoints == null)
         {
            predictedContactPoints = new ArrayList<>();
         }
         else
         {
            predictedContactPoints.clear();
         }
         for (Point2D contactPoint : footstepContactPoints)
         {
            predictedContactPoints.add(new Point2D(contactPoint));
         }
      }
      else
      {
         predictedContactPoints = null;
      }
      trajectoryType = footstep.getTrajectoryType();
      swingHeight = footstep.getSwingHeight();

      if (footstep.getSwingWaypoints().size() != 0)
      {
         trajectoryWaypoints = new Point3D[footstep.getSwingWaypoints().size()];
         for (int i = 0; i < footstep.getSwingWaypoints().size(); i++)
            trajectoryWaypoints[i] = new Point3D(footstep.getSwingWaypoints().get(i));
      }
   }

   public FootstepOrigin getOrigin()
   {
      return origin;
   }

   public ArrayList<Point2D> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public Point3D getLocation()
   {
      return location;
   }

   public void getLocation(Point3D locationToPack)
   {
      locationToPack.set(location);
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public void getOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(this.orientation);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setOrigin(FootstepOrigin origin)
   {
      this.origin = origin;
   }

   public void setLocation(Point3D location)
   {
      if (this.location == null) this.location = new Point3D();
      this.location.set(location);
   }

   public void setOrientation(Quaternion orientation)
   {
      if (this.orientation == null) this.orientation = new Quaternion();
      this.orientation.set(orientation);
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setPredictedContactPoints(ArrayList<Point2D> predictedContactPoints)
   {
      this.predictedContactPoints = predictedContactPoints;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public Point3D[] getTrajectoryWaypoints()
   {
      return trajectoryWaypoints;
   }

   public void setTrajectoryWaypoints(Point3D[] trajectoryWaypoints)
   {
      this.trajectoryWaypoints = trajectoryWaypoints;
   }

   public void setTimings(double swingDuration, double transferDuration)
   {
      setSwingDuration(swingDuration);
      setTransferDuration(transferDuration);
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   @Override
   public String toString()
   {
      String ret = "";

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), this.orientation);
      double[] ypr = frameOrientation.getYawPitchRoll();
      ret = location.toString();
      ret += ", YawPitchRoll = " + Arrays.toString(ypr) + "\n";
      ret += "Predicted Contact Points: ";
      if (predictedContactPoints != null)
      {
         ret += "size = " + predictedContactPoints.size() + "\n";
      }
      else
      {
         ret += "null";
      }

      ret += trajectoryType.name() + "\n";

      if(trajectoryWaypoints != null)
      {
         ret += "waypoints = " + trajectoryWaypoints.length + "\n";
      }
      else
      {
         ret += "no waypoints" + "\n";
      }

      return ret;
   }

   @Override
   public boolean epsilonEquals(FootstepDataMessage footstepData, double epsilon)
   {
      boolean robotSideEquals = robotSide == footstepData.robotSide;
      boolean locationEquals = location.epsilonEquals(footstepData.location, epsilon);

      boolean orientationEquals = orientation.epsilonEquals(footstepData.orientation, epsilon);
      if (!orientationEquals)
      {
         Quaternion temp = new Quaternion();
         temp.setAndNegate(orientation);
         orientationEquals = temp.epsilonEquals(footstepData.orientation, epsilon);
      }

      boolean contactPointsEqual = true;

      if ((this.predictedContactPoints == null) && (footstepData.predictedContactPoints != null))
         contactPointsEqual = false;
      else if ((this.predictedContactPoints != null) && (footstepData.predictedContactPoints == null))
         contactPointsEqual = false;
      else if (this.predictedContactPoints != null)
      {
         int size = predictedContactPoints.size();
         if (size != footstepData.predictedContactPoints.size())
            contactPointsEqual = false;
         else
         {
            for (int i = 0; i < size; i++)
            {
               Point2D pointOne = predictedContactPoints.get(i);
               Point2D pointTwo = footstepData.predictedContactPoints.get(i);

               if (!(pointOne.distanceSquared(pointTwo) < 1e-7))
                  contactPointsEqual = false;
            }
         }
      }

      boolean trajectoryWaypointsEqual = true;

      if ((this.trajectoryWaypoints == null) && (footstepData.trajectoryWaypoints != null))
         trajectoryWaypointsEqual = false;
      else if ((this.trajectoryWaypoints != null) && (footstepData.trajectoryWaypoints == null))
         trajectoryWaypointsEqual = false;
      else if (this.trajectoryWaypoints != null)
      {
         int size = trajectoryWaypoints.length;
         if (size != footstepData.trajectoryWaypoints.length)
            trajectoryWaypointsEqual = false;
         else
         {
            for (int i = 0; i < size; i++)
            {
               Point3D pointOne = trajectoryWaypoints[i];
               Point3D pointTwo = footstepData.trajectoryWaypoints[i];

               if (!(pointOne.distanceSquared(pointTwo) < 1e-7))
                  trajectoryWaypointsEqual = false;
            }
         }
      }

      boolean sameTimings = MathTools.epsilonEquals(swingDuration, footstepData.swingDuration, epsilon);
      sameTimings = sameTimings && MathTools.epsilonEquals(transferDuration, footstepData.transferDuration, epsilon);

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual && trajectoryWaypointsEqual && sameTimings;
   }

   @Override
   public FootstepDataMessage transform(RigidBodyTransform transform)
   {
      FootstepDataMessage ret = this.clone();

      // Point3D location;
      ret.location = TransformTools.getTransformedPoint(this.getLocation(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);

      // Waypoints if they exist:
      if (trajectoryWaypoints != null)
      {
         for (int i = 0; i < trajectoryWaypoints.length; i++)
            ret.trajectoryWaypoints[i] = TransformTools.getTransformedPoint(trajectoryWaypoints[i], transform);
      }

      return ret;
   }

   public FootstepDataMessage(Random random)
   {
      TrajectoryType[] trajectoryTypes = TrajectoryType.values();
      int randomOrdinal = random.nextInt(trajectoryTypes.length);

      origin = FootstepOrigin.AT_ANKLE_FRAME;
      this.robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      this.location = RandomGeometry.nextPoint3DWithEdgeCases(random, 0.05);
      this.orientation = RandomGeometry.nextQuaternion(random);
      int numberOfPredictedContactPoints = random.nextInt(10);
      this.predictedContactPoints = new ArrayList<>();

      for (int i = 0; i < numberOfPredictedContactPoints; i++)
      {
         predictedContactPoints.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      this.trajectoryType = trajectoryTypes[randomOrdinal];
      this.swingHeight = RandomNumbers.nextDoubleWithEdgeCases(random, 0.05);

      this.swingDuration = RandomNumbers.nextDouble(random, -1.0, 2.0);
      this.transferDuration = RandomNumbers.nextDouble(random, -1.0, 2.0);

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         trajectoryWaypoints = new Point3D[2];
         trajectoryWaypoints[0] = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         trajectoryWaypoints[1] = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
