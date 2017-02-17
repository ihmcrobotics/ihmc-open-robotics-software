package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;
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

   @RosExportedField(documentation = "Boolean that determines whether the controller should use swing and transfer times specifies in this message (if true) or"
         + "if the default swing and transfer times from the FootstepDataListMessage should be used (if false).")
   public boolean hasTimings = false;
   @RosExportedField(documentation = "Specifies the swing time for this footstep.")
   public double swingTime = Double.NaN;
   @RosExportedField(documentation = "Specifies the transfer time before this step.")
   public double transferTime = Double.NaN;

   @RosExportedField(documentation = "Boolean that determines whether the controller should attemp at keeping absolute timings for the execution of this footstep."
         + " This means that a time for foot lift-off (end of toe off) needs to be specified. The timing is set with respect to the start of the execution of the"
         + " FootstepDataList that this footstep is part of. Note, that if you choose to use absolute timings transfer times you set in this message will be ignored.")
   public boolean hasAbsoluteTime = false;
   @RosExportedField(documentation = "If using absolute timings this is the time at which the controller will start the swing. The time is with respect to the time"
         + " at which the controller recieves the walking command. The value of this time must be increasing throughout a FootstepDataListMessage, otherwise it is"
         + " ignored.")
   public double swingStartTime = 0.0;

   /**
    * Empty constructor for serialization.
    */
   public FootstepDataMessage()
   {
      origin = FootstepOrigin.AT_ANKLE_FRAME;
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

      this.hasTimings = footstepData.hasTimings;
      this.swingTime = footstepData.swingTime;
      this.transferTime = footstepData.transferTime;
      this.hasAbsoluteTime = footstepData.hasAbsoluteTime;
      this.swingStartTime = footstepData.swingStartTime;
   }

   public FootstepDataMessage clone()
   {
      return new FootstepDataMessage(this);
   }

   public FootstepDataMessage(Footstep footstep)
   {
      this(footstep, null);
   }

   public FootstepDataMessage(Footstep footstep, FootstepTiming timing)
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

      if (timing != null)
      {
         hasTimings = true;
         swingTime = timing.getSwingTime();
         transferTime = timing.getTransferTime();
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

   public void setTimings(double swingTime, double transferTime)
   {
      hasTimings = true;
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   public boolean hasTimings()
   {
      return hasTimings;
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public void setAbsoluteTime(double swingStartTime)
   {
      hasAbsoluteTime = true;
      this.swingStartTime = swingStartTime;
   }

   public boolean hasAbsoluteTime()
   {
      return hasAbsoluteTime;
   }

   public void removeAbsoluteTime()
   {
      hasAbsoluteTime = false;
      this.swingStartTime = 0.0;
   }

   public double getSwingStartTime()
   {
      return swingStartTime;
   }

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

      boolean sameTimings = hasTimings == footstepData.hasTimings;
      if (hasTimings)
      {
         sameTimings = sameTimings && MathTools.epsilonEquals(swingTime, footstepData.swingTime, epsilon);
         sameTimings = sameTimings && MathTools.epsilonEquals(transferTime, footstepData.transferTime, epsilon);
      }

      boolean sameAbsoluteTime = hasAbsoluteTime == footstepData.hasAbsoluteTime;
      if (hasAbsoluteTime)
         sameAbsoluteTime = sameAbsoluteTime && MathTools.epsilonEquals(swingStartTime, footstepData.swingStartTime, epsilon);

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual && trajectoryWaypointsEqual && sameTimings && sameAbsoluteTime;
   }

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
      this.location = RandomTools.generateRandomPointWithEdgeCases(random, 0.05);
      this.orientation = RandomTools.generateRandomQuaternion(random);
      int numberOfPredictedContactPoints = random.nextInt(10);
      this.predictedContactPoints = new ArrayList<>();

      for (int i = 0; i < numberOfPredictedContactPoints; i++)
      {
         predictedContactPoints.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      this.trajectoryType = trajectoryTypes[randomOrdinal];
      this.swingHeight = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.05);

      if (random.nextBoolean())
      {
         hasTimings = true;
         this.swingTime = RandomTools.generateRandomDoubleInRange(random, 0.05, 2.0);
         this.transferTime = RandomTools.generateRandomDoubleInRange(random, 0.05, 2.0);
      }

      if (random.nextBoolean())
      {
         hasAbsoluteTime = true;
         this.swingStartTime = RandomTools.generateRandomDoubleInRange(random, 0.0, 50.0);
      }

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         trajectoryWaypoints = new Point3D[2];
         trajectoryWaypoints[0] = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
         trajectoryWaypoints[1] = RandomTools.generateRandomPoint3d(random, -10.0, 10.0);
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
