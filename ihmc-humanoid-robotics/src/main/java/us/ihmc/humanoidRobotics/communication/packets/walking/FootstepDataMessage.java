package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

@RosMessagePacket(documentation = "This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class FootstepDataMessage extends Packet<FootstepDataMessage> implements TransformableDataObject<FootstepDataMessage>
{
   @RosExportedField(documentation = "Specifies which foot will swing to reach the foostep.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies the position of the footstep (sole frame) in world frame.")
   public Point3D location;
   @RosExportedField(documentation = "Specifies the orientation of the footstep (sole frame) in world frame.")
   public Quaternion orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to using the entire foot. Contact points are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public ArrayList<Point2D> predictedContactPoints;

   @RosExportedField(documentation = "This contains information on what the swing trajectory should be for each step. Recomended is DEFAULT.")
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   @RosExportedField(documentation = "Contains information on how high the robot should swing its foot. This affects trajectory types DEFAULT and OBSTACLE_CLEARANCE."
         + "If a value smaller then the minumal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.")
   public double swingHeight = 0.0;
   @RosExportedField(documentation = "In case the trajectory type is set to CUSTOM two swing waypoints can be specified here. The waypoints define sole positions."
         + "The controller will compute times and velocities at the waypoints. This is a convinient way to shape the trajectory of the swing. If full control over the swing"
         + "trajectory is desired use the trajectory type WAYPOINTS instead. The position waypoints are expected in the trajectory frame.")
   public Point3D[] positionWaypoints = new Point3D[0];
   @RosExportedField(documentation = "In case the trajectory type is set to WAYPOINTS, swing waypoints can be specified here. The waypoints do not include the"
         + "start point (which is set to the current foot state at lift-off) and the touch down point (which is specified by the location and orientation fields)."
         + "All waypoints are for the sole frame and expressed in the trajectory frame. The maximum number of points can be found in the Footstep class.")
   public SE3TrajectoryPointMessage[] swingTrajectory = null;
   @RosExportedField(documentation = "In case the trajectory type is set to WAYPOINTS, this value can be used to specify the trajectory blend duration "
         + " in seconds. If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning "
         + " the desired blend duration) will be adjusted to account for the initial error between the actual and expected position and orientation of the "
         + "swing foot. Note that the expectedInitialLocation and expectedInitialOrientation fields must be defined in order to enable trajectory blending.")
   public double swingTrajectoryBlendDuration = 0.0;

   @RosExportedField(documentation = "The swingDuration is the time a foot is not in ground contact during a step."
         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default swingDuration.")
   public double swingDuration = -1.0;
   @RosExportedField(documentation = "The transferDuration is the time spent with the feet in ground contact before a step."
         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default transferDuration.")
   public double transferDuration = -1.0;
   
//   this needs to be tested on the real robot before it's exposed in ROS. -shrews
//   @RosExportedField(documentation = "The touchdown duration is the time spent trying to do a soft touchdown."
//         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default transferDuration. If the default is set to zero, the touchdown state will be disabled")
   public double touchdownDuration = -1.0;

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   /**
    * Empty constructor for serialization.
    */
   public FootstepDataMessage()
   {
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
      this.swingTrajectoryBlendDuration = footstepData.swingTrajectoryBlendDuration;

      if (footstepData.positionWaypoints != null)
      {
         this.positionWaypoints = new Point3D[footstepData.positionWaypoints.length];
         for (int i = 0; i < footstepData.positionWaypoints.length; i++)
            positionWaypoints[i] = new Point3D(footstepData.positionWaypoints[i]);
      }

      this.swingDuration = footstepData.swingDuration;
      this.transferDuration = footstepData.transferDuration;
      this.touchdownDuration = footstepData.touchdownDuration;
      this.executionDelayTime = footstepData.executionDelayTime;
   }

   @Override
   public FootstepDataMessage clone()
   {
      return new FootstepDataMessage(this);
   }

   public FootstepDataMessage(Footstep footstep)
   {
      robotSide = footstep.getRobotSide();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.location = location.getPoint();
      this.orientation = new Quaternion(orientation);

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
         for (int i = 0; i < footstepContactPoints.size(); i++)
         {
            Point2D point = new Point2D(footstepContactPoints.get(i));
            predictedContactPoints.add(point);
         }
      }
      else
      {
         predictedContactPoints = null;
      }
      trajectoryType = footstep.getTrajectoryType();
      swingHeight = footstep.getSwingHeight();
      swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();

      if (footstep.getCustomPositionWaypoints().size() != 0)
      {
         positionWaypoints = new Point3D[footstep.getCustomPositionWaypoints().size()];
         for (int i = 0; i < footstep.getCustomPositionWaypoints().size(); i++)
         {
            FramePoint3D framePoint = footstep.getCustomPositionWaypoints().get(i);
            framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            positionWaypoints[i] = new Point3D(framePoint.getPoint());
         }
      }
   }

   public ArrayList<Point2D> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public Point3D getLocation()
   {
      return location;
   }

   public void getLocation(Point3DBasics locationToPack)
   {
      locationToPack.set(location);
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public void getOrientation(QuaternionBasics orientationToPack)
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

   public double getSwingTrajectoryBlendDuration()
   {
      return swingTrajectoryBlendDuration;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setLocation(Point3DReadOnly location)
   {
      if (this.location == null)
         this.location = new Point3D();
      this.location.set(location);
   }

   public void setOrientation(QuaternionReadOnly orientation)
   {
      if (this.orientation == null)
         this.orientation = new Quaternion();
      this.orientation.set(orientation);
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setSwingTrajectoryBlendDuration(double swingTrajectoryBlendDuration)
   {
      this.swingTrajectoryBlendDuration = swingTrajectoryBlendDuration;
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

   public Point3D[] getCustomPositionWaypoints()
   {
      return positionWaypoints;
   }

   public void setCustomPositionWaypoints(Point3D[] trajectoryWaypoints)
   {
      this.positionWaypoints = trajectoryWaypoints;
   }

   public SE3TrajectoryPointMessage[] getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setSwingTrajectory(SE3TrajectoryPointMessage[] swingTrajectory)
   {
      this.swingTrajectory = swingTrajectory;
   }

   public void setTimings(double swingDuration, double transferDuration)
   {
      setSwingDuration(swingDuration);
      setTransferDuration(transferDuration);
   }

   public void setTimings(double swingDuration, double touchdownDuration, double transferDuration)
   {
      setSwingDuration(swingDuration);
      setTouchdownDuration(touchdownDuration);
      setTransferDuration(transferDuration);
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public void setTouchdownDuration(double touchdownDuration)
   {
      this.touchdownDuration = touchdownDuration;
   }
   
   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }
   
   public double getTouchdownDuration()
   {
      return touchdownDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   @Override
   public String toString()
   {
      String ret = "";

      FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), this.orientation);
      double[] ypr = new double[3];
      frameOrientation.getYawPitchRoll(ypr);
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

      if (positionWaypoints != null)
      {
         ret += "waypoints = " + positionWaypoints.length + "\n";
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

      if ((this.positionWaypoints == null) && (footstepData.positionWaypoints != null))
         trajectoryWaypointsEqual = false;
      else if ((this.positionWaypoints != null) && (footstepData.positionWaypoints == null))
         trajectoryWaypointsEqual = false;
      else if (this.positionWaypoints != null)
      {
         int size = positionWaypoints.length;
         if (size != footstepData.positionWaypoints.length)
            trajectoryWaypointsEqual = false;
         else
         {
            for (int i = 0; i < size; i++)
            {
               Point3D pointOne = positionWaypoints[i];
               Point3D pointTwo = footstepData.positionWaypoints[i];

               if (!(pointOne.distanceSquared(pointTwo) < 1e-7))
                  trajectoryWaypointsEqual = false;
            }
         }
      }

      boolean sameTimings = MathTools.epsilonEquals(swingDuration, footstepData.swingDuration, epsilon);
      sameTimings = sameTimings && MathTools.epsilonEquals(transferDuration, footstepData.transferDuration, epsilon);
      sameTimings = sameTimings && MathTools.epsilonEquals(touchdownDuration, footstepData.touchdownDuration, epsilon);

      boolean swingTrajectoryBlendDurationEquals = MathTools.epsilonEquals(swingTrajectoryBlendDuration, footstepData.swingTrajectoryBlendDuration, epsilon);

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual && trajectoryWaypointsEqual && sameTimings
            && swingTrajectoryBlendDurationEquals;
   }

   @Override
   public FootstepDataMessage transform(RigidBodyTransform transform)
   {
      FootstepDataMessage ret = this.clone();

      ret.location.applyTransform(transform);
      ret.orientation.applyTransform(transform);

      // Waypoints if they exist:
      if (positionWaypoints != null)
      {
         for (int i = 0; i < positionWaypoints.length; i++)
            ret.positionWaypoints[i].applyTransform(transform);
      }

      return ret;
   }

   public FootstepDataMessage(Random random)
   {
      TrajectoryType[] trajectoryTypes = TrajectoryType.values();
      int randomOrdinal = random.nextInt(trajectoryTypes.length);

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
      //TODO: when the footstep data list ros message gets regenerated uncomment this! -shrews
//      this.touchdownDuration = RandomNumbers.nextDouble(random, -1.0, 2.0);

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         positionWaypoints = new Point3D[2];
         positionWaypoints[0] = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
         positionWaypoints[1] = RandomGeometry.nextPoint3D(random, -10.0, 10.0);
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
