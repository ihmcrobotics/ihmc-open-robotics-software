package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Arrays;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.trajectories.TrajectoryType;

@RosMessagePacket(documentation = "This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class FootstepDataMessage extends Packet<FootstepDataMessage>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public static final byte TRAJECTORY_TYPE_DEFAULT = 0;
   public static final byte TRAJECTORY_TYPE_OBSTACLE_CLEARANCE = 1;
   public static final byte TRAJECTORY_TYPE_CUSTOM = 2;
   public static final byte TRAJECTORY_TYPE_WAYPOINTS = 3;

   @RosExportedField(documentation = "Specifies which foot will swing to reach the foostep.")
   public byte robotSide;
   @RosExportedField(documentation = "Specifies the position of the footstep (sole frame) in world frame.")
   public Point3D location = new Point3D();
   @RosExportedField(documentation = "Specifies the orientation of the footstep (sole frame) in world frame.")
   public Quaternion orientation = new Quaternion();

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to using the entire foot. Contact points are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public PreallocatedList<Point2D> predictedContactPoints = new PreallocatedList<>(Point2D.class, Point2D::new, 10);

   @RosExportedField(documentation = "This contains information on what the swing trajectory should be for each step. Recomended is DEFAULT.")
   public byte trajectoryType = TrajectoryType.DEFAULT.toByte();
   @RosExportedField(documentation = "Contains information on how high the robot should swing its foot. This affects trajectory types DEFAULT and OBSTACLE_CLEARANCE."
         + "If a value smaller then the minumal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.")
   public double swingHeight = 0.0;
   @RosExportedField(documentation = "In case the trajectory type is set to CUSTOM two swing waypoints can be specified here. The waypoints define sole positions."
         + "The controller will compute times and velocities at the waypoints. This is a convinient way to shape the trajectory of the swing. If full control over the swing"
         + "trajectory is desired use the trajectory type WAYPOINTS instead. The position waypoints are expected in the trajectory frame.")
   public PreallocatedList<Point3D> positionWaypoints = new PreallocatedList<>(Point3D.class, Point3D::new, 2);
   @RosExportedField(documentation = "In case the trajectory type is set to WAYPOINTS, swing waypoints can be specified here. The waypoints do not include the"
         + "start point (which is set to the current foot state at lift-off) and the touch down point (which is specified by the location and orientation fields)."
         + "All waypoints are for the sole frame and expressed in the trajectory frame. The maximum number of points can be found in the Footstep class.")
   public PreallocatedList<SE3TrajectoryPointMessage> swingTrajectory = new PreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new,
                                                                                               50);
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

   @RosExportedField(documentation = "(Experimental) The touchdown duration is the time spent trying to do a soft touchdown."
         + "\nIf the value of this field is invalid (not positive) it will be replaced by a default transferDuration. If the default is set to zero, the touchdown state will be disabled")
   public double touchdownDuration = -1.0;

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   /**
    * Empty constructor for serialization.
    */
   public FootstepDataMessage()
   {
   }

   public FootstepDataMessage(FootstepDataMessage other)
   {
      this.robotSide = other.robotSide;
      this.location = new Point3D(other.location);
      this.orientation = new Quaternion(other.orientation);
      this.orientation.checkIfUnitary();
      MessageTools.copyData(other.predictedContactPoints, predictedContactPoints);
      MessageTools.copyData(other.positionWaypoints, positionWaypoints);
      MessageTools.copyData(other.swingTrajectory, swingTrajectory);
      this.trajectoryType = other.trajectoryType;
      this.swingHeight = other.swingHeight;
      this.swingTrajectoryBlendDuration = other.swingTrajectoryBlendDuration;
      this.swingDuration = other.swingDuration;
      this.transferDuration = other.transferDuration;
      this.touchdownDuration = other.touchdownDuration;
      this.executionDelayTime = other.executionDelayTime;
   }

   @Override
   public void set(FootstepDataMessage other)
   {
      robotSide = other.robotSide;
      location = new Point3D(other.location);
      orientation = new Quaternion(other.orientation);
      MessageTools.copyData(other.predictedContactPoints, predictedContactPoints);
      MessageTools.copyData(other.positionWaypoints, positionWaypoints);
      MessageTools.copyData(other.swingTrajectory, swingTrajectory);
      trajectoryType = other.trajectoryType;
      swingHeight = other.swingHeight;
      swingTrajectoryBlendDuration = other.swingTrajectoryBlendDuration;
      swingDuration = other.swingDuration;
      transferDuration = other.transferDuration;
      touchdownDuration = other.touchdownDuration;
      executionDelayTime = other.executionDelayTime;

      setPacketInformation(other);
   }

   public PreallocatedList<Point2D> getPredictedContactPoints()
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

   public byte getRobotSide()
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

   public void setRobotSide(byte robotSide)
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

   public void setPredictedContactPoints(List<Point2D> predictedContactPoints)
   {
      MessageTools.copyData(predictedContactPoints, this.predictedContactPoints);
   }

   public byte getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(byte trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public PreallocatedList<Point3D> getCustomPositionWaypoints()
   {
      return positionWaypoints;
   }

   public void setCustomPositionWaypoints(Point3D[] trajectoryWaypoints)
   {
      MessageTools.copyData(trajectoryWaypoints, this.positionWaypoints);
   }

   public PreallocatedList<SE3TrajectoryPointMessage> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setSwingTrajectory(SE3TrajectoryPointMessage[] swingTrajectory)
   {
      MessageTools.copyData(swingTrajectory, this.swingTrajectory);
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
    * 
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
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

      ret += TrajectoryType.fromByte(trajectoryType).name() + "\n";

      if (positionWaypoints != null)
      {
         ret += "waypoints = " + positionWaypoints.size() + "\n";
      }
      else
      {
         ret += "no waypoints" + "\n";
      }

      return ret;
   }

   @Override
   public boolean epsilonEquals(FootstepDataMessage other, double epsilon)
   {
      boolean robotSideEquals = robotSide == other.robotSide;
      boolean locationEquals = location.epsilonEquals(other.location, epsilon);

      boolean orientationEquals = orientation.epsilonEquals(other.orientation, epsilon);
      if (!orientationEquals)
      {
         Quaternion temp = new Quaternion();
         temp.setAndNegate(orientation);
         orientationEquals = temp.epsilonEquals(other.orientation, epsilon);
      }

      boolean contactPointsEqual = MessageTools.epsilonEquals(predictedContactPoints, other.predictedContactPoints, epsilon);
      boolean trajectoryWaypointsEqual = MessageTools.epsilonEquals(positionWaypoints, other.positionWaypoints, epsilon);
      boolean swingTrajectoriesEqual = MessageTools.epsilonEquals(swingTrajectory, other.swingTrajectory, epsilon);

      boolean sameTimings = MathTools.epsilonCompare(swingDuration, other.swingDuration, epsilon);
      sameTimings = sameTimings && MathTools.epsilonCompare(transferDuration, other.transferDuration, epsilon);
      sameTimings = sameTimings && MathTools.epsilonCompare(touchdownDuration, other.touchdownDuration, epsilon);

      boolean swingTrajectoryBlendDurationEquals = MathTools.epsilonCompare(swingTrajectoryBlendDuration, other.swingTrajectoryBlendDuration, epsilon);

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual && trajectoryWaypointsEqual && sameTimings
            && swingTrajectoryBlendDurationEquals && swingTrajectoriesEqual;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
