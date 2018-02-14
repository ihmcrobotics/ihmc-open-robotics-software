package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

@RosMessagePacket(documentation = "The intent of this message is to adjust a footstep when the robot is executing it (a foot is currently swinging to reach the footstep to be adjusted).", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class AdjustFootstepMessage extends Packet<AdjustFootstepMessage>
{
   @RosExportedField(documentation = "Specifies which foot is expected to be executing the footstep to be adjusted.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies the adjusted position of the footstep. It is expressed in world frame.")
   public Point3D location;
   @RosExportedField(documentation = "Specifies the adjusted orientation of the footstep. It is expressed in world frame.")
   public Quaternion orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to keep the contact points used for the original footstep. Contact points  are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public List<Point2D> predictedContactPoints;

   @RosExportedField(documentation = "The time to delay this command on the controller side before being executed.")
   public double executionDelayTime;

   /**
    * Empty constructor for serialization.
    */
   public AdjustFootstepMessage()
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      this(robotSide, location, orientation, null);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, ArrayList<Point2D> predictedContactPoints)
   {
      this(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, ArrayList<Point2D> predictedContactPoints,
                                TrajectoryType trajectoryType, double swingHeight)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      this.robotSide = robotSide;
      this.location = location;
      this.orientation = orientation;
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         this.predictedContactPoints = null;
      else
         this.predictedContactPoints = predictedContactPoints;
   }

   public AdjustFootstepMessage(AdjustFootstepMessage footstepData)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      this.robotSide = footstepData.robotSide;
      this.location = new Point3D(footstepData.location);
      this.orientation = new Quaternion(footstepData.orientation);
      this.executionDelayTime = footstepData.executionDelayTime;
      orientation.checkIfUnitary();
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
   }

   @Override
   public AdjustFootstepMessage clone()
   {
      return new AdjustFootstepMessage(this);
   }

   public AdjustFootstepMessage(Footstep footstep)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      robotSide = footstep.getRobotSide();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.location = new Point3D(location);
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
         for (Point2D contactPoint : footstepContactPoints)
         {
            predictedContactPoints.add(new Point2D(contactPoint));
         }
      }
      else
      {
         predictedContactPoints = null;
      }
   }

   public List<Point2D> getPredictedContactPoints()
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

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setLocation(Point3D location)
   {
      if (this.location == null)
         this.location = new Point3D();
      this.location.set(location);
   }

   public void setOrientation(Quaternion orientation)
   {
      if (this.orientation == null)
         this.orientation = new Quaternion();
      this.orientation.set(orientation);
   }

   public void setPredictedContactPoints(List<Point2D> predictedContactPoints)
   {
      this.predictedContactPoints = predictedContactPoints;
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

      return ret;
   }

   @Override
   public boolean epsilonEquals(AdjustFootstepMessage footstepData, double epsilon)
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

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
