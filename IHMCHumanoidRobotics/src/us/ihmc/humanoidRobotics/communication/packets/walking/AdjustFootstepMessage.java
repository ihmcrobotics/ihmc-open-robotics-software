package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

@RosMessagePacket(documentation = "The intent of this message is to adjust a footstep when the robot is executing it (a foot is currently swinging to reach the footstep to be adjusted).",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class AdjustFootstepMessage extends Packet<AdjustFootstepMessage> implements TransformableDataObject<AdjustFootstepMessage>
{
   @RosExportedField(documentation = "Specifies whether the given location is the location of the ankle or the sole.")
   public FootstepOrigin origin;
   @RosExportedField(documentation = "Specifies which foot is expected to be executing the footstep to be adjusted.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies the adjusted position of the footstep. It is expressed in world frame.")
   public Point3d location;
   @RosExportedField(documentation = "Specifies the adjusted orientation of the footstep. It is expressed in world frame.")
   public Quat4d orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to keep the contact points used for the original footstep. Contact points  are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public List<Point2d> predictedContactPoints;

   /**
    * Empty constructor for serialization.
    */
   public AdjustFootstepMessage()
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      origin = FootstepOrigin.AT_ANKLE_FRAME;
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3d location, Quat4d orientation)
   {
      this(robotSide, location, orientation, null);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3d location, Quat4d orientation, ArrayList<Point2d> predictedContactPoints)
   {
      this(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3d location, Quat4d orientation, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public AdjustFootstepMessage(RobotSide robotSide, Point3d location, Quat4d orientation, ArrayList<Point2d> predictedContactPoints,
         TrajectoryType trajectoryType, double swingHeight)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      origin = FootstepOrigin.AT_ANKLE_FRAME;
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
      this.origin = footstepData.origin;
      this.robotSide = footstepData.robotSide;
      this.location = new Point3d(footstepData.location);
      this.orientation = new Quat4d(footstepData.orientation);
      RotationTools.checkQuaternionNormalized(this.orientation);
      if (footstepData.predictedContactPoints == null || footstepData.predictedContactPoints.isEmpty())
      {
         this.predictedContactPoints = null;
      }
      else
      {
         this.predictedContactPoints = new ArrayList<>();
         for (Point2d contactPoint : footstepData.predictedContactPoints)
         {
            this.predictedContactPoints.add(new Point2d(contactPoint));
         }
      }
   }

   public AdjustFootstepMessage clone()
   {
      return new AdjustFootstepMessage(this);
   }

   public AdjustFootstepMessage(Footstep footstep)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      origin = FootstepOrigin.AT_ANKLE_FRAME;
      robotSide = footstep.getRobotSide();
      location = new Point3d();
      orientation = new Quat4d();
      footstep.getPositionInWorldFrame(location);
      footstep.getOrientationInWorldFrame(orientation);

      List<Point2d> footstepContactPoints = footstep.getPredictedContactPoints();
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
         for (Point2d contactPoint : footstepContactPoints)
         {
            predictedContactPoints.add((Point2d) contactPoint.clone());
         }
      }
      else
      {
         predictedContactPoints = null;
      }
   }

   public FootstepOrigin getOrigin()
   {
      return origin;
   }

   public List<Point2d> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public Point3d getLocation()
   {
      return location;
   }

   public void getLocation(Point3d locationToPack)
   {
      locationToPack.set(location);
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public void getOrientation(Quat4d orientationToPack)
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

   public void setOrigin(FootstepOrigin origin)
   {
      this.origin = origin;
   }

   public void setLocation(Point3d location)
   {
      if (this.location == null) this.location = new Point3d();
      this.location.set(location);
   }

   public void setOrientation(Quat4d orientation)
   {
      if (this.orientation == null) this.orientation = new Quat4d();
      this.orientation.set(orientation);
   }

   public void setPredictedContactPoints(List<Point2d> predictedContactPoints)
   {
      this.predictedContactPoints = predictedContactPoints;
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

      return ret;
   }

   public boolean epsilonEquals(AdjustFootstepMessage footstepData, double epsilon)
   {
      boolean robotSideEquals = robotSide == footstepData.robotSide;
      boolean locationEquals = location.epsilonEquals(footstepData.location, epsilon);

      boolean orientationEquals = orientation.epsilonEquals(footstepData.orientation, epsilon);
      if (!orientationEquals)
      {
         Quat4d temp = new Quat4d();
         temp.negate(orientation);
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
               Point2d pointOne = predictedContactPoints.get(i);
               Point2d pointTwo = footstepData.predictedContactPoints.get(i);

               if (!(pointOne.distanceSquared(pointTwo) < 1e-7))
                  contactPointsEqual = false;
            }
         }
      }

      return robotSideEquals && locationEquals && orientationEquals && contactPointsEqual;
   }

   public AdjustFootstepMessage transform(RigidBodyTransform transform)
   {
      AdjustFootstepMessage ret = this.clone();

      // Point3d location;
      ret.location = TransformTools.getTransformedPoint(this.getLocation(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      return ret;
   }

   public AdjustFootstepMessage(Random random)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      origin = FootstepOrigin.AT_ANKLE_FRAME;
      this.robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      this.location = RandomTools.generateRandomPointWithEdgeCases(random, 0.05);
      this.orientation = RandomTools.generateRandomQuaternion(random);
      int numberOfPredictedContactPoints = random.nextInt(10);
      this.predictedContactPoints = new ArrayList<>();

      for (int i = 0; i < numberOfPredictedContactPoints; i++)
      {
         predictedContactPoints.add(new Point2d(random.nextDouble(), random.nextDouble()));
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
