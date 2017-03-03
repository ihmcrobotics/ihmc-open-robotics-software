package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomGeometry;
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
   public Point3D location;
   @RosExportedField(documentation = "Specifies the adjusted orientation of the footstep. It is expressed in world frame.")
   public Quaternion orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to keep the contact points used for the original footstep. Contact points  are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public List<Point2D> predictedContactPoints;

   /**
    * Empty constructor for serialization.
    */
   public AdjustFootstepMessage()
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
      origin = FootstepOrigin.AT_ANKLE_FRAME;
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
      this.location = new Point3D(footstepData.location);
      this.orientation = new Quaternion(footstepData.orientation);
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

   public AdjustFootstepMessage clone()
   {
      return new AdjustFootstepMessage(this);
   }

   public AdjustFootstepMessage(Footstep footstep)
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
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
   }

   public FootstepOrigin getOrigin()
   {
      return origin;
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

   public void setPredictedContactPoints(List<Point2D> predictedContactPoints)
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

   public AdjustFootstepMessage transform(RigidBodyTransform transform)
   {
      AdjustFootstepMessage ret = this.clone();

      // Point3D location;
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
      this.location = RandomGeometry.nextPoint3DWithEdgeCases(random, 0.05);
      this.orientation = RandomGeometry.nextQuaternion(random);
      int numberOfPredictedContactPoints = random.nextInt(10);
      this.predictedContactPoints = new ArrayList<>();

      for (int i = 0; i < numberOfPredictedContactPoints; i++)
      {
         predictedContactPoints.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootstepDataMessage(this);
   }
}
