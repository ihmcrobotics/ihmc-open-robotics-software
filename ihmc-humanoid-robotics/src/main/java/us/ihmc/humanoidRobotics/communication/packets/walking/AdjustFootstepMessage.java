package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Arrays;
import java.util.List;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.idl.PreallocatedList;

@RosMessagePacket(documentation = "The intent of this message is to adjust a footstep when the robot is executing it (a foot is currently swinging to reach the footstep to be adjusted).", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class AdjustFootstepMessage extends Packet<AdjustFootstepMessage>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   @RosExportedField(documentation = "Specifies which foot is expected to be executing the footstep to be adjusted.")
   public byte robotSide;
   @RosExportedField(documentation = "Specifies the adjusted position of the footstep. It is expressed in world frame.")
   public Point3D location;
   @RosExportedField(documentation = "Specifies the adjusted orientation of the footstep. It is expressed in world frame.")
   public Quaternion orientation;

   @RosExportedField(documentation = "predictedContactPoints specifies the vertices of the expected contact polygon between the foot and\n"
         + "the world. A value of null or an empty list will default to keep the contact points used for the original footstep. Contact points  are expressed in sole frame. This ordering does not matter.\n"
         + "For example: to tell the controller to use the entire foot, the predicted contact points would be:\n" + "predicted_contact_points:\n"
         + "- {x: 0.5 * foot_length, y: -0.5 * toe_width}\n" + "- {x: 0.5 * foot_length, y: 0.5 * toe_width}\n"
         + "- {x: -0.5 * foot_length, y: -0.5 * heel_width}\n" + "- {x: -0.5 * foot_length, y: 0.5 * heel_width}\n")
   public PreallocatedList<Point2D> predictedContactPoints = new PreallocatedList<>(Point2D.class, Point2D::new, 10);

   @RosExportedField(documentation = "The time to delay this command on the controller side before being executed.")
   public double executionDelayTime;

   /**
    * Empty constructor for serialization.
    */
   public AdjustFootstepMessage()
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
   }

   public AdjustFootstepMessage(AdjustFootstepMessage other)
   {
      set(other);
   }

   @Override
   public void set(AdjustFootstepMessage other)
   {
      robotSide = other.robotSide;
      location = new Point3D(other.location);
      orientation = new Quaternion(other.orientation);
      executionDelayTime = other.executionDelayTime;
      orientation.checkIfUnitary();
      MessageTools.copyData(other.predictedContactPoints, predictedContactPoints);
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
      orientationToPack.set(orientation);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(byte robotSide)
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
      MessageTools.copyData(predictedContactPoints, this.predictedContactPoints);
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
      executionDelayTime = delayTime;
   }

   @Override
   public String toString()
   {
      String ret = "";

      FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), orientation);
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

      if (predictedContactPoints == null && footstepData.predictedContactPoints != null)
         contactPointsEqual = false;
      else if (predictedContactPoints != null && footstepData.predictedContactPoints == null)
         contactPointsEqual = false;
      else if (predictedContactPoints != null)
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
