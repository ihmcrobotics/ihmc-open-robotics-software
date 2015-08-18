package us.ihmc.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.DocumentedEnum;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 *Use PacketControllerTools to easily create HandPosePacket 
 *
 */

@ClassDocumentation(documentation = "This message commands the controller to move an arm end effector to a given\n"
                                  + "position and orientation. On Atlas, the end effector position is considered\n"
                                  + "as the end of the hand attachment plate, which is about 10 cm from the end\n"
                                  + "of the wrist. The position/orientation may be specified in world frame, chest\n"
                                  + "frame, or desired joint angles.")
public class HandPosePacket extends IHMCRosApiPacket<HandPosePacket> implements TransformableDataObject<HandPosePacket>, VisualizablePacket
{
   public enum Frame implements DocumentedEnum<Frame>
   {
      CHEST, WORLD;

      @Override
      public String getDocumentation(Frame var)
      {
         switch (var)
         {
         case CHEST:
            return "frame attached to the chest of the robot";
         case WORLD:
            return "world frame";

         default:
            return "no documentation available";
         }
      }

      @Override
      public Frame[] getDocumentedValues()
      {
         return values();
      }
   }

   public enum DataType implements DocumentedEnum<DataType>
   {
      HAND_POSE, JOINT_ANGLES;
      
      @Override
      public String getDocumentation(DataType var)
      {
         switch (var)
         {
         case HAND_POSE:
            return "the hand pose will be represented by a hand position and a hand orientation (joint angles will be ignored)";
         case JOINT_ANGLES:
            return "the joint angles contained in this package will be used for control (position and orientation will be ignored)";

         default:
            return "no documentation available";
         }
      }

      @Override
      public DataType[] getDocumentedValues()
      {
         return values();
      }
   }

   public RobotSide robotSide;
   @FieldDocumentation(documentation = "data_type specifies whether or not the IHMC Controller should use the pose fields\n"
                                     + "or the joint angles array for computing control output for the arms")
   public DataType dataType; 
   @FieldDocumentation(documentation = "when using HAND_POSE datatype commands, specify whether the pose should be held in world or chest frame. "
         + "Note that regardless of the frame specified here the position and orientation must be expressed in world frame.")
   public Frame referenceFrame;

   @FieldDocumentation(documentation = "to_home_position can be used to move the arm end effectors back to their starting\n"
                                     + "position, defined as down and beside the robot with slightly bent elbows")
   public boolean toHomePosition;
   @FieldDocumentation(documentation = "the position component of a HAND_POSE type message. See the data_type field.")
   public Point3d position;
   @FieldDocumentation(documentation = "the orientation component of a HAND_POSE type message. See the data_type field.")
   public Quat4d orientation;
   @FieldDocumentation(documentation = "trajectory_time specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;
   @FieldDocumentation(documentation = "joint_angles specifies the desired arm joint angles in order for a JOINT_ANGLES type messages."
                                     + "For Atlas V5 the controller assumes joint angles will be given in the following order:\n"
                                     + "shoulder yaw, shoulder roll, elbow pitch, elbow roll, upper wrist pitch, wrist roll, lower wrist pitch")
   public double[] jointAngles;

   
   @IgnoreField
   @Optional(value = "scripting")
   public boolean zeroUIWristForce = false;
   
   /**
    * By default it should be all true, meaning the hand orientation will be full constrained.
    */
   @IgnoreField
   public boolean[] controlledOrientationAxes;
   // Needs to be between 0.0 and 1.0 or set it to Double.NaN to perform a default hand pose (equivalent to 1.0).
   @IgnoreField
   public double percentOfTrajectoryWithHandOrientationBeingControlled = Double.NaN;
   
   @FieldDocumentation(documentation = "Specifies whether or not the orientation of the hand should be controller during HAND_POSE commands.\n")
   public boolean controlOrientation;
   
   public HandPosePacket()
   {
      // Empty constructor for deserialization
   }

   /**
    *  This constructor assumes non-load bearing
    */
   public HandPosePacket(RobotSide robotSide, Frame referenceFrame, DataType dataType, Point3d position, Quat4d orientation, double trajectoryTime,
         double[] jointAngles)
   {
      this(robotSide, referenceFrame, dataType, position, orientation, false, trajectoryTime, jointAngles);
   }
   

   /**
    * This constructor assumes going to home position and non-load bearing
    */
   public HandPosePacket(RobotSide robotSide, double[] jointAngles)
   {
      this(robotSide, null, DataType.JOINT_ANGLES, null, null, true, 5.0, jointAngles);
   }

   public HandPosePacket(RobotSide robotSide, double trajectoryTime, double[] jointAngles)
   {
      this(robotSide, null, DataType.JOINT_ANGLES, null, null, false, trajectoryTime, jointAngles);
   }

   public HandPosePacket(RobotSide robotSide, double trajectoryTime, double[] jointAngles, Frame referenceFrame, Point3d position, Quat4d orientation)
   {
      this(robotSide, referenceFrame, DataType.JOINT_ANGLES, position, orientation, false, trajectoryTime, jointAngles);
   }

   public HandPosePacket(RobotSide robotSide, Frame referenceFrame, Point3d position, Quat4d orientation, double trajectoryTime)
   {
      this(robotSide, referenceFrame, DataType.HAND_POSE, position, orientation, false, trajectoryTime, null);
   }

   public HandPosePacket(RobotSide robotSide, Frame referenceFrame, DataType dataType, Point3d position, Quat4d orientation, boolean toHomePosition,
         double trajectoryTime, double[] jointAngles)
   {
      this.robotSide = robotSide;
      this.referenceFrame = referenceFrame;
      this.dataType = dataType;
      this.position = position;
      this.orientation = orientation;
      this.toHomePosition = toHomePosition;
      this.trajectoryTime = trajectoryTime;
      this.jointAngles = jointAngles;
      this.percentOfTrajectoryWithHandOrientationBeingControlled = Double.NaN;
      
      
      this.zeroUIWristForce = false;
      this.controlledOrientationAxes = null;
      this.controlOrientation = false;
      
   }

   public HandPosePacket(HandPosePacket handPosePacket)
   {
      this(handPosePacket.robotSide, handPosePacket.referenceFrame, handPosePacket.dataType, handPosePacket.position, handPosePacket.orientation,
            handPosePacket.toHomePosition, handPosePacket.trajectoryTime, handPosePacket.jointAngles);
      
      this.zeroUIWristForce = handPosePacket.getZeroUIWristForce();
      
      if (handPosePacket.controlledOrientationAxes != null)
      {
         this.controlledOrientationAxes = new boolean[handPosePacket.controlledOrientationAxes.length];
         
         for(int i=0; i<this.controlledOrientationAxes.length; i++)
         {
            this.controlledOrientationAxes[i] = handPosePacket.controlledOrientationAxes[i];
         }
      }
      
      this.controlOrientation = handPosePacket.controlOrientation;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Frame getReferenceFrame()
   {
      return referenceFrame;
   }

   public DataType getDataType()
   {
      return dataType;
   }

   public boolean isToHomePosition()
   {
      return toHomePosition;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public double[] getJointAngles()
   {
      return jointAngles;
   }
   
   public boolean getControlOrientation()
   {
      if (controlledOrientationAxes == null)
         return true;
      else
         return controlledOrientationAxes[0] || controlledOrientationAxes[1] || controlledOrientationAxes[2];
   }

   public boolean[] getControlledOrientationAxes()
   {
      return controlledOrientationAxes;
   }

   public void setControlledOrientationAxes(boolean[] controlledOrientationAxes)
   {
      this.controlledOrientationAxes = controlledOrientationAxes;
   }

   public void setControlledOrientationAxes(boolean controlOrientationAroundX, boolean controlOrientationAroundY, boolean controlOrientationAroundZ)
   {
      if (controlledOrientationAxes == null)
         controlledOrientationAxes = new boolean[3];

      controlledOrientationAxes[0] = controlOrientationAroundX;
      controlledOrientationAxes[1] = controlOrientationAroundY;
      controlledOrientationAxes[2] = controlOrientationAroundZ;
   }

   public double getPercentOfTrajectoryWithHandOrientationBeingControlled()
   {
      return percentOfTrajectoryWithHandOrientationBeingControlled;
   }

   public void setPercentOfTrajectoryWithHandOrientationBeingControlled(double percentOfTrajectoryWithHandOrientationBeingControlled)
   {
      this.percentOfTrajectoryWithHandOrientationBeingControlled = percentOfTrajectoryWithHandOrientationBeingControlled;
   }

   @Override
   public HandPosePacket transform(RigidBodyTransform transform)
   {
      HandPosePacket ret = new HandPosePacket();
      ret.notes = this.notes;

      // RobotSide robotSide;
      ret.robotSide = this.getRobotSide();

      ret.trajectoryTime = this.trajectoryTime;
      ret.zeroUIWristForce = this.getZeroUIWristForce();

      if (this.isToHomePosition())
      {
         ret.toHomePosition = true;
      }
      else
      {
         // Frame referenceFrame;
         ret.referenceFrame = this.getReferenceFrame();

         // boolean toHomePosition;
         ret.toHomePosition = this.isToHomePosition();

         // Point3d position;
         if (this.getPosition() != null)
            ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

         // Quat4d orientation;
         if (this.getOrientation() != null)
            ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);
      }

      ret.dataType = this.dataType;

      if (this.jointAngles != null)
         ret.jointAngles = Arrays.copyOf(this.jointAngles, this.jointAngles.length);
      
      ret.percentOfTrajectoryWithHandOrientationBeingControlled = this.percentOfTrajectoryWithHandOrientationBeingControlled;
      
      if (this.controlledOrientationAxes != null)
         ret.controlledOrientationAxes = Arrays.copyOf(this.controlledOrientationAxes, this.controlledOrientationAxes.length);

      ret.controlOrientation = this.controlOrientation;

      return ret;
   }

   public boolean getZeroUIWristForce()
   {
      return zeroUIWristForce;
   }

   public void setZeroUIWristForce(boolean zeroUIWristForce)
   {
      this.zeroUIWristForce = zeroUIWristForce;
   }

   @Override
   public String toString()
   {
      return "HandPosePacket [robotSide=" + robotSide + ", dataType=" + dataType + ", referenceFrame=" + referenceFrame + ", toHomePosition=" + toHomePosition
            + ", position=" + position + ", orientation=" + orientation + ", trajectoryTime=" + trajectoryTime + ", jointAngles="
            + Arrays.toString(jointAngles) + "]";
   }

   @Override
   public boolean epsilonEquals(HandPosePacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());
      ret &= this.getDataType().equals(other.getDataType());
      ret &= this.getReferenceFrame() == other.getReferenceFrame();
      ret &= this.isToHomePosition() == other.isToHomePosition();
      
      if (this.getPosition() == null || other.getPosition() == null)
      {
         ret &= this.getPosition() == null && other.getPosition() == null;
      }
      else
      {
         ret &= this.getPosition().epsilonEquals(other.getPosition(), epsilon);
      }
      
      if (this.getOrientation() == null || other.getOrientation() == null)
      {
         ret &= this.getOrientation() == null && other.getOrientation() == null;
      } 
      else
      {
         ret &= RotationFunctions.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      }
      
      if (getControlledOrientationAxes() == null || other.getControlledOrientationAxes() == null)
      {
         ret &= this.getControlledOrientationAxes() == null && other.getControlledOrientationAxes() == null;
      }
      else
      {
         ret &= Arrays.equals(getControlledOrientationAxes(), other.getControlledOrientationAxes());
      }

      ret &= MathTools.epsilonEquals(getPercentOfTrajectoryWithHandOrientationBeingControlled(), other.getPercentOfTrajectoryWithHandOrientationBeingControlled(), epsilon);
      
      ret &= MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime, epsilon);
      
      if(jointAngles == null || other.jointAngles == null)
      {
         ret &= jointAngles == other.jointAngles;
      }
      else
      {
         for (int i = 0; i < jointAngles.length; i++)
         {
            ret &= MathTools.epsilonEquals(this.jointAngles[i], other.jointAngles[i], epsilon);
         }
      }
      
      ret &=  (Math.abs(this.percentOfTrajectoryWithHandOrientationBeingControlled - other.percentOfTrajectoryWithHandOrientationBeingControlled) < epsilon);
           
      ret &= (this.zeroUIWristForce && other.zeroUIWristForce);
      
      if (this.controlledOrientationAxes == null || other.controlledOrientationAxes == null)
      {
         ret &= this.controlledOrientationAxes==other.controlledOrientationAxes;
      }
      else
      {
         if (this.controlledOrientationAxes.length != other.controlledOrientationAxes.length)
            ret &= false;
         else
         {
            for(int i=0; i<this.controlledOrientationAxes.length; i++)
            {
               ret &= this.controlledOrientationAxes[i]==other.controlledOrientationAxes[i];
            }
         }
      }
      
      ret &= this.controlOrientation==other.controlOrientation;

      return ret;
   }

   public HandPosePacket(Random random)
   {
      final int MAX_NUMBER_OF_ARM_JOINTS = 7;


      double maxValue = Double.MAX_VALUE / 2;
      Point3d randomPoint = RandomTools.generateRandomPoint(random, maxValue, maxValue, maxValue);
      Quat4d randomQuat = new Quat4d();
      randomQuat.set(RandomTools.generateRandomRotation(random));
      randomQuat.normalize();
      
      
      this.referenceFrame = random.nextBoolean() ? HandPosePacket.Frame.WORLD : HandPosePacket.Frame.CHEST;
      this.dataType = random.nextBoolean() ? DataType.HAND_POSE : DataType.JOINT_ANGLES;
      this.position = randomPoint;
      this.orientation = randomQuat;
      
      int randomNumberOfJoints = random.nextInt(MAX_NUMBER_OF_ARM_JOINTS);
      double[] randomJointAngles = RandomTools.generateRandomDoubleArrayWithEdgeCases(random, randomNumberOfJoints, 0.02);

      this.robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      this.toHomePosition = random.nextBoolean();
      this.trajectoryTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1);
      this.jointAngles = randomJointAngles;
   }
   
   @Override
   public boolean rosConversionEpsilonEquals(HandPosePacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());
      ret &= this.isToHomePosition() == other.isToHomePosition();
      
      ret &= MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime, epsilon);
      
      if(jointAngles == null || other.jointAngles == null)
      {
         ret &= jointAngles == other.jointAngles;
      }
      else
      {
         for (int i = 0; i < jointAngles.length; i++)
         {
            ret &= MathTools.epsilonEquals(this.jointAngles[i], other.jointAngles[i], epsilon);
         }
      }

      return ret;
   }
}
