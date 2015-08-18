package us.ihmc.communication.packets.manipulation;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandPoseListPacket extends Packet<HandPoseListPacket> implements TransformableDataObject<HandPoseListPacket>
{
   public RobotSide robotSide;
   // TODO Implement for actual hand pose
   public DataType dataType;

   @FieldDocumentation(documentation = "trajectoryTime specifies how fast to move the hand to the desired position")
   public double trajectoryTime;
   @FieldDocumentation(documentation = "jointAngles specifies the desired individual arm joint angles. For Atlas, those are:\n"
         + "shoulder pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll.  jointAngles must contain more than one arm pose, for trajectory generator")
   public double[][] jointAngles;
   
   public Frame referenceFrame;
   public Point3d[] positions;
   public Quat4d[] orientations;

   public HandPoseListPacket()
   {
      // Empty constructor for deserialization
   }

   public HandPoseListPacket(RobotSide robotSide, double[][] jointAngles, double trajectoryTime)
   {
      int numberOfArmPoses = jointAngles[0].length;
      if ( numberOfArmPoses < 2 )
      {
         throw new RuntimeException("Must specify at least two poses for Trajectory Generator");
      }
      
      this.robotSide = robotSide;
      this.jointAngles = jointAngles;
      this.trajectoryTime = trajectoryTime;
   }
   
   public HandPoseListPacket(RobotSide robotSide, Frame referenceFrame, DataType dataType, Point3d[] positions, Quat4d[] orientations, boolean toHomePosition,
         double trajectoryTime, double[] jointAngles)
   {
      this.robotSide = robotSide;
      this.referenceFrame = referenceFrame;
      this.dataType = dataType;
      this.positions = positions;
      this.orientations = orientations;
      this.trajectoryTime = trajectoryTime;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public double[][] getJointAngles()
   {
      return jointAngles;
   }
   
   public Frame getReferenceFrame()
   {
      return referenceFrame;
   }

   public DataType getDataType()
   {
      return dataType;
   }
   
   public Point3d[] getPositions()
   {
      return positions;
   }
   
   public Quat4d[] getOrientations()
   {
      return orientations;
   }

   @Override
   public boolean epsilonEquals(HandPoseListPacket other, double epsilon)
   {
      boolean sameRobotSide = other.robotSide == robotSide;
      
      boolean sameTrajectoryTime = other.trajectoryTime == trajectoryTime;
      
      boolean sameReferenceFrames = true;
      boolean sameDataTypes = true;
      boolean samePositions = true;
      boolean sameOrientations = true;

      if (other.referenceFrame != null & referenceFrame != null)
         sameReferenceFrames = referenceFrame == other.referenceFrame;

      if (other.dataType != null & dataType != null)
         sameDataTypes = dataType == other.dataType;

      if (other.positions != null & positions != null)
         samePositions = other.positions == positions;
      
      if (other.orientations != null & orientations != null)
         sameOrientations = other.orientations == orientations;
      

      boolean sameJointAngles = true;
      
      if (jointAngles != null && other.jointAngles != null)
      {
         for (int i = 0; i < jointAngles.length; i++)
         {
            for (int j = 0; j < jointAngles[0].length; j++)
            {
               if (Math.abs(jointAngles[i][j] - other.jointAngles[i][j]) > epsilon)
               {
                  sameJointAngles = false;
                  break;
               }
            }
         }
      }

      return sameRobotSide && sameTrajectoryTime && sameJointAngles && sameReferenceFrames && sameDataTypes && samePositions && sameOrientations;
   }

   @Override
   public HandPoseListPacket transform(RigidBodyTransform transform)
   {
      HandPoseListPacket ret = new HandPoseListPacket();

      ret.robotSide = robotSide;
      ret.trajectoryTime = trajectoryTime;
      ret.jointAngles = jointAngles;

      return ret;
   }
}
