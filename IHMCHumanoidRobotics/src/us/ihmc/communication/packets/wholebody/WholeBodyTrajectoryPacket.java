package us.ihmc.communication.packets.wholebody;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

import us.ihmc.communication.TransformableDataObject;
import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.random.RandomTools;

@ClassDocumentation("Send whole body trajectories to the robot. A best effort is \n" +
               "made to execute the trajectory while balance is kept.\n"
               + "Internally the first waypoint at time 0.0 is set to the current\n"
               + "position, do not provide a waypoint at time 0.0.\n"
               + "Positions and orientations are set to null if no motion is desired\n"
               + "velocities are set to null if they are zero")
public class WholeBodyTrajectoryPacket extends IHMCRosApiPacket<WholeBodyTrajectoryPacket>
        implements VisualizablePacket, TransformableDataObject<WholeBodyTrajectoryPacket>
{

   @FieldDocumentation("Sequence of desired time at the waypoints of the trajecotry. \n"
         + "The execution starts at 0. Do not proivde time 0.0\n"
         + "Should be numWaypoint elements long")
   public double[] timeAtWaypoint;
   
   @FieldDocumentation("Sequence of desired positions of the pelvis in world coordinates. \n"
           + "Provide an empty list if no motion is desired\n"
           + "Should be numWaypoints elements long")
   public Point3d[] pelvisWorldPosition;
   
   @FieldDocumentation("Sequence of desired velocities of the pelvis. \n"
         + "Provide an empty list if zero velocity is required\n"
         + "Should be numWaypoints elements long")
   public Vector3d[] pelvisLinearVelocity;
   @FieldDocumentation("Sequence of desired angular velocities of the pelvis. \n"
         + "Provide an empty list if zero angular velocity is desired\n"
         + "Should be numWaypoints elements long")
   public Vector3d[] pelvisAngularVelocity;

   @FieldDocumentation("Sequence of desired quaternion (x,y,z,w) orientations of the pelvis in world coordinates. \n"
         + "Provide an empty list if no motion is desired\n"
         + "Should be numWaypoints elements long")
   public Quat4d[] pelvisWorldOrientation;
   @FieldDocumentation("Sequence of desired quaternion (x,y,z,w) orientations of the chest in world coordinates. \n"
         + "Provide an empty list if no motion is desired\n"
         + "Should be numWaypoints elements long")
   public Quat4d[] chestWorldOrientation;
   
   @FieldDocumentation("Sequence of desired angular velocities of the chest. \n"
         + "Provide an empty list if zero velocity is desired\n"
         + "Should be numWaypoints elements long")
   public Vector3d[] chestAngularVelocity;

   @FieldDocumentation("Arm trajectory for the right arm. Should have numWaypoints waypoints.\n"
         + "Time in trajectory_points should match the corresponding element of timeAtWaypoint")
   public ArmJointTrajectoryPacket rightArmTrajectory;

   @FieldDocumentation("Arm trajectory for the left arm. Should have numWaypoints waypoints.\n"
         + "Time in trajectory_points should match the corresponding element of timeAtWaypoint")
   public ArmJointTrajectoryPacket leftArmTrajectory;

   @FieldDocumentation("Number of waypoints in the trajectory. Should be at least 1")
   public int numWaypoints = 0;
   
   @FieldDocumentation("Number of joints in a single arm")
   public int numJointsPerArm = 0;

   @IgnoreField
   @Optional(value = "scripting")
   public FootPosePacket leftFootPosePacket;

   @IgnoreField
   @Optional(value = "scripting")
   public FootPosePacket rightFootPosePacket;

   @IgnoreField
   @Optional(value = "scripting")
   public HandPosePacket leftHandPosePacket;

   @IgnoreField
   @Optional(value = "scripting")
   public HandPosePacket rightHandPosePacket;

   @IgnoreField
   @Optional(value = "scripting")
   public int controlledDoFHandRight;

   @IgnoreField
   @Optional(value = "scripting")
   public int controlledDoFHandLeft;

   @IgnoreField
   @Optional(value = "scripting")
   public int lockLevel;

   @IgnoreField
   @Optional(value = "scripting")
   public HashMap<String, Double> jointPreferedAngle;

   @IgnoreField
   @Optional(value = "scripting")
   public PelvisPosePacket pelvisPosePacket;

   public WholeBodyTrajectoryPacket()
   {
   }

   public WholeBodyTrajectoryPacket(WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket)
   {
//    public double[] timeAtWaypoint;
      if (wholeBodyTrajectoryPacket.timeAtWaypoint != null)
         this.timeAtWaypoint = Arrays.copyOf(wholeBodyTrajectoryPacket.timeAtWaypoint, wholeBodyTrajectoryPacket.timeAtWaypoint.length);

//    public Point3d[] pelvisWorldPosition;
      if (wholeBodyTrajectoryPacket.pelvisWorldPosition != null)
      {
         this.pelvisWorldPosition = new Point3d[wholeBodyTrajectoryPacket.pelvisWorldPosition.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.pelvisWorldPosition.length; i++)
         {
            this.pelvisWorldPosition[i] = new Point3d(wholeBodyTrajectoryPacket.pelvisWorldPosition[i]);
         }
      }

//    public Vector3d[] pelvisLinearVelocity;
      if (wholeBodyTrajectoryPacket.pelvisLinearVelocity != null)
      {
         this.pelvisLinearVelocity = new Vector3d[wholeBodyTrajectoryPacket.pelvisLinearVelocity.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.pelvisLinearVelocity.length; i++)
         {
            this.pelvisLinearVelocity[i] = new Vector3d(wholeBodyTrajectoryPacket.pelvisLinearVelocity[i]);
         }
      }

//    public Vector3d[] pelvisAngularVelocity;
      if (wholeBodyTrajectoryPacket.pelvisAngularVelocity != null)
      {
         this.pelvisAngularVelocity = new Vector3d[wholeBodyTrajectoryPacket.pelvisAngularVelocity.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.pelvisAngularVelocity.length; i++)
         {
            this.pelvisAngularVelocity[i] = new Vector3d(wholeBodyTrajectoryPacket.pelvisAngularVelocity[i]);
         }
      }

//    public Quat4d[] pelvisWorldOrientation;
      if (wholeBodyTrajectoryPacket.pelvisWorldOrientation != null)
      {
         this.pelvisWorldOrientation = new Quat4d[wholeBodyTrajectoryPacket.pelvisWorldOrientation.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.pelvisWorldOrientation.length; i++)
         {
            this.pelvisWorldOrientation[i] = new Quat4d(wholeBodyTrajectoryPacket.pelvisWorldOrientation[i]);
         }
      }


//    public Quat4d[] chestWorldOrientation;
      if (wholeBodyTrajectoryPacket.chestWorldOrientation != null)
      {
         this.chestWorldOrientation = new Quat4d[wholeBodyTrajectoryPacket.chestWorldOrientation.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.chestWorldOrientation.length; i++)
         {
            this.chestWorldOrientation[i] = new Quat4d(wholeBodyTrajectoryPacket.chestWorldOrientation[i]);
         }
      }

//    public Vector3d[] chestAngularVelocity;
      if (wholeBodyTrajectoryPacket.chestAngularVelocity != null)
      {
         this.chestAngularVelocity = new Vector3d[wholeBodyTrajectoryPacket.chestAngularVelocity.length];

         for (int i = 0; i < wholeBodyTrajectoryPacket.chestAngularVelocity.length; i++)
         {
            this.chestAngularVelocity[i] = new Vector3d(wholeBodyTrajectoryPacket.chestAngularVelocity[i]);
         }
      }

//    public ArmJointTrajectoryPacket rightArmTrajectory;
      if (wholeBodyTrajectoryPacket.rightArmTrajectory != null)
         this.rightArmTrajectory = new ArmJointTrajectoryPacket(wholeBodyTrajectoryPacket.rightArmTrajectory);

//    public ArmJointTrajectoryPacket leftArmTrajectory;
      if (wholeBodyTrajectoryPacket.leftArmTrajectory != null)
         this.leftArmTrajectory = new ArmJointTrajectoryPacket(wholeBodyTrajectoryPacket.leftArmTrajectory);

//    public int numWaypoints = 0;
      this.numWaypoints = wholeBodyTrajectoryPacket.numWaypoints;

//    public int numJointsPerArm = 0;  
      this.numJointsPerArm = wholeBodyTrajectoryPacket.numJointsPerArm;

      // public FootPosePacket leftFootPosePacket;
      if (wholeBodyTrajectoryPacket.leftFootPosePacket != null)
         this.leftFootPosePacket = new FootPosePacket(wholeBodyTrajectoryPacket.leftFootPosePacket);

      // public FootPosePacket rightFootPosePacket;
      if (wholeBodyTrajectoryPacket.rightFootPosePacket != null)
         this.rightFootPosePacket = new FootPosePacket(wholeBodyTrajectoryPacket.rightFootPosePacket);

      // public HandPosePacket leftHandPosePacket;
      if (wholeBodyTrajectoryPacket.leftHandPosePacket != null)
         this.leftHandPosePacket = new HandPosePacket(wholeBodyTrajectoryPacket.leftHandPosePacket);

      // public HandPosePacket rightHandPosePacket;
      if (wholeBodyTrajectoryPacket.rightHandPosePacket != null)
         this.rightHandPosePacket = new HandPosePacket(wholeBodyTrajectoryPacket.rightHandPosePacket);

      // public int controlledDoFHandRight;
      this.controlledDoFHandRight = wholeBodyTrajectoryPacket.controlledDoFHandRight;

      // public int controlledDoFHandLeft;
      this.controlledDoFHandLeft = wholeBodyTrajectoryPacket.controlledDoFHandLeft;

      // public int lockLevel;
      this.lockLevel = wholeBodyTrajectoryPacket.lockLevel;

      // public HashMap<String, Double> jointPreferedAngle;
      if (wholeBodyTrajectoryPacket.getJointPreferedAngle() != null)
      {
         this.jointPreferedAngle = new HashMap<String, Double>();

         for (String string : wholeBodyTrajectoryPacket.getJointPreferedAngle().keySet())
         {
            this.jointPreferedAngle.put(string, wholeBodyTrajectoryPacket.getJointPreferedAngle().get(string));
         }
      }

      // public PelvisPosePacket pelvisPosePacket;
      if (wholeBodyTrajectoryPacket.pelvisPosePacket != null)
      {
         this.pelvisPosePacket = new PelvisPosePacket(wholeBodyTrajectoryPacket.pelvisPosePacket);
      }

   }

   public WholeBodyTrajectoryPacket(int numWaypoints, int numJointsPerArm)
   {
      this.numJointsPerArm = numJointsPerArm;
      this.numWaypoints = numWaypoints;

      timeAtWaypoint = new double[numWaypoints];
      allocateArmTrajectories();
      allocateChestTrajectory();
      allocatePelvisTrajectory();
   }

   public WholeBodyTrajectoryPacket(Random random)
   {
      this(random.nextInt(128) + 1, random.nextInt(16) + 1);
      
      for(int i = 0; i < numWaypoints; i++)
      {
         if(i == 0)
         {
            timeAtWaypoint[i] = random.nextDouble();            
         }
         else
         {
            timeAtWaypoint[i] = random.nextDouble() + timeAtWaypoint[i - 1];
         }
         
         
         pelvisWorldPosition[i] = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
         pelvisLinearVelocity[i] = RandomTools.generateRandomVector(random);
         pelvisAngularVelocity[i] = RandomTools.generateRandomVector(random);
         pelvisWorldOrientation[i] = RandomTools.generateRandomQuaternion(random);
         
         chestWorldOrientation[i] = RandomTools.generateRandomQuaternion(random);
         chestAngularVelocity[i] = RandomTools.generateRandomVector(random);
      }
      
      pelvisWorldPosition = random.nextBoolean() ? null : pelvisWorldPosition;
      pelvisLinearVelocity = random.nextBoolean() ? null : pelvisLinearVelocity;
      pelvisAngularVelocity = random.nextBoolean() ? null : pelvisAngularVelocity;
      pelvisWorldOrientation = random.nextBoolean() ? null : pelvisWorldOrientation;
      
      chestWorldOrientation = random.nextBoolean() ? null : chestWorldOrientation;
      chestAngularVelocity = random.nextBoolean() ? null : chestAngularVelocity;
      
      rightArmTrajectory = new ArmJointTrajectoryPacket(random, RobotSide.RIGHT, this.numWaypoints);
      leftArmTrajectory = new ArmJointTrajectoryPacket(random, RobotSide.LEFT, this.numWaypoints);
   }

   public void allocateArmTrajectories()
   {
      rightArmTrajectory = new ArmJointTrajectoryPacket(RobotSide.RIGHT, numWaypoints, numJointsPerArm);
      leftArmTrajectory = new ArmJointTrajectoryPacket(RobotSide.LEFT, numWaypoints, numJointsPerArm);
   }

   public void allocatePelvisTrajectory()
   {
      pelvisWorldPosition = new Point3d[numWaypoints];
      pelvisLinearVelocity = new Vector3d[numWaypoints];
      pelvisAngularVelocity = new Vector3d[numWaypoints];
      pelvisWorldOrientation = new Quat4d[numWaypoints];

      for (int i = 0; i < numWaypoints; i++)
      {
         pelvisWorldPosition[i] = new Point3d();
         pelvisLinearVelocity[i] = new Vector3d();
         pelvisAngularVelocity[i] = new Vector3d();
         pelvisWorldOrientation[i] = new Quat4d();
      }
   }

   public double getTotalTrajectoryTime()
   {
      return timeAtWaypoint[timeAtWaypoint.length - 1];
   }

   public void allocateChestTrajectory()
   {
      chestAngularVelocity = new Vector3d[numWaypoints];
      chestWorldOrientation = new Quat4d[numWaypoints];

      for (int i = 0; i < numWaypoints; i++)
      {
         chestAngularVelocity[i] = new Vector3d();
         chestWorldOrientation[i] = new Quat4d();
      }
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryPacket otherTrajectory, double epsilon)
   {
      if ((numWaypoints != otherTrajectory.numWaypoints) || (numJointsPerArm != otherTrajectory.numJointsPerArm))
      {
         return false;
      }

      for (int w = 0; w < numWaypoints; w++)
      {
         if (!this.leftArmTrajectory.epsilonEquals(otherTrajectory.leftArmTrajectory, epsilon))
         {
            return false;
         }

         if (!this.rightArmTrajectory.epsilonEquals(otherTrajectory.rightArmTrajectory, epsilon))
         {
            return false;
         }

         if (Math.abs(this.timeAtWaypoint[w] - otherTrajectory.timeAtWaypoint[w]) > epsilon)
         {
            return false;
         }

         if(this.pelvisWorldPosition == null)
         {
            return otherTrajectory.pelvisWorldPosition == null;
         }
         else if (this.pelvisWorldPosition[w].epsilonEquals(otherTrajectory.pelvisWorldPosition[w], epsilon) == false)
         {
            return false;
         }

         if(this.pelvisWorldOrientation == null)
         {
            return otherTrajectory.pelvisWorldOrientation == null;
         }
         else if (this.pelvisWorldOrientation[w].epsilonEquals(otherTrajectory.pelvisWorldOrientation[w], epsilon) == false)
         {
            return false;
         }

         if(this.chestWorldOrientation == null)
         {
            return otherTrajectory.chestWorldOrientation == null;
         }
         else if (this.chestWorldOrientation[w].epsilonEquals(otherTrajectory.chestWorldOrientation[w], epsilon) == false)
         {
            return false;
         }
         
         if(this.chestAngularVelocity == null)
         {
            return otherTrajectory.chestAngularVelocity == null;
         }
         else if (this.chestAngularVelocity[w].epsilonEquals(otherTrajectory.chestAngularVelocity[w], epsilon) == false)
         {
            return false;
         }

         if(this.pelvisLinearVelocity == null)
         {
            return otherTrajectory.pelvisLinearVelocity == null;
         }
         else if (this.pelvisLinearVelocity[w].epsilonEquals(otherTrajectory.pelvisLinearVelocity[w], epsilon) == false)
         {
            return false;
         }
         if(this.pelvisAngularVelocity == null)
         {
            return otherTrajectory.pelvisAngularVelocity == null;
         }
         else if (this.pelvisAngularVelocity[w].epsilonEquals(otherTrajectory.pelvisAngularVelocity[w], epsilon) == false)
         {
            return false;
         }
         
         
      }

      return true;
   }

   public ArmJointTrajectoryPacket getArmJointTrajectoryPacket(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return leftArmTrajectory;
      else if (robotSide == RobotSide.RIGHT)
         return rightArmTrajectory;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + robotSide);
   }


   @Override
   public String toString()
   {
      return toString(3);
   }

   public String toString(int decimals)
   {
      ByteArrayOutputStream stream = new ByteArrayOutputStream();
      PrintStream out = new PrintStream(stream);

      stream.reset();
      String F = " %." + decimals + "f ";

      out.format("WBTraj Packet, # pts=" + numWaypoints);


      for (int w = 0; w < numWaypoints; w++)
      {
         out.format("---------------------\n");

         if (timeAtWaypoint != null)
            out.format("timeSincePrevious:       " + F + "\n", timeAtWaypoint[w]);

         if (pelvisWorldPosition != null)
            out.println("pelvisWorldPosition:    " + pelvisWorldPosition[w]);

         if (pelvisWorldOrientation != null)
            out.println("pelvisWorldOrientation: " + pelvisWorldOrientation[w]);

         if (chestWorldOrientation != null)
            out.println("chestWorldOrientation:  " + chestWorldOrientation[w]);

         out.print("leftArmJointAngle:  ");

         if (leftArmTrajectory != null)
         {
            for (int j = 0; j < numJointsPerArm; j++)
            {

               out.format(F + "\t", leftArmTrajectory.trajectoryPoints[w].positions[j]);
            }
         }

         out.println();

         out.print("rightArmJointAngle: ");

         if (rightArmTrajectory != null)
         {
            for (int j = 0; j < numJointsPerArm; j++)
            {
               out.format(F + "\t", rightArmTrajectory.trajectoryPoints[w].positions[j]);
            }
         }

         out.println();
      }

      return stream.toString();
   }



   @Override
   public WholeBodyTrajectoryPacket transform(RigidBodyTransform transform)
   {
      WholeBodyTrajectoryPacket ret = new WholeBodyTrajectoryPacket(this);

      ret.leftFootPosePacket = ret.leftFootPosePacket.transform(transform);
      ret.rightFootPosePacket = ret.rightFootPosePacket.transform(transform);
      ret.leftHandPosePacket = ret.leftHandPosePacket.transform(transform);
      ret.rightHandPosePacket = ret.rightHandPosePacket.transform(transform);
      ret.pelvisPosePacket = ret.pelvisPosePacket.transform(transform);

      return ret;
   }

   public FootPosePacket getFootPosePacket(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return leftFootPosePacket;
      else if (robotSide == RobotSide.RIGHT)
         return rightFootPosePacket;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + robotSide);
   }

   public void setFootPosePacket(FootPosePacket footPosePacket)
   {
      if (footPosePacket.getRobotSide() == RobotSide.LEFT)
         this.leftFootPosePacket = footPosePacket;
      else if (footPosePacket.getRobotSide() == RobotSide.RIGHT)
         this.rightFootPosePacket = footPosePacket;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + footPosePacket.getRobotSide());
   }
   
   public HandPosePacket getHandPosePacket(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return leftHandPosePacket;
      else if (robotSide == RobotSide.RIGHT)
         return rightHandPosePacket;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + robotSide);
   }

   public void setHandPosePacket(HandPosePacket handPosePacket)
   {
      if (handPosePacket.getRobotSide() == RobotSide.LEFT)
         this.leftHandPosePacket = handPosePacket;
      else if (handPosePacket.getRobotSide() == RobotSide.RIGHT)
         this.rightHandPosePacket = handPosePacket;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + handPosePacket.getRobotSide());
   }

   public int getControlledDoFHand(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return controlledDoFHandLeft;
      else if (robotSide == RobotSide.RIGHT)
         return controlledDoFHandRight;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + robotSide);
   }

   public void setControlledDoFHand(int controlledDoFHand, RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         this.controlledDoFHandLeft = controlledDoFHand;
      else if (robotSide == RobotSide.RIGHT)
         this.controlledDoFHandRight = controlledDoFHand;
      else
         throw new RuntimeException("Side must be either LEFT or RIGHT, robotSide=" + robotSide);
   }

   public int getLockLevel()
   {
      return lockLevel;
   }

   public void setLockLevel(int lockLevel)
   {
      this.lockLevel = lockLevel;
   }

   public HashMap<String, Double> getJointPreferedAngle()
   {
      HashMap<String, Double> ret = new HashMap<String, Double>();
      for (String string : this.jointPreferedAngle.keySet())
      {
         ret.put(string, jointPreferedAngle.get(string));
      }

      return ret;
   }

   public void setJointPreferedAngle(HashMap<String, Double> jointPreferedAnglePassedIn)
   {
      if (this.jointPreferedAngle == null)
         this.jointPreferedAngle = new HashMap<String, Double>();
      else
         this.jointPreferedAngle.clear();

      for (String string : jointPreferedAnglePassedIn.keySet())
      {
         this.jointPreferedAngle.put(string, jointPreferedAnglePassedIn.get(string));
      }
   }

   public Point3d[] getPelvisWorldPosition()
   {
      return pelvisWorldPosition;
   }

   public Quat4d[] getPelvisWorldOrientation()
   {
      return pelvisWorldOrientation;
   }


   public PelvisPosePacket getPelvisPosePacket()
   {
      return pelvisPosePacket;
   }

   public void setPelvisPosePacket(PelvisPosePacket pelvisPosePacket)
   {
      this.pelvisPosePacket = pelvisPosePacket;
   }


}
