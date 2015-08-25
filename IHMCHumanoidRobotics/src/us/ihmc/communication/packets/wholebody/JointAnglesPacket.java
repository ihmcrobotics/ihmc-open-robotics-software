package us.ihmc.communication.packets.wholebody;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the IHMC joint position controller to move Atlas's joints to the desired angles.")
public class JointAnglesPacket extends IHMCRosApiPacket<JointAnglesPacket> implements VisualizablePacket
{
   @FieldDocumentation("trajectoryTime specifies how fast or how slow to move to the desired joint angles")
   public double trajectoryTime;
   @FieldDocumentation("neckJointAngle neck_ry")
   public double neckJointAngle;
   @FieldDocumentation("spineJointAngles back_bky back_bkx back_bkz")
   public double[] spineJointAngle;
   @FieldDocumentation("rightLegJointAngle r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx")
   public double[] rightLegJointAngle;
   @FieldDocumentation("leftLegJointAngle l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx")
   public double[] leftLegJointAngle;
   @FieldDocumentation("rightArmJointAngle r_arm_shz r_arm_shx r_arm_ely r_arm_elx r_arm_wry r_arm_wrx r_arm_wry2")
   public double[] rightArmJointAngle;
   @FieldDocumentation("leftArmJointAngle l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2")
   public double[] leftArmJointAngle;

   @FieldDocumentation("spineJointLimits back_bky back_bkx back_bkz")
   public int[] spineTorqueLimit;
   @FieldDocumentation("rightLegJointTorqueLimit r_leg_hpz r_leg_hpx r_leg_hpy r_leg_kny r_leg_aky r_leg_akx")
   public int[] rightLegTorqueLimit;
   @FieldDocumentation("leftLegJointTorqueLimit l_leg_hpz l_leg_hpx l_leg_hpy l_leg_kny l_leg_aky l_leg_akx")
   public int[] leftLegTorqueLimit;
   @FieldDocumentation("rightArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2")
   public int[] rightArmTorqueLimit;
   @FieldDocumentation("leftArmTorqueLimit l_arm_shz l_arm_shx l_arm_ely l_arm_elx l_arm_wry l_arm_wrx l_arm_wry2")
   public int[] leftArmTorqueLimit;
   
   @FieldDocumentation("keepLeftHandInTaskspacePosition specifies whether the position controller should try to maintain the left hand position in task space")
   public boolean keepLeftHandInTaskspacePosition;
   
   @FieldDocumentation("keepRightHandInTaskspacePosition specifies whether the position controller should try to maintain the right hand position in task space")
   public boolean keepRightHandInTaskspacePosition;
   
   @IgnoreField
   @FieldDocumentation("if flattenFeetAtTheEnd is true, the ankles will move at the end of the trajectory to adapt to the inclination of the ground")
   public boolean flattenFeetAtTheEnd;
   
   public JointAnglesPacket()
   {
   }

   public JointAnglesPacket(int numJointsPerArm, int numJointPerLeg, int numJointsSpine)
   {
      spineJointAngle    = new double[numJointsSpine];
      rightLegJointAngle = new double[numJointPerLeg];
      leftLegJointAngle  = new double[numJointPerLeg];
      rightArmJointAngle = new double[numJointsPerArm];
      leftArmJointAngle  = new double[numJointsPerArm];
      
      spineTorqueLimit    = new int[numJointsSpine];
      rightLegTorqueLimit = new int[numJointPerLeg];
      leftLegTorqueLimit  = new int[numJointPerLeg];
      rightArmTorqueLimit = new int[numJointsPerArm];
      leftArmTorqueLimit  = new int[numJointsPerArm];
      
      flattenFeetAtTheEnd = false;
   }

   public JointAnglesPacket(Random rand)
   {
      this(Math.abs(rand.nextInt(10000)), Math.abs(rand.nextInt(10000)), Math.abs(rand.nextInt(10000)));
      int numJointsPerArm = rightArmJointAngle.length;
      int numJointPerLeg = rightLegJointAngle.length;
      int numJointsSpine = spineJointAngle.length;

      for (int i = 0; i < numJointsPerArm; i++)
      {
         rightArmJointAngle[i] = rand.nextDouble();
         leftArmJointAngle[i] = rand.nextDouble();
         
         rightArmTorqueLimit[i] = rand.nextInt();
         leftArmTorqueLimit[i] = rand.nextInt();
      }

      for (int i = 0; i < numJointPerLeg; i++)
      {
         rightLegJointAngle[i] = rand.nextDouble();
         leftLegJointAngle[i] = rand.nextDouble();

         rightLegTorqueLimit[i] = rand.nextInt();
         leftLegTorqueLimit[i] = rand.nextInt();      
      }

      for (int i = 0; i < numJointsSpine; i++)
      {
         spineJointAngle[i] = rand.nextDouble();
         spineTorqueLimit[i] = rand.nextInt();
      }
      
      neckJointAngle = rand.nextDouble();
      
      keepLeftHandInTaskspacePosition = rand.nextBoolean();
      keepRightHandInTaskspacePosition = rand.nextBoolean();
      
      flattenFeetAtTheEnd =  rand.nextBoolean();
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Get number of joints
   
   public int getNumberOfArmJoints(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return this.leftArmTorqueLimit.length;
      else
         return this.rightArmTorqueLimit.length;
   }

   public int getNumberOfLegJoints(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return this.leftLegTorqueLimit.length;
      else
         return this.rightLegTorqueLimit.length;
   }

   public int getNumberOfSpineJoints()
   {
      return this.spineTorqueLimit.length;
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Set joint angles
   
   public void setLeftLegJointAngle(double[] leftLegJointAngle)
   {
      copyArray(leftLegJointAngle, this.leftLegJointAngle);
   }

   public void setRightLegJointAngle(double[] rightLegJointAngle)
   {
      copyArray(rightLegJointAngle, this.rightLegJointAngle);
   }

   public void setLegJointAngle(RobotSide robotSide, double[] legJointAngle)
   {
      if (robotSide == RobotSide.LEFT)
         setLeftLegJointAngle(legJointAngle);
      else
         setRightLegJointAngle(legJointAngle);
   }

   public void setLeftArmJointAngle(double[] leftArmJointAngle)
   {
      copyArray(leftArmJointAngle, this.leftArmJointAngle);
   }

   public void setRightArmJointAngle(double[] rightArmJointAngle)
   {
      copyArray(rightArmJointAngle, this.rightArmJointAngle);
   }

   public void setArmJointAngle(RobotSide robotSide, double[] armJointAngle)
   {
      if (robotSide == RobotSide.LEFT)
         setLeftArmJointAngle(armJointAngle);
      else
         setRightArmJointAngle(armJointAngle);
   }

   public void setSpineJointAngles(double[] spineJointAngle)
   {
      copyArray(spineJointAngle, this.spineJointAngle);
   }
   
   public void setNeckJointAngle(double neckJointAngle)
   {
      this.neckJointAngle = neckJointAngle;
   }
   
   public void setKeepHandInTaskspacePosition(RobotSide robotSide, boolean keepHandInTaskspacePosition)
   {
      if(robotSide.equals(RobotSide.LEFT)) 
         keepLeftHandInTaskspacePosition = keepHandInTaskspacePosition;
      else 
         keepRightHandInTaskspacePosition = keepHandInTaskspacePosition;
   }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Trajectory time
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      //      this.trajectoryTime = trajectoryTime;

      // limiting motor speed for safe joint speed. if the arms exceed (700 rad / sec) / (100 gear ratio) = 7 rad/sec we are in trouble
      this.trajectoryTime = MathTools.clipToMinMax(trajectoryTime, 2.0, Double.MAX_VALUE);
   }
   
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Pack joint angles

   public void packSpineJointAngle(double[] arrayToPack)
   {
      copyArray(this.spineJointAngle, arrayToPack);
   }
   
   public double getNeckJointAngle()
   {
      return neckJointAngle;
   }

   public void packRightLegJointAngle(double[] arrayToPack)
   {
      copyArray(this.rightLegJointAngle, arrayToPack);
   }

   public void packLeftLegJointAngle(double[] arrayToPack)
   {
      copyArray(this.leftLegJointAngle, arrayToPack);
   }

   public void packLegJointAngle(RobotSide robotSide, double[] arrayToPack)
   {
      if (robotSide == RobotSide.LEFT)
         packLeftLegJointAngle(arrayToPack);
      else
         packRightLegJointAngle(arrayToPack);
   }

   public void packRightArmJointAngle(double[] arrayToPack)
   {
      copyArray(this.rightArmJointAngle, arrayToPack);
   }

   public void packLeftArmJointAngle(double[] arrayToPack)
   {
      copyArray(this.leftArmJointAngle, arrayToPack);
   }

   public void packArmJointAngle(RobotSide robotSide, double[] arrayToPack)
   {
      if (robotSide == RobotSide.LEFT)
         packLeftArmJointAngle(arrayToPack);
      else
         packRightArmJointAngle(arrayToPack);
   }
   
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Set torque limits
   
   public void setLeftLegTorqueLimit(int[] leftLegTorqueLimit)
   {
      copyArray(leftLegTorqueLimit, this.leftLegTorqueLimit);
   }

   public void setRightLegTorqueLimit(int[] rightLegTorqueLimit)
   {
      copyArray(rightLegTorqueLimit, this.rightLegTorqueLimit);
   }

   public void setLegTorqueLimit(RobotSide robotSide, int[] legTorqueLimit)
   {
      if (robotSide == RobotSide.LEFT)
         setLeftLegTorqueLimit(legTorqueLimit);
      else
         setRightLegTorqueLimit(legTorqueLimit);
   }

   public void setLeftArmTorqueLimit(int[] leftArmTorqueLimit)
   {
      copyArray(leftArmTorqueLimit, this.leftArmTorqueLimit);
   }

   public void setRightArmTorqueLimit(int[] rightArmTorqueLimit)
   {
      copyArray(rightArmTorqueLimit, this.rightArmTorqueLimit);
   }

   public void setArmTorqueLimit(RobotSide robotSide, int[] armTorqueLimit)
   {
      if (robotSide == RobotSide.LEFT)
         setLeftArmTorqueLimit(armTorqueLimit);
      else
         setRightArmTorqueLimit(armTorqueLimit);
   }
   
   public void setSpineTorqueLimits(int[] spineTorqueLimits)
   {
      copyArray(spineTorqueLimits, this.spineTorqueLimit);
   }
   
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Pack torque limits
   
   public void packSpineTorqueLimit(int[] arrayToPack)
   {
      copyArray(this.spineTorqueLimit, arrayToPack);
   }

   public void packRightLegTorqueLimit(int[] arrayToPack)
   {
      copyArray(this.rightLegTorqueLimit, arrayToPack);
   }

   public void packLeftLegTorqueLimit(int[] arrayToPack)
   {
      copyArray(this.leftLegTorqueLimit, arrayToPack);
   }

   public void packLegTorqueLimit(RobotSide robotSide, int[] arrayToPack)
   {
      if (robotSide == RobotSide.LEFT)
         packLeftLegTorqueLimit(arrayToPack);
      else
         packRightLegTorqueLimit(arrayToPack);
   }

   public void packRightArmTorqueLimit(int[] arrayToPack)
   {
      copyArray(this.rightArmTorqueLimit, arrayToPack);
   }

   public void packLeftArmTorqueLimit(int[] arrayToPack)
   {
      copyArray(this.leftArmTorqueLimit, arrayToPack);
   }

   public void packArmTorqueLimit(RobotSide robotSide, int[] arrayToPack)
   {
      if (robotSide == RobotSide.LEFT)
         packLeftArmTorqueLimit(arrayToPack);
      else
         packRightArmTorqueLimit(arrayToPack);
   }   

   ////////////////////////////////////////////////////////////////////////////////////////////////////
   // Helper functions
   
   @Override
   public boolean epsilonEquals(JointAnglesPacket other, double epsilon)
   {
      boolean ret = true;
      
      if(this.leftArmJointAngle == null || other.leftArmJointAngle == null)
         ret &= this.leftArmJointAngle == other.leftArmJointAngle;
      else
         ret &= ArrayTools.deltaEquals(this.leftArmJointAngle, other.leftArmJointAngle, epsilon);
      
      if(this.rightArmJointAngle == null || other.rightArmJointAngle == null)
         ret &= this.rightArmJointAngle == other.rightArmJointAngle;
      else
         ret &= ArrayTools.deltaEquals(this.rightArmJointAngle, other.rightArmJointAngle, epsilon);
      
      if(this.leftLegJointAngle == null || other.leftLegJointAngle == null)
         ret &= this.leftLegJointAngle == other.leftLegJointAngle;
      else
         ret &= ArrayTools.deltaEquals(this.leftLegJointAngle, other.leftLegJointAngle, epsilon);
      
      if(this.rightLegJointAngle == null || other.rightLegJointAngle == null)
         ret &= this.rightLegJointAngle == other.rightLegJointAngle;
      else
         ret &= ArrayTools.deltaEquals(this.rightLegJointAngle, other.rightLegJointAngle, epsilon);
      
      if(this.spineJointAngle == null || other.spineJointAngle == null)
         ret &= this.spineJointAngle == other.spineJointAngle;
      else
         ret &= ArrayTools.deltaEquals(this.spineJointAngle, other.spineJointAngle, epsilon);
      
      ret &= MathTools.epsilonEquals(this.neckJointAngle, other.neckJointAngle, epsilon);
      
      return ret;
   }

   public String toString()
   {
      return "JointAnglePacket trajTime: " + FormattingTools.getFormattedDecimal2D(getTrajectoryTime());
   }

   public static void main(String[] args)
   {
      JointAnglesPacket jointAnglesPacket = new JointAnglesPacket();

      double[] angles = new double[] { 0.0, 1.0, 2.0, 3.0, 4.0 };

      ArrayTools.printArray(angles, System.out);

      jointAnglesPacket.setLeftLegJointAngle(angles);

      angles[0] = 99.0;

      ArrayTools.printArray(jointAnglesPacket.leftLegJointAngle, System.out);
   }

   private static void copyArray(double[] sourceArray, double[] destinationArray)
   {
      if (sourceArray.length != destinationArray.length)
         throw new RuntimeException("Arrays must be equal length: sourceArray.length=" + sourceArray.length + ", destinationArray.length="
               + destinationArray.length);

      for (int i = 0; i < sourceArray.length; i++)
      {
         destinationArray[i] = sourceArray[i];
      }
   }
   
   private static void copyArray(int[] sourceArray, int[] destinationArray)
   {
      if (sourceArray.length != destinationArray.length)
         throw new RuntimeException("Arrays must be equal length: sourceArray.length=" + sourceArray.length + ", destinationArray.length="
               + destinationArray.length);

      for (int i = 0; i < sourceArray.length; i++)
      {
         destinationArray[i] = sourceArray[i];
      }
   }
}
