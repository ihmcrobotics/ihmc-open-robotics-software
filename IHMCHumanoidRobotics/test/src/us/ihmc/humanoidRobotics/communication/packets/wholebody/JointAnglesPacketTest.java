package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.robotSide.RobotSide;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class JointAnglesPacketTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNumberOfJoints()
   {
      int numberOfArmJoints = 5;
      int numberOfLegJoints = 7;
      int numberOfWaistJoints = 10;
      JointAnglesPacket jointAnglesPacket = new JointAnglesPacket(numberOfArmJoints, numberOfLegJoints, numberOfWaistJoints);

      assertEquals(numberOfArmJoints, jointAnglesPacket.getNumberOfArmJoints(RobotSide.LEFT));
      assertEquals(numberOfArmJoints, jointAnglesPacket.getNumberOfArmJoints(RobotSide.RIGHT));
      assertEquals(numberOfLegJoints, jointAnglesPacket.getNumberOfLegJoints(RobotSide.LEFT));
      assertEquals(numberOfLegJoints, jointAnglesPacket.getNumberOfLegJoints(RobotSide.RIGHT));
      assertEquals(numberOfWaistJoints, jointAnglesPacket.getNumberOfSpineJoints());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetAndPackWaist()
   {
      int numberOfArmJoints = 5;
      int numberOfLegJoints = 7;
      int numberOfWaistJoints = 10;
      JointAnglesPacket jointAnglesPacket = new JointAnglesPacket(numberOfArmJoints, numberOfLegJoints, numberOfWaistJoints);

      double[] waistAngles = new double[numberOfWaistJoints];
      Random random = new Random();

      for (int i = 0; i < numberOfWaistJoints; i++)
      {
         waistAngles[i] = random.nextDouble();
      }

      jointAnglesPacket.setSpineJointAngles(waistAngles);


      // check to make sure these got set
      double[] waistAnglesPacked = new double[numberOfWaistJoints];
      jointAnglesPacket.packSpineJointAngle(waistAnglesPacked);

      for (int i = 0; i < numberOfWaistJoints; i++)
      {
         assertEquals(waistAngles[i], waistAnglesPacked[i], 0.0);
      }


      // Check to make sure that a defensive copy was made
      double[] waistAnglesCopy = Arrays.copyOf(waistAngles, waistAngles.length);

      // change the original
      for (int i = 0; i < numberOfWaistJoints; i++)
      {
         waistAngles[i] = random.nextDouble();
      }


      for (int i = 0; i < numberOfWaistJoints; i++)
      {
         assertEquals(waistAnglesPacked[i], waistAnglesCopy[i], 0.0);
      }
   }


	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetAndPackArmJointAngles()
   {
      int numberOfArmJoints = 5;
      int numberOfLegJoints = 7;
      int numberOfWaistJoints = 10;
      JointAnglesPacket jointAnglesPacket = new JointAnglesPacket(numberOfArmJoints, numberOfLegJoints, numberOfWaistJoints);


      for (RobotSide robotSide : RobotSide.values)
      {
         double[] armAngles = new double[numberOfArmJoints];
         Random random = new Random();

         for (int i = 0; i < numberOfArmJoints; i++)
         {
            armAngles[i] = random.nextDouble();
         }

         jointAnglesPacket.setArmJointAngle(robotSide, armAngles);


         // check to make sure these got set
         double[] armAnglesPacked = new double[numberOfArmJoints];
         jointAnglesPacket.packArmJointAngle(robotSide, armAnglesPacked);

         for (int i = 0; i < numberOfArmJoints; i++)
         {
            assertEquals(armAngles[i], armAnglesPacked[i], 0.0);
         }


         // Check to make sure that a defensive copy was made
         double[] armAnglesCopy = Arrays.copyOf(armAngles, armAngles.length);

         // change the original
         for (int i = 0; i < numberOfArmJoints; i++)
         {
            armAngles[i] = random.nextDouble();
         }


         for (int i = 0; i < numberOfArmJoints; i++)
         {
            assertEquals(armAnglesPacked[i], armAnglesCopy[i], 0.0);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetAndPackLegJointAngles()
   {
      int numberOfArmJoints = 5;
      int numberOfLegJoints = 7;
      int numberOfWaistJoints = 10;
      JointAnglesPacket jointAnglesPacket = new JointAnglesPacket(numberOfArmJoints, numberOfLegJoints, numberOfWaistJoints);


      for (RobotSide robotSide : RobotSide.values)
      {
         double[] legAngles = new double[numberOfLegJoints];
         Random random = new Random();

         for (int i = 0; i < numberOfLegJoints; i++)
         {
            legAngles[i] = random.nextDouble();
         }

         jointAnglesPacket.setLegJointAngle(robotSide, legAngles);


         // check to make sure these got set
         double[] legAnglesPacked = new double[numberOfLegJoints];
         jointAnglesPacket.packLegJointAngle(robotSide, legAnglesPacked);

         for (int i = 0; i < numberOfLegJoints; i++)
         {
            assertEquals(legAngles[i], legAnglesPacked[i], 0.0);
         }


         // Check to make sure that a defensive copy was made
         double[] legAnglesCopy = Arrays.copyOf(legAngles, legAngles.length);

         // change the original
         for (int i = 0; i < numberOfLegJoints; i++)
         {
            legAngles[i] = random.nextDouble();
         }


         for (int i = 0; i < numberOfLegJoints; i++)
         {
            assertEquals(legAnglesPacked[i], legAnglesCopy[i], 0.0);
         }
      }
   }




}
