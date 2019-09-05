package us.ihmc.sensorProcessing.communication.producers;

import static us.ihmc.robotics.Assert.*;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public abstract class RobotConfigurationDataBufferTest
{
   @Test
   public void testAddingStuff()
   {
      RobotConfigurationDataBuffer buffer = new RobotConfigurationDataBuffer();
      FullHumanoidRobotModel setterFullRobotModel = getFullRobotModel();
      FullHumanoidRobotModel getterFullRobotModel = getFullRobotModel();

      OneDoFJointBasics[] setterJoints = FullRobotModelUtils.getAllJointsExcludingHands(setterFullRobotModel);
      OneDoFJointBasics[] getterJoints = FullRobotModelUtils.getAllJointsExcludingHands(getterFullRobotModel);
      ForceSensorDefinition[] forceSensorDefinitions = setterFullRobotModel.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = setterFullRobotModel.getIMUDefinitions();

      assertTrue(buffer.updateFullRobotModel(false, 0, getterFullRobotModel, null) == -1);

      for (int i = 0; i < RobotConfigurationDataBuffer.BUFFER_SIZE * 2; i++)
      {
         RobotConfigurationData test = RobotConfigurationDataFactory.create(setterJoints, forceSensorDefinitions, imuDefinitions);
         test.setWallTime(i * 11);
         test.setMonotonicTime(i * 10);
         test.getJointAngles().add(i * 10);
         buffer.receivedPacket(test);
      }

      for (int i = 0; i < RobotConfigurationDataBuffer.BUFFER_SIZE; i++)
      {
         assertEquals(buffer.updateFullRobotModel(true, i * 10 + 5, getterFullRobotModel, null), -1);
      }

      for (int i = RobotConfigurationDataBuffer.BUFFER_SIZE; i < 2.0 * RobotConfigurationDataBuffer.BUFFER_SIZE - 1; i++)
      {
         long timestamp = buffer.updateFullRobotModel(true, i * 10 + 5, getterFullRobotModel, null);
         assertEquals(getterJoints[0].getQ(), i * 10.0, 1e-7);
         assertEquals(i * 10, timestamp);
      }
   }

   @Test
   public void testWaitForTimestamp()
   {
	   for (int numberOfTestIterations = 0; numberOfTestIterations < 100; numberOfTestIterations++)
	   {
         final int TEST_COUNT = 5;
         final CountDownLatch countdownA = new CountDownLatch(TEST_COUNT);
         final CountDownLatch countdownB = new CountDownLatch(TEST_COUNT + 1);

         final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
         Thread receivedThread = new Thread()
         {
            @Override
            public void run()
            {
               for (int i = 0; i < TEST_COUNT + 1; i++)
               {
                  int timestamp = i * (TEST_COUNT * 10);
                  robotConfigurationDataBuffer.waitForTimestamp(timestamp);
                  countdownA.countDown();
                  countdownB.countDown();
               }
            }
         };
         receivedThread.start();
         ThreadTools.sleep(1000);
         assertEquals(5, countdownA.getCount());
         for (int i = 0; i < TEST_COUNT; i++)
         {
            ThreadTools.sleep(100);
            RobotConfigurationData data = new RobotConfigurationData();
            data.setWallTime(i * (TEST_COUNT * 10) + i);
            data.setMonotonicTime(i * (TEST_COUNT * 11) + i);
            robotConfigurationDataBuffer.receivedPacket(data);
         }

         try
         {
            assertTrue(countdownA.await(1, TimeUnit.SECONDS));
         }
         catch (InterruptedException e)
         {
            fail();
         }
         assertEquals(1, countdownB.getCount());
         RobotConfigurationData data = new RobotConfigurationData();
         data.setWallTime(TEST_COUNT * (TEST_COUNT * 11));
         data.setMonotonicTime(TEST_COUNT * (TEST_COUNT * 10));
         robotConfigurationDataBuffer.receivedPacket(data);

         try
         {
            assertTrue(countdownB.await(1, TimeUnit.SECONDS));
         }
         catch (InterruptedException e)
         {
            fail();
         }
	   }
   }

   public abstract FullHumanoidRobotModel getFullRobotModel();
}
