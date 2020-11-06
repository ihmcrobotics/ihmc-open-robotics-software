package us.ihmc.humanoidRobotics.communication.packets;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class HumanoidMessageToolsTest
{
   private static final double epsilon = 1e-8;
   @Test
   public void testCreateFootstepDataListMessage()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < 100; iter++)
      {
         int numberOfSteps = 10;
         List<FootstepDataMessage> desiredSteps = new ArrayList<>();
         for (int i = 0; i < numberOfSteps; i++)
            desiredSteps.add(createRandomFootstepDataMessage(random));

         double swingDuration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double transferDuration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double finalTransferDuration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         ExecutionMode executionMode = ExecutionMode.fromByte((byte) RandomNumbers.nextInt(random, 0, 2));

         FootstepDataListMessage dataListMessage = HumanoidMessageTools
               .createFootstepDataListMessage(desiredSteps, swingDuration, transferDuration, finalTransferDuration, executionMode);

         assertEquals(numberOfSteps, dataListMessage.getFootstepDataList().size());
         for (int i = 0; i < numberOfSteps; i++)
            assertMessagesEqual(desiredSteps.get(i), dataListMessage.getFootstepDataList().get(i));
         assertEquals(swingDuration, dataListMessage.getDefaultSwingDuration(), epsilon);
         assertEquals(transferDuration, dataListMessage.getDefaultTransferDuration(), epsilon);
         assertEquals(finalTransferDuration, dataListMessage.getFinalTransferDuration(), epsilon);
         assertEquals(executionMode, ExecutionMode.fromByte(dataListMessage.getQueueingProperties().getExecutionMode()));
         assertEquals(executionMode.toByte(), dataListMessage.getQueueingProperties().getExecutionMode());
      }
   }

   private static FootstepDataMessage createRandomFootstepDataMessage(Random random)
   {
      FootstepDataMessage message = new FootstepDataMessage();
      message.setExecutionDelayTime(RandomNumbers.nextDouble(random, 0.0, 5.0));
      message.setLiftoffDuration(RandomNumbers.nextDouble(random, 0.0, 5.0));
      message.setSwingDuration(RandomNumbers.nextDouble(random, 0.0, 5.0));
      message.setTransferDuration(RandomNumbers.nextDouble(random, 0.0, 5.0));
      message.setSwingTrajectoryBlendDuration(RandomNumbers.nextDouble(random, 0.0, 1.0));
      message.setTouchdownDuration(RandomNumbers.nextDouble(random, 0.0, 1.0));
      message.setSwingHeight(RandomNumbers.nextDouble(random, 0.0, 1.0));
      message.setRobotSide((byte) RandomNumbers.nextInt(random, 0, 1));
      message.getLocation().set(EuclidCoreRandomTools.nextPoint3D(random));
      message.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random));

      return message;
   }

   private static void assertMessagesEqual(FootstepDataMessage messageA, FootstepDataMessage messageB)
   {
      assertEquals(messageA.getExecutionDelayTime(), messageB.getExecutionDelayTime(), epsilon);
      assertEquals(messageA.getLiftoffDuration(), messageB.getLiftoffDuration(), epsilon);
      assertEquals(messageA.getSwingDuration(), messageB.getSwingDuration(), epsilon);
      assertEquals(messageA.getTransferDuration(), messageB.getTransferDuration(), epsilon);
      assertEquals(messageA.getSwingTrajectoryBlendDuration(), messageB.getSwingTrajectoryBlendDuration(), epsilon);
      assertEquals(messageA.getTouchdownDuration(), messageB.getTouchdownDuration(), epsilon);
      assertEquals(messageA.getSwingHeight(), messageB.getSwingHeight(), epsilon);
      assertEquals(messageA.getRobotSide(), messageB.getRobotSide());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(messageA.getLocation(), messageB.getLocation(), epsilon);
      EuclidCoreTestTools.assertQuaternionEquals(messageA.getOrientation(), messageB.getOrientation(), epsilon);
   }

}
