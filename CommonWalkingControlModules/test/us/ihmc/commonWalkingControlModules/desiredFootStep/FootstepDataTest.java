package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;
import us.ihmc.utilities.io.streamingData.StreamingDataTCPClient;
import us.ihmc.utilities.io.streamingData.StreamingDataTCPServer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.test.JUnitTools;

/**
 * User: Matt
 * Date: 1/10/13
 */
public class FootstepDataTest
{
   /**
    * This test just verifies that FootstepData can be correctly serialized
    */
   @Test(timeout = 6000)
   public void testFootstepDataSerialization()
   {
      RigidBody endEffector = new RigidBody("rigid", ReferenceFrame.getWorldFrame());
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
      ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
      for (int i = 0; i < 3; i++)
      {
         FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), i, 0.0, 0.0);
         expectedContactPoints.add(framePoint);
      }

      boolean trustHeight = true;
      Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints, trustHeight);
      FootstepData footstepData = new FootstepData(footstep);
      JUnitTools.assertSerializable(footstepData);
   }

   /**
    * This test verifies that FootstepData can be sent and received using our current message passing utilities
    */
   @Test(timeout = 6000)
   public void testPassingFootstepData()
   {
      // setup comms
      long dataIdentifier = 1111L;
      int port = 7777;
      QueueBasedStreamingDataProducer<FootstepData> queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<FootstepData>(dataIdentifier);
      StreamingDataTCPServer tcpServer = createAndStartStreamingDataTCPServer(queueBasedStreamingDataProducer, port);
      FootstepDataConsumer footstepDataConsumer = new FootstepDataConsumer();
      StreamingDataTCPClient tcpClient = createStreamingDataConsumer(footstepDataConsumer, port);
      ThreadTools.sleep(100);
      queueBasedStreamingDataProducer.startProducingData();

      // create test footsteps
      ArrayList<Footstep> sentFootsteps = createRandomFootsteps(50);
      for (Footstep footstep : sentFootsteps)
      {
         FootstepData footstepData = new FootstepData(footstep);
         queueBasedStreamingDataProducer.queueDataToSend(footstepData);
      }

      ThreadTools.sleep(100);

      tcpServer.closeAndBlockTillFullyClosed();
      tcpClient.close();

      // verify received correctly
      ArrayList<Footstep> receivedFootsteps = footstepDataConsumer.getReconstructedFootsteps();
      compareFootstepsSentWithReceived(sentFootsteps, receivedFootsteps);
   }

   @Test(timeout = 6000)
   public void testPassingFootstepPath()
   {
      // setup comms
      long dataIdentifier = 2222L;
      int port = 2222;
      QueueBasedStreamingDataProducer<FootstepData[]> queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<FootstepData[]>(dataIdentifier);
      StreamingDataTCPServer tcpServer = createAndStartStreamingDataTCPServer(queueBasedStreamingDataProducer, port);

      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer();
      StreamingDataTCPClient tcpClient = createStreamingDataConsumer(footstepPathConsumer, port);
      ThreadTools.sleep(100);
      queueBasedStreamingDataProducer.startProducingData();

      // create test footsteps
      ArrayList<Footstep> sentFootsteps = createRandomFootsteps(50);
      FootstepData[] footstepsData = convertFootstepsToFootstepData(sentFootsteps);

      queueBasedStreamingDataProducer.queueDataToSend(footstepsData);
      ThreadTools.sleep(100);

      tcpServer.closeAndBlockTillFullyClosed();
      tcpClient.close();

      // verify received correctly
      ArrayList<Footstep> receivedFootsteps = footstepPathConsumer.getReconstructedFootsteps();
      compareFootstepsSentWithReceived(sentFootsteps, receivedFootsteps);
   }

   @Test(timeout = 6000)
   public void testPassingPauseCommand()
   {
      // setup comms
      long dataIdentifier = 3333L;
      int port = 3333;
      QueueBasedStreamingDataProducer<Boolean> queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<Boolean>(dataIdentifier);
      StreamingDataTCPServer tcpServer = createAndStartStreamingDataTCPServer(queueBasedStreamingDataProducer, port);

      PauseConsumer pauseConsumer = new PauseConsumer();
      StreamingDataTCPClient tcpClient = createStreamingDataConsumer(pauseConsumer, port);
      ThreadTools.sleep(100);
      queueBasedStreamingDataProducer.startProducingData();

      // create test commands
      ArrayList<Boolean> commands = new ArrayList<Boolean>();
      Random random = new Random(77);
      int numberToTest = 10;
      for (int i = 0; i < numberToTest; i++)
      {
         boolean isPaused = random.nextBoolean();
         commands.add(isPaused);
         queueBasedStreamingDataProducer.queueDataToSend(isPaused);
      }

      ThreadTools.sleep(100);

      tcpServer.closeAndBlockTillFullyClosed();
      tcpClient.close();

      // verify received correctly
      ArrayList<Boolean> reconstructedCommands = pauseConsumer.getReconstructedCommands();
      for (int i = 0; i < commands.size(); i++)
      {
         Boolean isPaused = commands.get(i);
         Boolean reconstructedCommand = reconstructedCommands.get(i);
         assertTrue(isPaused.booleanValue() == reconstructedCommand.booleanValue());
      }
   }

   @Test(timeout = 6000)
   public void testPassingFootstepPathAndPauseCommands()
   {
      // Create one server for two types of data
      int pathPort = 2222;
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(pathPort);

      QueueBasedStreamingDataProducer<FootstepData[]> pathQueueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<FootstepData[]>(2222L);
      streamingDataTCPServer.registerStreamingDataProducer(pathQueueBasedStreamingDataProducer);

      QueueBasedStreamingDataProducer<Boolean> pauseQueueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<Boolean>(3333L);
      streamingDataTCPServer.registerStreamingDataProducer(pauseQueueBasedStreamingDataProducer);

      streamingDataTCPServer.startOnAThread();

      // create one client for two types of data
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer();
      PauseConsumer pauseConsumer = new PauseConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", pathPort);
      streamingDataTCPClient.registerStreamingDataConsumer(footstepPathConsumer);
      streamingDataTCPClient.registerStreamingDataConsumer(pauseConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(100);
      pathQueueBasedStreamingDataProducer.startProducingData();
      pauseQueueBasedStreamingDataProducer.startProducingData();

      // send test footstep path
      ArrayList<Footstep> sentFootsteps = createRandomFootsteps(50);
      FootstepData[] footstepsData = convertFootstepsToFootstepData(sentFootsteps);

      pathQueueBasedStreamingDataProducer.queueDataToSend(footstepsData);
      ThreadTools.sleep(100);

      // send some commands
      ArrayList<Boolean> commands = new ArrayList<Boolean>();
      int numberToTest = 3;
      Random random = new Random(777);
      for (int i = 0; i < numberToTest; i++)
      {
         boolean isPaused = random.nextBoolean();
         commands.add(isPaused);
         pauseQueueBasedStreamingDataProducer.queueDataToSend(isPaused);
      }

      ThreadTools.sleep(100);

      // send another footstep path
      ArrayList<Footstep> sentFootsteps2 = createRandomFootsteps(50);
      footstepsData = convertFootstepsToFootstepData(sentFootsteps2);

      pathQueueBasedStreamingDataProducer.queueDataToSend(footstepsData);
      sentFootsteps.addAll(sentFootsteps2);
      ThreadTools.sleep(100);

      streamingDataTCPServer.closeAndBlockTillFullyClosed();
      streamingDataTCPClient.close();

      // verify footsteps received correctly
      ArrayList<Footstep> receivedFootsteps = footstepPathConsumer.getReconstructedFootsteps();
      compareFootstepsSentWithReceived(sentFootsteps, receivedFootsteps);

      // verify commands received correctly
      ArrayList<Boolean> reconstructedCommands = pauseConsumer.getReconstructedCommands();
      for (int i = 0; i < commands.size(); i++)
      {
         Boolean isPaused = commands.get(i);
         Boolean reconstructedCommand = reconstructedCommands.get(i);
         assertTrue(isPaused.booleanValue() == reconstructedCommand.booleanValue());
      }
   }

   @Test(timeout = 6000)
   public void testPassingFootstepStatus()
   {
      // setup comms
      long dataIdentifier = 1111L;
      int port = 7777;
      QueueBasedStreamingDataProducer<FootstepData> queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<FootstepData>(dataIdentifier);
      StreamingDataTCPServer tcpServer = createAndStartStreamingDataTCPServer(queueBasedStreamingDataProducer, port);

      FootstepStatusConsumer footstepStatusConsumer = new FootstepStatusConsumer();
      StreamingDataTCPClient tcpClient = createStreamingDataConsumer(footstepStatusConsumer, port);
      ThreadTools.sleep(100);
      queueBasedStreamingDataProducer.startProducingData();

      // create test footsteps
      Random random = new Random(777);
      ArrayList<Footstep> sentFootsteps = createRandomFootsteps(50);
      ArrayList<FootstepStatus> sentFootstepStatus = new ArrayList<FootstepStatus>();
      for (Footstep footstep : sentFootsteps)
      {
         FootstepStatus.Status status = FootstepStatus.Status.STARTED;
         boolean isComplete = random.nextBoolean();
         if (isComplete)
         {
            status = FootstepStatus.Status.COMPLETED;
         }

         FootstepStatus footstepStatus = new FootstepStatus(footstep, status);
         sentFootstepStatus.add(footstepStatus);
         queueBasedStreamingDataProducer.queueDataToSend(footstepStatus);
      }

      ThreadTools.sleep(100);

      tcpServer.closeAndBlockTillFullyClosed();
      tcpClient.close();

      // verify received correctly
      ArrayList<FootstepStatus> receivedFootsteps = footstepStatusConsumer.getReconstructedFootsteps();
      compareStatusSentWithReceived(sentFootstepStatus, receivedFootsteps);
   }

   private StreamingDataTCPServer createAndStartStreamingDataTCPServer(QueueBasedStreamingDataProducer<?> queueBasedStreamingDataProducer, int port)
   {
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.registerStreamingDataProducer(queueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      return streamingDataTCPServer;
   }

   private StreamingDataTCPClient createStreamingDataConsumer(StreamingDataConsumer streamingDataConsumer, int port)
   {
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", port);
      streamingDataTCPClient.registerStreamingDataConsumer(streamingDataConsumer);
      streamingDataTCPClient.connectToServer(false);

      return streamingDataTCPClient;
   }

   private ArrayList<Footstep> createRandomFootsteps(int number)
   {
      Random random = new Random(77);
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      for (int footstepNumber = 0; footstepNumber < number; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0),
                                        new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }

         boolean trustHeight = true;
         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints, trustHeight);
         footsteps.add(footstep);
      }

      return footsteps;
   }

   private void compareFootstepsSentWithReceived(ArrayList<Footstep> sentFootsteps, ArrayList<Footstep> receivedFootsteps)
   {
      for (int i = 0; i < sentFootsteps.size(); i++)
      {
         Footstep sentFootstep = sentFootsteps.get(i);
         Footstep receivedFootstep = receivedFootsteps.get(i);

         assertTrue(sentFootstep.getBody().getName().equals(receivedFootstep.getBody().getName()));
         assertTrue(sentFootstep.getPose().epsilonEquals(receivedFootstep.getPose(), 0.0001));

         for (int j = 0; j < sentFootstep.getExpectedContactPoints().size(); j++)
         {
            FramePoint sentFramePoint = sentFootstep.getExpectedContactPoints().get(j);
            FramePoint receivedFramePoint = receivedFootstep.getExpectedContactPoints().get(j);
            assertTrue(sentFramePoint.epsilonEquals(receivedFramePoint, 0.0001));
         }
      }
   }

   private void compareStatusSentWithReceived(ArrayList<FootstepStatus> sentFootstepStatus, ArrayList<FootstepStatus> receivedFootsteps)
   {
      for (int i = 0; i < sentFootstepStatus.size(); i++)
      {
         FootstepStatus footstepStatus = sentFootstepStatus.get(i);
         FootstepStatus reconstructedFootstepStatus = receivedFootsteps.get(i);
         assertTrue(footstepStatus.getStatus() == reconstructedFootstepStatus.getStatus());
         assertTrue(footstepStatus.getLocation().epsilonEquals(reconstructedFootstepStatus.getLocation(), 0.0001));
      }
   }

   private static FootstepData[] convertFootstepsToFootstepData(ArrayList<Footstep> footsteps)
   {
      FootstepData[] footstepsData = new FootstepData[footsteps.size()];

      int i = 0;
      for (Footstep footstep : footsteps)
      {
         FootstepData footstepData = new FootstepData(footstep);
         footstepsData[i++] = footstepData;
      }

      return footstepsData;
   }

   private class FootstepDataConsumer extends AbstractStreamingDataConsumer<FootstepData>
   {
      ArrayList<Footstep> reconstructedFootsteps = new ArrayList<Footstep>();

      public FootstepDataConsumer()
      {
         super(1111L, FootstepData.class);
      }

      protected void processPacket(FootstepData packet)
      {
         RigidBody endEffector = new RigidBody(packet.getRigidBodyName(), ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), packet.getLocation(), packet.getOrientation());
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (Point3d point : packet.getExpectedContactPoints())
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), point);
            expectedContactPoints.add(framePoint);
         }

         boolean trustHeight = true;
         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints, trustHeight);
         reconstructedFootsteps.add(footstep);
      }

      public ArrayList<Footstep> getReconstructedFootsteps()
      {
         return reconstructedFootsteps;
      }
   }


   private class FootstepPathConsumer extends AbstractStreamingDataConsumer<FootstepData[]>
   {
      ArrayList<Footstep> reconstructedFootstepPath = new ArrayList<Footstep>();

      public FootstepPathConsumer()
      {
         super(2222L, FootstepData[].class);
      }

      protected void processPacket(FootstepData[] packet)
      {
         for (FootstepData footstepData : packet)
         {
            RigidBody endEffector = new RigidBody(footstepData.getRigidBodyName(), ReferenceFrame.getWorldFrame());
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
            ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
            for (Point3d point : footstepData.getExpectedContactPoints())
            {
               FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), point);
               expectedContactPoints.add(framePoint);
            }

            boolean trustHeight = true;
            Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints, trustHeight);
            reconstructedFootstepPath.add(footstep);
         }
      }

      public ArrayList<Footstep> getReconstructedFootsteps()
      {
         return reconstructedFootstepPath;
      }
   }


   private class PauseConsumer extends AbstractStreamingDataConsumer<Boolean>
   {
      ArrayList<Boolean> reconstructedCommands = new ArrayList<Boolean>();

      public PauseConsumer()
      {
         super(3333L, Boolean.class);
      }

      protected void processPacket(Boolean packet)
      {
         Boolean isPaused = (Boolean) packet;
         reconstructedCommands.add(isPaused);
      }

      public ArrayList<Boolean> getReconstructedCommands()
      {
         return reconstructedCommands;
      }
   }


   private class FootstepStatusConsumer extends AbstractStreamingDataConsumer<FootstepStatus>
   {
      private ArrayList<FootstepStatus> reconstructedFootstepStatuses = new ArrayList<FootstepStatus>();

      public FootstepStatusConsumer()
      {
         super(1111L, FootstepStatus.class);
      }

      protected void processPacket(FootstepStatus footstepStatus)
      {
         reconstructedFootstepStatuses.add(footstepStatus);
      }

      public ArrayList<FootstepStatus> getReconstructedFootsteps()
      {
         return reconstructedFootstepStatuses;
      }
   }
}
