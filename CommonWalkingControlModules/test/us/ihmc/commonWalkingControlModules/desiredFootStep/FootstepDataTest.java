package us.ihmc.commonWalkingControlModules.desiredFootStep;

import org.junit.Test;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.io.streamingData.StreamingDataTCPClient;
import us.ihmc.utilities.io.streamingData.StreamingDataTCPServer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.test.JUnitTools;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertTrue;

/**
 * User: Matt
 * Date: 1/10/13
 */
public class FootstepDataTest
{
   @Test
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

      Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
      Serializable footstepData = new FootstepData(footstep);
      JUnitTools.assertSerializable(footstepData);
   }

   @Test
   public void testPassingFootstepData()
   {
      QueueBasedStreamingDataProducer queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer(1111L);
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(7777);
      streamingDataTCPServer.registerStreamingDataProducer(queueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      FootstepDataConsumer footstepDataConsumer = new FootstepDataConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", 7777);
      streamingDataTCPClient.registerStreamingDataConsumer(footstepDataConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(1000);

      queueBasedStreamingDataProducer.startProducingData();

      Random random = new Random(77);
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      int numberToTest = 50;
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }


         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         footsteps.add(footstep);
         //         System.out.println("footstep.getPose() = " + footstep.getPose());

         Serializable footstepData = new FootstepData(footstep);

         queueBasedStreamingDataProducer.queueDataToSend(footstepData);
      }
      ArrayList<Footstep> reconstructedFootsteps = footstepDataConsumer.getReconstructedFootsteps();

      ThreadTools.sleep(1000);
      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Footstep reconstructedFootstep = reconstructedFootsteps.get(i);
         //         System.out.println(" reconstructedFootstep.getPose() = " + reconstructedFootstep.getPose());
         assertTrue(footstep.getBody().getName().equals(reconstructedFootstep.getBody().getName()));
         assertTrue(footstep.getPose().epsilonEquals(reconstructedFootstep.getPose(), 0.0001));
         for (int j = 0; j < footstep.getExpectedContactPoints().size(); j++)
         {
            FramePoint framePoint = footstep.getExpectedContactPoints().get(j);
            FramePoint reconstructedFramePoint = reconstructedFootstep.getExpectedContactPoints().get(j);
            assertTrue(framePoint.epsilonEquals(reconstructedFramePoint, 0.0001));
         }
      }

      streamingDataTCPClient.close();
      streamingDataTCPServer.close();
   }

   @Test
   public void testPassingFootstepPath()
   {
      int port = 2222;
      QueueBasedStreamingDataProducer queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer(port);
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.registerStreamingDataProducer(queueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", port);
      streamingDataTCPClient.registerStreamingDataConsumer(footstepPathConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(1000);

      queueBasedStreamingDataProducer.startProducingData();

      Random random = new Random(77);
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      ArrayList<Serializable> footstepsData = new ArrayList<Serializable>();
      int numberToTest = 50;
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }


         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         footsteps.add(footstep);
         //         System.out.println("footstep.getPose() = " + footstep.getPose());

         Serializable footstepData = new FootstepData(footstep);
         footstepsData.add(footstepData);
      }

      queueBasedStreamingDataProducer.queueDataToSend(footstepsData);

      ThreadTools.sleep(1000);

      ArrayList<Footstep> reconstructedFootsteps = footstepPathConsumer.getReconstructedFootsteps();

      ThreadTools.sleep(1000);
      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Footstep reconstructedFootstep = reconstructedFootsteps.get(i);
         //         System.out.println(" reconstructedFootstep.getPose() = " + reconstructedFootstep.getPose());
         assertTrue(footstep.getBody().getName().equals(reconstructedFootstep.getBody().getName()));
         assertTrue(footstep.getPose().epsilonEquals(reconstructedFootstep.getPose(), 0.0001));
         for (int j = 0; j < footstep.getExpectedContactPoints().size(); j++)
         {
            FramePoint framePoint = footstep.getExpectedContactPoints().get(j);
            FramePoint reconstructedFramePoint = reconstructedFootstep.getExpectedContactPoints().get(j);
            assertTrue(framePoint.epsilonEquals(reconstructedFramePoint, 0.0001));
         }
      }

      streamingDataTCPClient.close();
      streamingDataTCPServer.close();
   }

   @Test
   public void testPassingPauseCommand()
   {
      int port = 3333;
      QueueBasedStreamingDataProducer queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer(port);
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.registerStreamingDataProducer(queueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      PauseConsumer pauseConsumer = new PauseConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", port);
      streamingDataTCPClient.registerStreamingDataConsumer(pauseConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(1000);

      queueBasedStreamingDataProducer.startProducingData();

      ArrayList<Boolean> commands = new ArrayList<Boolean>();
      Random random = new Random(77);
      int numberToTest = 10;
      for (int i = 0; i < numberToTest; i++)
      {
         boolean isPaused = random.nextBoolean();
         commands.add(isPaused);
         queueBasedStreamingDataProducer.queueDataToSend(isPaused);
      }
      ThreadTools.sleep(200);

      ArrayList<Boolean> reconstructedCommands = pauseConsumer.getReconstructedCommands();
      for (int i = 0; i < commands.size(); i++)
      {
         Boolean isPaused = commands.get(i);
         Boolean reconstructedCommand = reconstructedCommands.get(i);
         //         System.out.println(" reconstructedFootstep.getPose() = " + reconstructedFootstep.getPose());
         assertTrue(isPaused.booleanValue() == reconstructedCommand.booleanValue());
      }

      streamingDataTCPClient.close();
      streamingDataTCPServer.close();
   }

   @Test
   public void testPassingFootstepPathAndPauseCommands()
   {
      int pathPort = 2222;
      QueueBasedStreamingDataProducer pathQueueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer(2222L);
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(pathPort);
      streamingDataTCPServer.registerStreamingDataProducer(pathQueueBasedStreamingDataProducer);
      QueueBasedStreamingDataProducer pauseQueueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer(3333L);
      streamingDataTCPServer.registerStreamingDataProducer(pauseQueueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer();
      PauseConsumer pauseConsumer = new PauseConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", pathPort);
      streamingDataTCPClient.registerStreamingDataConsumer(footstepPathConsumer);
      streamingDataTCPClient.registerStreamingDataConsumer(pauseConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(1000);

      pathQueueBasedStreamingDataProducer.startProducingData();
      pauseQueueBasedStreamingDataProducer.startProducingData();

      Random random = new Random(77);
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      ArrayList<Serializable> footstepsData = new ArrayList<Serializable>();
      int numberToTest = 10;
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }


         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         footsteps.add(footstep);
         //         System.out.println("footstep.getPose() = " + footstep.getPose());

         Serializable footstepData = new FootstepData(footstep);
         footstepsData.add(footstepData);
      }
      pathQueueBasedStreamingDataProducer.queueDataToSend(footstepsData);

      ArrayList<Boolean> commands = new ArrayList<Boolean>();
      numberToTest = 3;
      for (int i = 0; i < numberToTest; i++)
      {
         boolean isPaused = random.nextBoolean();
         commands.add(isPaused);
         pauseQueueBasedStreamingDataProducer.queueDataToSend(isPaused);
      }
      ThreadTools.sleep(200);

      footstepsData = new ArrayList<Serializable>();
      numberToTest = 5;
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }


         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         footsteps.add(footstep);
         //         System.out.println("footstep.getPose() = " + footstep.getPose());

         Serializable footstepData = new FootstepData(footstep);
         footstepsData.add(footstepData);
      }
      pathQueueBasedStreamingDataProducer.queueDataToSend(footstepsData);

      ThreadTools.sleep(1000);

      ArrayList<Footstep> reconstructedFootsteps = footstepPathConsumer.getReconstructedFootsteps();

      ThreadTools.sleep(1000);
      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Footstep reconstructedFootstep = reconstructedFootsteps.get(i);
         //         System.out.println(" reconstructedFootstep.getPose() = " + reconstructedFootstep.getPose());
         assertTrue(footstep.getBody().getName().equals(reconstructedFootstep.getBody().getName()));
         assertTrue(footstep.getPose().epsilonEquals(reconstructedFootstep.getPose(), 0.0001));
         for (int j = 0; j < footstep.getExpectedContactPoints().size(); j++)
         {
            FramePoint framePoint = footstep.getExpectedContactPoints().get(j);
            FramePoint reconstructedFramePoint = reconstructedFootstep.getExpectedContactPoints().get(j);
            assertTrue(framePoint.epsilonEquals(reconstructedFramePoint, 0.0001));
         }
      }

      ArrayList<Boolean> reconstructedCommands = pauseConsumer.getReconstructedCommands();
      for (int i = 0; i < commands.size(); i++)
      {
         Boolean isPaused = commands.get(i);
         Boolean reconstructedCommand = reconstructedCommands.get(i);
         //         System.out.println(" reconstructedFootstep.getPose() = " + reconstructedFootstep.getPose());
         assertTrue(isPaused.booleanValue() == reconstructedCommand.booleanValue());
      }

      streamingDataTCPClient.close();
      streamingDataTCPServer.close();
   }

   @Test
   public void testPassingFootstepStatus()
   {
      QueueBasedStreamingDataProducer<FootstepStatus> queueBasedStreamingDataProducer = new QueueBasedStreamingDataProducer<FootstepStatus>(1111L);
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(7777);
      streamingDataTCPServer.registerStreamingDataProducer(queueBasedStreamingDataProducer);
      streamingDataTCPServer.startOnAThread();

      FootstepStatusConsumer footstepStatusConsumer = new FootstepStatusConsumer();
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient("localhost", 7777);
      streamingDataTCPClient.registerStreamingDataConsumer(footstepStatusConsumer);
      streamingDataTCPClient.connectToServer(false);

      ThreadTools.sleep(1000);

      queueBasedStreamingDataProducer.startProducingData();

      Random random = new Random(77);
      ArrayList<FootstepStatus> footstepStatuses = new ArrayList<FootstepStatus>();
      int numberToTest = 10;
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = new RigidBody("rigid_" + footstepNumber, ReferenceFrame.getWorldFrame());
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }


         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         FootstepData footstepData = new FootstepData(footstep);
         FootstepStatus.Status status = FootstepStatus.Status.STARTED;
         boolean isComplete = random.nextBoolean();
         if(isComplete)
         {
            status = FootstepStatus.Status.COMPLETED;
         }
         FootstepStatus footstepStatus = new FootstepStatus(footstepData, status);
         footstepStatuses.add(footstepStatus);
         queueBasedStreamingDataProducer.queueDataToSend(footstepStatus);
      }
      ArrayList<FootstepStatus> reconstructedFootsteps = footstepStatusConsumer.getReconstructedFootsteps();

      ThreadTools.sleep(1000);
      for (int i = 0; i < footstepStatuses.size(); i++)
      {
         FootstepStatus footstepStatus = footstepStatuses.get(i);
         FootstepStatus reconstructedFootstepStatus = reconstructedFootsteps.get(i);
         assertTrue(footstepStatus.getStatus() == reconstructedFootstepStatus.getStatus());
         assertTrue(footstepStatus.getFootstepData().getLocation().epsilonEquals(reconstructedFootstepStatus.getFootstepData().getLocation(), 0.0001));
      }

      streamingDataTCPClient.close();
      streamingDataTCPServer.close();
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

         Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
         reconstructedFootsteps.add(footstep);
      }

      public ArrayList<Footstep> getReconstructedFootsteps()
      {
         return reconstructedFootsteps;
      }
   }

   private class FootstepPathConsumer extends AbstractStreamingDataConsumer<ArrayList>
   {
      ArrayList<Footstep> reconstructedFootstepPath = new ArrayList<Footstep>();

      public FootstepPathConsumer()
      {
         super(2222L, ArrayList.class);
      }

      protected void processPacket(ArrayList packet)
      {
         for (Object aPacket : packet)
         {
            FootstepData footstepData = (FootstepData) aPacket;
            RigidBody endEffector = new RigidBody(footstepData.getRigidBodyName(), ReferenceFrame.getWorldFrame());
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
            ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
            for (Point3d point : footstepData.getExpectedContactPoints())
            {
               FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), point);
               expectedContactPoints.add(framePoint);
            }

            Footstep footstep = new Footstep(endEffector, pose, expectedContactPoints);
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
