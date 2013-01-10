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
      streamingDataTCPClient.connectToServer();

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
}
