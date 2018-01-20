package us.ihmc.robotics.referenceFrames;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class PoseReferenceFrameTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAsynchronousUpdatesOne()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame0 = new PoseReferenceFrame("poseFrame0", worldFrame);
      PoseReferenceFrame poseFrame00 = new PoseReferenceFrame("poseFrame00", poseFrame0);
      PoseReferenceFrame poseFrame000 = new PoseReferenceFrame("poseFrame000", poseFrame00);
      FramePoint3D framePoint = new FramePoint3D(poseFrame000, 1.0, 2.8, 4.4);

      Random random = new Random(1776L);

      doRandomPoseChangeAndUpdate(poseFrame0, random);
      doRandomPoseChangeAndUpdate(poseFrame00, random);
      doRandomPoseChangeAndUpdate(poseFrame000, random);

      // Make sure that after a pose change of an intermediate frame, that the point before and after is different.
      FramePoint3D framePointInWorldOne = new FramePoint3D(framePoint);
      framePointInWorldOne.changeFrame(worldFrame);
      doRandomPoseChangeAndUpdate(poseFrame0, random);
      FramePoint3D framePointInWorldTwo = new FramePoint3D(framePoint);
      framePointInWorldTwo.changeFrame(worldFrame);

      assertFalse(framePointInWorldOne.epsilonEquals(framePointInWorldTwo, 1e-7));

      // But make sure that additional updates doesn't make it any different.
      poseFrame0.update();
      poseFrame00.update();
      poseFrame000.update();
      
      FramePoint3D framePointInWorldThree = new FramePoint3D(framePoint);
      framePointInWorldThree.changeFrame(worldFrame);

      assertTrue(framePointInWorldThree.epsilonEquals(framePointInWorldTwo, 1e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testLongChainEfficiency()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      int numberOfFramesInChain = 100;
      
      ReferenceFrame previousFrame = worldFrame;
      for (int i=0; i<numberOfFramesInChain; i++)
      {
         PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame" + i, previousFrame);
         poseFrame.setPositionAndUpdate(new FramePoint3D(previousFrame, 1.0, 0.0, 0.0));
         previousFrame = poseFrame;
      }
      
      FramePoint3D finalPosition = new FramePoint3D(previousFrame);
      finalPosition.changeFrame(worldFrame);
      
      long startTime = System.currentTimeMillis();
      int numberOfChangeFrames = 1000000;
      for (int i=0; i<numberOfChangeFrames; i++)
      {
         FramePoint3D finalPosition2 = new FramePoint3D(previousFrame);
         finalPosition2.changeFrame(worldFrame);
      }
      long endTime = System.currentTimeMillis();
      
      double millisPerChangeFrame = ((double) (endTime-startTime)) / ((double) numberOfChangeFrames);
      System.out.println("millisPerChangeFrame = " + millisPerChangeFrame);
      
      assertTrue(millisPerChangeFrame < 0.01); //That would still be pretty slow, but allows for hiccups.
      FramePoint3D finalPosition3 = new FramePoint3D(previousFrame);
      finalPosition3.changeFrame(worldFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAsynchronousUpdatesTwo()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame0 = new PoseReferenceFrame("poseFrame0", worldFrame);
      PoseReferenceFrame poseFrame00 = new PoseReferenceFrame("poseFrame00", poseFrame0);
      PoseReferenceFrame poseFrame01 = new PoseReferenceFrame("poseFrame01", poseFrame0);
      PoseReferenceFrame poseFrame010 = new PoseReferenceFrame("poseFrame010", poseFrame01);
      PoseReferenceFrame poseFrame000 = new PoseReferenceFrame("poseFrame000", poseFrame00);
      PoseReferenceFrame poseFrame001 = new PoseReferenceFrame("poseFrame001", poseFrame00);
      PoseReferenceFrame poseFrame1 = new PoseReferenceFrame("poseFrame1", worldFrame);
      PoseReferenceFrame poseFrame10 = new PoseReferenceFrame("poseFrame10", poseFrame1);
      PoseReferenceFrame poseFrame100 = new PoseReferenceFrame("poseFrame100", poseFrame10);
      PoseReferenceFrame poseFrame101 = new PoseReferenceFrame("poseFrame101", poseFrame10);
      
      PoseReferenceFrame[] referenceFrames = new PoseReferenceFrame[]{poseFrame0, poseFrame00, poseFrame01, poseFrame010, poseFrame000, poseFrame001, poseFrame1, poseFrame10, poseFrame100, poseFrame101};
      
      Point3D position = new Point3D(1.0, 2.2, 3.4);
      FramePoint3D framePoint = new FramePoint3D(poseFrame010, position);
      
      updateAllFrames(referenceFrames);

      Random random = new Random(1776L);

      FramePoint3D newPoint = doRandomChangeFrames(referenceFrames, framePoint, random);
      newPoint.changeFrame(framePoint.getReferenceFrame());
      
      assertTrue(newPoint.epsilonEquals(framePoint, 1e-7)); 
      
      
      doRandomPoseChangeAndUpdate(poseFrame01, random);
      newPoint = doRandomChangeFrames(referenceFrames, framePoint, random);      

      FramePoint3D newPointInWorldOne = new FramePoint3D(newPoint);
      newPointInWorldOne.changeFrame(worldFrame);
      updateAllFrames(referenceFrames);
      FramePoint3D newPointInWorldTwo = new FramePoint3D(newPoint);
      newPointInWorldTwo.changeFrame(worldFrame);

      assertTrue(newPointInWorldOne.epsilonEquals(newPointInWorldTwo, 1e-7));

   }

   private void updateAllFrames(PoseReferenceFrame[] referenceFrames)
   {
      for (PoseReferenceFrame poseReferenceFrame : referenceFrames)
      {
         poseReferenceFrame.update();
      }
   }
   
   private void doRandomPoseChangeAndUpdate(PoseReferenceFrame poseReferenceFrame, Random random)
   {
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      FramePose3D framePose = new FramePose3D(poseReferenceFrame.getParent(), transform);
      poseReferenceFrame.setPoseAndUpdate(framePose);
   }
   
   private FramePoint3D doRandomChangeFrames(PoseReferenceFrame[] referenceFrames, FramePoint3D framePoint, Random random)
   {
      for (int i=0; i<10; i++)
      {
         int index = random.nextInt(referenceFrames.length);
         PoseReferenceFrame desiredFrame = referenceFrames[index];
         framePoint = new FramePoint3D(framePoint);
         framePoint.changeFrame(desiredFrame);
      }
      
      return framePoint;
   }

}
