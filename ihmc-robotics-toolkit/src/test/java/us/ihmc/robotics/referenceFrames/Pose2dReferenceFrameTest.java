
package us.ihmc.robotics.referenceFrames;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.random.RandomGeometry;

public class Pose2dReferenceFrameTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAsynchronousUpdatesOne()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Pose2dReferenceFrame poseFrame0 = new Pose2dReferenceFrame("poseFrame0", worldFrame);
      Pose2dReferenceFrame poseFrame00 = new Pose2dReferenceFrame("poseFrame00", poseFrame0);
      Pose2dReferenceFrame poseFrame000 = new Pose2dReferenceFrame("poseFrame000", poseFrame00);
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
         Pose2dReferenceFrame poseFrame = new Pose2dReferenceFrame("poseFrame" + i, previousFrame);
         poseFrame.setPositionAndUpdate(new FramePoint2D(previousFrame, 1.0, 0.0));
         previousFrame = poseFrame;
      }
      
      FramePoint2D finalPosition = new FramePoint2D(previousFrame);
      finalPosition.changeFrame(worldFrame);
      
      long startTime = System.currentTimeMillis();
      int numberOfChangeFrames = 1000000;
      for (int i=0; i<numberOfChangeFrames; i++)
      {
         FramePoint2D finalPosition2 = new FramePoint2D(previousFrame);
         finalPosition2.changeFrame(worldFrame);
      }
      long endTime = System.currentTimeMillis();
      
      double millisPerChangeFrame = ((double) (endTime-startTime)) / ((double) numberOfChangeFrames);
      System.out.println("millisPerChangeFrame = " + millisPerChangeFrame);
      
      assertTrue(millisPerChangeFrame < 0.01); //That would still be pretty slow, but allows for hiccups.
      FramePoint2D finalPosition3 = new FramePoint2D(previousFrame);
      finalPosition3.changeFrame(worldFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAsynchronousUpdatesTwo()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Pose2dReferenceFrame poseFrame0 = new Pose2dReferenceFrame("poseFrame0", worldFrame);
      Pose2dReferenceFrame poseFrame00 = new Pose2dReferenceFrame("poseFrame00", poseFrame0);
      Pose2dReferenceFrame poseFrame01 =  new Pose2dReferenceFrame("poseFrame01", poseFrame0);
      Pose2dReferenceFrame poseFrame010 = new Pose2dReferenceFrame("poseFrame010", poseFrame01);
      Pose2dReferenceFrame poseFrame000 = new Pose2dReferenceFrame("poseFrame000", poseFrame00);
      Pose2dReferenceFrame poseFrame001 = new Pose2dReferenceFrame("poseFrame001", poseFrame00);
      Pose2dReferenceFrame poseFrame1 =   new Pose2dReferenceFrame("poseFrame1", worldFrame);
      Pose2dReferenceFrame poseFrame10 =  new Pose2dReferenceFrame("poseFrame10", poseFrame1);
      Pose2dReferenceFrame poseFrame100 = new Pose2dReferenceFrame("poseFrame100", poseFrame10);
      Pose2dReferenceFrame poseFrame101 = new Pose2dReferenceFrame("poseFrame101", poseFrame10);
      
      Pose2dReferenceFrame[] referenceFrames = new Pose2dReferenceFrame[]{poseFrame0, poseFrame00, poseFrame01, poseFrame010, poseFrame000, poseFrame001, poseFrame1, poseFrame10, poseFrame100, poseFrame101};
      
      Point2D position = new Point2D(1.0, 2.2);
      FramePoint2D framePoint = new FramePoint2D(poseFrame010, position);
      
      updateAllFrames(referenceFrames);

      Random random = new Random(1776L);

      FramePoint2D newPoint = doRandomChangeFrames(referenceFrames, framePoint, random);
      newPoint.changeFrame(framePoint.getReferenceFrame());
      
      assertTrue(newPoint.epsilonEquals(framePoint, 1e-7)); 
      
      
      doRandomPoseChangeAndUpdate(poseFrame01, random);
      newPoint = doRandomChangeFrames(referenceFrames, framePoint, random);      

      FramePoint2D newPointInWorldOne = new FramePoint2D(newPoint);
      newPointInWorldOne.changeFrame(worldFrame);
      updateAllFrames(referenceFrames);
      FramePoint2D newPointInWorldTwo = new FramePoint2D(newPoint);
      newPointInWorldTwo.changeFrame(worldFrame);

      assertTrue(newPointInWorldOne.epsilonEquals(newPointInWorldTwo, 1e-7));

   }

   private void updateAllFrames(ReferenceFrame[] referenceFrames)
   {
      for (ReferenceFrame frame : referenceFrames)
      {
         frame.update();
      }
   }
   
   private void doRandomPoseChangeAndUpdate(Pose2dReferenceFrame poseReferenceFrame, Random random)
   {

      Point2D randomPoint2d = RandomGeometry.nextPoint2D(random, 1234, 1234);
      FramePose2D framePose = new FramePose2D(poseReferenceFrame.getParent(), randomPoint2d,random.nextGaussian());
      poseReferenceFrame.setPoseAndUpdate(framePose);
   }
   
   private FramePoint2D doRandomChangeFrames(ReferenceFrame[] referenceFrames, FramePoint2D framePoint, Random random)
   {
      for (int i=0; i<10; i++)
      {
         int index = random.nextInt(referenceFrames.length);
         ReferenceFrame desiredFrame = referenceFrames[index];
         framePoint.changeFrame(desiredFrame);
      }
      
      return framePoint;
   }

}
