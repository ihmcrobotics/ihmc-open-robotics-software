package us.ihmc.commonWalkingControlModules.desiredFootStep;

import org.junit.Test;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathCoordinatorTest
{
   @Test
   public void testPoll() throws Exception
   {
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(2222L,  null, footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(3333L, footstepPathCoordinator);
      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);
      for (Footstep footstep : footsteps)
      {
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }
      footstepPathCoordinator.close();
   }

   @Test
   public void testIsEmpty() throws Exception
   {
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(2222L,  null, footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(3333L, footstepPathCoordinator);;
      assertTrue(footstepPathCoordinator.isEmpty());

      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);

      for (Footstep footstep : footsteps)
      {
         assertFalse(footstepPathCoordinator.isEmpty());
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }

      assertTrue(footstepPathCoordinator.isEmpty());
      footstepPathCoordinator.close();

   }

   @Test
   public void testUpdatePath() throws Exception
   {
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(2222L,  null, footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(3333L, footstepPathCoordinator);
      assertTrue(footstepPathCoordinator.isEmpty());

      int numberTotest = 10;
      ArrayList<Footstep> footsteps = createRandomFootsteps(numberTotest);
      footstepPathCoordinator.updatePath(footsteps);
      assertFalse(footstepPathCoordinator.isEmpty());

      int count = 0;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      ArrayList<Footstep> footsteps2 = createRandomFootsteps(3);
      footstepPathCoordinator.updatePath(footsteps2);
      assertFalse(footstepPathCoordinator.isEmpty());
      for (Footstep footstep : footsteps2)
      {
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }
      assertTrue(footstepPathCoordinator.isEmpty());


      int pathLength = 5;
      ArrayList<Footstep> footsteps3 = createRandomFootsteps(pathLength);
      footstepPathCoordinator.updatePath(footsteps3);
      assertFalse(footstepPathCoordinator.isEmpty());
      int i = 0;
      for (; i < 3; i++)
      {
         compareFootsteps(footsteps.get(i), footstepPathCoordinator.poll());
      }
      assertFalse(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      for (; i < pathLength; i++)
      {
         compareFootsteps(footsteps.get(i), footstepPathCoordinator.poll());
      }
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.close();

   }

   @Test
   public void testSetPaused() throws Exception
   {
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(2222L,  null, footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(3333L, footstepPathCoordinator);
      int numberTotest = 10;
      ArrayList<Footstep> footsteps = createRandomFootsteps(numberTotest);
      footstepPathCoordinator.updatePath(footsteps);

      int count = 0;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());

      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.close();

   }

   private void compareFootsteps(Footstep footstep1, Footstep footstep2)
   {
      assertTrue(footstep1.getBody().getName().equals(footstep2.getBody().getName()));
      assertTrue(footstep1.getPose().epsilonEquals(footstep2.getPose(), 0.0001));
      for (int j = 0; j < footstep1.getExpectedContactPoints().size(); j++)
      {
         FramePoint framePoint = footstep1.getExpectedContactPoints().get(j);
         FramePoint reconstructedFramePoint = footstep2.getExpectedContactPoints().get(j);
         assertTrue(framePoint.epsilonEquals(reconstructedFramePoint, 0.0001));
      }
   }

   private ArrayList<Footstep> createRandomFootsteps(int numberToTest)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      Random random = new Random(777);
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
      }
      return footsteps;
   }
}
