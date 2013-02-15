package us.ihmc.commonWalkingControlModules.desiredFootStep;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.*;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathCoordinatorTest
{
   /**
    * This test only verifies that polling returns the first footstep in the list *
    */
   @Test
   public void testPoll() throws Exception
   {
      // create a random list of footsteps
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);

      // verify that poll returns the same footsteps
      for (Footstep footstep : footsteps)
      {
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }

      // verify that poll returns null is empty
      assertNull(footstepPathCoordinator.poll());

      footstepPathCoordinator.close();
   }

   @Test
   public void testIsEmpty() throws Exception
   {
      // verify list is initially empty
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      assertTrue(footstepPathCoordinator.isEmpty());

      // create a random list of footsteps
      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);

      // verify list is not empty while each step is removed
      for (Footstep footstep : footsteps)
      {
         assertFalse(footstepPathCoordinator.isEmpty());
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }

      // verify list is empty after all footsteps are removed
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.close();
   }

   @Test
   public void testUpdatePath() throws Exception
   {
      // create a random list of footsteps
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);

      // verify first two steps are the same
      int count = 0;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      // create another random list of footsteps
      ArrayList<Footstep> footsteps2 = createRandomFootsteps(3);
      footstepPathCoordinator.updatePath(footsteps2);

      // verify all steps match the newest footstep path
      assertFalse(footstepPathCoordinator.isEmpty());
      for (Footstep footstep : footsteps2)
      {
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }

      // verify the list is empty after removing all the newest footstep path steps
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.close();
   }

   @Test
   public void testSetPaused() throws Exception
   {
      // create a random list of footsteps
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator();
      ArrayList<Footstep> footsteps = createRandomFootsteps(10);
      footstepPathCoordinator.updatePath(footsteps);

      // verify first two steps are the same
      int count = 0;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      // pause and verify the list appears empty and that polling yields null
      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      assertNull(footstepPathCoordinator.poll());

      // resume and verify three more steps
      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());

      // resume and then resume again and verify the isEmpty method
      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      // pause and then pause again and verify the isEmpty method
      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      assertNull(footstepPathCoordinator.poll());

      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      assertNull(footstepPathCoordinator.poll());

      // resume
      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      // verify one more footstep
      count++;
      compareFootsteps(footsteps.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      // create another random list of footsteps
      ArrayList<Footstep> footsteps2 = createRandomFootsteps(3);
      footstepPathCoordinator.updatePath(footsteps2);

      // verify first footstep matches new path
      count = 0;
      compareFootsteps(footsteps2.get(count), footstepPathCoordinator.poll());
      assertFalse(footstepPathCoordinator.isEmpty());

      // pause and verify the list appears empty and that polling yields null
      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      assertNull(footstepPathCoordinator.poll());

      // create another random list of footsteps
      ArrayList<Footstep> footsteps3 = createRandomFootsteps(5);
      footstepPathCoordinator.updatePath(footsteps3);

      // verify the list appears empty and that polling yields null
      footstepPathCoordinator.setPaused(true);
      assertTrue(footstepPathCoordinator.isEmpty());
      assertNull(footstepPathCoordinator.poll());

      // resume
      footstepPathCoordinator.setPaused(false);
      assertFalse(footstepPathCoordinator.isEmpty());

      // verify all steps match the newest footstep path
      for (Footstep footstep : footsteps3)
      {
         compareFootsteps(footstep, footstepPathCoordinator.poll());
      }

      // verify the list is empty after removing all the newest footstep path steps
      assertTrue(footstepPathCoordinator.isEmpty());

      footstepPathCoordinator.close();

   }

   private void compareFootsteps(Footstep footstep1, Footstep footstep2)
   {
      assertTrue(footstep1.getBody().getName().equals(footstep2.getBody().getName()));
      assertTrue(footstep1.getPoseCopy().epsilonEquals(footstep2.getPoseCopy(), 0.0001));
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
         ContactablePlaneBody contactablePlaneBody = ContactablePlaneBodyTools.createRandomContactablePlaneBodyForTests(random, endEffector);

         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         ArrayList<FramePoint> expectedContactPoints = new ArrayList<FramePoint>();
         for (int i = 0; i < 3; i++)
         {
            FramePoint framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), footstepNumber, i, 0.0);
            expectedContactPoints.add(framePoint);
         }

         boolean trustHeight = true;
         Footstep footstep = new Footstep(contactablePlaneBody, pose, expectedContactPoints, trustHeight);
         footsteps.add(footstep);
      }
      return footsteps;
   }
}
