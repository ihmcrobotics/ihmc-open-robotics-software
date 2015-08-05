package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathCoordinatorTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   /**
    * This test only verifies that polling returns the first footstep in the list *
    */

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testPoll() throws Exception
   {
      // create a random list of footsteps
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      GlobalDataProducer objectCommunicator = null;
      BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator = null;
      ConstantSwingTimeCalculator constantSwingTimeCalculator = new ConstantSwingTimeCalculator(0.6, registry);
      ConstantTransferTimeCalculator constantTransferTimeCalculator = new ConstantTransferTimeCalculator(0.3, registry);
      
      @SuppressWarnings("deprecation")
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulationForTest();
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, blindWalkingToDestinationDesiredFootstepCalculator, constantSwingTimeCalculator, constantTransferTimeCalculator, registry);
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

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testIsEmpty() throws Exception
   {
      // verify list is initially empty
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      GlobalDataProducer objectCommunicator = null;
      BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator = null;
      ConstantSwingTimeCalculator constantSwingTimeCalculator = new ConstantSwingTimeCalculator(0.6, registry);
      ConstantTransferTimeCalculator constantTransferTimeCalculator = new ConstantTransferTimeCalculator(0.3, registry);
      
      @SuppressWarnings("deprecation")
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulationForTest();
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, blindWalkingToDestinationDesiredFootstepCalculator, constantSwingTimeCalculator, constantTransferTimeCalculator, registry);
      
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

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testUpdatePath() throws Exception
   {
      // create a random list of footsteps
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      GlobalDataProducer objectCommunicator = null;
      BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator = null;
      ConstantSwingTimeCalculator constantSwingTimeCalculator = new ConstantSwingTimeCalculator(0.6, registry);
      ConstantTransferTimeCalculator constantTransferTimeCalculator = new ConstantTransferTimeCalculator(0.3, registry);
      
      @SuppressWarnings("deprecation")
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulationForTest();
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, blindWalkingToDestinationDesiredFootstepCalculator, constantSwingTimeCalculator, constantTransferTimeCalculator, registry);
     
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

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testSetPaused() throws Exception
   {
      // create a random list of footsteps
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      GlobalDataProducer objectCommunicator = null;
      BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator = null;
      ConstantSwingTimeCalculator constantSwingTimeCalculator = new ConstantSwingTimeCalculator(0.6, registry);
      ConstantTransferTimeCalculator constantTransferTimeCalculator = new ConstantTransferTimeCalculator(0.3, registry);
      
      @SuppressWarnings("deprecation")
      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulationForTest();
      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, blindWalkingToDestinationDesiredFootstepCalculator, constantSwingTimeCalculator, constantTransferTimeCalculator, registry);
     
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
      assertTrue(footstep1.epsilonEquals(footstep2, 0.0001));
   }

   private ArrayList<Footstep> createRandomFootsteps(int numberToTest)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      Random random = new Random(777);
      for (int footstepNumber = 0; footstepNumber < numberToTest; footstepNumber++)
      {
         RigidBody endEffector = createRigidBody("rigid_" + footstepNumber);
         ContactablePlaneBody contactablePlaneBody = ContactablePlaneBodyTools.createRandomContactablePlaneBodyForTests(random, endEffector);

         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(footstepNumber, 0.0, 0.0), new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));
         PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", pose);

         boolean trustHeight = true;
         Footstep footstep = new Footstep(contactablePlaneBody.getRigidBody(), null, contactablePlaneBody.getSoleFrame(), poseReferenceFrame, trustHeight);
         
         footsteps.add(footstep);
      }
      return footsteps;
   }

   private RigidBody createRigidBody(String name)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint joint = new SixDoFJoint("joint", elevator, elevator.getBodyFixedFrame());
      return ScrewTools.addRigidBody(name, joint, new Matrix3d(), 0.0, new Vector3d());
   }
}
