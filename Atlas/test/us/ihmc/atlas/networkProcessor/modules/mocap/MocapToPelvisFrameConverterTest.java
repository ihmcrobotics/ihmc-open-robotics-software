package us.ihmc.atlas.networkProcessor.modules.mocap;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapToPelvisFrameConverter;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertTrue;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class MocapToPelvisFrameConverterTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper testHelper;
   private final Random random = new Random(456654321123L);

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 10000)
   public void testForFrameConversionNoPelvisMotion()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      ReferenceFrame pelvisFrame = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent("pelvisFrame", ReferenceFrame.getWorldFrame(), new Vector3d(1.0, 0.0, 0.0));
      ReferenceFrame mocapFrame = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent("mocapFrame", ReferenceFrame.getWorldFrame(), new Vector3d(0.0, 0.0, 1.0));
      MocapRigidBody markerRigidBody = createMocapRigidBody(pelvisFrame, mocapFrame);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(pelvisFrame, markerRigidBody);

      RigidBodyTransform computedPelvisToWorldTransform = new RigidBodyTransform();
      frameConverter.computePelvisToWorldTransform(markerRigidBody, computedPelvisToWorldTransform);
      RigidBodyTransform actualPelvisToWorldTransform = pelvisFrame.getTransformToWorldFrame();

      assertTrue(computedPelvisToWorldTransform.epsilonEquals(actualPelvisToWorldTransform, 1e-5));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 10000)
   public void testFrameConversionForRandomPelvisMotion()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      ReferenceFrame initialPelvisFrame = ReferenceFrame.generateRandomReferenceFrame("initialPelvisFrame", random, ReferenceFrame.getWorldFrame());
      ReferenceFrame mocapFrame = ReferenceFrame.generateRandomReferenceFrame("mocapFrame", random, ReferenceFrame.getWorldFrame());
      MocapRigidBody mocapRigidBody = createMocapRigidBody(initialPelvisFrame, mocapFrame);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(initialPelvisFrame, mocapRigidBody);

      for (int i = 0; i < 20; i++)
      {
         ReferenceFrame randomPelvisFrame = ReferenceFrame.generateRandomReferenceFrame("randomPelvisFrame" + i, random, ReferenceFrame.getWorldFrame());
         MocapRigidBody randomMocapRigidBody = createMocapRigidBody(randomPelvisFrame, mocapFrame);
         RigidBodyTransform computedPelvisToWorldTransform = new RigidBodyTransform();
         frameConverter.computePelvisToWorldTransform(randomMocapRigidBody, computedPelvisToWorldTransform);
         RigidBodyTransform actualPelvisToWorldTransform = randomPelvisFrame.getTransformToWorldFrame();
         assertTrue(actualPelvisToWorldTransform.epsilonEquals(computedPelvisToWorldTransform, 1e-5));
      }
   }

   private MocapRigidBody createMocapRigidBody(ReferenceFrame pelvisFrame, ReferenceFrame mocapFrame)
   {
      FramePoint markerPoint1 = new FramePoint(pelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(1));
      FramePoint markerPoint2 = new FramePoint(pelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(2));
      FramePoint markerPoint3 = new FramePoint(pelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(3));
      FramePoint markerPoint4 = new FramePoint(pelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(4));

      markerPoint1.changeFrame(mocapFrame);
      markerPoint2.changeFrame(mocapFrame);
      markerPoint3.changeFrame(mocapFrame);
      markerPoint4.changeFrame(mocapFrame);

      ArrayList<MocapMarker> mocapMarkers = new ArrayList<MocapMarker>();
      mocapMarkers.add(new MocapMarker(1, new Vector3d(markerPoint1.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(2, new Vector3d(markerPoint2.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(3, new Vector3d(markerPoint3.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(4, new Vector3d(markerPoint4.getPoint()), 0.024f));
      return new MocapRigidBody(1, new Vector3d(), new Quat4d(0.0, 0.0, 0.0, 1.0), mocapMarkers, false);
   }
}
