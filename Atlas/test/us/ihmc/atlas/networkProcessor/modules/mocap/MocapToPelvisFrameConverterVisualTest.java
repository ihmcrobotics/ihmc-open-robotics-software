package us.ihmc.atlas.networkProcessor.modules.mocap;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapToPelvisFrameConverter;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class MocapToPelvisFrameConverterVisualTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final String modelDirectory = "models/GFE/atlas_description/meshes_unplugged/";
   private final Random random = new Random(456654321123L);

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (true || simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 28.6)
   @Test(timeout = 14000)
   public void testVisuallyForMarkerToPelvisAlignment()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Vector3d pelvisTranslation = RandomTools.generateRandomVector(random, 1.5);
      Vector3d mocapTranslation = RandomTools.generateRandomVector(random, 1.5);
      Matrix3d pelvisRotation = RandomTools.generateRandomRotationMatrix3d(random);
      Matrix3d mocapRotation = RandomTools.generateRandomRotationMatrix3d(random);

      RigidBodyTransform pelvisToWorldTransform = new RigidBodyTransform(pelvisRotation, pelvisTranslation);
      RigidBodyTransform mocapToWorldTransform = new RigidBodyTransform(mocapRotation, mocapTranslation);

      RigidBodyTransform worldToMocapTransform = new RigidBodyTransform(mocapToWorldTransform);
      worldToMocapTransform.invert();

      RigidBodyTransform worldToPelvisTranform = new RigidBodyTransform(pelvisToWorldTransform);
      worldToPelvisTranform.invert();

      ReferenceFrame originalPelvisFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("originalPelvisFrame", ReferenceFrame.getWorldFrame(), pelvisToWorldTransform);
      ReferenceFrame mocapFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("mocapFrame", ReferenceFrame.getWorldFrame(), mocapToWorldTransform);

      FramePoint markerPoint1 = new FramePoint(originalPelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(1));
      FramePoint markerPoint2 = new FramePoint(originalPelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(2));
      FramePoint markerPoint3 = new FramePoint(originalPelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(3));
      FramePoint markerPoint4 = new FramePoint(originalPelvisFrame, MocapToPelvisFrameConverter.getMarkerOffsetInPelvisFrame(4));

      markerPoint1.changeFrame(ReferenceFrame.getWorldFrame());
      markerPoint2.changeFrame(ReferenceFrame.getWorldFrame());
      markerPoint3.changeFrame(ReferenceFrame.getWorldFrame());
      markerPoint4.changeFrame(ReferenceFrame.getWorldFrame());

      Graphics3DObject mocapAndPelvisGraphics = new Graphics3DObject();

      mocapAndPelvisGraphics.transform(pelvisToWorldTransform);
      mocapAndPelvisGraphics.addModelFile(modelDirectory + "pelvis.obj");
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.translate(markerPoint1.getPointCopy());
      mocapAndPelvisGraphics.addSphere(0.005, YoAppearance.White());
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.translate(markerPoint2.getPointCopy());
      mocapAndPelvisGraphics.addSphere(0.005, YoAppearance.White());
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.translate(markerPoint3.getPointCopy());
      mocapAndPelvisGraphics.addSphere(0.005, YoAppearance.White());
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.translate(markerPoint4.getPointCopy());
      mocapAndPelvisGraphics.addSphere(0.005, YoAppearance.White());
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.transform(mocapToWorldTransform);
      mocapAndPelvisGraphics.addCoordinateSystem(0.5);

      markerPoint1.changeFrame(mocapFrame);
      markerPoint2.changeFrame(mocapFrame);
      markerPoint3.changeFrame(mocapFrame);
      markerPoint4.changeFrame(mocapFrame);

      ArrayList<MocapMarker> mocapMarkers = new ArrayList<MocapMarker>();
      mocapMarkers.add(new MocapMarker(1, new Vector3d(markerPoint1.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(2, new Vector3d(markerPoint2.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(3, new Vector3d(markerPoint3.getPoint()), 0.024f));
      mocapMarkers.add(new MocapMarker(4, new Vector3d(markerPoint4.getPoint()), 0.024f));
      MocapRigidBody markerRigidBody = new MocapRigidBody(1, new Vector3d(), new Quat4d(0.0, 0.0, 0.0, 1.0), mocapMarkers, false);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(originalPelvisFrame, markerRigidBody);
      RigidBodyTransform computedPelvisTransform = new RigidBodyTransform();
      frameConverter.computePelvisToWorldTransform(markerRigidBody, computedPelvisTransform);

      mocapAndPelvisGraphics.identity();
      mocapAndPelvisGraphics.transform(computedPelvisTransform);
      mocapAndPelvisGraphics.addCoordinateSystem(0.5);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), new SimulationConstructionSetParameters());
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(mocapAndPelvisGraphics);
      scs.startOnAThread();
   }
}
