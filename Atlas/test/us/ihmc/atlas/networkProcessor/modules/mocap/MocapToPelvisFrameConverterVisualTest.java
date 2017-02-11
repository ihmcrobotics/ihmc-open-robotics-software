package us.ihmc.atlas.networkProcessor.modules.mocap;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapToPelvisFrameConverter;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
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
      if (simulationTestingParameters.getKeepSCSUp())
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

      ReferenceFrame pelvisFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("originalPelvisFrame", ReferenceFrame.getWorldFrame(), pelvisToWorldTransform);
      ReferenceFrame mocapFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("mocapFrame", ReferenceFrame.getWorldFrame(), mocapToWorldTransform);

      RigidBodyTransform pelvisToMarker2Transform = MocapToPelvisFrameConverter.getPelvisToMarker2Transform();
      Vector3d pelvisToMarker2Translation = new Vector3d();
      pelvisToMarker2Transform.getTranslation(pelvisToMarker2Translation);
      pelvisToMarker2Translation.negate();
      FramePoint markerPoint2 = new FramePoint(pelvisFrame, pelvisToMarker2Translation);

      markerPoint2.changeFrame(ReferenceFrame.getWorldFrame());

      Graphics3DObject mocapAndPelvisGraphics = new Graphics3DObject();

      mocapAndPelvisGraphics.transform(pelvisToWorldTransform);
      mocapAndPelvisGraphics.addModelFile(modelDirectory + "pelvis.obj");
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.translate(markerPoint2.getPointCopy());
      mocapAndPelvisGraphics.addSphere(0.005, YoAppearance.White());
      mocapAndPelvisGraphics.identity();

      mocapAndPelvisGraphics.transform(mocapToWorldTransform);
      mocapAndPelvisGraphics.addCoordinateSystem(0.5);

      RigidBodyTransform pelvisToMocapTransform = new RigidBodyTransform();
      pelvisFrame.getTransformToDesiredFrame(pelvisToMocapTransform, mocapFrame);
      Quat4d pelvisToMocapRotation = new Quat4d();
      pelvisToMocapTransform.getRotation(pelvisToMocapRotation);

      ArrayList<MocapMarker> mocapMarkers = new ArrayList<MocapMarker>();
      mocapMarkers.add(new MocapMarker(1, new Vector3d(), 0.024f));
      mocapMarkers.add(new MocapMarker(2, new Vector3d(), 0.024f));
      mocapMarkers.add(new MocapMarker(3, new Vector3d(), 0.024f));
      mocapMarkers.add(new MocapMarker(4, new Vector3d(), 0.024f));
      markerPoint2.changeFrame(mocapFrame);
      MocapRigidBody markerRigidBody = new MocapRigidBody(1, markerPoint2.getVectorCopy(), pelvisToMocapRotation, mocapMarkers, false);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(pelvisFrame, markerRigidBody);
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
