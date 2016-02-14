package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.NonFlatGroundPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorAchievedWrenchCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorVisualizer;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public class GroundReactionWrenchDistributorTest
{
   private static final boolean VISUALIZE = false;
   private static boolean DEBUG = false;

   //TODO: GeometricFlatGroundDistributor seems to be the only one working. Either delete the others, or fix them...
   
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

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testSimpleWrenchDistributionWithGeometricFlatGroundDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      
      GroundReactionWrenchDistributor distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);     
      testSimpleWrenchDistribution(centerOfMassFrame, distributor, parentRegistry, 1e-7);
   }

   @Ignore // LeeGoswami algorithm not currently used. Decide to either continue support or delete. 

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSimpleWrenchDistributionWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry, 1.0); 
      testSimpleWrenchDistribution(centerOfMassFrame, distributor, parentRegistry, 1e-7);
   }

   @Ignore // Fix or delete!

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSimpleWrenchDistributionWithContactPointDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = createContactPointDistributor(parentRegistry, centerOfMassFrame);
      testSimpleWrenchDistribution(centerOfMassFrame, distributor, parentRegistry, 1e-7);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testRandomFlatGroundExamplesWithGeometricFlatGroundDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = false;

      GroundReactionWrenchDistributor distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, false, centerOfMassFrame, distributor, 1.0, parentRegistry);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testRandomFlatGroundExamplesWithViableMomentSolutionsWithGeometricFlatGroundDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = true;

      GroundReactionWrenchDistributor distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, false, centerOfMassFrame, distributor, 1.0, parentRegistry);
   }

   @Ignore // LeeGoswami algorithm not currently used. Decide to either continue support or delete. 

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testRandomFlatGroundExamplesWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = false;
      double rotationalCoefficientOfFrictionMultiplier = 1.0;
     
      LeeGoswamiGroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry,
            rotationalCoefficientOfFrictionMultiplier);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, false, centerOfMassFrame, distributor,
            rotationalCoefficientOfFrictionMultiplier, parentRegistry);
   }

   @Ignore // LeeGoswami algorithm not currently used. Decide to either continue support or delete. 

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testRandomFlatGroundExamplesWithViableMomentSolutionsWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      double rotationalCoefficientOfFrictionMultiplier = 1.0;
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = true;
      
      LeeGoswamiGroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry,
            rotationalCoefficientOfFrictionMultiplier);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, false, centerOfMassFrame, distributor,
            rotationalCoefficientOfFrictionMultiplier, parentRegistry);
   }

   @Ignore // Fix or delete!

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testRandomFlatGroundExamplesWithContactPointDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = false;

      GroundReactionWrenchDistributor distributor = createContactPointDistributor(parentRegistry, centerOfMassFrame);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, true, centerOfMassFrame, distributor, 1.0, parentRegistry);
   }

   @Ignore // Fix or delete!

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testRandomFlatGroundExamplesWithViableMomentSolutionsWithContactPointDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);
      boolean verifyForcesAreInsideFrictionCones = false;
      boolean feasibleMomentSolutions = true;

      GroundReactionWrenchDistributor distributor = createContactPointDistributor(parentRegistry, centerOfMassFrame);
      testRandomFlatGroundExamples(verifyForcesAreInsideFrictionCones, feasibleMomentSolutions, true, centerOfMassFrame, distributor, 1.0, parentRegistry);
   }

   @Ignore // LeeGoswami algorithm not currently used. Decide to either continue support or delete. 

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSimpleNonFlatGroundExampleWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry, 1.0);
      testNonFlatGroundExample(centerOfMassFrame, distributor, parentRegistry);
   }

   @Ignore // Fix or delete!

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSimpleNonFlatGroundExampleWithContactPointDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.0, 0.0, 1.0);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = createContactPointDistributor(parentRegistry, centerOfMassFrame);
      testNonFlatGroundExample(centerOfMassFrame, distributor, parentRegistry);
   }
   
   @Ignore // LeeGoswami algorithm not currently used. Decide to either continue support or delete. 

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testTroublesomeExamplesWithLeeGoswamiDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = new LeeGoswamiGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry, 1.0);
      testTroublesomeExampleOne(centerOfMassFrame, distributor, parentRegistry);
      testTroublesomeExampleTwo(centerOfMassFrame, distributor, parentRegistry);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testTroublesomeExamplesWithGeometricFlatGroundDistributor()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("registry");
      Point3d centerOfMassPoint3d = new Point3d(0.2, 0.1, 1.07);
      PoseReferenceFrame centerOfMassFrame = createCenterOfMassFrame(centerOfMassPoint3d);

      GroundReactionWrenchDistributor distributor = new GeometricFlatGroundReactionWrenchDistributor(parentRegistry, null);
      testTroublesomeExampleOne(centerOfMassFrame, distributor, parentRegistry);
      testTroublesomeExampleTwo(centerOfMassFrame, distributor, parentRegistry);
   }

   private void testSimpleWrenchDistribution(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry,
         double epsilon)
   {
      double coefficientOfFriction = 1.0;
      double footLength = 0.3;
      double footWidth = 0.15;
      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      RigidBody leftFoot = new RigidBody("leftFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState leftFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, leftMidfootLocation, coefficientOfFriction,
            leftFoot);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      RigidBody rightFoot = new RigidBody("rightFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState rightFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, rightMidfootLocation, coefficientOfFriction,
            rightFoot);

      simpleTwoFootTest(centerOfMassFrame, distributor, parentRegistry, leftFootContactState, rightFootContactState, coefficientOfFriction, epsilon);
   }

   private void testSimpleWrenchDistributionWithCylinders(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor,
         YoVariableRegistry parentRegistry, double epsilon, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double coefficientOfFriction = 0.3;
      double footLength = 0.3;
      double footWidth = 0.15;
      double gripStrength = 30.01;
      double cylinderRadius = 0.03;
      double halfHandWidth = 0.025;
      double gripWeaknessFactor = 0.2;
      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      RigidBody leftFoot = new RigidBody("leftFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState leftFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, leftMidfootLocation, coefficientOfFriction,
            leftFoot);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      RigidBody rightFoot = new RigidBody("rightFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState rightFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, rightMidfootLocation, coefficientOfFriction,
            rightFoot);

      RigidBodyTransform leftHandFrameAfterJointTransformInWorld = new RigidBodyTransform(); // RandomTools.generateRandomTransform(random)

      leftHandFrameAfterJointTransformInWorld.setTranslation(new Vector3d(2.0, 2.0, 2.0));

      RigidBodyTransform leftHandCylinderFrameTransformInWorld = new RigidBodyTransform();
      leftHandCylinderFrameTransformInWorld.setEuler(new Vector3d(Math.PI / 6.0, 0.0, -Math.PI * 0.5));
      leftHandCylinderFrameTransformInWorld.setTranslation(new Vector3d(0.5, 0.75, 1.4));

      RigidBodyTransform rightHandFrameAfterJointTransformInWorld = new RigidBodyTransform();
      rightHandFrameAfterJointTransformInWorld.setEuler(new Vector3d(0.0, 0.0, Math.PI / 2));

      RigidBodyTransform rightHandCylinderFrameTransformInWorld = new RigidBodyTransform();
      rightHandCylinderFrameTransformInWorld.setEuler(new Vector3d(Math.PI / 6.0, 0.0, -Math.PI * 0.5));
      rightHandCylinderFrameTransformInWorld.setTranslation(new Vector3d(0.4, -0.75, 1.4));

      PlaneContactState[] feetContactStates = new PlaneContactState[] { leftFootContactState, rightFootContactState };

      Vector3d linearPart = new Vector3d(0.0, 0.0, 1000.0);
      Vector3d angularPart = new Vector3d();

      SpatialForceVector desiredNetSpatialForceVector = new SpatialForceVector(centerOfMassFrame, linearPart, angularPart);

      simpleNFootMCylinderTest(centerOfMassFrame, distributor, parentRegistry, feetContactStates, 1.0, 1e-3,
            yoGraphicsListRegistry, desiredNetSpatialForceVector);
   }

   private void testRandomMultiContact(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry,
         double epsilon, YoGraphicsListRegistry yoGraphicsListRegistry, Random random)
   {
      double coefficientOfFriction = 0.3;
      double footLength = 0.3;
      double footWidth = 0.15;
      double gripStrength = 30.01;
      double cylinderRadius = 0.03;
      double halfHandWidth = 0.025;
      double gripWeaknessFactor = 0.2;

      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      RigidBody leftFoot = new RigidBody("leftFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState leftFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, leftMidfootLocation, coefficientOfFriction,
            leftFoot);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      RigidBody rightFoot = new RigidBody("rightFoot", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState rightFootContactState = new FlatGroundPlaneContactState(footLength, footWidth, rightMidfootLocation, coefficientOfFriction,
            rightFoot);

      RigidBodyTransform leftHandFrameAfterJointTransformInWorld = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform leftHandCylinderFrameTransformInWorld = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform rightHandFrameAfterJointTransformInWorld = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform rightHandCylinderFrameTransformInWorld = RigidBodyTransform.generateRandomTransform(random);

      FramePose[] poses = new FramePose[4];

      PlaneContactState[] feetContactStates = new PlaneContactState[] { leftFootContactState, rightFootContactState };

      SpatialForceVector desiredNetSpatialForceVector = generateRandomAchievableSpatialForceVector(random, centerOfMassFrame, feetContactStates);

      simpleNFootMCylinderTest(centerOfMassFrame, distributor, parentRegistry, feetContactStates, 1.0, epsilon,
            yoGraphicsListRegistry, desiredNetSpatialForceVector);
   }

   private void testTroublesomeExampleOne(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry)
   {
      double[][] contactPointLocationsOne = new double[][] { { -0.18748618308569526, 0.4993664950063565 }, { -0.18748618308569526, 0.3107721369458269 },
            { -0.4272566950489601, 0.3107721369458269 }, { -0.4272566950489601, 0.4993664950063565 } };

      double[][] contactPointLocationsTwo = new double[][] { { -0.1133318132874206, -0.49087142187157046 }, { -0.1133318132874206, -0.6010308443768181 },
            { -0.33136117053395964, -0.6010308443768181 }, { -0.33136117053395964, -0.49087142187157046 } };

      testTroublesomeExample(centerOfMassFrame, distributor, contactPointLocationsOne, contactPointLocationsTwo, new Vector3d(-2.810834363235027,
            -9.249454803442402, 76.9108583580996), new Vector3d(-36.06373668027517, 39.43047643829655, 62.59792486812425));
   }

   private void testTroublesomeExampleTwo(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry)
   {
      double[][] contactPointLocationsOne = new double[][] { { 0.9795417664487718, 0.5885587484763559 }, { 0.9795417664487718, 0.4381570540985258 },
            { 0.8166018745568961, 0.4381570540985258 }, { 0.8166018745568961, 0.5885587484763559 } };

      double[][] contactPointLocationsTwo = new double[][] { { -0.8707570392292157, -0.6236048380817167 }, { -0.8707570392292157, -0.8067104155445308 },
            { -1.0414910774920054, -0.8067104155445308 }, { -1.0414910774920054, -0.6236048380817167 } };

      testTroublesomeExample(centerOfMassFrame, distributor, contactPointLocationsOne, contactPointLocationsTwo, new Vector3d(-36.955722108464826,
            36.82778916715679, 127.13798022142323), new Vector3d(-5.664078207221138, 95.25549492340134, -20.334006511537165));
   }

   private void testTroublesomeExample(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, double[][] contactPointLocationsOne,
         double[][] contactPointLocationsTwo, Vector3d linearPart, Vector3d angularPart)
   {
      SpatialForceVector desiredNetSpatialForceVector = new SpatialForceVector(centerOfMassFrame, linearPart, angularPart);

      double coefficientOfFriction = 0.87;
      RigidBody bodyOne = new RigidBody("bodyOne", ReferenceFrame.getWorldFrame());
      RigidBody bodyTwo = new RigidBody("bodyTwo", ReferenceFrame.getWorldFrame());
      FlatGroundPlaneContactState contactStateOne = new FlatGroundPlaneContactState(contactPointLocationsOne, coefficientOfFriction, bodyOne);
      FlatGroundPlaneContactState contactStateTwo = new FlatGroundPlaneContactState(contactPointLocationsTwo, coefficientOfFriction, bodyTwo);

      ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();

      contactStates.add(contactStateOne);
      contactStates.add(contactStateTwo);

      GroundReactionWrenchDistributorInputData inputData = new GroundReactionWrenchDistributorInputData();

      inputData.addPlaneContact(contactStateOne);
      inputData.addPlaneContact(contactStateTwo);

      GroundReactionWrenchDistributorVisualizer visualizer = null;
      SimulationConstructionSet scs = null;

      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
         scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2; // 6;
         int maxNumberOfVertices = 10;
         visualizer = new GroundReactionWrenchDistributorVisualizer(maxNumberOfFeet, maxNumberOfVertices, 0, scs.getRootRegistry(),
               yoGraphicsListRegistry);

         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

         addCoordinateSystem(scs);
         scs.startOnAThread();
      }

      inputData.setSpatialForceVectorAndUpcomingSupportSide(desiredNetSpatialForceVector, null);

      GroundReactionWrenchDistributorOutputData distributedWrench = new GroundReactionWrenchDistributorOutputData();
      distributor.solve(distributedWrench, inputData);

      verifyForcesAreInsideFrictionCones(distributedWrench, contactStates, coefficientOfFriction);
      verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, contactStates, distributedWrench, 1e-7, true);

      if (VISUALIZE)
      {
         visualizer.update(scs, distributedWrench, centerOfMassFrame, contactStates, desiredNetSpatialForceVector);
         ThreadTools.sleepForever();
      }
   }

   private void testRandomFlatGroundExamples(boolean verifyForcesAreInsideFrictionCones, boolean feasibleMomentSolutions, boolean contactPointDistributor,
         ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, double rotationalCoefficientOfFrictionMultiplier,
         YoVariableRegistry parentRegistry)
   {
      Random random = new Random(1776L);
      ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();

      double coefficientOfFriction = 0.87;

      GroundReactionWrenchDistributorVisualizer visualizer = null;
      SimulationConstructionSet scs = null;

      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
         scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2; // 6;
         int maxNumberOfVertices = 10;
         visualizer = new GroundReactionWrenchDistributorVisualizer(maxNumberOfFeet, maxNumberOfVertices, 0, scs.getRootRegistry(),
               yoGraphicsListRegistry);

         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

         addCoordinateSystem(scs);
         scs.startOnAThread();
      }

      int numberOfTests = 25;

      for (int i = 0; i < numberOfTests; i++)
      {
         contactStates.clear();

         RigidBody leftFoot = new RigidBody("leftFoot", ReferenceFrame.getWorldFrame());
         FlatGroundPlaneContactState leftFootContactState = FlatGroundPlaneContactState.createRandomFlatGroundContactState(random, true, coefficientOfFriction,
               leftFoot);
         RigidBody rightFoot = new RigidBody("rightFoot", ReferenceFrame.getWorldFrame());
         FlatGroundPlaneContactState rightFootContactState = FlatGroundPlaneContactState.createRandomFlatGroundContactState(random, false,
               coefficientOfFriction, rightFoot);

         contactStates.add(leftFootContactState);
         contactStates.add(rightFootContactState);

         GroundReactionWrenchDistributorInputData inputData = new GroundReactionWrenchDistributorInputData();

         inputData.addPlaneContact(leftFootContactState);
         inputData.addPlaneContact(rightFootContactState);

         SpatialForceVector desiredNetSpatialForceVector;

         if (contactPointDistributor)
         {
            desiredNetSpatialForceVector = generateRandomAchievableSpatialForceVectorUsingContactPoints(random, centerOfMassFrame, contactStates,
                  coefficientOfFriction, feasibleMomentSolutions);
         }
         else
         {
            desiredNetSpatialForceVector = generateRandomAchievableSpatialForceVector(random, centerOfMassFrame, contactStates, coefficientOfFriction,
                  coefficientOfFriction * rotationalCoefficientOfFrictionMultiplier, feasibleMomentSolutions);
         }

         inputData.setSpatialForceVectorAndUpcomingSupportSide(desiredNetSpatialForceVector, null);

         GroundReactionWrenchDistributorOutputData distributedWrench = new GroundReactionWrenchDistributorOutputData();
         distributor.solve(distributedWrench, inputData);

         if (verifyForcesAreInsideFrictionCones)
         {
            verifyForcesAreInsideFrictionCones(distributedWrench, contactStates, coefficientOfFriction);
         }

         if (VISUALIZE)
         {
            visualizer.update(scs, distributedWrench, centerOfMassFrame, contactStates, desiredNetSpatialForceVector);
         }

         verifyCentersOfPressureAreInsideContactPolygons(distributedWrench, contactStates);
         verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, contactStates, distributedWrench, 1e-4, !feasibleMomentSolutions);
      }

      if (VISUALIZE)
      {
         deleteFirstDataPointAndCropData(scs);
         ThreadTools.sleepForever();
      }
   }

   private void testNonFlatGroundExample(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry)
   {
      double coefficientOfFriction = 1.0;
      double footLength = 0.3;
      double footWidth = 0.15;

      Point3d leftMidfootLocation = new Point3d(0.0, 0.5, 0.0);
      Vector3d leftNormalToContactPlane = new Vector3d(0.1, 0.0, 1.0);
      leftNormalToContactPlane.normalize();
      RigidBody leftFoot = new RigidBody("leftFoot", ReferenceFrame.getWorldFrame());
      NonFlatGroundPlaneContactState leftFootContactState = new NonFlatGroundPlaneContactState(footLength, footWidth, leftMidfootLocation,
            leftNormalToContactPlane, coefficientOfFriction, leftFoot);

      Point3d rightMidfootLocation = new Point3d(0.0, -0.5, 0.0);
      Vector3d rightNormalToContactPlane = new Vector3d(-0.1, 0.0, 1.0);
      rightNormalToContactPlane.normalize();
      RigidBody rightFoot = new RigidBody("rightFoot", ReferenceFrame.getWorldFrame());
      NonFlatGroundPlaneContactState rightFootContactState = new NonFlatGroundPlaneContactState(footLength, footWidth, rightMidfootLocation,
            rightNormalToContactPlane, coefficientOfFriction, rightFoot);

      simpleTwoFootTest(centerOfMassFrame, distributor, parentRegistry, leftFootContactState, rightFootContactState, coefficientOfFriction, 1e-5);
   }

   private void simpleTwoFootTest(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry,
         PlaneContactState leftFootContactState, PlaneContactState rightFootContactState, double coefficientOfFriction, double epsilon)
   {
      simpleNFootTest(centerOfMassFrame, distributor, parentRegistry, new PlaneContactState[] { leftFootContactState, rightFootContactState },
            coefficientOfFriction, epsilon);
   }

   private void simpleNFootTest(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry,
         PlaneContactState[] feetContactStates, double coefficientOfFriction, double epsilon)
   {
      ArrayList<PlaneContactState> planeContactStates = new ArrayList<PlaneContactState>();

      GroundReactionWrenchDistributorInputData inputData = new GroundReactionWrenchDistributorInputData();

      for (int i = 0; i < feetContactStates.length; i++)
      {
         inputData.addPlaneContact(feetContactStates[i]);
         planeContactStates.add(feetContactStates[i]);
      }

      Vector3d linearPart = new Vector3d(0.0, 0.0, 1000.0); // 50.0, 60.0, 1000.0);
      Vector3d angularPart = new Vector3d(); // 10.0, 12.0, 13.0);

      SpatialForceVector desiredNetSpatialForceVector = new SpatialForceVector(centerOfMassFrame, linearPart, angularPart);

      inputData.setSpatialForceVectorAndUpcomingSupportSide(desiredNetSpatialForceVector, null);

      GroundReactionWrenchDistributorOutputData distributedWrench = new GroundReactionWrenchDistributorOutputData();
      distributor.solve(distributedWrench, inputData);

      for (int i = 0; i < feetContactStates.length; i++)
      {
         printIfDebug("force" + i + " = " + distributedWrench.getForce(feetContactStates[i]));
         printIfDebug("leftNormalTorque" + i + " = " + distributedWrench.getNormalTorque(feetContactStates[i]));
         printIfDebug("leftCenterOfPressure" + i + " = " + distributedWrench.getCenterOfPressure(feetContactStates[i]));
      }

      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2;
         int maxNumberOfVertices = 10;
         GroundReactionWrenchDistributorVisualizer visualizer = new GroundReactionWrenchDistributorVisualizer(maxNumberOfFeet, maxNumberOfVertices, 0,
               scs.getRootRegistry(), yoGraphicsListRegistry);

         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         addCoordinateSystem(scs);

         scs.startOnAThread();

         visualizer.update(scs, distributedWrench, centerOfMassFrame, planeContactStates, desiredNetSpatialForceVector);

         ThreadTools.sleepForever();
      }

      verifyForcesAreInsideFrictionCones(distributedWrench, planeContactStates, coefficientOfFriction);
      verifyCentersOfPressureAreInsideContactPolygons(distributedWrench, planeContactStates);
      verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, planeContactStates, distributedWrench, epsilon, false);
   }

   private void simpleNFootMCylinderTest(ReferenceFrame centerOfMassFrame, GroundReactionWrenchDistributor distributor, YoVariableRegistry parentRegistry,
         PlaneContactState[] feetContactStates, double coefficientOfFriction, double epsilon,
         YoGraphicsListRegistry yoGraphicsListRegistry, SpatialForceVector desiredNetSpatialForceVector)
   {
      ArrayList<PlaneContactState> planeContactState = new ArrayList<PlaneContactState>();

      GroundReactionWrenchDistributorInputData inputData = new GroundReactionWrenchDistributorInputData();

      for (int i = 0; i < feetContactStates.length; i++)
      {
         inputData.addPlaneContact(feetContactStates[i]);
         planeContactState.add(feetContactStates[i]);
      }

      inputData.setSpatialForceVectorAndUpcomingSupportSide(desiredNetSpatialForceVector, null);

      GroundReactionWrenchDistributorOutputData distributedWrench = new GroundReactionWrenchDistributorOutputData();
      distributor.solve(distributedWrench, inputData);

      for (int i = 0; i < feetContactStates.length; i++)
      {
         printIfDebug("force" + i + " = " + distributedWrench.getForce(feetContactStates[i]));
         printIfDebug("leftNormalTorque" + i + " = " + distributedWrench.getNormalTorque(feetContactStates[i]));
         printIfDebug("leftCenterOfPressure" + i + " = " + distributedWrench.getCenterOfPressure(feetContactStates[i]));
      }

      if (VISUALIZE)
      {
         Robot robot = new Robot("null");

         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         int maxNumberOfFeet = 2;
         int maxNumberOfVertices = 10;
         int maxNumberOfCylinders = 2;
         GroundReactionWrenchDistributorVisualizer visualizer = new GroundReactionWrenchDistributorVisualizer(maxNumberOfFeet, maxNumberOfVertices,
               maxNumberOfCylinders, scs.getRootRegistry(), yoGraphicsListRegistry);

         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         addCoordinateSystem(scs);

         scs.startOnAThread();

         visualizer
               .update(scs, distributedWrench, centerOfMassFrame, planeContactState, desiredNetSpatialForceVector);

         ThreadTools.sleepForever();
      }

      verifyForcesAreInsideFrictionCones(distributedWrench, planeContactState, coefficientOfFriction);
      verifyCentersOfPressureAreInsideContactPolygons(distributedWrench, planeContactState);
      verifyWrenchesSumToExpectedTotal(centerOfMassFrame, desiredNetSpatialForceVector, planeContactState, distributedWrench, epsilon, false);
   }

   private void deleteFirstDataPointAndCropData(SimulationConstructionSet scs)
   {
      scs.gotoInPointNow();
      scs.tick(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   private void addCoordinateSystem(SimulationConstructionSet scs)
   {
      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(0.2);
      scs.addStaticLinkGraphics(coordinateSystem, Graphics3DNodeType.VISUALIZATION);
   }

   private PoseReferenceFrame createCenterOfMassFrame(Point3d centerOfMassPosition)
   {
      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("com", ReferenceFrame.getWorldFrame());
      FramePose centerOfMassPose = new FramePose(ReferenceFrame.getWorldFrame(), centerOfMassPosition, new Quat4d());
      centerOfMassFrame.setPoseAndUpdate(centerOfMassPose);
      centerOfMassFrame.update();

      return centerOfMassFrame;
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   private void verifyWrenchesSumToExpectedTotal(ReferenceFrame centerOfMassFrame, SpatialForceVector totalBodyWrench,
         ArrayList<PlaneContactState> contactStates, GroundReactionWrenchDistributorOutputData distributedWrench, double epsilon, boolean onlyForces)
   {
      ReferenceFrame expressedInFrame = totalBodyWrench.getExpressedInFrame();

      SpatialForceVector totalWrench = new SpatialForceVector(centerOfMassFrame);
      totalWrench.add(GroundReactionWrenchDistributorAchievedWrenchCalculator.computeAchievedWrench(distributedWrench, expressedInFrame, contactStates));

      FrameVector totalBodyForce = totalBodyWrench.getLinearPartAsFrameVectorCopy();
      assertTrue("Wrenches are not equal:\nachievedWrench = " + totalWrench + ", \ntotalBodyWrench = " + totalBodyWrench + "\nfor epsilon=" + epsilon,
            totalWrench.getLinearPartAsFrameVectorCopy().epsilonEquals(totalBodyForce, epsilon));

      FrameVector totalBodyMoment = totalBodyWrench.getAngularPartAsFrameVectorCopy();

      if (!onlyForces)
      {
         assertTrue("Wrenches are not equal:\nachievedWrench = " + totalWrench + ", \ntotalBodyWrench = " + totalBodyWrench + "\nfor epsilon=" + epsilon,
               totalWrench.getAngularPartAsFrameVectorCopy().epsilonEquals(totalBodyMoment, epsilon));
      }
   }

   private void verifyForcesAreInsideFrictionCones(GroundReactionWrenchDistributorOutputData distributedWrench, ArrayList<PlaneContactState> contactStates,
         double coefficientOfFriction)
   {
      for (PlaneContactState contactState : contactStates)
      {
         FrameVector force = distributedWrench.getForce(contactState);

         verifyForceIsInsideFrictionCone(force, contactState, coefficientOfFriction);
      }
   }

   private void verifyCentersOfPressureAreInsideContactPolygons(GroundReactionWrenchDistributorOutputData distributedWrench,
         ArrayList<PlaneContactState> contactStates)
   {
      for (PlaneContactState contactState : contactStates)
      {
         FramePoint2d centerOfPressure = distributedWrench.getCenterOfPressure(contactState);
         verifyCenterOfPressureIsInsideFoot(centerOfPressure, contactState);
      }
   }

   private void verifyCenterOfPressureIsInsideFoot(FramePoint2d centerOfPressure, PlaneContactState planeContactState)
   {
      centerOfPressure.checkReferenceFrameMatch(planeContactState.getPlaneFrame());
      List<FramePoint2d> contactPoints = planeContactState.getContactFramePoints2dInContactCopy();
      FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(contactPoints);

      assertTrue("footPolygon.distance(centerOfPressure) should be negative " + footPolygon.distance(centerOfPressure),
            footPolygon.distance(centerOfPressure) < 1e-7);
   }

   private void verifyForceIsInsideFrictionCone(FrameVector forceVector, PlaneContactState planeContactState, double coefficientOfFriction)
   {
      forceVector.changeFrame(planeContactState.getPlaneFrame());

      double normalForce = forceVector.getZ();
      double parallelForce = Math.sqrt(forceVector.getX() * forceVector.getX() + forceVector.getY() * forceVector.getY());

      if (parallelForce > coefficientOfFriction * normalForce)
         fail("Outside of Friction Cone! forceVector = " + forceVector + ", planeContactState = " + planeContactState);
   }

   private static SpatialForceVector generateRandomAchievableSpatialForceVector(Random random, ReferenceFrame centerOfMassFrame,
         ArrayList<PlaneContactState> contactStates, double coefficientOfFriction, double normalTorqueCoefficientOfFriction, boolean feasibleMomentSolution)
   {
      SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame);

      for (PlaneContactState contactState : contactStates)
      {
         ReferenceFrame contactPlaneFrame = contactState.getPlaneFrame();

         double normalForce = RandomTools.generateRandomDouble(random, 10.0, feasibleMomentSolution ? 12.0 : 100.0);
         double parallelForceMagnitude = random.nextDouble() * coefficientOfFriction * normalForce;

         Vector2d parallelForce2d = RandomTools.generateRandomVector2d(random, parallelForceMagnitude);
         Vector3d totalForce = new Vector3d(parallelForce2d.getX(), parallelForce2d.getY(), normalForce);

         double normalTorque = random.nextDouble() * normalTorqueCoefficientOfFriction * normalForce;
         Vector3d totalTorque = new Vector3d(0.0, 0.0, normalTorque);

         ReferenceFrame centerOfPressureFrame = generateRandomCenterOfPressureFrame(random, contactState, contactPlaneFrame);
         SpatialForceVector spatialForceVectorContributedByThisContact = new SpatialForceVector(centerOfPressureFrame, totalForce, totalTorque);

         spatialForceVectorContributedByThisContact.changeFrame(centerOfMassFrame);
         spatialForceVector.add(spatialForceVectorContributedByThisContact);
      }

      return spatialForceVector;
   }

   private static SpatialForceVector generateRandomAchievableSpatialForceVector(Random random, ReferenceFrame centerOfMassFrame, PlaneContactState[] planes)
   {
      SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame);

      for (int i = 0; i < planes.length; i++)
      {
         spatialForceVector.add(getRandomValidBasisPlaneWrench(random, centerOfMassFrame, planes[i]));
      }

      return spatialForceVector;
   }

   public static SpatialForceVector getRandomValidBasisPlaneWrench(Random random, ReferenceFrame centerOfMassFrame, PlaneContactState plane)
   {
      FrameVector totalTorque = new FrameVector(centerOfMassFrame, 0.0, 0.0, 0.0);
      FrameVector totalForce = new FrameVector(centerOfMassFrame, 0.0, 0.0, 0.0);
      ReferenceFrame contactPlaneFrame = plane.getPlaneFrame();

      List<FrameVector> normalizedSupportVectors = new ArrayList<FrameVector>();

      for (int i = 0; i < 4; i++)
      {
         normalizedSupportVectors.add(new FrameVector(contactPlaneFrame));
      }

      WrenchDistributorTools.getSupportVectors(normalizedSupportVectors, plane.getCoefficientOfFriction(), contactPlaneFrame);

      FrameVector tempSupportVector = new FrameVector(centerOfMassFrame);
      FrameVector tempCrossVector = new FrameVector(centerOfMassFrame);
      FramePoint tempContactPoint = new FramePoint(centerOfMassFrame);

      for (FramePoint2d contactPoint : plane.getContactFramePoints2dInContactCopy())
      {
         tempContactPoint.setIncludingFrame(contactPoint.getReferenceFrame(), contactPoint.getX(), contactPoint.getY(), 0.0);
         tempContactPoint.changeFrame(centerOfMassFrame);

         for (FrameVector supportVector : normalizedSupportVectors)
         {
            double scale = RandomTools.generateRandomDouble(random, 10.0, 50.0);
            tempSupportVector.setIncludingFrame(centerOfMassFrame, supportVector.getX(), supportVector.getY(), supportVector.getZ());
            tempSupportVector.scale(scale);

            tempCrossVector.cross(tempContactPoint, tempSupportVector);

            totalForce.add(tempSupportVector);
            totalTorque.add(tempCrossVector);
         }
      }

      SpatialForceVector result = new SpatialForceVector(centerOfMassFrame, totalForce.getVector(), totalTorque.getVector());

      return result;
   }

   private static SpatialForceVector generateRandomAchievableSpatialForceVectorUsingContactPoints(Random random, ReferenceFrame centerOfMassFrame,
         ArrayList<PlaneContactState> contactStates, double coefficientOfFriction, boolean feasibleSolution)
   {
      SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame);

      FrameVector totalTorque = new FrameVector(centerOfMassFrame, 0.0, 0.0, 0.0);
      FrameVector totalForce = new FrameVector(centerOfMassFrame, 0.0, 0.0, 0.0);

      for (PlaneContactState contactState : contactStates)
      {
         ReferenceFrame contactPlaneFrame = contactState.getPlaneFrame();

         List<FrameVector> normalizedSupportVectors = new ArrayList<FrameVector>();

         for (int i = 0; i < 4; i++)
         {
            normalizedSupportVectors.add(new FrameVector(contactPlaneFrame));
         }

         WrenchDistributorTools.getSupportVectors(normalizedSupportVectors, coefficientOfFriction, contactPlaneFrame);

         FrameVector tempSupportVector = new FrameVector(centerOfMassFrame);
         FrameVector tempCrossVector = new FrameVector(centerOfMassFrame);
         FramePoint tempContactPoint = new FramePoint(centerOfMassFrame);

         for (FramePoint2d contactPoint : contactState.getContactFramePoints2dInContactCopy())
         {
            tempContactPoint.setIncludingFrame(contactPoint.getReferenceFrame(), contactPoint.getX(), contactPoint.getY(), 0.0);
            tempContactPoint.changeFrame(centerOfMassFrame);

            for (FrameVector supportVector : normalizedSupportVectors)
            {
               double scale = RandomTools.generateRandomDouble(random, 10.0, 50.0);
               tempSupportVector.setIncludingFrame(centerOfMassFrame, supportVector.getX(), supportVector.getY(), supportVector.getZ());
               tempSupportVector.scale(scale);

               tempCrossVector.cross(tempContactPoint, tempSupportVector);

               totalForce.add(tempSupportVector);
               totalTorque.add(tempCrossVector);
            }
         }
      }
      spatialForceVector = new SpatialForceVector(centerOfMassFrame, totalForce.getVector(), totalTorque.getVector());

      return spatialForceVector;
   }

   private static ReferenceFrame generateRandomCenterOfPressureFrame(Random random, PlaneContactState contactState, ReferenceFrame contactPlaneFrame)
   {
      PoseReferenceFrame centerOfPressureFrame = new PoseReferenceFrame("centerOfPressure", contactPlaneFrame);
      Point2d pointInsideContact = generateRandomPointInsideContact(random, contactState);
      Point3d centerOfPressurePosition = new Point3d(pointInsideContact.getX(), pointInsideContact.getY(), 0.0);
      FramePose framePose = new FramePose(contactPlaneFrame, centerOfPressurePosition, new Quat4d());
      centerOfPressureFrame.setPoseAndUpdate(framePose);
      centerOfPressureFrame.update();

      return centerOfPressureFrame;
   }

   private static Point2d generateRandomPointInsideContact(Random random, PlaneContactState contactState)
   {
      Point2d ret = new Point2d();
      double totalWeight = 0.0;

      List<FramePoint2d> contactPoints = contactState.getContactFramePoints2dInContactCopy();
      for (FramePoint2d contactPoint : contactPoints)
      {
         Point2d point2d = contactPoint.getPointCopy();
         double weight = random.nextDouble();
         point2d.scale(weight);
         ret.add(point2d);
         totalWeight += weight;
      }

      ret.scale(1.0 / totalWeight);

      return ret;
   }
   
   private ContactPointGroundReactionWrenchDistributor createContactPointDistributor(YoVariableRegistry parentRegistry, PoseReferenceFrame centerOfMassFrame)
   {
      ContactPointGroundReactionWrenchDistributor distributor = new ContactPointGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry);

      double[] diagonalCWeights = new double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
      double epsilonRho = 0.0;
      distributor.setWeights(diagonalCWeights, 0.0, epsilonRho);

      return distributor;
   }
}
