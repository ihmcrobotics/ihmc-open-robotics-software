package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.random.RandomGeometry;

public class FourBarKinematicLoopTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RigidBody elevator = new RigidBody("elevator", worldFrame);

   private RigidBody rigidBodyAB, rigidBodyBC, rigidBodyCD, rigidBodyDA;
   private RevoluteJoint masterJointA;
   private PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private FourBarKinematicLoop fourBarKinematicLoop;
   private final Random random = new Random(329023L);

   private final static double eps = 1e-7;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquareWithParallelReferenceFrames()
   {
      // initialize to a square of unit length
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(0.0, 1.0, 0.0);
      Vector3D jointBtoC = new Vector3D(1.0, 0.0, 0.0);
      Vector3D jointCtoD = new Vector3D(0.0, -1.0, 0.0);
      Vector3D jointAtoD = new Vector3D(1.0, 0.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      boolean recomputeJointLimits = false;

      testPlanarSquare(recomputeJointLimits, jointAtoD);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquareWithRotatedJointFrames()
   {
      // initialize to a square of unit length
      FrameVector3D jointAxis = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FrameVector3D jointAtoD = new FrameVector3D(worldFrame, 1.0, 0.0, 0.0);
      FramePoint3D jointAPosition = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      FramePoint3D jointBPosition = new FramePoint3D(worldFrame, 0.0, 1.0, 0.0);
      FramePoint3D jointCPosition = new FramePoint3D(worldFrame, 1.0, 1.0, 0.0);
      FramePoint3D jointDPosition = new FramePoint3D(worldFrame, 1.0, 0.0, 0.0);

      initializeFourBarWithRandomlyRotatedJointFrames(jointAPosition, jointBPosition, jointCPosition, jointDPosition, jointAxis, jointAxis, jointAxis,
            jointAxis);

      jointAtoD.changeFrame(masterJointA.getFrameBeforeJoint());

      boolean recomputeJointLimits = false;

      testPlanarSquare(recomputeJointLimits, jointAtoD);
   }

   private void testPlanarSquare(boolean recomputeJointLimits, Vector3DReadOnly jointAtoD)
   {
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);

      // try making a four bar with no joint limits
      try
      {
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);
         fail();
      }
      catch(Exception e)
      {
      }

      initializeJointLimits(-0.5 * Math.PI, 0.5 * Math.PI, -0.5 * Math.PI, 0.5 * Math.PI, -0.5 * Math.PI, 0.5 * Math.PI, 0.0, Math.PI);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);

      // master joint is 0 degrees, 0 velocity/acceleration
      masterJointA.setQ(0.0);
      masterJointA.setQd(0.0);
      masterJointA.setQdd(0.0);
      fourBarKinematicLoop.update();

      assertEquals(passiveJointB.getQ(), 0.0, eps);
      assertEquals(passiveJointC.getQ(), 0.0, eps);
      assertEquals(passiveJointD.getQ(), -0.5 * Math.PI, eps);

      assertEquals(passiveJointB.getQd(), 0.0, eps);
      assertEquals(passiveJointC.getQd(), 0.0, eps);
      assertEquals(passiveJointD.getQd(), 0.0, eps);

      assertEquals(passiveJointB.getQdd(), 0.0, eps);
      assertEquals(passiveJointC.getQdd(), 0.0, eps);
      assertEquals(passiveJointD.getQdd(), 0.0, eps);

      // master joint is 45 degrees, non-zero velocity/acceleration
      masterJointA.setQ(0.25 * Math.PI);
      masterJointA.setQd(1.0);
      masterJointA.setQdd(1.0);
      fourBarKinematicLoop.update();

      assertEquals(passiveJointB.getQ(), -0.25 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), 0.25 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.75 * Math.PI, eps);

      assertEquals(passiveJointB.getQd(), -1.0, eps);
      assertEquals(passiveJointC.getQd(), 1.0, eps);
      assertEquals(passiveJointD.getQd(), -1.0, eps);

      assertEquals(passiveJointB.getQdd(), -1.0, eps);
      assertEquals(passiveJointC.getQdd(), 1.0, eps);
      assertEquals(passiveJointD.getQdd(), -1.0, eps);

      // try to set it outside of joint limits
      try
      {
         masterJointA.setQ(-0.6 * Math.PI);
         fourBarKinematicLoop.update();
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         masterJointA.setQ(0.6 * Math.PI);
         fourBarKinematicLoop.update();
         fail();
      }
      catch (Exception e)
      {
      }

      assertTrue(fourBarKinematicLoop.getPassiveRevoluteJointB() == passiveJointB);
      assertTrue(fourBarKinematicLoop.getPassiveRevoluteJointC() == passiveJointC);
      assertTrue(fourBarKinematicLoop.getPassiveRevoluteJointD() == passiveJointD);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSquareWithoutClosureAtZeroAngles()
   {
      // initialize a four bar which is a unit square in the xy plane and whose joint offsets aren't colinear in the xy plane
      double aToBOffsetZ = random.nextDouble();
      double bToCOffsetZ = random.nextDouble();
      double cToDOffsetZ = random.nextDouble();

      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, aToBOffsetZ);
      Vector3D jointBtoC = new Vector3D(Math.sqrt(0.5), Math.sqrt(0.5), bToCOffsetZ);
      Vector3D jointCtoD = new Vector3D(0.0, 1.0, cToDOffsetZ);
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, aToBOffsetZ + bToCOffsetZ + cToDOffsetZ);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      boolean recomputeJointLimits = false;

      initializeJointLimits(-0.5 * Math.PI, 0.5 * Math.PI, -0.25 * Math.PI, 0.75 * Math.PI, -0.25 * Math.PI, 0.75 * Math.PI, 0.0, Math.PI);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

      // master joint is 90 degrees, non-zero velocity
      masterJointA.setQ(0.0);
      masterJointA.setQd(1.0);
      fourBarKinematicLoop.update();

      assertEquals(passiveJointB.getQ(), 0.25 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), -1.0, eps);
      assertEquals(passiveJointC.getQ(), 0.25 * Math.PI, eps);
      assertEquals(passiveJointC.getQd(), 1.0, eps);
      assertEquals(passiveJointD.getQ(), 0.5 * Math.PI, eps);
      assertEquals(passiveJointD.getQd(), -1.0, eps);

      // test random angles [0, 180]
      for(int i = 0; i < 30; i++)
      {
         double masterQ = generateDoubleInBounds(random, -0.5 * Math.PI, 0.5 * Math.PI);
         double masterQd = generateDoubleInBounds(random, -1.0, 1.0);
         masterJointA.setQ(masterQ);
         masterJointA.setQd(masterQd);
         fourBarKinematicLoop.update();
         assertEquals(passiveJointB.getQ(), - masterQ + 0.25 * Math.PI, eps);
         assertEquals(passiveJointB.getQd(), - masterQd, eps);
         assertEquals(passiveJointC.getQ(), masterQ + 0.25 * Math.PI, eps);
         assertEquals(passiveJointC.getQd(), masterQd, eps);
         assertEquals(passiveJointD.getQ(), - masterQ + 0.5 * Math.PI, eps);
         assertEquals(passiveJointD.getQd(), - masterQd, eps);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRectangleWithJointOutOfPlane()
   {
      // initialize to rectangle of side lengths 1.0 and 2.0
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, 0.0);
      Vector3D jointBtoC = new Vector3D(0.0, 2.0, 0.2);
      Vector3D jointCtoD = new Vector3D(-1.0, 0.0, -0.2);
      Vector3D jointAtoD = new Vector3D(0.0, 2.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      boolean recomputeJointLimits = false;
      initializeJointLimits(-0.5 * Math.PI, 0.5 * Math.PI, -0.5 * Math.PI, 0.5 * Math.PI, -0.5 * Math.PI, 0.5 * Math.PI, 0.0, Math.PI);

      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);

      // master joint is 0 degrees, 0 velocity
      masterJointA.setQ(0.0);
      masterJointA.setQd(0.0);
      fourBarKinematicLoop.update();
      assertEquals(passiveJointB.getQ(), 0.0, eps);
      assertEquals(passiveJointC.getQ(), 0.0, eps);
      assertEquals(passiveJointD.getQ(), 0.5 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), 0.0, eps);
      assertEquals(passiveJointC.getQd(), 0.0, eps);
      assertEquals(passiveJointD.getQd(), 0.0, eps);

      // master joint is near maximum angle, non-zero velocity
      double masterJointVelocity = 0.2;
      double angleEpsilon = 1e-4;
      masterJointA.setQ(0.5 * Math.PI - angleEpsilon);
      masterJointA.setQd(masterJointVelocity);
      fourBarKinematicLoop.update();
      assertEquals(passiveJointB.getQ(), -0.5 * Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), 0.5 * Math.PI - angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), angleEpsilon, eps);
      assertEquals(passiveJointB.getQd(), -masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), -masterJointVelocity, eps);

      // master joint in near minimum angle, non-zero velocity
      masterJointA.setQ(-0.5 * Math.PI + angleEpsilon);
      masterJointA.setQd(masterJointVelocity);
      fourBarKinematicLoop.update();
      assertEquals(passiveJointB.getQ(), 0.5 * Math.PI - angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), -0.5 * Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), Math.PI - angleEpsilon, eps);
      assertEquals(passiveJointB.getQd(), -masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), -masterJointVelocity, eps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParallelJointAxesIsEnforced_TranslatedJointsFrames()
   {
      // unit length square, and two slightly different rotation axes
      double epsilonAngle = 1e-3;
      double sideLength = 1.0;
      Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D epsilonRotatedZAxis = new Vector3D(0.0, Math.sin(epsilonAngle), Math.cos(epsilonAngle));
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, 0.0);
      boolean recomputeJointLimits = true;

      // try making a four bar with non-parallel axes
      try
      {
         initializeSquareFourBarInXYPlane(sideLength, epsilonRotatedZAxis, zAxis, zAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBarInXYPlane(sideLength, zAxis, epsilonRotatedZAxis, zAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBarInXYPlane(sideLength, zAxis, zAxis, epsilonRotatedZAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBarInXYPlane(sideLength, zAxis, zAxis, zAxis, epsilonRotatedZAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      // make a four bar with joint axes facing in opposite directions
      Vector3D negatedZAxis = new Vector3D(0.0, 0.0, -1.0);

      try
      {
         initializeSquareFourBarInXYPlane(sideLength, negatedZAxis, zAxis, zAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

         initializeSquareFourBarInXYPlane(sideLength, zAxis, negatedZAxis, zAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

         initializeSquareFourBarInXYPlane(sideLength, zAxis, zAxis, negatedZAxis, zAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

         initializeSquareFourBarInXYPlane(sideLength, zAxis, zAxis, zAxis, negatedZAxis, 1);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         fail();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJointLimitsNearFourBarConstraint()
   {
      // initialize to quadrilateral with side lengths 2.0, 1.0, 0.5, 1.0
      FrameVector3D jointAxis = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FrameVector3D jointAtoD = new FrameVector3D(worldFrame, 0.0, 1.0, random.nextDouble());
      FramePoint3D jointAPosition = new FramePoint3D(worldFrame, 0.0, 0.0, random.nextDouble());
      FramePoint3D jointBPosition = new FramePoint3D(worldFrame, 2.0, 0.0, random.nextDouble());
      FramePoint3D jointCPosition = new FramePoint3D(worldFrame, 2.0 + Math.sqrt(0.5), Math.sqrt(0.5), random.nextDouble());
      FramePoint3D jointDPosition = new FramePoint3D(worldFrame, 2.0 + Math.sqrt(0.5), 0.5 + Math.sqrt(0.5), random.nextDouble());

      initializeFourBarWithRandomlyRotatedJointFrames(jointAPosition, jointBPosition, jointCPosition, jointDPosition, jointAxis, jointAxis, jointAxis, jointAxis);
      boolean recomputeJointLimits = false;

      jointAtoD.changeFrame(masterJointA.getFrameBeforeJoint());

      double interiorAngleAMin = getInteriorAngleFromCosineRule(2.0, 1.0 + 0.5, 1.0);
      double interiorAngleAMax = getInteriorAngleFromCosineRule(2.0, 1.0, 0.5 + 1.0);
      double angleEpsilon = 1e-6;

      // test all permutations of at least one bad bound
      masterJointA.setJointLimitUpper(0.5 * Math.PI - (interiorAngleAMin - angleEpsilon));
      masterJointA.setJointLimitLower(0.5 * Math.PI - (interiorAngleAMax - angleEpsilon));
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

      masterJointA.setJointLimitUpper(0.5 * Math.PI - (interiorAngleAMin + angleEpsilon));
      masterJointA.setJointLimitLower(0.5 * Math.PI - (interiorAngleAMax + angleEpsilon));
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

      masterJointA.setJointLimitUpper(0.5 * Math.PI - (interiorAngleAMin - angleEpsilon));
      masterJointA.setJointLimitLower(0.5 * Math.PI - (interiorAngleAMax + angleEpsilon));
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);

      // test just inside bounds
      masterJointA.setJointLimitUpper(0.5 * Math.PI - (interiorAngleAMin + angleEpsilon));
      masterJointA.setJointLimitLower(0.5 * Math.PI - (interiorAngleAMax - angleEpsilon));
      new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_NoLimitsAreSet_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointBtoC = new Vector3D(0.0, 1.0, random.nextDouble());
      Vector3D jointCtoD = new Vector3D(-1.0, 0.0, random.nextDouble());
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, random.nextDouble());
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      boolean recomputeJointLimits = true;

      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);

      // check that joint limits are consistent with four bar kinematic constraints
      double jointAMin = masterJointA.getJointLimitLower();
      double jointAMax = masterJointA.getJointLimitUpper();
      assertEquals(jointAMin, -0.5 * Math.PI, 1e-7);
      assertEquals(jointAMax, 0.5 * Math.PI, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_NoLimitsAreSet_RandomQuadrilateral()
   {
      for(int i = 0; i < 100; i++)
      {
         // generate random quadrilateral with joint axes along z and random joint offsets along z
         Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
         Vector3D elevatorToJointA = RandomGeometry.nextVector3D(random);
         double[] sideLengths = generateRandomQuadrilateralSideLengths(random, 0.05, 2.0);
         Vector3D jointAtoB = new Vector3D(sideLengths[0], 0.0, random.nextDouble());
         Vector3D jointBtoC = new Vector3D(Math.sqrt(0.5) * sideLengths[1], Math.sqrt(0.5) * sideLengths[1], random.nextDouble());
         Vector3D jointCtoD = new Vector3D(0.0, sideLengths[2], random.nextDouble());
         Vector3D jointAtoD = new Vector3D(0.0, sideLengths[3], random.nextDouble());
         initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
         boolean recomputeJointLimits = true;

         fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
               recomputeJointLimits);

         // check that joint limits are consistent with four bar kinematic constraints
         double sideLengthAB = sideLengths[0];
         double sideLengthBC = sideLengths[1];
         double sideLengthCD = sideLengths[2];
         double sideLengthDA = sideLengths[3];

         double minMasterInteriorAngle, maxMasterInteriorAngle;

         if(sideLengthAB + sideLengthBC > sideLengthCD + sideLengthDA)
            minMasterInteriorAngle = getInteriorAngleFromCosineRule(sideLengthAB, sideLengthDA + sideLengthCD, sideLengthBC);
         else
            minMasterInteriorAngle = getInteriorAngleFromCosineRule(sideLengthAB + sideLengthBC, sideLengthDA, sideLengthCD);

         if(sideLengthAB + sideLengthDA > sideLengthBC + sideLengthCD)
            maxMasterInteriorAngle = getInteriorAngleFromCosineRule(sideLengthAB, sideLengthDA, sideLengthBC + sideLengthCD);
         else
            maxMasterInteriorAngle = Math.PI;

         double jointAMinActual = masterJointA.getJointLimitLower();
         double jointAMaxActual = masterJointA.getJointLimitUpper();

         assertEquals(minMasterInteriorAngle, 0.5 * Math.PI - jointAMaxActual, 1e-6);
         assertEquals(maxMasterInteriorAngle, 0.5 * Math.PI - jointAMinActual, 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_UserSetRestrictiveJointLimits_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointBtoC = new Vector3D(0.0, 1.0, random.nextDouble());
      Vector3D jointCtoD = new Vector3D(-1.0, 0.0, random.nextDouble());
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, random.nextDouble());
      boolean recomputeJointLimits = true;

      // joint limits for b are [-eps, eps]
      double angleEpsilon = 1e-4;

      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointB.setJointLimitLower(- angleEpsilon);
      passiveJointB.setJointLimitUpper(angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      double masterJointALower = masterJointA.getJointLimitLower();
      double masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, - angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, angleEpsilon, 0.01 * angleEpsilon);

      // try to set joint outside of this limit
      try
      {
         masterJointA.setQ(- 2 * angleEpsilon);
         fourBarKinematicLoop.update();
         fail();
      }
      catch(Exception e)
      {
      }

      // joint limits for c are [-eps, eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointC.setJointLimitLower(- angleEpsilon);
      passiveJointC.setJointLimitUpper(angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, angleEpsilon, 0.01 * angleEpsilon);

      // joint limits for d are [90-eps, 90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointD.setJointLimitLower(0.5 * Math.PI - angleEpsilon);
      passiveJointD.setJointLimitUpper(0.5 * Math.PI + angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, angleEpsilon, 0.01 * angleEpsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_UserSetLimitsNearFourBarConstraints_UnitSquare()
   {
      // initialize to a square of unit length and random offsets out of plane
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointBtoC = new Vector3D(0.0, 1.0, random.nextDouble());
      Vector3D jointCtoD = new Vector3D(-1.0, 0.0, random.nextDouble());
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, random.nextDouble());
      boolean recomputeJointLimits = true;
      double angleEpsilon = 1e-4;

      // the constraints of the four bar inherently restrict a to [-90, 90]

      // joint limits for a are [-90+eps, 90-eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      masterJointA.setJointLimitLower(-0.5 * Math.PI + angleEpsilon);
      masterJointA.setJointLimitUpper(0.5 * Math.PI - angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      double masterJointALower = masterJointA.getJointLimitLower();
      double masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI + angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI - angleEpsilon, 0.01 * angleEpsilon);

      // joint limits for a are [-90-eps, 90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      masterJointA.setJointLimitLower(-0.5 * Math.PI - angleEpsilon);
      masterJointA.setJointLimitUpper(0.5 * Math.PI + angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI, 0.01 * angleEpsilon);

      // joint limits for b are [-90+eps, 90-eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointB.setJointLimitLower(-0.5 * Math.PI + angleEpsilon);
      passiveJointB.setJointLimitUpper(0.5 * Math.PI - angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI + angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI - angleEpsilon, 0.01 * angleEpsilon);

      // joint limits for b are [-90-eps, 90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointB.setJointLimitLower(-0.5 * Math.PI - angleEpsilon);
      passiveJointB.setJointLimitUpper(0.5 * Math.PI + angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI, 0.01 * angleEpsilon);

      // joint limits for c are [-90+eps, 90-eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointC.setJointLimitLower(-0.5 * Math.PI + angleEpsilon);
      passiveJointC.setJointLimitUpper(0.5 * Math.PI - angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI + angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI - angleEpsilon, 0.01 * angleEpsilon);

      // joint limits for c are [-90-eps, 90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointC.setJointLimitLower(-0.5 * Math.PI - angleEpsilon);
      passiveJointC.setJointLimitUpper(0.5 * Math.PI + angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI, 0.01 * angleEpsilon);

      // joint limits for d are [-90+eps, 90-eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointD.setJointLimitLower(angleEpsilon);
      passiveJointD.setJointLimitUpper(Math.PI - angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI + angleEpsilon, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI - angleEpsilon, 0.01 * angleEpsilon);

      // joint limits for d are [-90-eps, 90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);
      passiveJointD.setJointLimitLower(- angleEpsilon);
      passiveJointD.setJointLimitUpper(Math.PI + angleEpsilon);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, -0.5 * Math.PI, 0.01 * angleEpsilon);
      assertEquals(masterJointAUpper, 0.5 * Math.PI, 0.01 * angleEpsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_MasterJointLimitsMostRestrictive_UnitSquare()
   {

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJointOrderIsChecked_PlanarSquare()
   {
      // initialize to a square of unit length and random offsets out of plane
      Vector3D jointAxis = new Vector3D(0.0, 0.0, 1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointBtoC = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointCtoD = new Vector3D(1.0, 0.0, random.nextDouble());
      Vector3D jointDtoA = new Vector3D(1.0, 0.0, random.nextDouble());
      boolean recomputeJointLimits = true;

      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, 1);

      // check that exception is thrown when joints are passed in out of order
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointD, passiveJointC, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointC, passiveJointB, passiveJointD, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointC, passiveJointD, passiveJointB, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointD, passiveJointB, passiveJointC, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointD, passiveJointC, passiveJointB, jointDtoA, recomputeJointLimits);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAntiParallelJointAxesWithoutRecomputingJointLimits_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3D jointAxisUp = new Vector3D(0.0, 0.0, 1.0);
      Vector3D jointAxisDown = new Vector3D(0.0, 0.0, -1.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(1.0, 0.0, 0.0);
      Vector3D jointBtoC = new Vector3D(0.0, 1.0, 0.0);
      Vector3D jointCtoD = new Vector3D(-1.0, 0.0, 0.0);
      Vector3D jointAtoD = new Vector3D(0.0, 1.0, 0.0);
      boolean recomputeJointLimits = true;

      double angleEpsilon0 = 1e-4;
      double angleEpsilon1 = 3e-4;

      // try setting joint B, C, D to [45 deg, 135 deg] and make sure A's joint limits are recomputed accordingly
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown, 1);
      passiveJointB.setJointLimitLower(-0.5 * Math.PI + angleEpsilon0);
      passiveJointB.setJointLimitUpper(0.5 * Math.PI - angleEpsilon1);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);
      assertEquals(masterJointA.getJointLimitLower(), -0.5 * Math.PI + angleEpsilon0, eps);
      assertEquals(masterJointA.getJointLimitUpper(), 0.5 * Math.PI - angleEpsilon1, eps);

      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown, 1);
      passiveJointC.setJointLimitLower(-0.5 * Math.PI + angleEpsilon0);
      passiveJointC.setJointLimitUpper(0.5 * Math.PI - angleEpsilon1);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);
      assertEquals(masterJointA.getJointLimitLower(), -0.5 * Math.PI + angleEpsilon1, eps);
      assertEquals(masterJointA.getJointLimitUpper(), 0.5 * Math.PI - angleEpsilon0, eps);

      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown, 1);
      passiveJointD.setJointLimitLower(- Math.PI + angleEpsilon0);
      passiveJointD.setJointLimitUpper(- angleEpsilon1);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD, recomputeJointLimits);
      assertEquals(masterJointA.getJointLimitLower(), - 0.5 * Math.PI + angleEpsilon0, eps);
      assertEquals(masterJointA.getJointLimitUpper(), 0.5 * Math.PI - angleEpsilon1, eps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOutputJoint()
   {
      // initialize to a square of unit length
      Vector3D jointAxis = new Vector3D(0.0, 1.0, 0.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(0.0, 0.0, -1.0);
      Vector3D jointBtoC = new Vector3D(1.0, 0.0, 0.0);
      Vector3D jointCtoD = new Vector3D(0.0, 0.0, 1.0);
      Vector3D jointAtoD = new Vector3D(1.0, 0.0, 0.0);
      boolean recomputeJointLimits = true;

      int outputJointIndex = 1;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      assertTrue(passiveJointB == fourBarKinematicLoop.getFourBarOutputJoint());

      outputJointIndex = 2;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      assertTrue(passiveJointC == fourBarKinematicLoop.getFourBarOutputJoint());

      outputJointIndex = 3;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      assertTrue(passiveJointD == fourBarKinematicLoop.getFourBarOutputJoint());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJacobian_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3D posY = new Vector3D(0.0, 1.0, 0.0);
      Vector3D negY = new Vector3D(0.0, - 1.0, 0.0);
      Vector3D elevatorToJointA = new Vector3D();
      Vector3D jointAtoB = new Vector3D(0.0, 0.0, -1.0);
      Vector3D jointBtoC = new Vector3D(1.0, 0.0, 0.0);
      Vector3D jointCtoD = new Vector3D(0.0, 0.0, 1.0);
      Vector3D jointAtoD = new Vector3D(1.0, 0.0, 0.0);
      boolean recomputeJointLimits = true;

      // test jacobian with joint axes aligned
      int outputJointIndex = 1;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

      outputJointIndex = 2;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

      outputJointIndex = 3;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

      // test jacobian with joint axes reversed
      outputJointIndex = 1;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, negY, negY, negY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

      outputJointIndex = 2;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, negY, negY, negY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

      outputJointIndex = 3;
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, posY, negY, negY, negY, outputJointIndex);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointAtoD,
            recomputeJointLimits);
      unitSquareJacobianTestForSpecificOutputJoint(outputJointIndex);

   }

   private void unitSquareJacobianTestForSpecificOutputJoint(int outputJointIndex)
   {
      int numTests = 50;
      Random random = new Random(538L);
      DenseMatrix64F jacobian = new DenseMatrix64F(6, 1);
      FrameVector3D tempLinearVelocity = new FrameVector3D();
      double expectedLinearX, expectedLinearY, expectedLinearZ, expectedAngularX, expectedAngularY, expectedAngularZ;
      ReferenceFrame outputJointFrame;

      for(int i = 0; i < numTests; i++)
      {
         double qMaster = RandomNumbers.nextDouble(random, - 0.5 * Math.PI + 0.01, 0.5 * Math.PI - 0.01);
         double qdMaster = RandomNumbers.nextDouble(random, -2.0, 2.0);

         masterJointA.setQ(qMaster);
         masterJointA.setQd(qdMaster);
         fourBarKinematicLoop.update();
         fourBarKinematicLoop.getJacobian(jacobian);

         switch (outputJointIndex)
         {
         case 1:
            expectedLinearX = - qdMaster * Math.cos(qMaster);
            expectedLinearY = 0.0;
            expectedLinearZ = qdMaster * Math.sin(qMaster);
            expectedAngularX = 0.0;
            expectedAngularY = qdMaster;
            expectedAngularZ = 0.0;
            outputJointFrame = passiveJointB.getFrameAfterJoint();
            break;
         case 2:
            expectedLinearX = - qdMaster * Math.cos(qMaster);
            expectedLinearY = 0.0;
            expectedLinearZ = qdMaster * Math.sin(qMaster);
            expectedAngularX = 0.0;
            expectedAngularY = 0.0;
            expectedAngularZ = 0.0;
            outputJointFrame = passiveJointC.getFrameAfterJoint();
            break;
         case 3:
            expectedLinearX = 0.0;
            expectedLinearY = 0.0;
            expectedLinearZ = 0.0;
            expectedAngularX = 0.0;
            expectedAngularY = qdMaster;
            expectedAngularZ = 0.0;
            outputJointFrame = passiveJointD.getFrameAfterJoint();
            break;
         default:
            throw new RuntimeException("invalid output joint index: " + outputJointIndex);
         }

         tempLinearVelocity.setIncludingFrame(outputJointFrame, jacobian.get(3, 0), jacobian.get(4, 0), jacobian.get(5, 0));
         tempLinearVelocity.changeFrame(worldFrame);

         assertEquals(expectedAngularX, jacobian.get(0, 0), eps);
         assertEquals(expectedAngularY, jacobian.get(1, 0), eps);
         assertEquals(expectedAngularZ, jacobian.get(2, 0), eps);
         assertEquals(expectedLinearX, tempLinearVelocity.getX(), eps);
         assertEquals(expectedLinearY, tempLinearVelocity.getY(), eps);
         assertEquals(expectedLinearZ, tempLinearVelocity.getZ(), eps);
      }
   }

   private static void failIfFourBarConstructsWithoutAnException(RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB,
         PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3DReadOnly closurePointFromLastPassiveJoint, boolean recomputeJointLimits)
   {
      try
      {
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, closurePointFromLastPassiveJoint, recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }
   }

   private void initializeFourBar(Vector3D elevatorToJointA, Vector3D jointAtoB, Vector3D jointBtoC, Vector3D jointCtoD, Vector3D jointAxis,
         int outputJointIndex)
   {
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, jointAxis, jointAxis, jointAxis, outputJointIndex);
   }

   private void initializeSquareFourBarInXYPlane(double sideLength, Vector3D jointAxisA, Vector3D jointAxisB, Vector3D jointAxisC, Vector3D jointAxisD,
         int outputJointIndex)
   {
      initializeFourBar(new Vector3D(), new Vector3D(sideLength, 0.0, 0.0), new Vector3D(0.0, sideLength, 0.0), new Vector3D(-sideLength, 0.0, 0.0), jointAxisA,
            jointAxisB, jointAxisC, jointAxisD, outputJointIndex);
   }

   private void initializeFourBar(Vector3D elevatorToJointA, Vector3D jointAtoB, Vector3D jointBtoC, Vector3D jointCtoD, Vector3D jointAxisA,
         Vector3D jointAxisB, Vector3D jointAxisC, Vector3D jointAxisD, int outputJointIndex)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, elevatorToJointA, jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointAtoB, jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointBtoC, jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointCtoD, jointAxisD, true);
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyCD", passiveJointD);

      RigidBody outputBody;
      switch(outputJointIndex)
      {
      case 1:
         outputBody = rigidBodyBC;
         break;
      case 2:
         outputBody = rigidBodyCD;
         break;
      case 3:
         outputBody = rigidBodyDA;
         break;
      default:
         throw new RuntimeException("Invalid output joint index: " + outputJointIndex);
      }

      RevoluteJoint outputChildJoint = ScrewTools.addRevoluteJoint("outputChildJoint", outputBody, new Vector3D(), jointAxisA);
      createAndAttachCylinderRB("outputChild_RB", outputChildJoint);

      masterJointA.setQ(random.nextDouble());
      passiveJointB.setQ(random.nextDouble());
      passiveJointC.setQ(random.nextDouble());
      passiveJointD.setQ(random.nextDouble());
   }

   private void initializeFourBar(RigidBodyTransform jointAtoElevator, RigidBodyTransform jointBtoA, RigidBodyTransform jointCtoB, RigidBodyTransform jointDtoC,
         Vector3D jointAxisA, Vector3D jointAxisB, Vector3D jointAxisC, Vector3D jointAxisD)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, jointAtoElevator, jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointBtoA, jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointCtoB, jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointDtoC, jointAxisD, true);
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyCD", passiveJointD);

      masterJointA.setQ(random.nextDouble());
      passiveJointB.setQ(random.nextDouble());
      passiveJointC.setQ(random.nextDouble());
      passiveJointD.setQ(random.nextDouble());
   }

   private void initializeFourBarWithRandomlyRotatedJointFrames(FramePoint3D jointAPosition, FramePoint3D jointBPosition, FramePoint3D jointCPosition, FramePoint3D jointDPosition,
         FrameVector3D jointAxisA, FrameVector3D jointAxisB, FrameVector3D jointAxisC, FrameVector3D jointAxisD)
   {
      ReferenceFrame jointAFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("jointAFrame", jointAPosition, Axis.Z, new FrameVector3D(worldFrame, RandomGeometry.nextVector3D(random, 1.0)));
      ReferenceFrame jointBFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("jointBFrame", jointBPosition, Axis.Z, new FrameVector3D(worldFrame, RandomGeometry.nextVector3D(random, 1.0)));
      ReferenceFrame jointCFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("jointCFrame", jointCPosition, Axis.Z, new FrameVector3D(worldFrame, RandomGeometry.nextVector3D(random, 1.0)));
      ReferenceFrame jointDFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("jointDFrame", jointDPosition, Axis.Z, new FrameVector3D(worldFrame, RandomGeometry.nextVector3D(random, 1.0)));

      Vector3D jointAxisAFrameA = new Vector3D();
      Vector3D jointAxisBFrameB = new Vector3D();
      Vector3D jointAxisCFrameC = new Vector3D();
      Vector3D jointAxisDFrameD = new Vector3D();

      jointAxisA.changeFrame(jointAFrame);
      jointAxisAFrameA.set(jointAxisA);
      jointAxisB.changeFrame(jointBFrame);
      jointAxisBFrameB.set(jointAxisB);
      jointAxisC.changeFrame(jointCFrame);
      jointAxisCFrameC.set(jointAxisC);
      jointAxisD.changeFrame(jointDFrame);
      jointAxisDFrameD.set(jointAxisD);

      RigidBodyTransform jointAtoElevator = jointAFrame.getTransformToDesiredFrame(worldFrame);
      RigidBodyTransform jointBtoA = jointBFrame.getTransformToDesiredFrame(jointAFrame);
      RigidBodyTransform jointCtoB = jointCFrame.getTransformToDesiredFrame(jointBFrame);
      RigidBodyTransform jointDtoC = jointDFrame.getTransformToDesiredFrame(jointCFrame);

      initializeFourBar(jointAtoElevator, jointBtoA, jointCtoB, jointDtoC, jointAxisAFrameA, jointAxisBFrameB, jointAxisCFrameC, jointAxisDFrameD);
   }

   // the RigidBodies are independent of the calculations done by FourBarKinematicLoop, so this suffices to make all the RigidBodies
   private static RigidBody createAndAttachCylinderRB(String name, RevoluteJoint parentJoint)
   {
      Matrix3D inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(1.0, 1.0, 1.0, Axis.Z);
      return ScrewTools.addRigidBody(name, parentJoint, inertiaCylinder, 1.0, new Vector3D());
   }

   private void initializeAllJointsToSameLimits(double lowerLimit, double upperLimit)
   {
      masterJointA.setJointLimitLower(lowerLimit);
      masterJointA.setJointLimitUpper(upperLimit);
      passiveJointB.setJointLimitLower(lowerLimit);
      passiveJointB.setJointLimitUpper(upperLimit);
      passiveJointC.setJointLimitLower(lowerLimit);
      passiveJointC.setJointLimitUpper(upperLimit);
      passiveJointD.setJointLimitLower(lowerLimit);
      passiveJointD.setJointLimitUpper(upperLimit);
   }

   private void initializeJointLimits(double lowerLimitA, double upperLimitA, double lowerLimitB, double upperLimitB, double lowerLimitC, double upperLimitC, double lowerLimitD, double upperLimitD)
   {
      masterJointA.setJointLimitLower(lowerLimitA);
      masterJointA.setJointLimitUpper(upperLimitA);
      passiveJointB.setJointLimitLower(lowerLimitB);
      passiveJointB.setJointLimitUpper(upperLimitB);
      passiveJointC.setJointLimitLower(lowerLimitC);
      passiveJointC.setJointLimitUpper(upperLimitC);
      passiveJointD.setJointLimitLower(lowerLimitD);
      passiveJointD.setJointLimitUpper(upperLimitD);
   }

   private static double getInteriorAngleFromCosineRule(double adjacentSide1, double adjacentSide2, double oppositeSide)
   {
      return Math.acos((adjacentSide1 * adjacentSide1 + adjacentSide2 * adjacentSide2 - oppositeSide * oppositeSide) / (2.0 * adjacentSide1 * adjacentSide2));
   }

   private static double generateDoubleInBounds(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private static double[] generateRandomQuadrilateralSideLengths(Random random, double min, double max)
   {
      double l0 = generateDoubleInBounds(random, min, max);
      double l1 = generateDoubleInBounds(random, min, max);
      double l2 = generateDoubleInBounds(random, min, max);
      double lastSideMin = min;
      if(l0 - l1 - l2 > 0)       lastSideMin = Math.max(min, l0 - l1 - l2);
      else if(l1 - l0 - l2 > 0)  lastSideMin = Math.max(min, l1 - l0 - l2);
      else if(l2 - l0 - l1 > 0)  lastSideMin = Math.max(min, l2 - l0 - l1);
      double lastSideMax = Math.min(l0 + l1 + l2, max);
      double l3 = generateDoubleInBounds(random, lastSideMin, lastSideMax);
      return new double[]{l0, l1, l2, l3};
   }

   private void printJointPositionsAndAngles()
   {
      System.out.println("joint angles:");
      System.out.println("joint a = " + masterJointA.getQ() / Math.PI);
      System.out.println("joint b = " + passiveJointB.getQ() / Math.PI);
      System.out.println("joint c = " + passiveJointC.getQ() / Math.PI);
      System.out.println("joint d = " + passiveJointD.getQ() / Math.PI);

      FramePoint3D jointAPosition = new FramePoint3D(masterJointA.getFrameBeforeJoint());
      FramePoint3D jointBPosition = new FramePoint3D(passiveJointB.getFrameBeforeJoint());
      FramePoint3D jointCPosition = new FramePoint3D(passiveJointC.getFrameBeforeJoint());
      FramePoint3D jointDPosition = new FramePoint3D(passiveJointD.getFrameBeforeJoint());

      jointAPosition.changeFrame(worldFrame);
      jointBPosition.changeFrame(worldFrame);
      jointCPosition.changeFrame(worldFrame);
      jointDPosition.changeFrame(worldFrame);

      System.out.println("joint a position = " + jointAPosition);
      System.out.println("joint b position = " + jointBPosition);
      System.out.println("joint c position = " + jointCPosition);
      System.out.println("joint d position = " + jointDPosition);
   }

   private void printInitialJointPositions()
   {
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);

      FramePoint3D jointAPosition = new FramePoint3D(masterJointA.getFrameBeforeJoint());
      FramePoint3D jointBPosition = new FramePoint3D(passiveJointB.getFrameBeforeJoint());
      FramePoint3D jointCPosition = new FramePoint3D(passiveJointC.getFrameBeforeJoint());
      FramePoint3D jointDPosition = new FramePoint3D(passiveJointD.getFrameBeforeJoint());

      jointAPosition.changeFrame(worldFrame);
      jointBPosition.changeFrame(worldFrame);
      jointCPosition.changeFrame(worldFrame);
      jointDPosition.changeFrame(worldFrame);

      System.out.println("\njoint a position = " + jointAPosition);
      System.out.println("joint b position = " + jointBPosition);
      System.out.println("joint c position = " + jointCPosition);
      System.out.println("joint d position = " + jointDPosition);
   }
}
