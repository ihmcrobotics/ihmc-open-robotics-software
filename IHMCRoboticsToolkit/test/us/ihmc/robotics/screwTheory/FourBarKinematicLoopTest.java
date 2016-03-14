package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FourBarKinematicLoopTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
   private final RigidBody elevator = new RigidBody("elevator", elevatorFrame);
   
   private RigidBody rigidBodyAB, rigidBodyBC, rigidBodyCD;
   private RevoluteJoint masterJointA;
   private PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private FourBarKinematicLoop fourBarKinematicLoop;
   private final Random random = new Random(329023L);

   private final static double eps = 1e-7;
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquare()
   {
      // initialize to a square of unit length
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;      
      
      // try making a four bar with no joint limits
      try
      {
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch(Exception e)
      {         
      }
      
      initializeAllJointsToSameLimits(0.0, Math.PI);      
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);

      // master joint is 90 degrees, 0 velocity
      masterJointA.setQ(0.5 * Math.PI);
      masterJointA.setQd(0.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), 0.0, eps);
      assertEquals(passiveJointC.getQd(), 0.0, eps);
      assertEquals(passiveJointD.getQd(), 0.0, eps);

      // master joint is 45 degrees, non-zero velocity
      masterJointA.setQ(0.25 * Math.PI);
      masterJointA.setQd(1.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -0.25 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), -0.75 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.25 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), -1.0, eps);
      assertEquals(passiveJointC.getQd(), 1.0, eps);
      assertEquals(passiveJointD.getQd(), -1.0, eps);
      
      // try to set it outside of joint limits
      try
      {
         masterJointA.setQ(-0.5);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         masterJointA.setQ(1.5 * Math.PI);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         fail();
      }
      catch (Exception e)
      {
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquareWithRotatedJointFrames()
   {
      // initialize to a square of unit length
      FrameVector jointAxis = new FrameVector(worldFrame, 0.0, 0.0, 1.0);

      FramePoint jointAPosition = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      FramePoint jointBPosition = new FramePoint(worldFrame, 1.0, 0.0, 0.0);
      FramePoint jointCPosition = new FramePoint(worldFrame, 2.0, 0.0, 0.0);
      FramePoint jointDPosition = new FramePoint(worldFrame, 3.0, 0.0, 0.0);
      
      ReferenceFrame jointAFrame = ReferenceFrame.constructReferenceFrameFromPointAndAxis("xFrame", jointAPosition, Axis.Z, new FrameVector(worldFrame, RandomTools.generateRandomVector(random, 1.0)));
      ReferenceFrame jointBFrame = ReferenceFrame.constructReferenceFrameFromPointAndAxis("xFrame", jointBPosition, Axis.Z, new FrameVector(worldFrame, RandomTools.generateRandomVector(random, 1.0)));
      ReferenceFrame jointCFrame = ReferenceFrame.constructReferenceFrameFromPointAndAxis("xFrame", jointCPosition, Axis.Z, new FrameVector(worldFrame, RandomTools.generateRandomVector(random, 1.0)));
      ReferenceFrame jointDFrame = ReferenceFrame.constructReferenceFrameFromPointAndAxis("xFrame", jointDPosition, Axis.Z, new FrameVector(worldFrame, RandomTools.generateRandomVector(random, 1.0)));
      
      FrameVector jointDtoA = new FrameVector(worldFrame, 1.0, 0.0, 0.0);
      jointDtoA.changeFrame(jointDFrame);
      Vector3d jointDtoAInFrameD = new Vector3d();
      jointDtoA.get(jointDtoAInFrameD);
      
      Vector3d jointAxisA = new Vector3d();
      Vector3d jointAxisB = new Vector3d();
      Vector3d jointAxisC = new Vector3d();
      Vector3d jointAxisD = new Vector3d();
      
      jointAxis.changeFrame(jointAFrame);
      jointAxis.get(jointAxisA);
      jointAxis.changeFrame(jointBFrame);
      jointAxis.get(jointAxisB);
      jointAxis.changeFrame(jointCFrame);
      jointAxis.get(jointAxisC);
      jointAxis.changeFrame(jointDFrame);
      jointAxis.get(jointAxisD);
      
      RigidBodyTransform jointAtoElevator = jointAFrame.getTransformToDesiredFrame(elevatorFrame);
      RigidBodyTransform jointBtoA = jointBFrame.getTransformToDesiredFrame(jointAFrame);
      RigidBodyTransform jointCtoB = jointCFrame.getTransformToDesiredFrame(jointBFrame);
      RigidBodyTransform jointDtoC = jointDFrame.getTransformToDesiredFrame(jointCFrame);
      
      initializeFourBar(jointAtoElevator, jointBtoA, jointCtoB, jointDtoC, jointAxisA, jointAxisB, jointAxisC, jointAxisD);
      
      boolean recomputeJointLimits = false;
      
      // try making a four bar with no joint limits
      try
      {
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoAInFrameD,
               recomputeJointLimits);
         fail();
      }
      catch(Exception e)
      {         
      }
      
      initializeAllJointsToSameLimits(0.0, Math.PI);      
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoAInFrameD,
            recomputeJointLimits);

      // master joint is 90 degrees, 0 velocity
      masterJointA.setQ(0.5 * Math.PI);
      masterJointA.setQd(0.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), 0.0, eps);
      assertEquals(passiveJointC.getQd(), 0.0, eps);
      assertEquals(passiveJointD.getQd(), 0.0, eps);

      // master joint is 45 degrees, non-zero velocity
      masterJointA.setQ(0.25 * Math.PI);
      masterJointA.setQd(1.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -0.25 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), -0.75 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.25 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), -1.0, eps);
      assertEquals(passiveJointC.getQd(), 1.0, eps);
      assertEquals(passiveJointD.getQd(), -1.0, eps);
      
      // try to set it outside of joint limits
      try
      {
         masterJointA.setQ(-0.5);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         masterJointA.setQ(1.5 * Math.PI);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         fail();
      }
      catch (Exception e)
      {
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSquareWithoutColinearOffsets()
   {
      // initialize a four bar which is a unit square in the xy plane and whose joint offsets aren't colinear in the xy plane
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(Math.sqrt(0.5), Math.sqrt(0.5), random.nextDouble());
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(Math.sqrt(0.5), Math.sqrt(0.5), random.nextDouble());
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;
      
      initializeAllJointsToSameLimits(0.0, Math.PI);      
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      
      // master joint is 90 degrees, non-zero velocity
      masterJointA.setQ(0.5 * Math.PI);
      masterJointA.setQd(1.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), - 0.75 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), -1.0, eps);
      assertEquals(passiveJointC.getQ(), - 0.25 * Math.PI, eps);
      assertEquals(passiveJointC.getQd(), 1.0, eps);
      assertEquals(passiveJointD.getQ(), - 0.75 * Math.PI, eps);
      assertEquals(passiveJointD.getQd(), -1.0, eps);
      
      // test random angles [0, 180]
      for(int i = 0; i < 30; i++)
      {
         double masterQ = generateDoubleInBounds(random, 0.0, Math.PI);
         double masterQd = random.nextDouble();
         masterJointA.setQ(masterQ);
         masterJointA.setQd(masterQd);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         assertEquals(passiveJointB.getQ(), - masterQ - 0.25 * Math.PI, eps);
         assertEquals(passiveJointB.getQd(), - masterQd, eps);
         assertEquals(passiveJointC.getQ(), masterQ - 0.75 * Math.PI, eps);
         assertEquals(passiveJointC.getQd(), masterQd, eps);
         assertEquals(passiveJointD.getQ(), - masterQ - 0.25 * Math.PI, eps);
         assertEquals(passiveJointD.getQd(), - masterQd, eps);
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRectangleWithJointOutOfPlane()
   {
      // initialize to rectangle of side lengths 1.0 and 2.0
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointBtoC = new Vector3d(2.0, 0.0, 0.2);
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, -0.2);
      Vector3d jointDtoA = new Vector3d(2.0, 0.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;
      initializeAllJointsToSameLimits(0.0, Math.PI);

      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);

      // master joint is 90 degrees, 0 velocity
      masterJointA.setQ(0.5 * Math.PI);
      masterJointA.setQd(0.0);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointC.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointD.getQ(), -0.5 * Math.PI, eps);
      assertEquals(passiveJointB.getQd(), 0.0, eps);
      assertEquals(passiveJointC.getQd(), 0.0, eps);
      assertEquals(passiveJointD.getQd(), 0.0, eps);

      // master joint is near maximum angle, non-zero velocity
      double masterJointVelocity = 0.2;
      double angleEpsilon = 1e-4;
      masterJointA.setQ(Math.PI - angleEpsilon);
      masterJointA.setQd(masterJointVelocity);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), -angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), -Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointB.getQd(), -masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), -masterJointVelocity, eps);

      // master joint in near minimum angle, non-zero velocity
      masterJointA.setQ(angleEpsilon);
      masterJointA.setQd(masterJointVelocity);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), -angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), -Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), -angleEpsilon, eps);
      assertEquals(passiveJointB.getQd(), -masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), -masterJointVelocity, eps);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParallelJointAxesIsEnforced_TranslatedJointsFrames()
   {
      // unit length square, and two slightly different rotation axes
      double epsilonAngle = 1e-3;
      double sideLength = 1.0;
      Vector3d jointAxis1 = RandomTools.generateRandomVector(random, 1.0);
      Vector3d jointAxis2 = new Vector3d(jointAxis1.getX(), Math.cos(epsilonAngle) * jointAxis1.getY() + Math.sin(epsilonAngle) * jointAxis1.getZ(),
            Math.cos(epsilonAngle) * jointAxis1.getZ() - Math.sin(epsilonAngle) * jointAxis1.getY());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, 0.0);
      boolean recomputeJointLimits = false;
      
      // try making a four bar with non-parallel axes
      try
      {
         initializeSquareFourBar(sideLength, jointAxis2, jointAxis1, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis2, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis2, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis1, jointAxis2);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }      
      
      // make a four bar with joint axes facing in opposite directions

      jointAxis2.set(jointAxis1);
      jointAxis2.negate();
      
      try
      {
         initializeSquareFourBar(sideLength, jointAxis2, jointAxis1, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis2, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis2, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis1, jointAxis2);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
      }
      catch (Exception e)
      {
         fail();
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJointLimitsNearFourBarConstraint()
   {
      // initialize to quadrilateral with side lengths 2.0, 1.0, 0.5, 1.0
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(2.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointCtoD = new Vector3d(0.5, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, random.nextDouble());
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;

      double jointAMin = getInteriorAngleFromCosineRule(2.0, 1.0 + 0.5, 1.0);
      double jointAMax = getInteriorAngleFromCosineRule(2.0, 1.0, 0.5 + 1.0);
      double angleEpsilon = 1e-6;

      // test all permutations of at least one bad bound
      masterJointA.setJointLimitLower(jointAMin - angleEpsilon);
      masterJointA.setJointLimitUpper(jointAMax - angleEpsilon);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);

      masterJointA.setJointLimitLower(jointAMin + angleEpsilon);
      masterJointA.setJointLimitUpper(jointAMax + angleEpsilon);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);

      masterJointA.setJointLimitLower(jointAMin - angleEpsilon);
      masterJointA.setJointLimitUpper(jointAMax + angleEpsilon);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);

      // test just inside bounds
      masterJointA.setJointLimitLower(jointAMin + angleEpsilon);
      masterJointA.setJointLimitUpper(jointAMax - angleEpsilon);
      new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_NoLimitsAreSet_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, random.nextDouble());
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = true;
      
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      
      // check that joint limits are consistent with four bar kinematic constraints
      double jointAMin = masterJointA.getJointLimitLower();
      double jointAMax = masterJointA.getJointLimitUpper();
      assertEquals(jointAMin, 0.0, 1e-7);
      assertEquals(jointAMax, Math.PI, 1e-7);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_NoLimitsAreSet_RandomQuadrilateral()
   {
      for(int i = 0; i < 100; i++)
      {
         // generate random quadrilateral with joint axes along z and random joint offsets along z
         Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
         Vector3d elevatorToJointA = RandomTools.generateRandomVector(random);
         double[] sideLengths = generateRandomQuadrilateralSideLengths(random, 0.05, 2.0);
         Vector3d jointAtoB = new Vector3d(sideLengths[0], 0.0, random.nextDouble());
         Vector3d jointBtoC = new Vector3d(sideLengths[1], 0.0, random.nextDouble());
         Vector3d jointCtoD = new Vector3d(sideLengths[2], 0.0, random.nextDouble());
         Vector3d jointDtoA = new Vector3d(sideLengths[3], 0.0, random.nextDouble());
         initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
         boolean recomputeJointLimits = true;
         
         fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         // check that joint limits are consistent with four bar kinematic constraints
         double sideLengthAB = sideLengths[0];
         double sideLengthBC = sideLengths[1];
         double sideLengthCD = sideLengths[2];
         double sideLengthDA = sideLengths[3];
         
         double jointAMinExpected, jointAMaxExpected;
         
         if(sideLengthAB + sideLengthBC > sideLengthCD + sideLengthDA)
            jointAMinExpected = getInteriorAngleFromCosineRule(sideLengthAB, sideLengthDA + sideLengthCD, sideLengthBC);
         else
            jointAMinExpected = getInteriorAngleFromCosineRule(sideLengthAB + sideLengthBC, sideLengthDA, sideLengthCD);
         
         if(sideLengthAB + sideLengthDA > sideLengthBC + sideLengthCD)
            jointAMaxExpected = getInteriorAngleFromCosineRule(sideLengthAB, sideLengthDA, sideLengthBC + sideLengthCD);
         else
            jointAMaxExpected = Math.PI;

         double jointAMinActual = masterJointA.getJointLimitLower();
         double jointAMaxActual = masterJointA.getJointLimitUpper();
         
         assertEquals(jointAMinExpected, jointAMinActual, 1e-6);
         assertEquals(jointAMaxExpected, jointAMaxActual, 1e-6);        
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_UserSetRestrictiveJointLimits_UnitSquare()
   {
      // initialize to a square of unit length
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, random.nextDouble());
      boolean recomputeJointLimits = true;
      
      // joint limits for b are [-90-eps, -90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointB.setJointLimitLower(-0.5 * Math.PI - 1e-4);
      passiveJointB.setJointLimitUpper(-0.5 * Math.PI + 1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      double masterJointALower = masterJointA.getJointLimitLower();
      double masterJointAUpper = masterJointA.getJointLimitUpper();      
      assertEquals(masterJointALower, 0.5 * Math.PI - 1e-4, 1e-6);
      assertEquals(masterJointAUpper, 0.5 * Math.PI + 1e-4, 1e-6);
      
      // try to set joint outside of this limit
      try
      {
         masterJointA.setQ(0.5 * Math.PI - 2e-4);
         fourBarKinematicLoop.updateAnglesAndVelocities();
         fail();
      }
      catch(Exception e)
      {
      }
      
      // joint limits for c are [-90-eps, -90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointC.setJointLimitLower(-0.5 * Math.PI - 1e-4);
      passiveJointC.setJointLimitUpper(-0.5 * Math.PI + 1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0.5 * Math.PI - 1e-4, 1e-6);
      assertEquals(masterJointAUpper, 0.5 * Math.PI + 1e-4, 1e-6);
      
      // joint limits for d are [-90-eps, -90+eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointD.setJointLimitLower(-0.5 * Math.PI - 1e-4);
      passiveJointD.setJointLimitUpper(-0.5 * Math.PI + 1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0.5 * Math.PI - 1e-4, 1e-6);
      assertEquals(masterJointAUpper, 0.5 * Math.PI + 1e-4, 1e-6);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_UserSetLimitsNearFourBarConstraints_UnitSquare()
   {
      // initialize to a square of unit length and random offsets out of plane
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, random.nextDouble());
      boolean recomputeJointLimits = true;
      
      // the constraints of the four bar inherently restrict a to [0, 180]
      
      // joint limits for a are [eps, 180 - eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      masterJointA.setJointLimitLower(1e-4);
      masterJointA.setJointLimitUpper(Math.PI - 1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      double masterJointALower = masterJointA.getJointLimitLower();
      double masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 1e-4, 1e-8);
      assertEquals(masterJointAUpper, Math.PI - 1e-4, 1e-8);
      
      // joint limits for a are [-eps, 180 + eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      masterJointA.setJointLimitLower(-1e-4);
      masterJointA.setJointLimitUpper(Math.PI + 1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0, 1e-8);
      assertEquals(masterJointAUpper, Math.PI, 1e-8);
      
      // the constraints of the four bar inherently restrict b, c, and d to [-180, 0]
      
      // joint limits for b are [-180 + eps, - eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointB.setJointLimitLower(-Math.PI + 1e-4);
      passiveJointB.setJointLimitUpper(-1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 1e-4, 1e-8);
      assertEquals(masterJointAUpper, Math.PI - 1e-4, 1e-8);
      
      // joint limits for b are [-180 - eps, eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointB.setJointLimitLower(-Math.PI - 1e-4);
      passiveJointB.setJointLimitUpper(1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0.0, 1e-8);
      assertEquals(masterJointAUpper, Math.PI, 1e-8);
      
      // joint limits for c are [-180 + eps, - eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointC.setJointLimitLower(-Math.PI + 1e-4);
      passiveJointC.setJointLimitUpper(-1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 1e-4, 1e-8);
      assertEquals(masterJointAUpper, Math.PI - 1e-4, 1e-8);
      
      // joint limits for c are [-180 - eps, eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointC.setJointLimitLower(-Math.PI - 1e-4);
      passiveJointC.setJointLimitUpper(1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0.0, 1e-8);
      assertEquals(masterJointAUpper, Math.PI, 1e-8);

      // joint limits for d are [-180 + eps, - eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointC.setJointLimitLower(-Math.PI + 1e-4);
      passiveJointC.setJointLimitUpper(-1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 1e-4, 1e-8);
      assertEquals(masterJointAUpper, Math.PI - 1e-4, 1e-8);
      
      // joint limits for d are [-180 - eps, eps]
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      passiveJointD.setJointLimitLower(-Math.PI - 1e-4);
      passiveJointD.setJointLimitUpper(1e-4);
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
            recomputeJointLimits);
      masterJointALower = masterJointA.getJointLimitLower();
      masterJointAUpper = masterJointA.getJointLimitUpper();
      assertEquals(masterJointALower, 0.0, 1e-8);
      assertEquals(masterJointAUpper, Math.PI, 1e-8);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRecomputingJointLimits_MasterJointLimitsMostRestrictive_UnitSquare()
   {
      
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJointOrderIsChecked_PlanarSquare()
   {
      // initialize to a square of unit length and random offsets out of plane
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, random.nextDouble());
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, random.nextDouble());
      boolean recomputeJointLimits = true;
      
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      
      // check that exception is thrown when joints are passed in out of order
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointB, passiveJointD, passiveJointC, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointC, passiveJointB, passiveJointD, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointC, passiveJointD, passiveJointB, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointD, passiveJointB, passiveJointC, jointDtoA, recomputeJointLimits);
      failIfFourBarConstructsWithoutAnException(masterJointA, passiveJointD, passiveJointC, passiveJointB, jointDtoA, recomputeJointLimits);
   }
   
//   @DeployableTestMethod(estimatedDuration = 0.0)
//   @Test(timeout = 30000)
//   public void testAntiParallelJointAxesWithoutRecomputingJointLimits_UnitSquare()
//   {
//      // initialize to a square of unit length
//      Vector3d jointAxisUp = new Vector3d(0.0, 0.0, 1.0);
//      Vector3d jointAxisDown = new Vector3d(0.0, 0.0, -1.0);
//      Vector3d elevatorToJointA = new Vector3d();
//      Vector3d jointAtoB = new Vector3d(1.0, 0.0, 0.0);
//      Vector3d jointBtoC = new Vector3d(1.0, 0.0, 0.0);
//      Vector3d jointCtoD = new Vector3d(1.0, 0.0, 0.0);
//      Vector3d jointDtoA = new Vector3d(1.0, 0.0, 0.0);
//      boolean recomputeJointLimits = true;
//
//      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown);
//      passiveJointB.setJointLimitLower(0.25 * Math.PI);
//      passiveJointB.setJointLimitUpper(0.75 * Math.PI);
//      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);
//      assertEquals(masterJointA.getJointLimitLower(), 0.25 * Math.PI, eps);
//      assertEquals(masterJointA.getJointLimitUpper(), 0.75 * Math.PI, eps);
//      
//      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown);
//      passiveJointC.setJointLimitLower(0.25 * Math.PI);
//      passiveJointC.setJointLimitUpper(0.75 * Math.PI);
//      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);
//      assertEquals(masterJointA.getJointLimitLower(), 0.25 * Math.PI, eps);
//      assertEquals(masterJointA.getJointLimitUpper(), 0.75 * Math.PI, eps);
//      
//      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxisUp, jointAxisDown, jointAxisDown, jointAxisDown);
//      passiveJointD.setJointLimitLower(0.25 * Math.PI);
//      passiveJointD.setJointLimitUpper(0.75 * Math.PI);
//      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA, recomputeJointLimits);
//      assertEquals(masterJointA.getJointLimitLower(), 0.25 * Math.PI, eps);
//      assertEquals(masterJointA.getJointLimitUpper(), 0.75 * Math.PI, eps);
//   }
   
   private static void failIfFourBarConstructsWithoutAnException(RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB,
         PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d closurePointFromLastPassiveJoint, boolean recomputeJointLimits)
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

   private static void createFourBar(Vector3d elevatorToJointA, Vector3d jointAtoB, Vector3d jointBtoC, Vector3d jointCtoD, Vector3d jointAxisA,
         Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD, boolean recomputeJointLimits)
   {
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      RevoluteJoint masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, elevatorToJointA, jointAxisA);
      RigidBody rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      PassiveRevoluteJoint passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointAtoB, jointAxisB, true);
      RigidBody rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      PassiveRevoluteJoint passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointBtoC, jointAxisC, true);
      RigidBody rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      PassiveRevoluteJoint passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointCtoD, jointAxisD, true);
      
   }

   private void initializeFourBar(Vector3d elevatorToJointA, Vector3d jointAtoB, Vector3d jointBtoC, Vector3d jointCtoD, Vector3d jointAxis)
   {
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis, jointAxis, jointAxis, jointAxis);
   }
   
   private void initializeSquareFourBar(double sideLength, Vector3d jointAxisA, Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD)
   {
      initializeFourBar(new Vector3d(), new Vector3d(sideLength, 0.0, 0.0), new Vector3d(sideLength, 0.0, 0.0), new Vector3d(sideLength, 0.0, 0.0), jointAxisA,
            jointAxisB, jointAxisC, jointAxisD);
   }
   
   private void initializeFourBar(Vector3d elevatorToJointA, Vector3d jointAtoB, Vector3d jointBtoC, Vector3d jointCtoD, Vector3d jointAxisA,
         Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, elevatorToJointA, jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointAtoB, jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointBtoC, jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointCtoD, jointAxisD, true);
      
      masterJointA.setQ(random.nextDouble());
      passiveJointB.setQ(random.nextDouble());
      passiveJointC.setQ(random.nextDouble());
      passiveJointD.setQ(random.nextDouble());
   }
   
   private void initializeFourBar(RigidBodyTransform jointAtoElevator, RigidBodyTransform jointBtoA, RigidBodyTransform jointCtoB, RigidBodyTransform jointDtoC,
         Vector3d jointAxisA, Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, jointAtoElevator, jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointBtoA, jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointCtoB, jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointDtoC, jointAxisD, true);
      
      masterJointA.setQ(random.nextDouble());
      passiveJointB.setQ(random.nextDouble());
      passiveJointC.setQ(random.nextDouble());
      passiveJointD.setQ(random.nextDouble());
   }

   // the RigidBodies are independent of the calculations done by FourBarKinematicLoop, so this suffices to make all the RigidBodies
   private static RigidBody createAndAttachCylinderRB(String name, RevoluteJoint parentJoint)
   {
      Matrix3d inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(1.0, 1.0, 1.0, Axis.Z);
      return ScrewTools.addRigidBody(name, parentJoint, inertiaCylinder, 1.0, new Vector3d());
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
   
   private void initializeARandomFourBar(Random random)
   {
      // joint axis is along z
      Matrix3d worldToJointAxisRotation = RandomTools.generateRandomRotationMatrix3d(random);
      RigidBodyTransform worldToJointAxis = new RigidBodyTransform();
      worldToJointAxis.setRotationAndZeroTranslation(worldToJointAxisRotation);
      ReferenceFrame frameWithZAlongJointAxis = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("frameWithZAlongJointAxis", worldFrame,
            worldToJointAxis);
      
      // master joint frame
      ReferenceFrame masterJointFrame = ReferenceFrame.generateRandomReferenceFrame("masterJointFrame", random, elevatorFrame);
      
      // generate random quadrilateral
//      double[] 
   }
}
