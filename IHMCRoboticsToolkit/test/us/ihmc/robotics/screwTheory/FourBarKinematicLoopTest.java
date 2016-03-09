package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import javax.swing.plaf.synth.SynthSeparatorUI;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FourBarKinematicLoopTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private RigidBody elevator, rigidBodyAB, rigidBodyBC, rigidBodyCD;
   private ReferenceFrame elevatorFrame;
   private RevoluteJoint masterJointA;
   private PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private FourBarKinematicLoop fourBarKinematicLoop;

   private final static double eps = 1e-7;

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquare()
   {
      // initialize to a square of unit length
      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch(Exception e)
      {         
      }
      
      initializeAllJointsToSameLimits(0.0, Math.PI);      
      fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
   public void testRectangleWithJointOutOfPlane()
   {
      // initialize to rectangle of side lengths 1.0 and 2.0
      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointBtoC = new Vector3d(2.0, 0.0, 0.2);
      Vector3d jointCtoD = new Vector3d(1.0, 0.0, -0.2);
      Vector3d jointDtoA = new Vector3d(2.0, 0.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;
      initializeAllJointsToSameLimits(0.0, Math.PI);

      fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
   public void testParallelJointAxesIsEnforced()
   {
      // random elevator frame, unit length square, and two slightly different rotation axes
      Random random = new Random(483942L);
      elevatorFrame = ReferenceFrame.generateRandomReferenceFrame("elevatorFrame", random, worldFrame);
      elevator = new RigidBody("elevator", elevatorFrame);
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis2, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis2, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         
         initializeSquareFourBar(sideLength, jointAxis1, jointAxis1, jointAxis1, jointAxis2);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
      // initialize to quadrulateral with side lengths 2.0, 1.0, 0.5, 1.0
      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      Vector3d jointAxis = new Vector3d(0.0, 0.0, 1.0);
      Vector3d elevatorToJointA = new Vector3d();
      Vector3d jointAtoB = new Vector3d(2.0, 0.0, 0.0);
      Vector3d jointBtoC = new Vector3d(1.0, 0.0, 0.0);
      Vector3d jointCtoD = new Vector3d(0.5, 0.0, 0.0);
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, 0.0);
      initializeFourBar(elevatorToJointA, jointAtoB, jointBtoC, jointCtoD, jointAxis);
      boolean recomputeJointLimits = false;

      double jointAMin = getInteriorAngleFromCosineRule(2.0, 1.0 + 0.5, 1.0);
      double jointAMax = getInteriorAngleFromCosineRule(2.0, 1.0, 0.5 + 1.0);
      double angleEpsilon = 1e-6;
      
      System.out.println("\nTest computed min angle: " + jointAMin);
      System.out.println("Test computed max angle: " + jointAMax);
      
      try
      {
         masterJointA.setJointLimitLower(jointAMin - angleEpsilon);
         masterJointA.setJointLimitUpper(jointAMax - angleEpsilon);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);         
         fail();
      }
      catch(Exception e)
      {
      }

      try
      {
         masterJointA.setJointLimitLower(jointAMin + angleEpsilon);
         masterJointA.setJointLimitUpper(jointAMax + angleEpsilon);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);         
         fail();
      }
      catch(Exception e)
      {
      }
      
      try
      {
         masterJointA.setJointLimitLower(jointAMin - angleEpsilon);
         masterJointA.setJointLimitUpper(jointAMax + angleEpsilon);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);         
         fail();
      }
      catch(Exception e)
      {
      }
      
      try
      {
         masterJointA.setJointLimitLower(jointAMin + angleEpsilon);
         masterJointA.setJointLimitUpper(jointAMax - angleEpsilon);
         new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);         
      }
      catch(Exception e)
      {
         fail();
      }
   }
   
   

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJointLimitsNotSet()
   {
      
   }
   
   private void initializeFourBar(Vector3d elevatorToJointA, Vector3d jointAtoB, Vector3d jointBtoC, Vector3d jointCtoD, Vector3d jointAxis)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, elevatorToJointA, jointAxis);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointAtoB, jointAxis, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointBtoC, jointAxis, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointCtoD, jointAxis, true);
   }

   private void initializeSquareFourBar(double sideLength, Vector3d jointAxisA, Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, new RigidBodyTransform(), jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, new Vector3d(sideLength, 0.0, 0.0), jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, new Vector3d(sideLength, 0.0, 0.0), jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, new Vector3d(sideLength, 0.0, 0.0), jointAxisD, true);
   }

   private void initializeFourBar(RigidBodyTransform elevatorToJointA, RigidBodyTransform jointAtoB, RigidBodyTransform jointBtoC, RigidBodyTransform jointCtoD,
         Vector3d jointAxisA, Vector3d jointAxisB, Vector3d jointAxisC, Vector3d jointAxisD)
   {
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, elevatorToJointA, jointAxisA);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, jointAtoB, jointAxisB, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, jointBtoC, jointAxisC, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, jointCtoD, jointAxisD, true);
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
}
