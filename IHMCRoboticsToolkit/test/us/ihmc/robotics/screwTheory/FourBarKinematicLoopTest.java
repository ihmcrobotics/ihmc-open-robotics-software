package us.ihmc.robotics.screwTheory;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class FourBarKinematicLoopTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private RigidBody elevator, rigidBodyAB, rigidBodyBC, rigidBodyCD, rigidBodyDA;
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
         fail();
      }
      catch (Exception e)
      {
      }

      try
      {
         masterJointA.setQ(1.5 * Math.PI);
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
      // random elevator frame, unit length square, and rotation around the z axis
      Random random = new Random(483942L);
      elevatorFrame = ReferenceFrame.generateRandomReferenceFrame("elevatorFrame", random, worldFrame);
      elevator = new RigidBody("elevator", elevatorFrame);
      double epsilonAngle = 1e-3;
      double sideLength = 1.0;
      Vector3d jointAxis1 = new Vector3d(0.0, 0.0, 1.0);
      Vector3d jointAxis2 = new Vector3d(0.0, Math.sin(epsilonAngle), Math.cos(epsilonAngle));
      Vector3d jointDtoA = new Vector3d(1.0, 0.0, 0.0);
      boolean recomputeJointLimits = false;

      try
      {
         initializeSquareFourBar(sideLength, jointAxis2, jointAxis1, jointAxis1, jointAxis1);
         initializeAllJointsToSameLimits(0.0, Math.PI);
         fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
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
         fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, jointDtoA,
               recomputeJointLimits);
         fail();
      }
      catch (Exception e)
      {
      }

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
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyDA", passiveJointD);
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
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyDA", passiveJointD);
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
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyDA", passiveJointD);
   }

   // the RigidBodies are independent of the calculations done by FourBarKinematicLoop
   private RigidBody createAndAttachCylinderRB(String name, RevoluteJoint parentJoint)
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
}
