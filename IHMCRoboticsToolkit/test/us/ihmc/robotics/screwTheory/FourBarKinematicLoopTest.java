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
   private Vector3d jointAxis;
   private RevoluteJoint masterJointA;
   private PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private FourBarKinematicLoop fourBarKinematicLoop;
   private Vector3d closurePointFromLastPassiveJoint;
   
   private final static double eps = 1e-7;
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPlanarSquare()
   {
      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      jointAxis = new Vector3d(0.0, 0.0, 1.0);
      double sideLength = 1.0;

      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, new RigidBodyTransform(), jointAxis);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", 10.0, 0.1, sideLength, masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, new Vector3d(sideLength, 0.0, 0.0), jointAxis, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", 10.0, 0.1, sideLength, passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, new Vector3d(sideLength, 0.0, 0.0), jointAxis, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", 10.0, 0.1, sideLength, passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, new Vector3d(sideLength, 0.0, 0.0), jointAxis, true);
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyDA", 10.0, 0.1, sideLength, passiveJointD);
      
      boolean recomputeJointLimits = false;      
      initializeAllJointsToSameLimits(0.0, Math.PI);
      
      closurePointFromLastPassiveJoint = new Vector3d(1.0, 0.0, 0.0);
      
      fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, closurePointFromLastPassiveJoint, recomputeJointLimits);
      
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
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRectangleWithJointOutOfPlane()
   {
      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      jointAxis = new Vector3d(0.0, 0.0, 1.0);
      double sideLength1 = 1.0;
      double sideLength2 = 1.5;
      double zOffset = 0.2;
      
      masterJointA = ScrewTools.addRevoluteJoint("jointA", elevator, new RigidBodyTransform(), jointAxis);
      rigidBodyAB = createAndAttachCylinderRB("rigidBodyAB", 10.0, 0.1, sideLength1, masterJointA);
      passiveJointB = ScrewTools.addPassiveRevoluteJoint("jointB", rigidBodyAB, new Vector3d(sideLength1, 0.0, 0.0), jointAxis, true);
      rigidBodyBC = createAndAttachCylinderRB("rigidBodyBC", 10.0, 0.1, sideLength2, passiveJointB);
      passiveJointC = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, new Vector3d(sideLength2, 0.0, zOffset), jointAxis, true);
      rigidBodyCD = createAndAttachCylinderRB("rigidBodyCD", 10.0, 0.1, sideLength1, passiveJointC);
      passiveJointD = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, new Vector3d(sideLength1, 0.0, -zOffset), jointAxis, true);
      rigidBodyDA = createAndAttachCylinderRB("rigidBodyDA", 10.0, 0.1, sideLength2, passiveJointD);

      boolean recomputeJointLimits = false;
      initializeAllJointsToSameLimits(0.0, Math.PI);

      closurePointFromLastPassiveJoint = new Vector3d(sideLength2, 0.0, 0.0);
      
      fourBarKinematicLoop = new FourBarKinematicLoop("simpleSquare", masterJointA, passiveJointB, passiveJointC, passiveJointD, closurePointFromLastPassiveJoint, recomputeJointLimits);

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
      assertEquals(passiveJointB.getQ(), - Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), - angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), - Math.PI + angleEpsilon, eps);      
      assertEquals(passiveJointB.getQd(), - masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), - masterJointVelocity, eps);
      
      // master joint in near minimum angle, non-zero velocity
      masterJointA.setQ(angleEpsilon);
      masterJointA.setQd(masterJointVelocity);
      fourBarKinematicLoop.updateAnglesAndVelocities();
      assertEquals(passiveJointB.getQ(), - angleEpsilon, eps);
      assertEquals(passiveJointC.getQ(), - Math.PI + angleEpsilon, eps);
      assertEquals(passiveJointD.getQ(), - angleEpsilon, eps);      
      assertEquals(passiveJointB.getQd(), - masterJointVelocity, eps);
      assertEquals(passiveJointC.getQd(), masterJointVelocity, eps);
      assertEquals(passiveJointD.getQd(), - masterJointVelocity, eps);
      
   }
   
   private FourBarKinematicLoop createRandomFourBarKinematicLoop(Random random)
   {      
      Matrix3d randomRotation = RandomTools.generateRandomRotationMatrix3d(random);
      AxisAngle4d randomAxisAngle = new AxisAngle4d();
      randomAxisAngle.set(randomRotation);
      jointAxis = new Vector3d(randomAxisAngle.x, randomAxisAngle.y, randomAxisAngle.z);
      
      elevatorFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame", random, worldFrame);
      elevator = new RigidBody("elevator", elevatorFrame);
      
      RigidBodyTransform elevatorToMasterTransform = RigidBodyTransform.generateRandomTransform(random);
      masterJointA = ScrewTools.addRevoluteJoint("masterJointA", elevator, elevatorToMasterTransform, jointAxis);
      
      Vector3d translationAB = RandomTools.generateRandomVector(random);
      Vector3d translationBC = RandomTools.generateRandomVector(random);
      Vector3d translationCD = RandomTools.generateRandomVector(random);
      
      return null;

//      FourBarKinematicLoop fourBarKinematicLoop = new FourBarKinematicLoop("fourBarTestLoop", masterJointA, passiveJointB, passiveJointC, passiveJointD, closurePointFromLastPassiveJointInWorld, recomputeJointLimits);
   }
   
   private RigidBody createAndAttachCylinderRB(String name, double mass, double radius, double length, RevoluteJoint parentJoint)
   {
      Matrix3d inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);
      Vector3d comOffsetCylinder = new Vector3d(0.0, 0.0, -length / 2.0);
      return ScrewTools.addRigidBody(name, parentJoint, inertiaCylinder, mass, comOffsetCylinder);
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
