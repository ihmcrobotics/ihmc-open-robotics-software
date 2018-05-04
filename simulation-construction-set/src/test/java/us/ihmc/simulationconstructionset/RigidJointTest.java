package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.RigidJointPhysics;

public class RigidJointTest
{
   private static final boolean doDynamics = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneRigidJoint()
   {
      Robot robot = new Robot("Test");

      RigidJoint rigidJointOne = new RigidJoint("rigidJointOne", new Vector3D(), robot);

      Link rigidLinkOne = new Link("rigidLinkOne");
      double massOne = 1.0;
      rigidLinkOne.setMassAndRadiiOfGyration(massOne, 0.1, 0.1, 0.1);
      Vector3D centerOfMassOffset = new Vector3D(1.1, 2.2, 3.3);
      rigidLinkOne.setComOffset(centerOfMassOffset);
      rigidJointOne.setLink(rigidLinkOne);

      robot.addRootJoint(rigidJointOne);

      Point3D centerOfMass = new Point3D();
      double totalMass = robot.computeCenterOfMass(centerOfMass);

      assertEquals(massOne, totalMass, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(centerOfMassOffset, centerOfMass, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneRigidJointWithTranslation()
   {
      Robot robot = new Robot("Test");

      RigidJoint rigidJointOne = new RigidJoint("rigidJointOne", new Vector3D(), robot);

      Vector3D translation = new Vector3D(1.1, 2.2, 3.3);
      rigidJointOne.setRigidTranslation(translation);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);

      Link rigidLinkOne = new Link("rigidLinkOne");
      double massOne = 1.0;
      rigidLinkOne.setMassAndRadiiOfGyration(massOne, 0.1, 0.1, 0.1);

      Vector3D centerOfMassOffset = new Vector3D(0.99, -0.4, 1.1);
      rigidLinkOne.setComOffset(centerOfMassOffset);
      rigidJointOne.setLink(rigidLinkOne);

      robot.addRootJoint(rigidJointOne);

      robot.update();
      Point3D centerOfMass = new Point3D();
      double totalMass = robot.computeCenterOfMass(centerOfMass);

      assertEquals(massOne, totalMass, 1e-7);
      Point3D expectedCenterOfMass = new Point3D(centerOfMassOffset);
      expectedCenterOfMass.add(translation);

      EuclidCoreTestTools.assertTuple3DEquals(expectedCenterOfMass, centerOfMass, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneRigidJointWithRotation()
   {
      Robot robot = new Robot("Test");

      RigidJoint rigidJointOne = new RigidJoint("rigidJointOne", new Vector3D(), robot);

      RotationMatrix rotation = new RotationMatrix();

      Vector3D eulerAngles = new Vector3D(0.3, 0.1, 0.2);
      rotation.setEuler(eulerAngles);
      rigidJointOne.setRigidRotation(rotation);

      Link rigidLinkOne = new Link("rigidLinkOne");
      double massOne = 1.0;
      rigidLinkOne.setMassAndRadiiOfGyration(massOne, 0.1, 0.1, 0.1);
      Vector3D centerOfMassOffset = new Vector3D(0.3, 0.7, 1.11);
      rigidLinkOne.setComOffset(centerOfMassOffset);
      rigidJointOne.setLink(rigidLinkOne);

      robot.addRootJoint(rigidJointOne);

      robot.update();
      Point3D centerOfMass = new Point3D();
      double totalMass = robot.computeCenterOfMass(centerOfMass);

      assertEquals(massOne, totalMass, 1e-7);
      Point3D expectedCenterOfMass = new Point3D(centerOfMassOffset);
      rotation.transform(expectedCenterOfMass);

      EuclidCoreTestTools.assertTuple3DEquals(expectedCenterOfMass, centerOfMass, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testOneRigidJointWithRotationAndTranslation()
   {
      Robot robot = new Robot("Test");

      RigidJoint rigidJointOne = new RigidJoint("rigidJointOne", new Vector3D(), robot);

      RotationMatrix rotation = new RotationMatrix();

      Vector3D eulerAngles = new Vector3D(0.3, 0.1, 0.2);
      rotation.setEuler(eulerAngles);
      rigidJointOne.setRigidRotation(rotation);

      Vector3D translation = new Vector3D(0.3, -0.99, 1.11);
      rigidJointOne.setRigidTranslation(translation);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);
      transform.setRotation(rotation);

      Link rigidLinkOne = new Link("rigidLinkOne");
      double massOne = 1.0;
      rigidLinkOne.setMassAndRadiiOfGyration(massOne, 0.1, 0.1, 0.1);
      Vector3D centerOfMassOffset = new Vector3D(0.3, 0.7, 1.11);
      rigidLinkOne.setComOffset(centerOfMassOffset);
      rigidJointOne.setLink(rigidLinkOne);

      robot.addRootJoint(rigidJointOne);

      robot.update();
      Point3D centerOfMass = new Point3D();
      double totalMass = robot.computeCenterOfMass(centerOfMass);

      assertEquals(massOne, totalMass, 1e-7);
      Point3D expectedCenterOfMass = new Point3D(centerOfMassOffset);
      transform.transform(expectedCenterOfMass);

      EuclidCoreTestTools.assertTuple3DEquals(expectedCenterOfMass, centerOfMass, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSinglePinJoint() throws UnreasonableAccelerationException
   {
      Robot robotA = new Robot("RobotA");
      Robot robotB = new Robot("RobotB");

      Vector3D rigidJointOffset = new Vector3D(1.1, 2.2, 3.3);
      RotationMatrix rigidJointRotation = new RotationMatrix();
      rigidJointRotation.setEuler(0.66, 0.1, 0.776);

      Vector3D rotationAxis = new Vector3D(0.1, 1.05, 0.356);
      rotationAxis.normalize();
      Vector3D rotatedRotationAxis = new Vector3D(rotationAxis);
      rigidJointRotation.transform(rotatedRotationAxis);

      PinJoint pinJointA = new PinJoint("pinA", rigidJointOffset, robotA, rotatedRotationAxis);

      RigidJoint rigidJointB = new RigidJoint("rigidJointB", new Vector3D(), robotB);
      rigidJointB.setRigidRotation(rigidJointRotation);
      rigidJointB.setRigidTranslation(rigidJointOffset);
      Link rigidLinkB = new Link("rigidLinkB");
      rigidJointB.setLink(rigidLinkB);

      PinJoint pinJointB = new PinJoint("pinB", new Vector3D(), robotB, rotationAxis);

      Link linkOneA = new Link("linkOneA");
      Link linkOneB = new Link("linkOneB");

      double massOne = 1.056;
      Vector3D radiiOfGyrationOne = new Vector3D(0.1, 0.2, 0.3);

      linkOneA.setMass(massOne);
      linkOneB.setMassAndRadiiOfGyration(massOne, radiiOfGyrationOne);

      Matrix3D momentOfInertiaOne = new Matrix3D();
      linkOneB.getMomentOfInertia(momentOfInertiaOne);
      Matrix3D rotatedMomentOfInertiaOne = new Matrix3D(momentOfInertiaOne);
      rigidJointRotation.transform(rotatedMomentOfInertiaOne);

      linkOneA.setMomentOfInertia(rotatedMomentOfInertiaOne);

      Vector3D centerOfMassOffsetOne = new Vector3D(0.1, 0.2, 1.0);
      Vector3D rotatedCenterOfMassOffsetOne = new Vector3D(centerOfMassOffsetOne);
      rigidJointRotation.transform(rotatedCenterOfMassOffsetOne);

      linkOneA.setComOffset(rotatedCenterOfMassOffsetOne);
      linkOneB.setComOffset(centerOfMassOffsetOne);

      pinJointA.setLink(linkOneA);
      pinJointB.setLink(linkOneB);

      robotA.addRootJoint(pinJointA);
      rigidJointB.addJoint(pinJointB);
      robotB.addRootJoint(rigidJointB);

      pinJointA.setQ(0.189);
      pinJointB.setQ(0.189);

      pinJointA.setQd(0.145);
      pinJointB.setQd(0.145);

      robotA.update();
      robotB.update();

      robotA.doDynamicsButDoNotIntegrate();
      robotB.doDynamicsButDoNotIntegrate();

      checkRobotsHaveSameState(robotA, robotB);

      if (doDynamics)
      {
         robotA.doDynamicsAndIntegrate(0.0001);
         robotB.doDynamicsAndIntegrate(0.0001);

         checkRobotsHaveSameState(robotA, robotB);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSingleSliderJoint() throws UnreasonableAccelerationException
   {
      Robot robotA = new Robot("RobotA");
      Robot robotB = new Robot("RobotB");

      Vector3D rigidJointOffset = new Vector3D(1.1, 2.2, 3.3);
      RotationMatrix rigidJointRotation = new RotationMatrix();
      rigidJointRotation.setEuler(0.5, 0.1, 0.99);

      Vector3D rotationAxis = new Vector3D(0.1, 1.0, 0.7);
      rotationAxis.normalize();
      Vector3D rotatedRotationAxis = new Vector3D(rotationAxis);
      rigidJointRotation.transform(rotatedRotationAxis);

      SliderJoint sliderJointA = new SliderJoint("sliderA", rigidJointOffset, robotA, rotatedRotationAxis);

      RigidJoint rigidJointB = new RigidJoint("rigidJointB", new Vector3D(), robotB);
      rigidJointB.setRigidRotation(rigidJointRotation);
      rigidJointB.setRigidTranslation(rigidJointOffset);
      Link rigidLinkB = new Link("rigidLinkB");
      rigidJointB.setLink(rigidLinkB);

      SliderJoint sliderJointB = new SliderJoint("sliderB", new Vector3D(), robotB, rotationAxis);

      Link linkOneA = new Link("linkOneA");
      Link linkOneB = new Link("linkOneB");

      double massOne = 1.0;
      Vector3D radiiOfGyrationOne = new Vector3D(0.1, 0.2, 0.3);
      Vector3D rotatedRadiiOfGyrationOne = new Vector3D(radiiOfGyrationOne);
      rigidJointRotation.transform(rotatedRadiiOfGyrationOne);

      linkOneA.setMass(massOne);
      linkOneB.setMassAndRadiiOfGyration(massOne, radiiOfGyrationOne);

      Matrix3D momentOfInertiaOne = new Matrix3D();
      linkOneB.getMomentOfInertia(momentOfInertiaOne);
      Matrix3D rotatedMomentOfInertiaOne = new Matrix3D(momentOfInertiaOne);
      rigidJointRotation.transform(rotatedMomentOfInertiaOne);

      linkOneA.setMomentOfInertia(rotatedMomentOfInertiaOne);

      Vector3D centerOfMassOffsetOne = new Vector3D(0.1, 0.2, 1.0);
      Vector3D rotatedCenterOfMassOffsetOne = new Vector3D(centerOfMassOffsetOne);
      rigidJointRotation.transform(rotatedCenterOfMassOffsetOne);

      linkOneA.setComOffset(rotatedCenterOfMassOffsetOne);
      linkOneB.setComOffset(centerOfMassOffsetOne);

      sliderJointA.setLink(linkOneA);
      sliderJointB.setLink(linkOneB);

      robotA.addRootJoint(sliderJointA);
      rigidJointB.addJoint(sliderJointB);
      robotB.addRootJoint(rigidJointB);

      sliderJointA.setQ(0.178);
      sliderJointB.setQ(0.178);

      sliderJointA.setQd(0.145);
      sliderJointB.setQd(0.145);

      robotA.update();
      robotB.update();

      robotA.doDynamicsButDoNotIntegrate();
      robotB.doDynamicsButDoNotIntegrate();

      checkRobotsHaveSameState(robotA, robotB);

      if (doDynamics)
      {
         robotA.doDynamicsAndIntegrate(0.0001);
         robotB.doDynamicsAndIntegrate(0.0001);

         checkRobotsHaveSameState(robotA, robotB);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPinJointThenRigidJointThenSliderOne() throws UnreasonableAccelerationException
   {
      // Rigid joint rotation but no offset.

      Vector3D rigidJointOffset = new Vector3D(0.0, 0.0, 0.0);
      RotationMatrix rigidJointRotation = new RotationMatrix();
      rigidJointRotation.setEuler(0.3, -0.55, 0.72);

      Vector3D centerOfMassOffsetOne = new Vector3D(); //0.3, 0.7, 10.11);
      Vector3D centerOfMassOffsetTwo = new Vector3D(); //100.3, 0.7, 1.11);

      Vector3D radiiOfGyrationOne = new Vector3D(0.25, 0.19, 0.6);
      Vector3D radiiOfGyrationTwo = new Vector3D(0.13, 0.14, 0.17);

      double qOne = 2.2;
      double qdOne = 10.145;
      double qTwo = 0.0;
      double qdTwo = 0.0;

      runTestWithPinThenRigidThenSliderJoints(rigidJointOffset, rigidJointRotation, centerOfMassOffsetOne, centerOfMassOffsetTwo, radiiOfGyrationOne,
                                              radiiOfGyrationTwo, qOne, qdOne, qTwo, qdTwo);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPinJointThenRigidJointThenSliderTwo() throws UnreasonableAccelerationException
   {
      // No Rigid joint rotation or offset, but center of mass offsets.

      Vector3D rigidJointOffset = new Vector3D(0.0, 0.0, 0.0);
      RotationMatrix rigidJointRotation = new RotationMatrix();

      Vector3D centerOfMassOffsetOne = new Vector3D();
      Vector3D centerOfMassOffsetTwo = new Vector3D(100.0, 0.0, 0.0);

      Vector3D radiiOfGyrationOne = new Vector3D(0.25, 0.19, 0.6);
      Vector3D radiiOfGyrationTwo = new Vector3D(0.13, 0.14, 0.17);

      double qOne = 0.0;
      double qdOne = 0.0;
      double qTwo = 0.0;
      double qdTwo = 0.0;

      runTestWithPinThenRigidThenSliderJoints(rigidJointOffset, rigidJointRotation, centerOfMassOffsetOne, centerOfMassOffsetTwo, radiiOfGyrationOne,
                                              radiiOfGyrationTwo, qOne, qdOne, qTwo, qdTwo);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPinJointThenRigidJointThenSliderThree() throws UnreasonableAccelerationException
   {
      // No Rigid joint rotation or offset, but translation of second joint.

      Vector3D rigidJointOffset = new Vector3D();//1.1, 2.2, 3.3);
      RotationMatrix rigidJointRotation = new RotationMatrix();
//      rigidJointRotation.setEuler(0.3, -0.55, 0.72);

      Vector3D centerOfMassOffsetOne = new Vector3D();
      Vector3D centerOfMassOffsetTwo = new Vector3D();

      Vector3D radiiOfGyrationOne = new Vector3D(0.25, 0.19, 0.6);
      Vector3D radiiOfGyrationTwo = new Vector3D(0.13, 0.14, 0.17);

      double qOne = 0.0;
      double qdOne = 0.0;
      double qTwo = 100.0;
      double qdTwo = 0.0;

      runTestWithPinThenRigidThenSliderJoints(rigidJointOffset, rigidJointRotation, centerOfMassOffsetOne, centerOfMassOffsetTwo, radiiOfGyrationOne,
                                              radiiOfGyrationTwo, qOne, qdOne, qTwo, qdTwo);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPinJointThenRigidJointThenSliderFour() throws UnreasonableAccelerationException
   {
      // Offsets, rotations, joint positions, and joint velocities.

      Vector3D rigidJointOffset = new Vector3D(1.1, 2.2, 3.3);
      RotationMatrix rigidJointRotation = new RotationMatrix();
      rigidJointRotation.setEuler(0.3, -0.55, 0.72);

      Vector3D centerOfMassOffsetOne = new Vector3D(0.3, 0.7, 10.11);
      Vector3D centerOfMassOffsetTwo = new Vector3D(100.3, 0.7, 1.11);

      Vector3D radiiOfGyrationOne = new Vector3D(0.25, 0.19, 0.6);
      Vector3D radiiOfGyrationTwo = new Vector3D(0.13, 0.14, 0.17);

      double qOne = 1.378;
      double qdOne = 12.96;
      double qTwo = 109.0;
      double qdTwo = 20.45;

      runTestWithPinThenRigidThenSliderJoints(rigidJointOffset, rigidJointRotation, centerOfMassOffsetOne, centerOfMassOffsetTwo, radiiOfGyrationOne,
                                              radiiOfGyrationTwo, qOne, qdOne, qTwo, qdTwo);
   }

   private void runTestWithPinThenRigidThenSliderJoints(Vector3D rigidJointOffset, RotationMatrix rigidJointRotation, Vector3D centerOfMassOffsetOne,
                                                        Vector3D centerOfMassOffsetTwo, Vector3D radiiOfGyrationOne, Vector3D radiiOfGyrationTwo, double qOne,
                                                        double qdOne, double qTwo, double qdTwo)
         throws UnreasonableAccelerationException
   {
      Robot robotA = new Robot("RobotA");
      Robot robotB = new Robot("RobotB");

      PinJoint pinJointA = new PinJoint("pinA", new Vector3D(), robotA, Axis.Y);
      PinJoint pinJointB = new PinJoint("pinB", new Vector3D(), robotB, Axis.Y);

      Link linkOneA = new Link("linkOneA");
      Link linkOneB = new Link("linkOneB");

      double massOne = 1.0;
      linkOneA.setMassAndRadiiOfGyration(massOne, radiiOfGyrationOne);
      linkOneB.setMassAndRadiiOfGyration(massOne, radiiOfGyrationOne);

      linkOneA.setComOffset(centerOfMassOffsetOne);
      linkOneB.setComOffset(centerOfMassOffsetOne);

      pinJointA.setLink(linkOneA);
      pinJointB.setLink(linkOneB);

      Vector3D sliderAxis = new Vector3D(1.0, 0.0, 0.0);
      Vector3D rotatedSliderAxis = new Vector3D(sliderAxis);
      rigidJointRotation.transform(rotatedSliderAxis);

      SliderJoint sliderJointA = new SliderJoint("sliderA", rigidJointOffset, robotA, rotatedSliderAxis);

      RigidJoint rigidJointB = new RigidJoint("rigidB", new Vector3D(), robotB);
      rigidJointB.setRigidRotation(rigidJointRotation);
      rigidJointB.setRigidTranslation(rigidJointOffset);
      Link rigidLinkB = new Link("rigidLinkB");
      rigidJointB.setLink(rigidLinkB);

      pinJointB.addJoint(rigidJointB);

      SliderJoint sliderJointB = new SliderJoint("sliderB", new Vector3D(), robotB, sliderAxis);

      Link linkTwoA = new Link("linkTwoA");
      Link linkTwoB = new Link("linkTwoB");

      double massTwo = 1.15;

      linkTwoA.setMass(massTwo);
      //      linkTwoB.setMass(massTwo);
      linkTwoB.setMassAndRadiiOfGyration(massTwo, radiiOfGyrationTwo);

      Matrix3D momentOfInertiaTwo = new Matrix3D();
      linkTwoB.getMomentOfInertia(momentOfInertiaTwo);
      Matrix3D rotatedMomentOfInertiaTwo = new Matrix3D(momentOfInertiaTwo);
      rigidJointRotation.transform(rotatedMomentOfInertiaTwo);

      linkTwoA.setMomentOfInertia(rotatedMomentOfInertiaTwo);

      Vector3D rotatedCenterOfMassOffsetTwo = new Vector3D(centerOfMassOffsetTwo);
      rigidJointRotation.transform(rotatedCenterOfMassOffsetTwo);
      linkTwoA.setComOffset(rotatedCenterOfMassOffsetTwo);
      linkTwoB.setComOffset(centerOfMassOffsetTwo);

      sliderJointA.setLink(linkTwoA);
      sliderJointB.setLink(linkTwoB);

      pinJointA.addJoint(sliderJointA);
      rigidJointB.addJoint(sliderJointB);

      robotA.addRootJoint(pinJointA);
      robotB.addRootJoint(pinJointB);

      pinJointA.setQ(qOne);
      pinJointB.setQ(qOne);

      pinJointA.setQd(qdOne);
      pinJointB.setQd(qdOne);

      sliderJointA.setQ(qTwo);
      sliderJointB.setQ(qTwo);

      sliderJointA.setQd(qdTwo);
      sliderJointB.setQd(qdTwo);

      robotA.update();
      robotB.update();

      robotA.doDynamicsButDoNotIntegrate();
      robotB.doDynamicsButDoNotIntegrate();

      checkRobotsHaveSameState(robotA, robotB);

      if (doDynamics)
      {
         robotA.doDynamicsAndIntegrate(0.0001);
         robotB.doDynamicsAndIntegrate(0.0001);

         checkRobotsHaveSameState(robotA, robotB);
      }
   }

   private void checkRobotsHaveSameState(Robot robotA, Robot robotB)
   {
      Point3D centerOfMassA = new Point3D();
      Point3D centerOfMassB = new Point3D();
      double totalMassA = robotA.computeCenterOfMass(centerOfMassA);
      double totalMassB = robotB.computeCenterOfMass(centerOfMassB);

      Vector3D angularMomentumA = new Vector3D();
      Vector3D angularMomentumB = new Vector3D();
      robotA.computeAngularMomentum(angularMomentumA);
      robotB.computeAngularMomentum(angularMomentumB);

      Vector3D linearMomentumA = new Vector3D();
      Vector3D linearMomentumB = new Vector3D();
      robotA.computeLinearMomentum(linearMomentumA);
      robotB.computeLinearMomentum(linearMomentumB);

      assertEquals(totalMassA, totalMassB, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(centerOfMassA, centerOfMassB, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumA, linearMomentumB, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumA, angularMomentumB, 1e-9);

      ArrayList<Joint> jointsA = robotA.getRootJoints();
      ArrayList<Joint> jointsB = robotB.getRootJoints();

      recursivelyTestJointStateIsTheSame(jointsA, jointsB);
   }

   private void recursivelyTestJointStateIsTheSame(ArrayList<Joint> jointsA, ArrayList<Joint> jointsB)
   {
      int indexA = 0;
      int indexB = 0;

      while ((jointsA.size() < indexA) && (jointsB.size() < indexB))
      {
         Joint jointA = jointsA.get(indexA);
         Joint jointB = jointsB.get(indexB);

         if (jointA instanceof RigidJoint)
         {
            indexA++;
            continue;
         }

         if (jointB instanceof RigidJoint)
         {
            indexB++;
            continue;
         }

         if (jointA instanceof OneDegreeOfFreedomJoint)
         {
            OneDegreeOfFreedomJoint pinJointA = (OneDegreeOfFreedomJoint) jointA;
            OneDegreeOfFreedomJoint pinJointB = (OneDegreeOfFreedomJoint) jointB;

            assertEquals(pinJointA.getQ(), pinJointB.getQ(), 1e-10);
            assertEquals(pinJointA.getQD(), pinJointB.getQD(), 1e-10);
            assertEquals(pinJointA.getQDD(), pinJointB.getQDD(), 1e-10);
         }

         else
         {
            throw new RuntimeException("Only testing OneDegreeOfFreedomJoints right now...");
         }

         ArrayList<Joint> childrenJointsA = jointA.getChildrenJoints();
         ArrayList<Joint> childrenJointsB = jointB.getChildrenJoints();

         recursivelyTestJointStateIsTheSame(childrenJointsA, childrenJointsB);

         indexA++;
         indexB++;
      }
   }

   public static void main(String[] args)
   {
      //      MutationTestFacilitator.facilitateMutationTestForClass(RigidJoint.class, RigidJointTest.class);
      MutationTestFacilitator.facilitateMutationTestForClass(RigidJointPhysics.class, RigidJointTest.class);
   }
}
