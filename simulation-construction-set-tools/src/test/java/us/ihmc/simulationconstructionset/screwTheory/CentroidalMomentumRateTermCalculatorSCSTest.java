package us.ihmc.simulationconstructionset.screwTheory;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class CentroidalMomentumRateTermCalculatorSCSTest
{
   private static final double EPSILON = 1.0e-5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int NUMBER_OF_ITERATIONS = 10;

   private final double controlDT = 1.0e-8;

   private final DenseMatrix64F a = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aPrevVal = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aDot = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aTermCalculator = new DenseMatrix64F(0,0);

   private final DenseMatrix64F aDotVNumerical = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F aDotVAnalytical = new DenseMatrix64F(6, 1);

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void chainTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomGeometry.nextVector3D(random, 1.0);

      joints.addAll(MultiBodySystemRandomTools.nextRevoluteJointChain(random, "blop", elevator, jointAxes));
      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", elevator.getChildrenJoints().get(0));

      assertAAndADotV(random, joints, elevator, robot,numberOfJoints);
   }

	@Test
   public void treeTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      RevoluteJoint rootJoint = MultiBodySystemRandomTools.nextRevoluteJoint(random, "rootJoint", elevator); // Just to make sure there is only one root joint for the SCS robot
      RigidBodyBasics rootBody = MultiBodySystemRandomTools.nextRigidBody(random, "rootBody", rootJoint);

      int numberOfJoints = 10; 
      joints.addAll(MultiBodySystemRandomTools.nextRevoluteJointTree(random, rootBody, numberOfJoints - 1));
      joints.add(0, rootJoint);
      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", rootJoint);

      assertAAndADotV(random, joints, elevator, robot, numberOfJoints);
   }

	@Test
   public void floatingChainTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      int numberOfJoints = 12; 
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomGeometry.nextVector3D(random, 1.0);

      RandomFloatingRevoluteJointChain idRobot = new RandomFloatingRevoluteJointChain(random, jointAxes);
      RigidBodyBasics elevator = idRobot.getElevator();
      joints.addAll(idRobot.getRevoluteJoints());

      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", idRobot.getRootJoint());

      assertAAndADotV(random, joints, elevator, robot, numberOfJoints + 1);
   }

   private void assertAAndADotV(Random random, ArrayList<RevoluteJoint> joints, RigidBodyBasics elevator, SCSRobotFromInverseDynamicsRobotModel robot,int numJoints)
         throws UnreasonableAccelerationException
   {
      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(MultiBodySystemTools.collectSubtreeJoints(elevator));
      DenseMatrix64F v = new DenseMatrix64F(numberOfDoFs, 1);

      JointBasics[] idJoints = new JointBasics[numJoints]; 
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);

      CentroidalMomentumCalculator centroidalMomentumMatrixCalculator = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);

      a.reshape(6, numberOfDoFs);
      aPrevVal.reshape(6, numberOfDoFs);
      aDot.reshape(6, numberOfDoFs);
      aTermCalculator.reshape(6,numberOfDoFs);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      
      CentroidalMomentumRateCalculator centroidalMomentumRateTermCalculator = new CentroidalMomentumRateCalculator(elevator, centerOfMassFrame);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

         robot.updateJointPositions_ID_to_SCS();
         robot.updateJointVelocities_ID_to_SCS();
         robot.updateJointTorques_ID_to_SCS();
         elevator.updateFramesRecursively();
         centerOfMassFrame.update();

         centroidalMomentumMatrixCalculator.reset();
         aPrevVal.set(centroidalMomentumMatrixCalculator.getCentroidalMomentumMatrix());

         robot.doDynamicsAndIntegrate(controlDT);
         robot.updateVelocities();
         robot.updateJointPositions_SCS_to_ID();
         robot.updateJointVelocities_SCS_to_ID();
         elevator.updateFramesRecursively();
         centerOfMassFrame.update();
         
         robot.packIdJoints(idJoints);
         MultiBodySystemTools.extractJointsState(idJoints, JointStateType.VELOCITY, v);

         centroidalMomentumRateTermCalculator.reset();
         aDotVAnalytical.set(centroidalMomentumRateTermCalculator.getBiasSpatialForceMatrix());

         aTermCalculator.set(centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix());
         // Compute aDotV numerically
         centroidalMomentumMatrixCalculator.reset();
         a.set(centroidalMomentumMatrixCalculator.getCentroidalMomentumMatrix());
         MatrixTools.numericallyDifferentiate(aDot, aPrevVal, a, controlDT);
         CommonOps.mult(aDot, v, aDotVNumerical);

         smartPrintOutADotV(EPSILON);

         JUnitTools.assertMatrixEquals(aDotVNumerical, aDotVAnalytical, EPSILON);
         JUnitTools.assertMatrixEquals(a,aTermCalculator,EPSILON);
      }
   }

   private void smartPrintOutADotV(double epsilon)
   {
      DenseMatrix64F difference = new DenseMatrix64F(aDotVNumerical.numRows, aDotVNumerical.numCols);
      CommonOps.subtract(aDotVNumerical, aDotVAnalytical, difference);

      for (int i = 0; i < difference.numRows; i++)
         if(Math.abs(difference.get(i, 0)) > epsilon)
         {
            printOutADotV();
            break;
         }
   }

   private void printOutADotV()
   {
      int numChar = 6;
      int precision = 3;
      String format = "%" + numChar + "." + precision + "f ";

      System.out.println("----------- ADotV -----------");
      System.out.println("Numerical: ||\tAnalytical:");
      for (int i = 0; i < aDotVNumerical.numRows; i++)
         System.out.printf(format + "\t   ||\t" + format + "\n", aDotVNumerical.get(i, 0), aDotVAnalytical.get(i, 0));
      System.out.println();
   }
}
