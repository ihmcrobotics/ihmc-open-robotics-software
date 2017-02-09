package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.Before;
import org.junit.Test;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;

public class ScrewToolsTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private RigidBody elevator;
   private Random random;
   private List<RigidBody> firstLevelSubTrees;
   private List<RigidBody> secondLevelSubTrees;
   private Set<RigidBody> exclusions;
   private Set<InverseDynamicsJoint> exclusionsJoints;
   private ArrayList<RevoluteJoint> joints;

   protected static final double epsilon = 1e-10;
   protected ReferenceFrame theFrame = ReferenceFrame.constructARootFrame("theFrame", false, true, true);
   protected ReferenceFrame aFrame = ReferenceFrame.constructARootFrame("aFrame", false, true, true);

   @Before
   public void setUp()
   {
      elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      random = new Random(1986L);

      setUpRandomTree(elevator);

      firstLevelSubTrees = new ArrayList<RigidBody>();

      for (InverseDynamicsJoint childJoint : elevator.getChildrenJoints())
      {
         firstLevelSubTrees.add(childJoint.getSuccessor());
      }

      secondLevelSubTrees = new ArrayList<RigidBody>();
      for(int i = 0; i < 3; i++)
      {
         for (InverseDynamicsJoint childJoint : firstLevelSubTrees.get(i).getChildrenJoints())
         {
            secondLevelSubTrees.add(childJoint.getSuccessor());
         }
      }

      exclusions = new LinkedHashSet<RigidBody>();
      exclusionsJoints = new LinkedHashSet<InverseDynamicsJoint>();
      exclusions.add(firstLevelSubTrees.get(1));

      for (InverseDynamicsJoint excludedJoint : firstLevelSubTrees.get(1).getChildrenJoints())
      {
         exclusionsJoints.add(excludedJoint);
      }

      InverseDynamicsJoint[] subtreeJoints = ScrewTools.computeSubtreeJoints(firstLevelSubTrees.get(2));
      RigidBody[] lastSubTree = ScrewTools.computeSuccessors(subtreeJoints);
      RigidBody halfwayDownLastSubTree = lastSubTree[3];
      exclusions.add(halfwayDownLastSubTree);

      for (InverseDynamicsJoint excludedJoint : halfwayDownLastSubTree.getChildrenJoints())
      {
         exclusionsJoints.add(excludedJoint);
      }
   }

   private void setUpRandomTree(RigidBody elevator)
   {
      joints = new ArrayList<RevoluteJoint>();

      Vector3d[] jointAxes1 = {X, Y, Z, Y, X};
      ScrewTestTools.createRandomChainRobot("chainA", joints, elevator, jointAxes1, random);

      Vector3d[] jointAxes2 = {Z, X, Y, X, X};
      ScrewTestTools.createRandomChainRobot("chainB", joints, elevator, jointAxes2, random);

      Vector3d[] jointAxes3 = {Y, Y, X, X, X};
      ScrewTestTools.createRandomChainRobot("chainC", joints, elevator, jointAxes3, random);
   }

   private Set<RigidBody> getExcludedRigidBodies()
   {
      Set<RigidBody> excludedBodies = new LinkedHashSet<RigidBody>();
      for (RigidBody rigidBody : exclusions)
      {
         excludedBodies.add(rigidBody);
         RigidBody[] subTree = ScrewTools.computeSuccessors(ScrewTools.computeSubtreeJoints(rigidBody));
         excludedBodies.addAll(Arrays.asList(subTree));
      }

      return excludedBodies;
   }

   private Set<InverseDynamicsJoint> getExcludedJoints()
   {
      Set<RigidBody> excludedBodies = getExcludedRigidBodies();
      Set<InverseDynamicsJoint> excludedJoints = new LinkedHashSet<InverseDynamicsJoint>();
      for (RigidBody rigidBody : excludedBodies)
      {
         excludedJoints.addAll(rigidBody.getChildrenJoints());
      }

      return excludedJoints;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddRevoluteJoint_String_RigidBody_Vector3d_Vector3d()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      chain.setRandomPositionsAndVelocities(random);

      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());
      RigidBody[] partialBodiesArray = ScrewTools.computeSubtreeSuccessors(chain.getElevator());
      RigidBody[] bodiesArray = new RigidBody[partialBodiesArray.length + 1];
      bodiesArray[0] = chain.getElevator();
      for(int i = 0; i < partialBodiesArray.length; i++)
      {
         bodiesArray[i+1] = partialBodiesArray[i];
      }

      String jointName = "joint";
      RigidBody parentBody = bodiesArray[bodiesArray.length - 1];
      Vector3d jointOffset = RandomTools.generateRandomVector(random, 5.0);
      Vector3d jointAxis = RandomTools.generateRandomVector(random, 5.0);

      RevoluteJoint joint = ScrewTools.addRevoluteJoint(jointName, parentBody, jointOffset, jointAxis);

      assertEquals("Should be equal", jointName, joint.getName());
      assertTrue(parentBody.equals(joint.getPredecessor()));
      assertTrue(jointAxis.equals(joint.getJointAxis().getVector()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddRevoluteJoint_String_RigidBody_Transform3D_Vector3d()
   {
      String jointName = "joint";
      RigidBody parentBody = new RigidBody(null, null);
      RigidBodyTransform transformToParent = RigidBodyTransform.generateRandomTransform(random);
      Vector3d jointAxis = RandomTools.generateRandomVector(random, 5.0);

      RevoluteJoint joint = ScrewTools.addRevoluteJoint(jointName, parentBody, transformToParent, jointAxis);

      assertEquals("Should be equal", jointName, joint.getName());
      assertTrue(parentBody.equals(joint.getPredecessor()));
      assertTrue(jointAxis.equals(joint.getJointAxis().getVector()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddPrismaticJoint_String_RigidBody_Vector3d_Vector3d()
   {
      String jointName = "joint";
      RigidBody parentBody = new RigidBody(null, null);
      Vector3d jointOffset = RandomTools.generateRandomVector(random, 5.0);
      Vector3d jointAxis = RandomTools.generateRandomVector(random, 5.0);

      PrismaticJoint joint = ScrewTools.addPrismaticJoint(jointName, parentBody, jointOffset, jointAxis);

      assertEquals("Should be equal", jointName, joint.getName());
      assertTrue(parentBody.equals(joint.getPredecessor()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddPrismaticJoint_String_RigidBody_Transform3D_Vector3d()
   {
      String jointName = "joint";
      RigidBody parentBody = new RigidBody(null, null);
      RigidBodyTransform transformToParent = RigidBodyTransform.generateRandomTransform(random);
      Vector3d jointAxis = RandomTools.generateRandomVector(random, 5.0);

      PrismaticJoint joint = ScrewTools.addPrismaticJoint(jointName, parentBody, transformToParent, jointAxis);

      assertEquals("Should be equal", jointName, joint.getName());
      assertTrue(parentBody.equals(joint.getPredecessor()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddRigidBody_String_InverseDynamicsJoint_Matrix3d_double_Vector3d()
   {
      String name = "body";
      RigidBody predecessor = new RigidBody("Predecessor", theFrame);
      PlanarJoint parentJoint = new PlanarJoint(name, predecessor, theFrame);
      Matrix3d momentOfInertia = new Matrix3d();
      double mass = random.nextDouble();

      RigidBody body = ScrewTools.addRigidBody(name, parentJoint, momentOfInertia, mass, X);

      assertEquals("Should be equal", name, body.getName());
      assertTrue(parentJoint.equals(body.getParentJoint()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddRigidBody_String_InverseDynamicsJoint_Matrix3d_double_Transform3D()
   {
      String name = "body";
      RigidBody predecessor = new RigidBody("Predecessor", theFrame);
      PlanarJoint parentJoint = new PlanarJoint(name, predecessor, theFrame);
      Matrix3d momentOfInertia = new Matrix3d();
      double mass = random.nextDouble();
      RigidBodyTransform inertiaPose = new RigidBodyTransform();

      RigidBody body = ScrewTools.addRigidBody(name, parentJoint, momentOfInertia, mass, inertiaPose);

      assertEquals("Should be equal", name, body.getName());
      assertTrue(parentJoint.equals(body.getParentJoint()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateOffsetFrame_ReferenceFrame_Transform3D_String()
   {
      ReferenceFrame parentFrame = theFrame;
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      String frameName = "woof";
      ReferenceFrame frame = ScrewTools.createOffsetFrame(parentFrame, transformToParent, frameName);

      parentFrame.checkReferenceFrameMatch(frame.getRootFrame());
      assertEquals("Should be equal", frameName, frame.getName());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSuccessors()
   {  
      int numJoints = 3;
      RigidBody[] bodyArray = new RigidBody[numJoints];
      for (int i = 0; i < numJoints; i++)
      {
         InverseDynamicsJoint joint = joints.get(i);
         bodyArray[i] = joint.getSuccessor();
      }

      RigidBody[] bodies = ScrewTools.computeSuccessors(joints.get(0), joints.get(1), joints.get(2));

      assertEquals("Should be equal", bodyArray.length, bodies.length);
      for(int i = 0; i < bodies.length; i++)
      {
         assertTrue(bodies[i].equals(bodyArray[i]));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubtreeSuccessors_InverseDynamicsJoint_RigidBody()
   {
      RigidBody[] bodies = ScrewTools.computeSubtreeSuccessors(elevator);

      Set<InverseDynamicsJoint> jointsToExclude = new HashSet<InverseDynamicsJoint>();
      RigidBody[] subtreeSuccessors = ScrewTools.computeSubtreeSuccessors(jointsToExclude, elevator);
      assertEquals("Should be equal", bodies.length, subtreeSuccessors.length);
      for(int i = 0; i < bodies.length; i++)
      {
         assertTrue(bodies[i].equals(subtreeSuccessors[i]));
      }

      jointsToExclude.addAll(joints);
      subtreeSuccessors = ScrewTools.computeSubtreeSuccessors(jointsToExclude, elevator);
      assertEquals("Should be equal", 0.0, subtreeSuccessors.length, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubtreeSuccessors_RigidBody()
   {
      RigidBody[] successors = ScrewTools.computeSubtreeSuccessors(elevator);

      Set<InverseDynamicsJoint> jointsToExclude = new HashSet<InverseDynamicsJoint>();
      RigidBody[] otherSuccessors = ScrewTools.computeSubtreeSuccessors(jointsToExclude, elevator);

      assertEquals("Should be equal", successors.length, otherSuccessors.length);
      for(int i = 0; i < successors.length; i++)
      {
         assertTrue(successors[i].equals(otherSuccessors[i]));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubtreeSuccessors_InverseDynamicsJoint()
   {
      List<InverseDynamicsJoint> jointsList = new ArrayList<InverseDynamicsJoint>();
      jointsList.addAll(elevator.getChildrenJoints());

      RigidBody[] successors = ScrewTools.computeSubtreeSuccessors(jointsList.get(0), jointsList.get(1));


      Set<InverseDynamicsJoint> jointsToExclude = new HashSet<InverseDynamicsJoint>();
      RigidBody[] otherSuccessors = ScrewTools.computeSubtreeSuccessors(jointsToExclude, elevator, elevator);

      assertEquals("Should be equal", successors.length, otherSuccessors.length);
      for(int i = 0; i < successors.length; i++)
      {
         assertTrue(successors[i].equals(otherSuccessors[i]));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSupportAndSubtreeSuccessors_RigidBody()
   {
      int numberOfBodiesOnChain = 6;
      int numberOfBodies = 16;
      RigidBody[] successors = ScrewTools.computeSupportAndSubtreeSuccessors(secondLevelSubTrees.get(0));
      assertEquals(numberOfBodiesOnChain - 1, successors.length);

      successors = ScrewTools.computeSupportAndSubtreeSuccessors(elevator);
      assertEquals(numberOfBodies - 1, successors.length);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSupportAndSubtreeJoints_RigidBody()
   {
      int numberOfJointsOnChain = 5;
      int numberOfJoints = 15;
      InverseDynamicsJoint [] successors = ScrewTools.computeSupportAndSubtreeJoints(secondLevelSubTrees.get(0));
      assertEquals(numberOfJointsOnChain, successors.length);

      successors = ScrewTools.computeSupportAndSubtreeJoints(elevator);
      assertEquals(numberOfJoints, successors.length);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSupportJoints_RigidBody()
   {
      InverseDynamicsJoint[] supportJoints = ScrewTools.computeSupportJoints(elevator);
      assertTrue(elevator.isRootBody());
      assertEquals(0, supportJoints.length);

      int jointsSupportingSecondLevelSubTree = 2, numberOfChainsUsed = 2;

      supportJoints = ScrewTools.computeSupportJoints(secondLevelSubTrees.get(0), secondLevelSubTrees.get(1));

      assertEquals(jointsSupportingSecondLevelSubTree * numberOfChainsUsed, supportJoints.length);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubtreeJoints_RigidBody()
   {
      List<RigidBody> bodies = new ArrayList<RigidBody>();
      bodies.add(elevator);
      bodies.add(elevator);

      InverseDynamicsJoint[] fromBodies = ScrewTools.computeSubtreeJoints(elevator, elevator);
      InverseDynamicsJoint[] fromBodiesList = ScrewTools.computeSubtreeJoints(bodies);

      assertEquals("These should be equal", fromBodies.length, fromBodiesList.length);
      for(int i = 0; i < fromBodies.length; i++)
      {
         assertTrue(fromBodies[i].equals(fromBodiesList[i]));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSubtreeJoints_RigidBodyLIST()
   {
      ArrayList<RigidBody> rootBodies = new ArrayList<RigidBody>();
      rootBodies.add(elevator);
      InverseDynamicsJoint[] subtreeJoints = ScrewTools.computeSubtreeJoints(rootBodies);

      ArrayList<InverseDynamicsJoint> subtree = new ArrayList<InverseDynamicsJoint>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      rigidBodyStack.addAll(rootBodies);

      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            RigidBody successor = joint.getSuccessor();
            rigidBodyStack.add(successor);
            subtree.add(joint);
         }
      }

      assertEquals("These should be equal", subtreeJoints.length, subtree.size());
      for(int i = 0; i < subtreeJoints.length; i++)
      {
         assertTrue(subtreeJoints[i].equals(subtree.get(i)));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetRootBody()
   {
      RigidBody randomBody = ScrewTools.getRootBody(joints.get(joints.size() - 1).getPredecessor());
      assertTrue(randomBody.isRootBody());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateParentMap()
   {
      int numberOfBodies = ScrewTools.computeSubtreeSuccessors(elevator).length + 1;
      RigidBody[] mostBodies = ScrewTools.computeSubtreeSuccessors(elevator);
      RigidBody[] allRigidBodiesInOrder = new RigidBody[numberOfBodies];
      allRigidBodiesInOrder[0] = elevator;
      for(int i = 0; i < numberOfBodies -1; i++)
      {
         allRigidBodiesInOrder[i+1] = mostBodies[i];
      }

      int[] parentMap = new int[allRigidBodiesInOrder.length];
      parentMap = ScrewTools.createParentMap(allRigidBodiesInOrder);
      assertEquals(-1, parentMap[0]); //root
      assertEquals(0, parentMap[1]); //first subtree of bodies
      assertEquals(0, parentMap[2]);
      assertEquals(0, parentMap[3]);

      for(int i = 4; i < 16; i++) //members of chains A, B, and C
      {
         assertEquals(i - 3, parentMap[i]);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTauMatrix()
   {
      InverseDynamicsJoint[] jointsInOrder = ScrewTools.computeSubtreeJoints(elevator);

      DenseMatrix64F tauMatrix = ScrewTools.getTauMatrix(jointsInOrder);
      assertEquals(jointsInOrder.length, tauMatrix.numRows);
      assertEquals(1, tauMatrix.numCols);
      for(int i = 0; i < jointsInOrder.length; i++)
      {
         assertEquals("These should be equal", jointsInOrder[i].getDegreesOfFreedom() - 1, tauMatrix.get(i, 0), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateJointPath()
   {
      int numberOfJoints = joints.size(), numberOfBodies = numberOfJoints + 1;
      RigidBody[] allBodies = new RigidBody[numberOfBodies];
      allBodies[0] = elevator;
      for(int i = 0; i < numberOfJoints; i++)
      {
         allBodies[i+1] = joints.get(i).getSuccessor();
      }

      RigidBody start = allBodies[0] , end = allBodies[allBodies.length - 1];
      InverseDynamicsJoint[] jointPath = ScrewTools.createJointPath(start, end);
      for(int i = 0; i < jointPath.length; i++)
      {
         assertTrue(jointPath[i].getName().equalsIgnoreCase("chainCjoint" + i));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsAncestor()
   {
      int numberOfJoints = joints.size(), numberOfBodies = numberOfJoints + 1;
      RigidBody[] allBodies = new RigidBody[numberOfBodies];
      allBodies[0] = elevator;
      for(int i = 0; i < numberOfJoints; i++)
      {
         allBodies[i+1] = joints.get(i).getSuccessor();
      }

      RigidBody d0 = allBodies[0]; //elevator
      RigidBody d1 = allBodies[1]; //chainAbody0
      RigidBody d2 = allBodies[2]; //chainAbody1
      RigidBody d3 = allBodies[3]; //chainAbody2

      assertTrue(ScrewTools.isAncestor(d0, d0)); //self
      assertTrue(ScrewTools.isAncestor(d3, d0)); //ancestor
      assertFalse(ScrewTools.isAncestor(d0, d3)); //descendant 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDistanceToAncestor()
   {
      int numberOfJoints = joints.size(), numberOfBodies = numberOfJoints + 1;
      RigidBody[] allBodies = new RigidBody[numberOfBodies];
      allBodies[0] = elevator;
      for(int i = 0; i < numberOfJoints; i++)
      {
         allBodies[i+1] = joints.get(i).getSuccessor();
      }

      RigidBody d0 = allBodies[0]; //elevator
      RigidBody d1 = allBodies[1]; //chainAbody0
      RigidBody d2 = allBodies[2]; //chainAbody1
      RigidBody d3 = allBodies[3]; //chainAbody2

      assertEquals(0, ScrewTools.computeDistanceToAncestor(d0, d0)); //self
      assertEquals(3, ScrewTools.computeDistanceToAncestor(d3, d0)); //ancestor
      assertEquals(-1, ScrewTools.computeDistanceToAncestor(d0, d3)); //descendant 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackJointVelocitiesMatrix_Array()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());

      DenseMatrix64F originalVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      for(int i = 0; i < originalVelocities.getNumRows() * originalVelocities.getNumCols(); i++)
      {       //create original matrix
         originalVelocities.set(i, random.nextDouble());
      }
      ScrewTools.setVelocities(jointsArray, originalVelocities); //set velocities from matrix
      DenseMatrix64F newVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);

      ScrewTools.getJointVelocitiesMatrix(jointsArray, newVelocities);//pack new matrix
      for(int i = 0; i < jointsArray.length; i++)
      {
         assertEquals("Should be equal velocities", originalVelocities.get(i), newVelocities.get(i), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackJointVelocitiesMatrix_Iterable()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());
      ArrayList<InverseDynamicsJoint> jointsList = new ArrayList<InverseDynamicsJoint>();
      for(int i = 0; i < jointsArray.length; i++)
      {
         jointsList.add(jointsArray[i]);
      }

      DenseMatrix64F originalVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      for(int i = 0; i < originalVelocities.getNumRows() * originalVelocities.getNumCols(); i++)
      {       //create original matrix
         originalVelocities.set(i, random.nextDouble());
      }
      ScrewTools.setVelocities(jointsArray, originalVelocities); //set velocities from matrix
      DenseMatrix64F newVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);

      ScrewTools.getJointVelocitiesMatrix(jointsList, newVelocities);//pack new matrix
      for(int i = 0; i < jointsArray.length; i++)
      {
         assertEquals("Should be equal velocities", originalVelocities.get(i), newVelocities.get(i), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackDesiredJointAccelerationsMatrix()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());

      DenseMatrix64F originalAccel = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      for(int i = 0; i < originalAccel.getNumRows() * originalAccel.getNumCols(); i++)
      {       //create original matrix
         originalAccel.set(i, random.nextDouble());
      }
      ScrewTools.setDesiredAccelerations(jointsArray, originalAccel); //set velocities from matrix
      DenseMatrix64F newAccelerations = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);

      ScrewTools.getDesiredJointAccelerationsMatrix(jointsArray, newAccelerations);//pack new matrix
      for(int i = 0; i < jointsArray.length; i++)
      {
         assertEquals("Should be equal velocities", originalAccel.get(i), newAccelerations.get(i), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDegreesOfFreedom_Array()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      chain.setRandomPositionsAndVelocities(random);

      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());
      RigidBody[] partialBodiesArray = ScrewTools.computeSubtreeSuccessors(chain.getElevator());
      RigidBody[] bodiesArray = new RigidBody[partialBodiesArray.length + 1];
      bodiesArray[0] = chain.getElevator();
      for(int i = 0; i < partialBodiesArray.length; i++)
      {
         bodiesArray[i+1] = partialBodiesArray[i];
      }

      int result = ScrewTools.computeDegreesOfFreedom(jointsArray);
      assertEquals(11, result);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeDegreesOfFreedom_Iterable()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      chain.setRandomPositionsAndVelocities(random);

      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());
      ArrayList<InverseDynamicsJoint> jointsList = new ArrayList<InverseDynamicsJoint>(jointsArray.length);

      RigidBody[] partialBodiesArray = ScrewTools.computeSubtreeSuccessors(chain.getElevator());
      RigidBody[] bodiesArray = new RigidBody[partialBodiesArray.length + 1];
      bodiesArray[0] = chain.getElevator();
      for(int i = 0; i < partialBodiesArray.length; i++)
      {
         bodiesArray[i+1] = partialBodiesArray[i];
      }

      ScrewTools.computeDegreesOfFreedom(jointsList);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateGravitationalSpatialAcceleration()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      chain.setRandomPositionsAndVelocities(random);

      double gravity = RandomTools.generateRandomDouble(random, 100.0);
      SpatialAccelerationVector result = ScrewTools.
            createGravitationalSpatialAcceleration(chain.getElevator(), gravity);

      Vector3d angularPart = result.getAngularPart();
      Vector3d zeroes = new Vector3d(0.0, 0.0, 0.0);

      assertTrue(angularPart.epsilonEquals(zeroes, epsilon));

      Vector3d linearPart = result.getLinearPart();
      assertEquals(zeroes.getX(), linearPart.getX(), epsilon);
      assertEquals(zeroes.getY(), linearPart.getY(), epsilon);
      assertEquals(gravity, linearPart.getZ(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetDesiredAccelerations()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());

      DenseMatrix64F jointAccelerations = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      for(int i = 0; i < jointAccelerations.getNumRows() * jointAccelerations.getNumCols(); i++)
      {
         jointAccelerations.set(i, random.nextDouble());
      }

      ScrewTools.setDesiredAccelerations(jointsArray, jointAccelerations);

      DenseMatrix64F sixDoFAccel = new DenseMatrix64F(6, 1);
      jointsArray[0].getDesiredAccelerationMatrix(sixDoFAccel, 0);
      for(int i = 0; i < 6; i++)
      {
         assertEquals("Should be equal accelerations", jointAccelerations.get(i), sixDoFAccel.get(i), epsilon);
      }

      OneDoFJoint joint;

      for(int i = 6; i < jointAccelerations.getNumRows() * jointAccelerations.getNumCols(); i++)
      {
         joint = (OneDoFJoint)jointsArray[i - 5]; //1 - 6
         assertEquals("Should be equal accelerations", jointAccelerations.get(i), joint.getQddDesired(), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetVelocities()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());

      DenseMatrix64F jointVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      for(int i = 0; i < jointVelocities.getNumRows() * jointVelocities.getNumCols(); i++)
      {
         jointVelocities.set(i, random.nextDouble());
      }

      ScrewTools.setVelocities(jointsArray, jointVelocities);

      DenseMatrix64F sixDoFVeloc = new DenseMatrix64F(6, 1);
      jointsArray[0].getVelocityMatrix(sixDoFVeloc, 0);
      for(int i = 0; i < 6; i++)
      {
         assertEquals("Should be equal velocitiess", jointVelocities.get(i), sixDoFVeloc.get(i), epsilon);
      }

      OneDoFJoint joint;

      for(int i = 6; i < jointVelocities.getNumRows() * jointVelocities.getNumCols(); i++)
      {
         joint = (OneDoFJoint)jointsArray[i - 5]; //1 - 6
         assertEquals("Should be equal velocities", jointVelocities.get(i), joint.getQd(), epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeIndicesForJoint()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArr = ScrewTools.computeSubtreeJoints(chain.getElevator());
      InverseDynamicsJoint rootJoint = jointsArr[0];
      InverseDynamicsJoint testJoint4 = jointsArr[5];

      
      TIntArrayList indices = new TIntArrayList();
      ScrewTools.computeIndicesForJoint(jointsArr, indices, testJoint4, rootJoint);
      assertEquals(7, indices.size());

      for(int i = 0; i < rootJoint.getDegreesOfFreedom(); i++)
      {
         assertEquals(i, indices.get(i));
      }
      assertEquals(10, indices.get(indices.size() - 1));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testExtractRevoluteJoints()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArr = ScrewTools.computeSubtreeJoints(chain.getElevator());

      RevoluteJoint[] revoluteJoints = ScrewTools.extractRevoluteJoints(jointsArr);
      assertEquals(jointsArr.length - 1, revoluteJoints.length);
      for(int i = 0; i < revoluteJoints.length; i++)
      {
         assertEquals("testjoint" + i, revoluteJoints[i].getName());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeNumberOfJointsOfType()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArr = ScrewTools.computeSubtreeJoints(chain.getElevator());

      int number6DoF = ScrewTools.computeNumberOfJointsOfType(SixDoFJoint.class, jointsArr);
      int numberRev = ScrewTools.computeNumberOfJointsOfType(RevoluteJoint.class, jointsArr);

      assertEquals(1, number6DoF);
      assertEquals(jointsArr.length - 1, numberRev);      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFilterJoints()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArr = ScrewTools.computeSubtreeJoints(chain.getElevator());

      RevoluteJoint[] justRevolutes = ScrewTools.filterJoints(jointsArr, RevoluteJoint.class);
      assertEquals(jointsArr.length - 1, justRevolutes.length);

      SixDoFJoint[] justSix = ScrewTools.filterJoints(jointsArr, SixDoFJoint.class);
      assertEquals(1, justSix.length);
      assertTrue(justSix[0] instanceof SixDoFJoint);

      Boolean clean = false;
      for(InverseDynamicsJoint joint: justRevolutes)
      {
         if(joint instanceof RevoluteJoint)
         {
            clean = true;
         }
         else
         {
            clean = false;
         }
         assertTrue(clean);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFilterJoints_dest()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArr = ScrewTools.computeSubtreeJoints(chain.getElevator());

      RevoluteJoint[] justRevolutes = new RevoluteJoint[jointsArr.length - 1];
      ScrewTools.filterJoints(jointsArr, justRevolutes, RevoluteJoint.class);
      assertEquals(jointsArr.length - 1, justRevolutes.length);

      SixDoFJoint[] justSix = new SixDoFJoint[1];
      ScrewTools.filterJoints(jointsArr, justSix, SixDoFJoint.class);
      assertEquals(1, justSix.length);
      assertTrue(justSix[0] instanceof SixDoFJoint);

      Boolean clean = false;
      for(InverseDynamicsJoint joint: justRevolutes)
      {
         if(joint instanceof RevoluteJoint)
         {
            clean = true;
         }
         else
         {
            clean = false;
         }
         assertTrue(clean);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFindJointsWithNames()
   {
      int numberOfJoints = joints.size();
      InverseDynamicsJoint[] allJoints = new InverseDynamicsJoint[joints.size()];
      for(int i = 0; i < numberOfJoints; i++)
      {
         allJoints[i] = joints.get(i);
      }

      InverseDynamicsJoint[] matches;
      try
      {
         matches = ScrewTools.findJointsWithNames(allJoints, "woof");
         fail("Should throw RuntimeException");
      }
      catch(RuntimeException rte)
      {
         //good  
      }
      matches = ScrewTools.findJointsWithNames(allJoints, "chainAjoint0");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFindRigidBodiesWithNames_RigidBody_String()
   {
      int numberOfJoints = joints.size();
      RigidBody[] allBodies = new RigidBody[joints.size() + 1];
      allBodies[0] = elevator;
      for(int i = 0; i < numberOfJoints; i++)
      {
         allBodies[i+1] = joints.get(i).getSuccessor();
      }

      RigidBody[] matches;
      try
      {
         matches = ScrewTools.findRigidBodiesWithNames(allBodies, "elevatorOOPS");
         fail("Should throw RuntimeException");
      }
      catch(RuntimeException rte)
      {
         //good  
      }
      matches = ScrewTools.findRigidBodiesWithNames(allBodies, "elevator", "chainAbody0", 
            "chainAbody1", "chainAbody2", "chainAbody4", "chainBbody0", "chainBbody1", "chainBbody2", 
            "chainBbody3", "chainBbody4", "chainCbody0", "chainCbody1", "chainCbody2", "chainCbody3", "chainCbody4");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAddExternalWrenches()
   {
      Vector3d[] jointAxes = {X, Y, Z, Y, X};
      RandomFloatingChain chain = new RandomFloatingChain(random, jointAxes);
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(chain.getElevator());

      LinkedHashMap<RigidBody, Wrench> external = new LinkedHashMap<RigidBody, Wrench>();
      LinkedHashMap<RigidBody, Wrench> toAdd = new LinkedHashMap<RigidBody, Wrench>();

      RigidBody rigidBody1 = jointsArray[2].getSuccessor(); //testBody1
      RigidBody rigidBody2 = jointsArray[0].getSuccessor(); //rootBody
      RigidBody rigidBody3 = jointsArray[4].getSuccessor(); //testBody3

      ReferenceFrame frame1 = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frame1", theFrame, RigidBodyTransform
            .generateRandomTransform(random));

      Wrench externalWrench1 = new Wrench(rigidBody1.getBodyFixedFrame(), theFrame, RandomTools.generateRandomDoubleArray(random, 6, 100.0));
      Wrench externalWrench2 = new Wrench(rigidBody3.getBodyFixedFrame(), theFrame, RandomTools.generateRandomDoubleArray(random, 6, 100.0));
      Wrench addedWrench1 = new Wrench(rigidBody2.getBodyFixedFrame(), frame1, RandomTools.generateRandomDoubleArray(random, 6, 100.0));
      Wrench addedWrench2 = new Wrench(rigidBody3.getBodyFixedFrame(), theFrame, RandomTools.generateRandomDoubleArray(random, 6, 100.0));

      external.put(rigidBody1, new Wrench(externalWrench1));
      external.put(rigidBody3, new Wrench(externalWrench2));
      toAdd.put(rigidBody2, new Wrench(addedWrench1));
      toAdd.put(rigidBody3, new Wrench(addedWrench2));

      assertEquals(2, external.keySet().size());
      assertTrue(external.keySet().contains(rigidBody1));
      assertTrue(external.keySet().contains(rigidBody3));
      
      ScrewTools.addExternalWrenches(external, toAdd);

      assertEquals(3, external.keySet().size());
      assertTrue(external.keySet().contains(rigidBody1));
      assertTrue(external.keySet().contains(rigidBody2));
      assertTrue(external.keySet().contains(rigidBody3));

      Wrench expectedWrench = new Wrench();
      expectedWrench.set(externalWrench2);
      expectedWrench.add(addedWrench2);
      
      assertTrue(expectedWrench.getAngularPartAsFrameVectorCopy().epsilonEquals(external.get(rigidBody3).getAngularPartAsFrameVectorCopy(), epsilon));
      assertTrue(expectedWrench.getLinearPartAsFrameVectorCopy().epsilonEquals(external.get(rigidBody3).getLinearPartAsFrameVectorCopy(), epsilon));
   }
}
