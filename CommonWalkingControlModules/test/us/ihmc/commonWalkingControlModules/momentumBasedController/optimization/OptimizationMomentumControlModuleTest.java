package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculator;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;
import us.ihmc.utilities.test.JUnitTools;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;
import java.util.*;

import static org.junit.Assert.assertEquals;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertRootJointWrenchZero;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesInFrictionCones;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesSumUpToMomentumDot;

/**
 * @author twan
 *         Date: 5/4/13
 */
public class OptimizationMomentumControlModuleTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private final double controlDT = 1e-5; // 5e-3;
   private final double gravityZ = 9.81;

   @Test
   public void testMomentumAndJointSpaceConstraints()
   {
      Random random = new Random(1223525L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X // , Y, Z, Z, X, Y, X, Y
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);
      randomFloatingChain.getElevator().updateFramesRecursively();

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings();
      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(), controlDT, centerOfMassFrame,
            momentumOptimizationSettings);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      double coefficientOfFriction = 1.0;
      addContactState(coefficientOfFriction, randomFloatingChain.getLeafBody(), contactStates);

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      double totalMass = TotalMassCalculator.computeSubTreeMass(randomFloatingChain.getElevator());
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     momentumOptimizationSettings.getRhoMinScalar());
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      InverseDynamicsJoint[] revoluteJointsArray = revoluteJoints.toArray(new InverseDynamicsJoint[revoluteJoints.size()]);
      DenseMatrix64F desiredJointAccelerations = setRandomJointAccelerations(random, momentumControlModule, revoluteJoints);

      momentumControlModule.compute(contactStates, null, null);

      SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();
      DenseMatrix64F jointAccelerationsBack = new DenseMatrix64F(desiredJointAccelerations.getNumRows(), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(revoluteJointsArray, jointAccelerationsBack);

      assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches().values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame, 1e-3);
      assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-3);
      EjmlUnitTests.assertEquals(desiredJointAccelerations, jointAccelerationsBack, 1e-9);
      assertRootJointWrenchZero(momentumControlModule.getExternalWrenches(), rootJoint, gravityZ, 1e-2);
   }

   @Test
   public void testMomentumAndTaskSpaceConstraints()
   {
      Random random = new Random(1223525L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, Z, Y, Y, X, Z, Y, X, X, Y, Z
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings();
      momentumOptimizationSettings.setRhoMin(-10000.0);
      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(), controlDT, centerOfMassFrame,
            momentumOptimizationSettings);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);

      RigidBody base = elevator; // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      jacobian.compute();
      TaskspaceConstraintData taskSpaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector endEffectorSpatialAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                                    endEffector.getBodyFixedFrame());
      endEffectorSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));
      endEffectorSpatialAcceleration.setLinearPart(RandomTools.generateRandomVector(random));
      taskSpaceConstraintData.set(endEffectorSpatialAcceleration);
      momentumControlModule.setDesiredSpatialAcceleration(jacobian, taskSpaceConstraintData); // , 10.0);

      momentumControlModule.compute(contactStates, null, null);
      SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector endEffectorAccelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(endEffectorAccelerationBack, base, endEffector);
      JUnitTools.assertSpatialMotionVectorEquals(endEffectorSpatialAcceleration, endEffectorAccelerationBack, 1e-3);

      assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches().values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame, 1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(momentumControlModule.getExternalWrenches(), rootJoint, gravityZ, 1e-2);
   }

   @Test
   public void testMomentumAndPointAccelerationConstraints()
   {
      Random random = new Random(1223525L);
      Vector3d[] jointAxes = new Vector3d[]
            {
                  X, Y, Z, Y, Y, X, Z, Y, X, X, Y, Z
            };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings();
      momentumOptimizationSettings.setRhoMin(-10000.0);
      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(), controlDT, centerOfMassFrame,
            momentumOptimizationSettings);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
            0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);

      RigidBody base = elevator; // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      jacobian.compute();

      FramePoint bodyFixedPoint = new FramePoint(endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random));
      FrameVector desiredPointAcceleration = new FrameVector(base.getBodyFixedFrame(), RandomTools.generateRandomVector(random));
      momentumControlModule.setDesiredPointAcceleration(jacobian, bodyFixedPoint, desiredPointAcceleration);


      momentumControlModule.compute(contactStates, null, null);
      SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      twistCalculator.compute();
      Twist twist = new Twist();
      twistCalculator.packRelativeTwist(twist, base, endEffector);

      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector acceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(acceleration, base, endEffector);
      acceleration.changeFrame(jacobian.getBaseFrame(), twist, twist);

      FrameVector desiredPointAccelerationBack = new FrameVector(jacobian.getBaseFrame());
      bodyFixedPoint.changeFrame(jacobian.getBaseFrame());
      twist.changeFrame(jacobian.getBaseFrame());
      acceleration.packAccelerationOfPointFixedInBodyFrame(twist, bodyFixedPoint, desiredPointAccelerationBack);

      assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches().values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame, 1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(momentumControlModule.getExternalWrenches(), rootJoint, gravityZ, 1e-2);
      JUnitTools.assertFrameVectorEquals(desiredPointAccelerationBack, desiredPointAcceleration, 1e-9);
   }

   @Test
   public void testSingleRigidBody()
   {
      Random random = new Random(125152L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(),
            new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);


      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), elevator);
      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings();
      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, new ArrayList<RevoluteJoint>(), controlDT, centerOfMassFrame,
            momentumOptimizationSettings);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      double coefficientOfFriction = 1.0;
      addContactState(coefficientOfFriction, rootBody, contactStates);
      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(elevator));
      double rhoMin = momentumOptimizationSettings.getRhoMinScalar();


      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();
      momentumControlModule.initialize();

      for (int i = 0; i < 100; i++)
      {
         momentumControlModule.reset();

         ScrewTestTools.setRandomVelocity(rootJoint, random);
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);

         elevator.updateFramesRecursively();
         centerOfMassFrame.update();

         SpatialForceVector desiredRateOfChangeOfMomentum = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random, rhoMin);
         MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
         momentumRateOfChangeData.set(desiredRateOfChangeOfMomentum);
         momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);

         momentumControlModule.compute(contactStates, null, null);


         SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();
         assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches().values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame, 1e-3);
         JUnitTools.assertSpatialForceVectorEquals(desiredRateOfChangeOfMomentum, momentumRateOfChangeOut, 1e-1);
         assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);
         assertRootJointWrenchZero(momentumControlModule.getExternalWrenches(), rootJoint, gravityZ, 1e-2);
      }
   }

   @Test
   public void testPrimaryAndSecondaryConstraints()
   {
      Random random = new Random(1223525L);
      Vector3d[] jointAxes = new Vector3d[]
            {
                  X, Y, Z, Y, Y, X, Z, Y, X, X, Y, Z
            };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings();
      momentumOptimizationSettings.setRhoMin(-10000.0);
      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(), controlDT, centerOfMassFrame,
            momentumOptimizationSettings);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
            0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);

      RigidBody base = elevator; // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      jacobian.compute();
      TaskspaceConstraintData taskSpaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector endEffectorSpatialAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
            endEffector.getBodyFixedFrame());
      endEffectorSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));
      endEffectorSpatialAcceleration.setLinearPart(RandomTools.generateRandomVector(random));
      taskSpaceConstraintData.set(endEffectorSpatialAcceleration);
      momentumControlModule.setDesiredSpatialAcceleration(jacobian, taskSpaceConstraintData); // , 10.0);

      Map<RevoluteJoint, Double> desiredJointAccelerations = new HashMap<RevoluteJoint, Double>();
      desiredJointAccelerations.put(randomFloatingChain.getRevoluteJoints().get(3), random.nextDouble());
      desiredJointAccelerations.put(randomFloatingChain.getRevoluteJoints().get(8), random.nextDouble());

      for (RevoluteJoint revoluteJoint : desiredJointAccelerations.keySet())
      {
         DenseMatrix64F desiredJointAcceleration = new DenseMatrix64F(1, 1);
         desiredJointAcceleration.set(0, 0, desiredJointAccelerations.get(revoluteJoint));
         double weight = 1.0;
         momentumControlModule.setDesiredJointAcceleration(revoluteJoint, desiredJointAcceleration, weight);
      }

      momentumControlModule.compute(contactStates, null, null);
      SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector endEffectorAccelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(endEffectorAccelerationBack, base, endEffector);
      JUnitTools.assertSpatialMotionVectorEquals(endEffectorSpatialAcceleration, endEffectorAccelerationBack, 1e-3);

      for (RevoluteJoint revoluteJoint : desiredJointAccelerations.keySet())
      {
         assertEquals(revoluteJoint.getQddDesired(), desiredJointAccelerations.get(revoluteJoint), 1e-9);
      }
      assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches().values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame, 1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(momentumControlModule.getExternalWrenches(), rootJoint, gravityZ, 1e-2);
   }

   private OptimizationMomentumControlModule createAndInitializeMomentumControlModule(SixDoFJoint rootJoint, List<RevoluteJoint> revoluteJoints, double dt,
                                                                                      ReferenceFrame centerOfMassFrame,
                                                                                      MomentumOptimizationSettings momentumOptimizationSettings)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      InverseDynamicsJoint[] jointsToOptimizeFor = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, registry,
            jointsToOptimizeFor, momentumOptimizationSettings,
            gravityZ, twistCalculator);
      momentumControlModule.initialize();

      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(revoluteJoints, dt);
      twistCalculator.compute();

      rootJoint.getPredecessor().updateFramesRecursively();

      return momentumControlModule;
   }

   private static SpatialAccelerationCalculator createSpatialAccelerationCalculator(TwistCalculator twistCalculator, RigidBody elevator)
   {
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);

      return spatialAccelerationCalculator;
   }

   private static DenseMatrix64F setRandomJointAccelerations(Random random, OptimizationMomentumControlModule momentumControlModule,
           List<? extends InverseDynamicsJoint> joints)
   {
      DenseMatrix64F desiredJointAccelerations = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(joints), 1);
      int index = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointAcceleration, random);
         momentumControlModule.setDesiredJointAcceleration(joint, jointAcceleration);
         CommonOps.insert(jointAcceleration, desiredJointAccelerations, index, 0);
         index += joint.getDegreesOfFreedom();
      }

      return desiredJointAccelerations;
   }

   private void addContactState(double coefficientOfFriction, RigidBody endEffector, LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates)
   {
      ReferenceFrame soleFrame = endEffector.getBodyFixedFrame();
      ContactablePlaneBody contactablePlaneBody = new RectangularContactableBody(endEffector, soleFrame, 1.0, -2.0, 3.0, -4.0);
      YoPlaneContactState contactState = new YoPlaneContactState("testContactState", endEffector.getParentJoint().getFrameAfterJoint(), soleFrame,
                                            new YoVariableRegistry("bla"));
      contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);
      contactStates.put(contactablePlaneBody, contactState);
   }

   public static SpatialForceVector generateRandomFeasibleMomentumRateOfChange(ReferenceFrame centerOfMassFrame,
           LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates, double totalMass, double gravityZ, Random random, double rhoMin)

   {
      ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator = new ContactPointWrenchMatrixCalculator(centerOfMassFrame,
                                                                                 MomentumOptimizerNative.nSupportVectors, MomentumOptimizerNative.rhoSize);
      contactPointWrenchMatrixCalculator.computeMatrix(contactStates.values());
      DenseMatrix64F rho = new DenseMatrix64F(MomentumOptimizerNative.rhoSize, 1);
      RandomMatrices.setRandom(rho, random);
      CommonOps.add(rho, rhoMin);
      Map<RigidBody,Wrench> rigidBodyWrenchMap = contactPointWrenchMatrixCalculator.computeWrenches(convertContactStates(contactStates), rho);
      Wrench ret = TotalWrenchCalculator.computeTotalWrench(rigidBodyWrenchMap.values(), centerOfMassFrame);
      ret.setLinearPartZ(ret.getLinearPartZ() - totalMass * gravityZ);

      return ret;
   }

   private static LinkedHashMap<RigidBody, PlaneContactState> convertContactStates(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates)
   {
      LinkedHashMap<RigidBody, PlaneContactState> ret = new LinkedHashMap<RigidBody, PlaneContactState>();
      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         ret.put(contactablePlaneBody.getRigidBody(), contactStates.get(contactablePlaneBody));
      }

      return ret;
   }

   private static MomentumOptimizationSettings createStandardOptimizationSettings()
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(new YoVariableRegistry("test1"));
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 1.0, 1.0);
//      momentumOptimizationSettings.setDampedLeastSquaresFactor(1e-11);
//      momentumOptimizationSettings.setGroundReactionForceRegularization(1e-9);
      momentumOptimizationSettings.setRhoMin(0.0);

      return momentumOptimizationSettings;
   }
}
