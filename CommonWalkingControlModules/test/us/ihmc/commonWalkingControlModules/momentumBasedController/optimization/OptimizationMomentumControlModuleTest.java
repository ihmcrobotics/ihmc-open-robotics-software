package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertRootJointWrenchZero;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesInFrictionCones;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesSumUpToMomentumDot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TotalWrenchCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.utilities.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

/**
 * @author twan
 *         Date: 5/4/13
 */
public class OptimizationMomentumControlModuleTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private final double controlDT = 1e-5;    // 5e-3;
   private final double gravityZ = 9.81;

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testMomentumAndJointSpaceConstraints() throws NoConvergenceException
   {
      Random random = new Random(1223521L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, Z, Z, X, Y, X, Y
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);
      randomFloatingChain.getElevator().updateFramesRecursively();

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      double coefficientOfFriction = 1.0;
      addContactState(coefficientOfFriction, randomFloatingChain.getLeafBody(), contactStates);

      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(),
                                                                   controlDT, centerOfMassFrame, momentumOptimizationSettings, contactStates.values());

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      double totalMass = TotalMassCalculator.computeSubTreeMass(randomFloatingChain.getElevator());
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     momentumOptimizationSettings.getRhoMinScalar());
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      InverseDynamicsJoint[] revoluteJointsArray = revoluteJoints.toArray(new InverseDynamicsJoint[revoluteJoints.size()]);
      DenseMatrix64F desiredJointAccelerations = setRandomJointAccelerations(random, momentumControlModule, revoluteJoints);

      MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute(contactStates, null);

      SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      
      DenseMatrix64F jointAccelerationsBack = new DenseMatrix64F(desiredJointAccelerations.getNumRows(), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(revoluteJointsArray, jointAccelerationsBack);

      assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame,
                                       1e-3);
      assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-3);
      EjmlUnitTests.assertEquals(desiredJointAccelerations, jointAccelerationsBack, 1e-3);
      assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-3);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
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

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      momentumOptimizationSettings.setRhoMin(-10000.0);
      
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(),
                                                                   controlDT, centerOfMassFrame, momentumOptimizationSettings, contactStates.values());

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

      RigidBody base = elevator;    // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      jacobian.compute();
      TaskspaceConstraintData taskSpaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector endEffectorSpatialAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                                    endEffector.getBodyFixedFrame());
      endEffectorSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));
      endEffectorSpatialAcceleration.setLinearPart(RandomTools.generateRandomVector(random));
      taskSpaceConstraintData.set(endEffectorSpatialAcceleration);
      
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskSpaceConstraintData); // , 10.0);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);    
      
      MomentumModuleSolution momentumModuleSolution; 
      try
      {
         momentumModuleSolution = momentumControlModule.compute(contactStates, null);
      } catch (MomentumControlModuleException momentumControlModuleException)
      {
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }
      SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector endEffectorAccelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(endEffectorAccelerationBack, base, endEffector);
      JUnitTools.assertSpatialMotionVectorEquals(endEffectorSpatialAcceleration, endEffectorAccelerationBack, 1e-3);

      assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame,
                                       1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testMomentumAndPointAccelerationConstraints() throws NoConvergenceException
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

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      momentumOptimizationSettings.setRhoMin(-10000.0);
      
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(),
                                                                   controlDT, centerOfMassFrame, momentumOptimizationSettings, contactStates.values());

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

      RigidBody base = elevator;    // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      jacobian.compute();

      FramePoint bodyFixedPoint = new FramePoint(endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random));
      FrameVector desiredPointAcceleration = new FrameVector(base.getBodyFixedFrame(), RandomTools.generateRandomVector(random));
      
      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(jacobian, bodyFixedPoint, desiredPointAcceleration);
      momentumControlModule.setDesiredPointAcceleration(desiredPointAccelerationCommand);

      MomentumModuleSolution momentumModuleSolution;
      try
      {
         momentumModuleSolution = momentumControlModule.compute(contactStates, null);
      } catch (MomentumControlModuleException momentumControlModuleException)
      {
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }
      SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();

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

      assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame,
                                       1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-3);
      assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-3);
      JUnitTools.assertFrameVectorEquals(desiredPointAccelerationBack, desiredPointAcceleration, 1e-3);
   }

	@EstimatedDuration(duration = 0.1)
	@Test(timeout = 30000)
   public void testSingleRigidBody() throws NoConvergenceException
   {
      Random random = new Random(125152L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(),
                                        new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);


      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), elevator);
      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      double coefficientOfFriction = 1.0;
      addContactState(coefficientOfFriction, rootBody, contactStates);

      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, new ArrayList<RevoluteJoint>(), controlDT,
                                                                   centerOfMassFrame, momentumOptimizationSettings, contactStates.values());
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

         SpatialForceVector desiredRateOfChangeOfMomentum = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ,
                                                               random, rhoMin);
         MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
         momentumRateOfChangeData.set(desiredRateOfChangeOfMomentum);
         
         DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
         momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

         MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute(contactStates, null);


         SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
         
         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass,
                                          centerOfMassFrame, 1e-2);
         JUnitTools.assertSpatialForceVectorEquals(desiredRateOfChangeOfMomentum, momentumRateOfChangeOut, 1e-1);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
         assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
      }
   }

	@EstimatedDuration(duration = 0.1)
	@Test(timeout = 30000)
   public void testPrimaryAndSecondaryConstraints() throws NoConvergenceException
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

      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      momentumOptimizationSettings.setRhoMin(-10000.0);
      
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);

      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, randomFloatingChain.getRevoluteJoints(),
                                                                   controlDT, centerOfMassFrame, momentumOptimizationSettings, contactStates.values());

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      RigidBody elevator = randomFloatingChain.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     0.0);
      momentumRateOfChangeData.set(momentumRateOfChangeIn);
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

      RigidBody base = elevator;    // rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      jacobian.compute();
      TaskspaceConstraintData taskSpaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector endEffectorSpatialAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                                    endEffector.getBodyFixedFrame());
      endEffectorSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));
      endEffectorSpatialAcceleration.setLinearPart(RandomTools.generateRandomVector(random));
      taskSpaceConstraintData.set(endEffectorSpatialAcceleration);
      
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskSpaceConstraintData);    // , 10.0);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);

      Map<RevoluteJoint, Double> desiredJointAccelerations = new HashMap<RevoluteJoint, Double>();
      desiredJointAccelerations.put(randomFloatingChain.getRevoluteJoints().get(3), random.nextDouble());
      desiredJointAccelerations.put(randomFloatingChain.getRevoluteJoints().get(8), random.nextDouble());

      for (RevoluteJoint revoluteJoint : desiredJointAccelerations.keySet())
      {
         DenseMatrix64F desiredJointAcceleration = new DenseMatrix64F(1, 1);
         desiredJointAcceleration.set(0, 0, desiredJointAccelerations.get(revoluteJoint));
         double weight = 1.0;
         
         DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(revoluteJoint, desiredJointAcceleration, weight);
         momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand);
      }

      MomentumModuleSolution momentumModuleSolution;
      try
      {
         momentumModuleSolution = momentumControlModule.compute(contactStates, null);
      } catch (MomentumControlModuleException momentumControlModuleException)
      {
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }
      SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector endEffectorAccelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(endEffectorAccelerationBack, base, endEffector);
      JUnitTools.assertSpatialMotionVectorEquals(endEffectorSpatialAcceleration, endEffectorAccelerationBack, 1e-3);

      for (RevoluteJoint revoluteJoint : desiredJointAccelerations.keySet())
      {
         assertEquals(revoluteJoint.getQddDesired(), desiredJointAccelerations.get(revoluteJoint), 1e-5);
      }

      assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame,
                                       1e-3);
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testNullspaceMultipliers() throws NoConvergenceException
   {
      Random random = new Random(2534L);
      Vector3d[] jointAxes = new Vector3d[]
            {
                  X, Y, Z, Y, Y, X
            };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();

      int kneeIndex = 3;
      RevoluteJoint kneeJoint = revoluteJoints.get(kneeIndex);
      kneeJoint.setQ(0.0);
      randomFloatingChain.getElevator().updateFramesRecursively();

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), elevator);
      centerOfMassFrame.update();


      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(rootJoint);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.0);
      momentumOptimizationSettings.setRhoMin(-10000.0);

      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      double coefficientOfFriction = 1000.0;
      addContactState(coefficientOfFriction, endEffector, contactStates);


      OptimizationMomentumControlModule momentumControlModule = createAndInitializeMomentumControlModule(rootJoint, revoluteJoints, controlDT,
            centerOfMassFrame, momentumOptimizationSettings, contactStates.values());

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      SpatialForceVector desiredMomentumRate = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
            0.0);
      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      momentumRateOfChangeData.set(desiredMomentumRate);
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);

      RigidBody base = rootJoint.getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());

      jacobian.compute();


      SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
            endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
            RandomTools.generateRandomVector(random));

      int nullity = 1;
      DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(nullity, 1);
      CommonOps.fill(nullspaceMultipliers, 0.0);
      RandomMatrices.setRandom(nullspaceMultipliers, random);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(taskSpaceAcceleration, nullspaceMultipliers);
      
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);

      MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute(contactStates, null);


      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(jacobian.getNumberOfColumns(), true);
      nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), nullity);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      InverseDynamicsJoint[] jointList = ScrewTools.computeSubtreeJoints(base);
      DenseMatrix64F vdotTaskSpace = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointList), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(jointList, vdotTaskSpace);
      DenseMatrix64F nullspaceMultiplierCheck = new DenseMatrix64F(nullspace.getNumCols(), vdotTaskSpace.getNumCols());
      CommonOps.multTransA(nullspace, vdotTaskSpace, nullspaceMultiplierCheck);
      CommonOps.subtractEquals(nullspaceMultiplierCheck, nullspaceMultipliers);

      assertTrue(MatrixFeatures.isConstantVal(nullspaceMultiplierCheck, 0.0, 1e-6));

      SpatialForceVector momentumRateOfChangeOut = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
      
      assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame,
            1e-3);
      JUnitTools.assertSpatialForceVectorEquals(desiredMomentumRate, momentumRateOfChangeOut, 1e-1);
      assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
   }

   private OptimizationMomentumControlModule createAndInitializeMomentumControlModule(SixDoFJoint rootJoint, List<RevoluteJoint> revoluteJoints, double dt,
         ReferenceFrame centerOfMassFrame, MomentumOptimizationSettings momentumOptimizationSettings,
         Collection<? extends PlaneContactState> planeContactStates)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());

      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, gravityZ,
            momentumOptimizationSettings, twistCalculator, null, planeContactStates, null, registry);
      
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
         
         DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(joint, jointAcceleration);
         momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand);
         CommonOps.insert(jointAcceleration, desiredJointAccelerations, index, 0);
         index += joint.getDegreesOfFreedom();
      }

      return desiredJointAccelerations;
   }

   private void addContactState(double coefficientOfFriction, RigidBody endEffector, LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates)
   {
      ReferenceFrame soleFrame = endEffector.getBodyFixedFrame();
      ContactablePlaneBody contactablePlaneBody = new RectangularContactableBody(endEffector, soleFrame, 1.0, -2.0, 3.0, -4.0);
      YoPlaneContactState contactState = new YoPlaneContactState("testContactState", endEffector, soleFrame, contactablePlaneBody.getContactPoints2d(),
                                            coefficientOfFriction, new YoVariableRegistry("bla"));
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
      Map<RigidBody, Wrench> rigidBodyWrenchMap = contactPointWrenchMatrixCalculator.computeWrenches(convertContactStates(contactStates), rho);
      TotalWrenchCalculator totalWrenchCalculator = new TotalWrenchCalculator();
      Wrench ret = new Wrench();
      totalWrenchCalculator.computeTotalWrench(ret, rigidBodyWrenchMap.values(), centerOfMassFrame);
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

   private static MomentumOptimizationSettings createStandardOptimizationSettings(SixDoFJoint rootJoint)
   {
      InverseDynamicsJoint[] jointsToOptimizeFor = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, new YoVariableRegistry("test1"));
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 1.0, 1.0);

    momentumOptimizationSettings.setDampedLeastSquaresFactor(0.0);
//    momentumOptimizationSettings.setGroundReactionForceRegularization(1e-9);
      momentumOptimizationSettings.setRhoMin(0.0);

      return momentumOptimizationSettings;
   }
}
