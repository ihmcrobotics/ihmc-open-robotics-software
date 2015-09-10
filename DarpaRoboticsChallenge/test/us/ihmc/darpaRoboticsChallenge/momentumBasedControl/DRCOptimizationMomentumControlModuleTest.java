package us.ihmc.darpaRoboticsChallenge.momentumBasedControl;

import static org.junit.Assert.assertTrue;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertRootJointWrenchZero;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesInFrictionCones;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesSumUpToMomentumDot;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

import javax.xml.bind.JAXBException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.utilities.exceptions.NoConvergenceException;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * @author twan
 *         Date: 5/7/13
 */
public abstract class DRCOptimizationMomentumControlModuleTest implements MultiRobotTestInterface
{

	@DeployableTestMethod(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testAllJointAccelerationsZero() throws IOException, JAXBException
   {
      Random random = new Random(1252515L);

      DRCRobotModel robotModel = getRobotModel();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);
      
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      DRCRobotInitialSetup<SDFHumanoidRobot> intialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      initializeRobot(fullRobotModel, sdfRobot, referenceFrames, intialSetup, jointMap);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      ScrewTestTools.setRandomVelocities(allJoints, random);
      OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);

      SideDependentList<ContactablePlaneBody> feet = createFeet(fullRobotModel, referenceFrames,robotModel.getWalkingControllerParameters());
      YoVariableRegistry registry = new YoVariableRegistry("test");
      double coefficientOfFriction = 1.0;

      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = createContactStates(feet, registry, coefficientOfFriction);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();
      
      OneDoFJoint lidarJoint = null;
      if(lidarParameters != null)
      {
    	  lidarJoint = fullRobotModel.getOneDoFJointByName(lidarParameters.getLidarSpindleJointName());
      }
      
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = 1e-4;
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 1e-3, 1e-9, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, gravityZ,
            optimizationSettings, twistCalculator, null, contactStates.values(), null, registry);
      momentumControlModule.initialize();

      double mass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));
      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);
         ScrewTestTools.integrateVelocities(Arrays.asList(oneDoFJoints), controlDT);
         fullRobotModel.updateFrames();
         centerOfMassFrame.update();

         momentumControlModule.reset();

         for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
         {
            DenseMatrix64F vdDesired = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
            
            
            DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(inverseDynamicsJoint, vdDesired);
            momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand);
         }

         MomentumModuleSolution momentumModuleSolution;
         try
         {
            momentumModuleSolution = momentumControlModule.compute(contactStates, null);
         }
         catch (MomentumControlModuleException momentumControlModuleException)
         {
            throw new RuntimeException();
         }
        
         for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
         {
            DenseMatrix64F vdDesired = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
            inverseDynamicsJoint.packDesiredAccelerationMatrix(vdDesired, 0);
            assertTrue(MatrixFeatures.isConstantVal(vdDesired, 0.0, 1e-5));
         }

         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();

         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumModuleSolution.getCentroidalMomentumRateSolution(),
                                          gravityZ, mass, centerOfMassFrame, 1e-3);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      }
   }

	@DeployableTestMethod(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testStandingInDoubleSupport() throws NoConvergenceException
   {
      Random random = new Random(1252515L);
      DRCRobotModel robotModel = getRobotModel();
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot robot = robotModel.createSdfRobot(false);
      
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      DRCRobotInitialSetup<SDFHumanoidRobot> intialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      initializeRobot(fullRobotModel, robot, referenceFrames, intialSetup, jointMap);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      RigidBody elevator = fullRobotModel.getElevator();

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());

//    ScrewTestTools.setRandomVelocities(allJoints, random);
      OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);

      SideDependentList<ContactablePlaneBody> feet = createFeet(fullRobotModel, referenceFrames,robotModel.getWalkingControllerParameters());
      YoVariableRegistry registry = new YoVariableRegistry("test");
      double coefficientOfFriction = 1.0;

      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = createContactStates(feet, registry, coefficientOfFriction);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      OneDoFJoint lidarJoint = null;
      if(lidarParameters != null)
      {
    	  lidarJoint = fullRobotModel.getOneDoFJointByName(lidarParameters.getLidarSpindleJointName());
      }
      
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = robotModel.getControllerDT();
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 0.0, 1e-5, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, gravityZ,
            optimizationSettings, twistCalculator, null, contactStates.values(), null, registry);
      momentumControlModule.initialize();

      double mass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);
         ScrewTestTools.integrateVelocities(Arrays.asList(oneDoFJoints), controlDT);
         fullRobotModel.updateFrames();
         centerOfMassFrame.update();

         momentumControlModule.reset();

         Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap = new HashMap<GeometricJacobian, TaskspaceConstraintData>();

         constrainFeet(elevator, feet, momentumControlModule, taskspaceConstraintDataMap);
         constrainPelvis(random, fullRobotModel, momentumControlModule, taskspaceConstraintDataMap);

         MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute(contactStates, null);
         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
         
         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumModuleSolution.getCentroidalMomentumRateSolution(),
                                          gravityZ, mass, centerOfMassFrame, 1e-3);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);

         for (GeometricJacobian jacobian : taskspaceConstraintDataMap.keySet())
         {
            assertSpatialAccelerationCorrect(jacobian.getBase(), jacobian.getEndEffector(), taskspaceConstraintDataMap.get(jacobian));
         }

         assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
      }
   }


   private void assertSpatialAccelerationCorrect(RigidBody base, RigidBody endEffector, TaskspaceConstraintData taskspaceConstraintData)
   {
      RigidBody elevator = ScrewTools.getRootBody(base);
      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector accelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(accelerationBack, base, endEffector);

      DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();
      DenseMatrix64F accelerationBackSelection = computeAccelerationSelection(accelerationBack, selectionMatrix);
      DenseMatrix64F accelerationInputSelection = computeAccelerationSelection(taskspaceConstraintData.getSpatialAcceleration(), selectionMatrix);

      EjmlUnitTests.assertEquals(accelerationInputSelection, accelerationBackSelection, 1e-5);
   }

   private DenseMatrix64F computeAccelerationSelection(SpatialAccelerationVector acceleration, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F accelerationBackMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
      acceleration.packMatrix(accelerationBackMatrix, 0);

      DenseMatrix64F accelerationSelection = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, accelerationBackMatrix, accelerationSelection);

      return accelerationSelection;
   }

   private void constrainPelvis(Random random, SDFFullRobotModel fullRobotModel, OptimizationMomentumControlModule momentumControlModule,
                                Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap)
   {
      RigidBody pelvis = fullRobotModel.getRootJoint().getSuccessor();
      RigidBody elevator = fullRobotModel.getElevator();
      GeometricJacobian rootJointJacobian = new GeometricJacobian(pelvis, elevator, pelvis.getBodyFixedFrame());
      TaskspaceConstraintData pelvisTaskspaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector pelvisSpatialAcceleration = new SpatialAccelerationVector(rootJointJacobian.getEndEffectorFrame(),
                                                               rootJointJacobian.getBaseFrame(), rootJointJacobian.getJacobianFrame());
      pelvisSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));

//    pelvisSpatialAcceleration.setAngularPart(new Vector3d());
      DenseMatrix64F pelvisNullspaceMultipliers = new DenseMatrix64F(0, 1);
      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(3, Momentum.SIZE);
      CommonOps.setIdentity(orientationSelectionMatrix);
      pelvisTaskspaceConstraintData.set(pelvisSpatialAcceleration, pelvisNullspaceMultipliers, orientationSelectionMatrix);
      
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(rootJointJacobian, pelvisTaskspaceConstraintData);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      taskspaceConstraintDataMap.put(rootJointJacobian, pelvisTaskspaceConstraintData);
   }

   private void constrainFeet(RigidBody elevator, SideDependentList<ContactablePlaneBody> feet, OptimizationMomentumControlModule momentumControlModule,
                              Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide).getRigidBody();
         GeometricJacobian jacobian = new GeometricJacobian(elevator, foot, foot.getBodyFixedFrame());
         TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
         SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
                                                            foot.getBodyFixedFrame());
         taskspaceConstraintData.set(spatialAcceleration);
         
         DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
         momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
         taskspaceConstraintDataMap.put(jacobian, taskspaceConstraintData);
      }
   }

   private void initializeRobot(SDFFullHumanoidRobotModel fullRobotModel, SDFHumanoidRobot robot, HumanoidReferenceFrames referenceFrames, DRCRobotInitialSetup<SDFHumanoidRobot> intialSetup, DRCRobotJointMap jointMap)
   {
      intialSetup.initializeRobot(robot, jointMap);
      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, referenceFrames);
      sensorReader.read();
   }

   private LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> createContactStates(SideDependentList<ContactablePlaneBody> feet,
           YoVariableRegistry registry, double coefficientOfFriction)
   {
      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
      for (ContactablePlaneBody contactablePlaneBody : feet)
      {
         String contactStateName = contactablePlaneBody.getName() + "ContactState";
         YoPlaneContactState contactState = new YoPlaneContactState(contactStateName, contactablePlaneBody.getRigidBody(),
                                               contactablePlaneBody.getSoleFrame(), contactablePlaneBody.getContactPoints2d(), coefficientOfFriction, registry);
         contactStates.put(contactablePlaneBody, contactState);
      }

      return contactStates;
   }

   private SideDependentList<ContactablePlaneBody> createFeet(SDFFullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControlParamaters)
   {
      double footForward = walkingControlParamaters.getFootForwardOffset();
      double footBack = walkingControlParamaters.getFootBackwardOffset();
      double footWidth = walkingControlParamaters.getFootWidth();

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }

   private static MomentumOptimizationSettings createOptimizationSettings(InverseDynamicsJoint[] jointsToOptimizeFor, double momentumWeight, double lambda, double wRho, double rhoMin)
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, new YoVariableRegistry("test1"));
      momentumOptimizationSettings.setMomentumWeight(momentumWeight, momentumWeight, momentumWeight, momentumWeight);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(lambda);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(wRho);
      momentumOptimizationSettings.setRhoMin(rhoMin);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.01);
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);

      return momentumOptimizationSettings;
   }
}
