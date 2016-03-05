package us.ihmc.darpaRoboticsChallenge.momentumBasedControl;

import static org.junit.Assert.assertTrue;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertRootJointWrenchZero;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesInFrictionCones;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesSumUpToMomentumDot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.xml.bind.JAXBException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
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
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
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
      
      InverseDynamicsJoint[] jointsToOptimizeFor = MomentumBasedController.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = 1e-4;
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 1e-3, 1e-9, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, gravityZ, optimizationSettings,
            twistCalculator, null, new ArrayList<>(feet.values()), registry);
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
            
            
            JointspaceAccelerationCommand desiredJointAccelerationCommand = new JointspaceAccelerationCommand(inverseDynamicsJoint, vdDesired);
            momentumControlModule.setInverseDynamicsCommand(desiredJointAccelerationCommand);
         }

         for (PlaneContactState planeContactState : contactStates.values())
         {
            PlaneContactStateCommand command = new PlaneContactStateCommand();
            planeContactState.getPlaneContactStateCommand(command);
            momentumControlModule.setInverseDynamicsCommand(command);
         }

         MomentumModuleSolution momentumModuleSolution;
         try
         {
            momentumModuleSolution = momentumControlModule.compute();
         }
         catch (MomentumControlModuleException momentumControlModuleException)
         {
            throw new RuntimeException();
         }
        
         for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
         {
            DenseMatrix64F vdDesired = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
            inverseDynamicsJoint.getDesiredAccelerationMatrix(vdDesired, 0);
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
      
      InverseDynamicsJoint[] jointsToOptimizeFor = MomentumBasedController.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = robotModel.getControllerDT();
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 0.0, 1e-5, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      GeometricJacobianHolder jacobianHolder = new GeometricJacobianHolder();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, gravityZ, optimizationSettings,
            twistCalculator, jacobianHolder, new ArrayList<>(feet.values()), registry);
      momentumControlModule.initialize();

      double mass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);
         ScrewTestTools.integrateVelocities(Arrays.asList(oneDoFJoints), controlDT);
         fullRobotModel.updateFrames();
         centerOfMassFrame.update();

         momentumControlModule.reset();

         List<SpatialAccelerationCommand> spatialAccelerationCommands = new ArrayList<>();

         constrainFeet(elevator, feet, momentumControlModule, spatialAccelerationCommands, jacobianHolder);
         constrainPelvis(random, fullRobotModel, momentumControlModule, spatialAccelerationCommands, jacobianHolder);

         for (PlaneContactState planeContactState : contactStates.values())
         {
            PlaneContactStateCommand command = new PlaneContactStateCommand();
            planeContactState.getPlaneContactStateCommand(command);
            momentumControlModule.setInverseDynamicsCommand(command);
         }

         jacobianHolder.compute();
         MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute();
         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
         
         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumModuleSolution.getCentroidalMomentumRateSolution(),
                                          gravityZ, mass, centerOfMassFrame, 1e-3);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);

         for (SpatialAccelerationCommand spatialAccelerationCommand : spatialAccelerationCommands)
         {
            assertSpatialAccelerationCorrect(spatialAccelerationCommand.getBase(), spatialAccelerationCommand.getEndEffector(), spatialAccelerationCommand);
         }


         assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
      }
   }


   private void assertSpatialAccelerationCorrect(RigidBody base, RigidBody endEffector, SpatialAccelerationCommand spatialAccelerationCommand)
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
      spatialAccelerationCalculator.getRelativeAcceleration(accelerationBack, base, endEffector);

      DenseMatrix64F selectionMatrix = spatialAccelerationCommand.getSelectionMatrix();
      DenseMatrix64F accelerationBackSelection = computeAccelerationSelection(accelerationBack, selectionMatrix);
      DenseMatrix64F accelerationInputSelection = computeAccelerationSelection(spatialAccelerationCommand.getSpatialAcceleration(), selectionMatrix);

      EjmlUnitTests.assertEquals(accelerationInputSelection, accelerationBackSelection, 1e-5);
   }

   private DenseMatrix64F computeAccelerationSelection(SpatialAccelerationVector acceleration, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F accelerationBackMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
      acceleration.getMatrix(accelerationBackMatrix, 0);

      DenseMatrix64F accelerationSelection = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, accelerationBackMatrix, accelerationSelection);

      return accelerationSelection;
   }

   private void constrainPelvis(Random random, SDFFullRobotModel fullRobotModel, OptimizationMomentumControlModule momentumControlModule,
         List<SpatialAccelerationCommand> spatialAccelerationCommands, GeometricJacobianHolder jacobianHolder)
   {
      RigidBody pelvis = fullRobotModel.getRootJoint().getSuccessor();
      RigidBody elevator = fullRobotModel.getElevator();
      SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand(jacobianHolder.getOrCreateGeometricJacobian(elevator, pelvis, pelvis.getBodyFixedFrame()));
      SpatialAccelerationVector pelvisSpatialAcceleration = new SpatialAccelerationVector(pelvis.getBodyFixedFrame(), elevator.getBodyFixedFrame(), pelvis.getBodyFixedFrame());
      pelvisSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));

//    pelvisSpatialAcceleration.setAngularPart(new Vector3d());
      DenseMatrix64F pelvisNullspaceMultipliers = new DenseMatrix64F(0, 1);
      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(3, Momentum.SIZE);
      CommonOps.setIdentity(orientationSelectionMatrix);
      spatialAccelerationCommand.set(pelvisSpatialAcceleration, pelvisNullspaceMultipliers, orientationSelectionMatrix);
      spatialAccelerationCommand.set(elevator, pelvis);
      
      momentumControlModule.setInverseDynamicsCommand(spatialAccelerationCommand);
      spatialAccelerationCommands.add(spatialAccelerationCommand);
   }

   private void constrainFeet(RigidBody elevator, SideDependentList<ContactablePlaneBody> feet, OptimizationMomentumControlModule momentumControlModule,
                              List<SpatialAccelerationCommand> spatialAccelerationCommands, GeometricJacobianHolder jacobianHolder)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide).getRigidBody();
         SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(), foot.getBodyFixedFrame());
         SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand(jacobianHolder.getOrCreateGeometricJacobian(elevator, foot, foot.getBodyFixedFrame()));
         spatialAccelerationCommand.set(spatialAcceleration);
         spatialAccelerationCommand.set(elevator, foot);
         
         momentumControlModule.setInverseDynamicsCommand(spatialAccelerationCommand);
         spatialAccelerationCommands.add(spatialAccelerationCommand);
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
