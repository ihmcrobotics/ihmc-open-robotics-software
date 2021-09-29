package us.ihmc.avatar.multiContact;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class HumanoidRobotTransformOptimizerTest
{
   public abstract DRCRobotModel createNewRobotModel();

   private DRCRobotModel robotModelA;
   private DRCRobotModel robotModelB;
   private DRCRobotModel robotModelBCorrected;

   private static final YoAppearanceRGBColor robotBApperance = new YoAppearanceRGBColor(Color.decode("#9e8329"), 0.25); // Some darkish orangish
   private static final YoAppearanceRGBColor robotBCorrectedApperance = new YoAppearanceRGBColor(Color.decode("#35824a"), 0.75); // Some darkish green

   @BeforeEach
   public void setup()
   {
      robotModelA = createNewRobotModel();
      robotModelB = createNewRobotModel();
      robotModelBCorrected = createNewRobotModel();

      robotModelA.getRobotDescription().setName("RobotA");
      robotModelB.getRobotDescription().setName("RobotB");
      robotModelBCorrected.getRobotDescription().setName("RobotBCorrected");
      HumanoidKinematicsToolboxControllerTest.recursivelyModifyGraphics(robotModelB.getRobotDescription().getChildrenJoints().get(0), robotBApperance);
      HumanoidKinematicsToolboxControllerTest.recursivelyModifyGraphics(robotModelBCorrected.getRobotDescription().getChildrenJoints().get(0),
                                                                robotBCorrectedApperance);
   }

   @AfterEach
   public void tearDown()
   {
      robotModelA = null;
      robotModelB = null;
      robotModelBCorrected = null;
   }

   public void visualizeRobots(Robot... robots)
   {
      if (ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet(robots);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public void runTest(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetupA, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetupB,
                       double epsilon)
   {
      HumanoidFloatingRootJointRobot scsRobotA = robotModelA.createHumanoidFloatingRootJointRobot(false);
      HumanoidFloatingRootJointRobot scsRobotB = robotModelB.createHumanoidFloatingRootJointRobot(false);
      HumanoidFloatingRootJointRobot scsRobotBCorrected = robotModelBCorrected.createHumanoidFloatingRootJointRobot(false);

      FullHumanoidRobotModel idRobotA = robotModelA.createFullRobotModel();
      FullHumanoidRobotModel idRobotB = robotModelB.createFullRobotModel();
      FullHumanoidRobotModel idRobotBCorrected = robotModelBCorrected.createFullRobotModel();

      initialSetupA.initializeRobot(scsRobotA, robotModelA.getJointMap());
      initialSetupB.initializeRobot(scsRobotB, robotModelB.getJointMap());
      initialSetupB.initializeRobot(scsRobotBCorrected, robotModelBCorrected.getJointMap());

      copyRobotState(scsRobotA, idRobotA);
      copyRobotState(scsRobotB, idRobotB);
      copyRobotState(scsRobotBCorrected, idRobotBCorrected);

      RobotTransformOptimizer robotTransformOptimizer = new RobotTransformOptimizer(idRobotA.getElevator(), idRobotB.getElevator());
      robotTransformOptimizer.addDefaultRigidBodyLinearErrorCalculators((bodyA, bodyB) -> !bodyA.isRootBody());
      robotTransformOptimizer.setInitializeWithHeaviestBody(true);
      robotTransformOptimizer.compute();

      RigidBodyTransform transform = new RigidBodyTransform();
      scsRobotBCorrected.getRootJoint().getTransformToWorld(transform);
      transform.preMultiply(robotTransformOptimizer.getTransformFromBToA());
      scsRobotBCorrected.getRootJoint().setRotationAndTranslation(transform);
      scsRobotBCorrected.update();
      idRobotBCorrected.getRootJoint().getJointPose().set(transform);
      idRobotBCorrected.updateFrames();

      visualizeRobots(scsRobotA, scsRobotB, scsRobotBCorrected);

      OneDoFJointBasics[] jointsA = idRobotA.getOneDoFJoints();
      OneDoFJointBasics[] jointsBCorrected = idRobotBCorrected.getOneDoFJoints();

      double errorMagnitude = 0.0;

      for (int i = 0; i < jointsA.length; i++)
      {
         OneDoFJointBasics jointA = jointsA[i];
         OneDoFJointBasics jointBCorrected = jointsBCorrected[i];

         errorMagnitude += jointA.getFrameAfterJoint().getTransformToDesiredFrame(jointBCorrected.getFrameAfterJoint()).getTranslation().length();
      }

      errorMagnitude /= jointsA.length;
      assertTrue(errorMagnitude < epsilon, "Error magnitude is larger than expected: " + errorMagnitude);
   }

   private static void copyRobotState(HumanoidFloatingRootJointRobot source, FullHumanoidRobotModel destination)
   {
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(source, 0);
      drcPerfectSensorReaderFactory.build(destination.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
      destination.updateFrames();
   }
}
