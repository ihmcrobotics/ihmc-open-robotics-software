package us.ihmc.avatar.multiContact;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;

public abstract class HumanoidRobotTransformOptimizerTest
{
   public abstract DRCRobotModel createNewRobotModel();

   private DRCRobotModel robotModelA;
   private DRCRobotModel robotModelB;
   private DRCRobotModel robotModelBCorrected;

   private static final MaterialDefinition robotBMaterial = new MaterialDefinition(ColorDefinitions.parse("#9e8329").derive(0, 1, 1, 0.75)); // Some darkish orangish
   private static final MaterialDefinition robotBCorrectedMaterial = new MaterialDefinition(ColorDefinitions.parse("#35824a").derive(0, 1, 1, 0.25)); // Some darkish green

   @BeforeEach
   public void setup()
   {
      robotModelA = createNewRobotModel();
      robotModelB = createNewRobotModel();
      robotModelBCorrected = createNewRobotModel();

      robotModelA.getRobotDefinition().setName("RobotA");
      robotModelB.getRobotDefinition().setName("RobotB");
      robotModelBCorrected.getRobotDefinition().setName("RobotBCorrected");
      RobotDefinitionTools.setRobotDefinitionMaterial(robotModelB.getRobotDefinition(), robotBMaterial);
      RobotDefinitionTools.setRobotDefinitionMaterial(robotModelBCorrected.getRobotDefinition(), robotBCorrectedMaterial);
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

      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.setJavaFXThreadImplicitExit(false);
      scs.addRobots(Arrays.asList(robots));
      scs.waitUntilVisualizerDown();
   }

   public void runTest(RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetupA,
                       RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetupB,
                       double epsilon)
   {
      Robot scsRobotA = new Robot(robotModelA.getRobotDefinition(), SimulationConstructionSet2.inertialFrame);
      Robot scsRobotB = new Robot(robotModelB.getRobotDefinition(), SimulationConstructionSet2.inertialFrame);
      Robot scsRobotBCorrected = new Robot(robotModelBCorrected.getRobotDefinition(), SimulationConstructionSet2.inertialFrame);

      FullHumanoidRobotModel idRobotA = robotModelA.createFullRobotModel();
      FullHumanoidRobotModel idRobotB = robotModelB.createFullRobotModel();
      FullHumanoidRobotModel idRobotBCorrected = robotModelBCorrected.createFullRobotModel();

      initialSetupA.initializeRobot(scsRobotA.getRootBody());
      initialSetupB.initializeRobot(scsRobotB.getRootBody());
      initialSetupB.initializeRobot(scsRobotBCorrected.getRootBody());

      copyRobotState(scsRobotA, idRobotA);
      copyRobotState(scsRobotB, idRobotB);
      copyRobotState(scsRobotBCorrected, idRobotBCorrected);

      RobotTransformOptimizer robotTransformOptimizer = new RobotTransformOptimizer(idRobotA.getElevator(), idRobotB.getElevator());
      robotTransformOptimizer.addDefaultRigidBodyLinearErrorCalculators((bodyA, bodyB) -> !bodyA.isRootBody());
      robotTransformOptimizer.setInitializeWithHeaviestBody(true);
      robotTransformOptimizer.compute();

      RigidBodyTransform transform = new RigidBodyTransform();
      scsRobotBCorrected.getFloatingRootJoint().getJointPose().get(transform);
      transform.preMultiply(robotTransformOptimizer.getTransformFromBToA());
      scsRobotBCorrected.getFloatingRootJoint().getJointPose().set(transform);
      scsRobotBCorrected.updateFrames();
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

   private static void copyRobotState(Robot source, FullHumanoidRobotModel destination)
   {
      List<? extends SimJointBasics> allSourceJoints = source.getFloatingRootJoint().subtreeList();
      List<? extends JointBasics> allDestinationJoints = destination.getRootJoint().subtreeList();
      for (JointStateType jointStateType : JointStateType.values())
         MultiBodySystemTools.copyJointsState(allSourceJoints, allDestinationJoints, jointStateType);
      destination.updateFrames();
   }
}
