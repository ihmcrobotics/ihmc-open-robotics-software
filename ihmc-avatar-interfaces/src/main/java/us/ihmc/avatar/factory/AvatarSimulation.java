package us.ihmc.avatar.factory;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;

public class AvatarSimulation
{
   private SimulationConstructionSet simulationConstructionSet;
   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private YoVariableServer yoVariableServer;
   private DisposableRobotController robotController;
   private AvatarEstimatorThread stateEstimationThread;
   private AvatarControllerThread controllerThread;
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel controllerFullRobotModel;
   private DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private DRCRobotModel robotModel;

   public void start()
   {
      if (yoVariableServer != null)
      {
         yoVariableServer.start();
      }

      simulationConstructionSet.startOnAThread();
   }

   public void simulate()
   {
      simulationConstructionSet.simulate();
   }

   public void dispose()
   {
      robotController.dispose();
      robotController = null;
   }

   public void resetRobot()
   {
      simulationConstructionSet.stop();

      // TODO: instead of sleeping wait for all tasks in the barrier scheduler to finish.
      ThreadTools.sleep(100);

      robotInitialSetup.initializeRobot(humanoidFloatingRootJointRobot, robotModel.getJointMap());
      AvatarSimulationFactory.initializeEstimator(humanoidFloatingRootJointRobot, stateEstimationThread);
      controllerThread.initialize();
      simulate();
   }

   public void updateEnvironment(CommonAvatarEnvironmentInterface commonAvatarEnvironment)
   {
      if (commonAvatarEnvironment != null && commonAvatarEnvironment.getEnvironmentRobots() != null)
      {
         commonAvatarEnvironment.addContactPoints(humanoidFloatingRootJointRobot.getAllGroundContactPoints());
         commonAvatarEnvironment.createAndSetContactControllerToARobot();
      }
      if (commonAvatarEnvironment != null && commonAvatarEnvironment.getTerrainObject3D() != null)
      {
         simulationConstructionSet.addStaticLinkGraphics(commonAvatarEnvironment.getTerrainObject3D().getLinkGraphics());
      }
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      controllerThread.addRobotController(controller);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return controllerThread.getFullRobotModelCorruptor();
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return highLevelHumanoidControllerFactory;
   }

   public HumanoidFloatingRootJointRobot getHumanoidFloatingRootJointRobot()
   {
      return humanoidFloatingRootJointRobot;
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return simulatedRobotTimeProvider;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet simulationConstructionSet)
   {
      this.simulationConstructionSet = simulationConstructionSet;
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory momentumBasedControllerFactory)
   {
      this.highLevelHumanoidControllerFactory = momentumBasedControllerFactory;
   }

   public void setYoVariableServer(YoVariableServer yoVariableServer)
   {
      this.yoVariableServer = yoVariableServer;
   }

   public void setRobotController(DisposableRobotController robotController)
   {
      this.robotController = robotController;
   }

   public void setStateEstimationThread(AvatarEstimatorThread stateEstimationThread)
   {
      this.stateEstimationThread = stateEstimationThread;
   }

   public void setControllerThread(AvatarControllerThread controllerThread)
   {
      this.controllerThread = controllerThread;
   }

   public void setHumanoidFloatingRootJointRobot(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      this.humanoidFloatingRootJointRobot = humanoidFloatingRootJointRobot;
   }

   public void setSimulatedRobotTimeProvider(SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider)
   {
      this.simulatedRobotTimeProvider = simulatedRobotTimeProvider;
   }

   public void setFullHumanoidRobotModel(FullHumanoidRobotModel controllerFullRobotModel)
   {
      this.controllerFullRobotModel = controllerFullRobotModel;
   }

   public void addRobotControllerOnEstimatorThread(RobotController controller)
   {
      stateEstimationThread.addRobotController(controller);
   }

   public void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }
}
