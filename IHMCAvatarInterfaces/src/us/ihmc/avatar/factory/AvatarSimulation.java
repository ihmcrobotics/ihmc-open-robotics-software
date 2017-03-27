package us.ihmc.avatar.factory;

import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public class AvatarSimulation
{
   private SimulationConstructionSet simulationConstructionSet;
   private MomentumBasedControllerFactory momentumBasedControllerFactory;
   private YoVariableServer yoVariableServer;
   private AbstractThreadedRobotController threadedRobotController;
   private HumanoidGlobalDataProducer humanoidGlobalDataProducer;
   private DRCEstimatorThread stateEstimationThread;
   private DRCControllerThread controllerThread;
   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private ThreadDataSynchronizerInterface threadDataSynchronizer;

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
      threadedRobotController.stop();

      if (humanoidGlobalDataProducer != null)
      {
         humanoidGlobalDataProducer.stop();
      }

      stateEstimationThread.dispose();
      stateEstimationThread = null;

      controllerThread.dispose();
      controllerThread = null;

      threadedRobotController = null;
      humanoidGlobalDataProducer = null;

      closeableAndDisposableRegistry.closeAndDispose();
      closeableAndDisposableRegistry = null;
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      stateEstimationThread.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);
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
      return threadDataSynchronizer.getControllerFullRobotModel();
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

   public MomentumBasedControllerFactory getMomentumBasedControllerFactory()
   {
      return momentumBasedControllerFactory;
   }

   public HumanoidFloatingRootJointRobot getHumanoidFloatingRootJointRobot()
   {
      return humanoidFloatingRootJointRobot;
   }

   public ThreadDataSynchronizerInterface getThreadDataSynchronizer()
   {
      return threadDataSynchronizer;
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return simulatedRobotTimeProvider;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet simulationConstructionSet)
   {
      this.simulationConstructionSet = simulationConstructionSet;
   }

   public void setMomentumBasedControllerFactory(MomentumBasedControllerFactory momentumBasedControllerFactory)
   {
      this.momentumBasedControllerFactory = momentumBasedControllerFactory;
   }

   public void setYoVariableServer(YoVariableServer yoVariableServer)
   {
      this.yoVariableServer = yoVariableServer;
   }

   public void setThreadedRobotController(AbstractThreadedRobotController threadedRobotController)
   {
      this.threadedRobotController = threadedRobotController;
   }

   public void setHumanoidGlobalDataProducer(HumanoidGlobalDataProducer humanoidGlobalDataProducer)
   {
      this.humanoidGlobalDataProducer = humanoidGlobalDataProducer;
   }

   public void setStateEstimationThread(DRCEstimatorThread stateEstimationThread)
   {
      this.stateEstimationThread = stateEstimationThread;
   }

   public void setControllerThread(DRCControllerThread controllerThread)
   {
      this.controllerThread = controllerThread;
   }

   public void setCloseableAndDisposableRegistry(CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this.closeableAndDisposableRegistry = closeableAndDisposableRegistry;
   }

   public void setHumanoidFloatingRootJointRobot(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      this.humanoidFloatingRootJointRobot = humanoidFloatingRootJointRobot;
   }

   public void setSimulatedRobotTimeProvider(SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider)
   {
      this.simulatedRobotTimeProvider = simulatedRobotTimeProvider;
   }

   public void setThreadDataSynchronizer(ThreadDataSynchronizerInterface threadDataSynchronizer)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
   }
}
