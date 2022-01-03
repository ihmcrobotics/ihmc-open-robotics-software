package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AvatarSimulation
{
   private SimulationConstructionSet simulationConstructionSet;
   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private DisposableRobotController robotController;
   private AvatarEstimatorThread stateEstimationThread;
   private AvatarControllerThread controllerThread;
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel controllerFullRobotModel;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private DRCRobotModel robotModel;

   public void start()
   {
      if (intraprocessYoVariableLogger != null)
      {
         intraprocessYoVariableLogger.start();
      }
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

   public void destroy()
   {
      dispose();
      if (yoVariableServer != null)
         yoVariableServer.close();
      ThreadTools.startAsDaemon(() -> simulationConstructionSet.stopSimulationThread(), "WaitForSimulationThreadToStop");
      simulationConstructionSet.closeAndDispose();
   }

   public void resetRobot()
   {
      resetRobot(true);
   }

   public void resetRobot(boolean simulateAfterReset)
   {
      simulationConstructionSet.stop();

      // TODO: instead of sleeping wait for all tasks in the barrier scheduler to finish.
      ThreadTools.sleep(100);

      List<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      humanoidFloatingRootJointRobot.getAllOneDegreeOfFreedomJoints(joints);

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         joint.setQd(0.0);
         joint.setQdd(0.0);
         joint.setTau(0.0);
      }

      humanoidFloatingRootJointRobot.getRootJoint().setAngularVelocityInBody(new Vector3D());
      humanoidFloatingRootJointRobot.getRootJoint().setVelocity(0, 0, 0);

      robotInitialSetup.initializeRobot(humanoidFloatingRootJointRobot);
      AvatarSimulationFactory.initializeEstimator(humanoidFloatingRootJointRobot, stateEstimationThread);
      controllerThread.initialize();

      if (simulateAfterReset)
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

   public void setIntraprocessYoVariableLogger(IntraprocessYoVariableLogger intraprocessYoVariableLogger)
   {
      this.intraprocessYoVariableLogger = intraprocessYoVariableLogger;
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

   public void setRobotInitialSetup(RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public YoRegistry getStateEstimationThreadRegistry()
   {
      return stateEstimationThread.getYoRegistry();
   }

   public YoRegistry getControllerThreadRegistry()
   {
      return controllerThread.getYoVariableRegistry();
   }
}
