package us.ihmc.avatar.scs2;

import javafx.application.Platform;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.simulationconstructionset.util.RobotController;

public class SCS2AvatarSimulation
{
   private RobotDefinition robotDefinition;
   private SimulationSession simulationSession;
   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private DisposableRobotController robotController;
   private AvatarEstimatorThread stateEstimationThread;
   private AvatarControllerThread controllerThread;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel controllerFullRobotModel;
   private DRCRobotModel robotModel;
   private boolean showGUI;
   private boolean automaticallyStartSimulation;
   private RealtimeROS2Node realtimeROS2Node;

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

      simulationSession.setSessionState(SessionState.ACTIVE);

      if (showGUI)
         SessionVisualizer.startSessionVisualizer(simulationSession);
      if (automaticallyStartSimulation)
         simulationSession.setSessionMode(SessionMode.RUNNING);
      if (realtimeROS2Node != null)
         realtimeROS2Node.spin();
   }

   public void destroy()
   {
      if (robotController != null)
      {
         robotController.dispose();
         robotController = null;
      }

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }

      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }

      if (simulationSession != null)
      {
         if (showGUI)
            Platform.exit();
         else
            simulationSession.shutdownSession();
         simulationSession = null;
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

   public SimulationSession getSimulationSession()
   {
      return simulationSession;
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return highLevelHumanoidControllerFactory;
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return simulatedRobotTimeProvider;
   }

   public void setSimulationSession(SimulationSession simulationSession)
   {
      this.simulationSession = simulationSession;
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

   public void setRobotDefinition(RobotDefinition robotDefinition)
   {
      this.robotDefinition = robotDefinition;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public void setShowGUI(boolean showGUI)
   {
      this.showGUI = showGUI;
   }

   public void setAutomaticallyStartSimulation(boolean automaticallyStartSimulation)
   {
      this.automaticallyStartSimulation = automaticallyStartSimulation;
   }

   public void setRealTimeROS2Node(RealtimeROS2Node realtimeROS2Node)
   {
      this.realtimeROS2Node = realtimeROS2Node;
   }
}
