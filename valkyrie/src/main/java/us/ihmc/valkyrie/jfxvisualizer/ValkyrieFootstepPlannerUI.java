package us.ihmc.valkyrie.jfxvisualizer;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.valkyrie.ValkyrieNetworkProcessor;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieUIAuxiliaryData;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

/**
 * This class provides a visualizer for the footstep planner module.
 * It allows user to create plans, log and load plans from disk, tune parameters,
 * and debug plans.
 */
public class ValkyrieFootstepPlannerUI extends Application
{
   private SharedMemoryJavaFXMessager messager;
   private RemoteUIMessageConverter messageConverter;
   private FootstepPlannerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel model = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
      ValkyrieRobotModel previewModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
      previewModel.setTransparency(0.0);
      previewModel.setUseOBJGraphics(true);

      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      messageConverter = RemoteUIMessageConverter.createConverter(messager, model.getSimpleRobotName(), DomainFactory.PubSubImplementation.FAST_RTPS);

      messager.startMessager();

      ui = FootstepPlannerUI.createMessagerUI(primaryStage,
                                              messager,
                                              model.getVisibilityGraphsParameters(),
                                              model.getFootstepPlannerParameters(),
                                              model.getSwingPlannerParameters(),
                                              model,
                                              previewModel,
                                              model.getJointMap(),
                                              model.getContactPointParameters(),
                                              model.getWalkingControllerParameters(),
                                              new ValkyrieUIAuxiliaryData());
      ui.show();

      if(!ValkyrieNetworkProcessor.isFootstepPlanningModuleStarted())
      {
         FootstepPlanningModule plannerModule = FootstepPlanningModuleLauncher.createModule(model, DomainFactory.PubSubImplementation.FAST_RTPS);

         // Create logger and connect to messager
         FootstepPlannerLogger logger = new FootstepPlannerLogger(plannerModule);
         Runnable loggerRunnable = () -> logger.logSessionAndReportToMessager(messager);
         messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestGenerateLog, b -> new Thread(loggerRunnable).start());

         // Automatically send graph data over messager
         plannerModule.addStatusCallback(status -> handleMessagerCallbacks(plannerModule, status));
      }
   }

   private void handleMessagerCallbacks(FootstepPlanningModule planningModule, FootstepPlannerOutput status)
   {
      if (status.getFootstepPlanningResult().terminalResult())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.GraphData,
                                Triple.of(planningModule.getEdgeDataMap(), planningModule.getIterationData(), planningModule.getVariableDescriptors()));
         messager.submitMessage(FootstepPlannerMessagerAPI.StartVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getStartMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.GoalVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getGoalMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.InterRegionVisibilityMap, planningModule.getBodyPathPlanner().getSolution().getInterRegionVisibilityMap());
         messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData, planningModule.getBodyPathPlanner().getSolution().getVisibilityMapsWithNavigableRegions());
      }
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      messager.closeMessager();
      messageConverter.destroy();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
