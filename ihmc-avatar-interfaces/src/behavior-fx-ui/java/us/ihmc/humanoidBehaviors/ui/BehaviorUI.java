package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.behaviors.DirectRobotUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.StepInPlaceBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   @FXML private PatrolBehaviorUIController patrolBehaviorUIController;
   @FXML private StepInPlaceBehaviorUIController stepInPlaceBehaviorUIController;
   @FXML private DirectRobotUIController directRobotUIController;

   public BehaviorUI(Stage primaryStage,
                     Messager behaviorMessager,
                     DRCRobotModel robotModel,
                     PubSubImplementation pubSubImplementation) throws Exception
   {
      this.primaryStage = primaryStage;

      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_ui");

//      YoVariableRegistry registry = new YoVariableRegistry("vrui");
//      ParameterLoaderHelper.loadParameters(getClass(), ClassLoader.getSystemClassLoader().getResourceAsStream("vrParameters.xml"), registry);
//      YoVariableServer yoVariableServer = new YoVariableServer(robotModel.getSimpleRobotName() + getClass().getSimpleName(),
//                                                               null,
//                                                               new DataServerSettings(false),
//                                                               0.01);
//      yoVariableServer.setMainRegistry(registry, null, null);
//      try
//      {
//         yoVariableServer.start();
//      }
//      catch(Throwable e)
//      {
//         e.printStackTrace();
//      }
//      PeriodicNonRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicNonRealtimeThreadSchedulerFactory();
//      PeriodicThreadScheduler updateScheduler = schedulerFactory.createPeriodicThreadScheduler("YoVariableUpdate");
//      AtomicLong timestamp = new AtomicLong();
//      updateScheduler.schedule(() -> {
//         yoVariableServer.update(timestamp.getAndAdd(10000));
//      }, 10, TimeUnit.MILLISECONDS);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 2000.0,true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      {
         /** TODO: Replace with View3DFactory.addDefaultLighting() when javafx-toolkit 0.12.8+ is released */
         double ambientValue = 0.7;
         double pointValue = 0.2;
         double pointDistance = 1000.0;
         Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
         view3dFactory.addNodeToView(new AmbientLight(ambientColor));
         Color indoorColor = Color.color(pointValue, pointValue, pointValue);
         view3dFactory.addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);
      }
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

      stepInPlaceBehaviorUIController.init(behaviorMessager);
      patrolBehaviorUIController.init(subScene, behaviorMessager, robotModel);
      directRobotUIController.init(ros2Node, robotModel);

      view3dFactory.addNodeToView(patrolBehaviorUIController);
      view3dFactory.addNodeToView(new LivePlanarRegionsGraphic(ros2Node));
      view3dFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1200, 800);

      primaryStage.setScene(mainScene);
   }

   public void show()
   {
      primaryStage.show();
   }

   public static void claimEditing(Object claimingEditor)
   {
      if (BehaviorUI.ACTIVE_EDITOR != null)
      {
         throw new RuntimeException("Only one editor may be active at a time.");
      }
      else
      {
         BehaviorUI.ACTIVE_EDITOR = claimingEditor;
         LogTools.debug("editor activated: {}", claimingEditor.getClass().getSimpleName());
      }
   }
}
