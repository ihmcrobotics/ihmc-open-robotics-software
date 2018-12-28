package us.ihmc.quadrupedUI;

import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedUI.controllers.BodyPoseController;
import us.ihmc.quadrupedUI.uiControllers.MainTabController;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class QuadrupedUserInterface
{
   private final Stage primaryStage;

   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;
   private final ScheduledExecutorService executorService = Executors
         .newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   @FXML
   private MainTabController mainTabController;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                 QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly xGaitSettings, YoVariableRegistry registry)
         throws Exception
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      BorderPane mainPane = loader.load();

      mainTabController.attachMessager(messager);

      mainTabController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      robotVisualizer = new JavaFXQuadrupedVisualizer(messager, modelFactory);

      view3dFactory.addNodeToView(robotVisualizer.getRootNode());

      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);

      cameraTracking = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            FramePoint3D rootJointPosition = new FramePoint3D(robotVisualizer.getFullRobotModel().getRootJoint().getFrameAfterJoint());
            rootJointPosition.changeFrame(ReferenceFrame.getWorldFrame());
            rootJointOffset.setX(rootJointPosition.getX());
            rootJointOffset.setY(rootJointPosition.getY());
            rootJointOffset.setZ(rootJointPosition.getZ());
         }
      };

      QuadrupedJoystick joystick;
      try
      {
         joystick = new QuadrupedJoystick();
      }
      catch (JoystickNotFoundException e)
      {
         LogTools.error("Could not find joystick. Running without xbox controller");
         joystick = null;
      }

      BodyPoseController bodyPoseController = new BodyPoseController(joystick, messager, executorService, physicalProperties.getNominalBodyHeight());

      YoQuadrupedXGaitSettings yoXGaitSettings = new YoQuadrupedXGaitSettings(xGaitSettings, registry);
      yoXGaitSettings.addVariableChangedListener(v -> messager.submitMessage(QuadrupedUIMessagerAPI.XGaitSettingsTopic, yoXGaitSettings));
      messager.registerTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, yoXGaitSettings::set);

      robotVisualizer.start();
      cameraTracking.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());

   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         robotVisualizer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static QuadrupedUserInterface createUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                                            QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                            YoVariableRegistry registry) throws Exception
   {
      return new QuadrupedUserInterface(primaryStage, messager, modelFactory, physicalProperties, xGaitSettings, registry);
   }
}
