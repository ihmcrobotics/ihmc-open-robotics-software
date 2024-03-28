package us.ihmc.avatar.joystickBasedJavaFXController;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXController.SecondaryControlOption;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.JavaFXRobotVisualizer;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.ui.JavaFXPlanarRegionsViewer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;

public class JoystickBasedSteppingMainUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final JavaFXRobotVisualizer robotVisualizer;
   private final StepGeneratorJavaFXController stepGeneratorJavaFXController;
   private final AnimationTimer cameraTracking;
   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(StepGeneratorJavaFXTopics.API);
   private final XBoxOneJavaFXController xBoxOneJavaFXController;
   private final JavaFXPlanarRegionsViewer planarRegionsViewer;

   @FXML
   private StepGeneratorParametersPaneController stepGeneratorParametersPaneController;

   public JoystickBasedSteppingMainUI(String robotName,
                                      Stage primaryStage,
                                      ROS2Node ros2Node,
                                      FullHumanoidRobotModelFactory fullRobotModelFactory,
                                      WalkingControllerParameters walkingControllerParameters,
                                      HumanoidRobotKickMessenger kickMessenger,
                                      HumanoidRobotPunchMessenger punchMessenger,
                                      RobotLowLevelMessenger lowLevelMessenger,
                                      SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons)
         throws Exception
   {
      this(robotName,
           primaryStage,
           ros2Node,
           null,
           fullRobotModelFactory,
           walkingControllerParameters,
           kickMessenger,
           punchMessenger,
           lowLevelMessenger,
           footPolygons);
   }

   public JoystickBasedSteppingMainUI(String robotName,
                                      Stage primaryStage,
                                      ROS2NodeInterface ros2Node,
                                      String workingDirectoryPath,
                                      FullHumanoidRobotModelFactory fullRobotModelFactory,
                                      WalkingControllerParameters walkingControllerParameters,
                                      HumanoidRobotKickMessenger kickMessenger,
                                      HumanoidRobotPunchMessenger punchMessenger,
                                      RobotLowLevelMessenger lowLevelMessenger,
                                      SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons)
         throws Exception
   {
      this.primaryStage = primaryStage;
      xBoxOneJavaFXController = new XBoxOneJavaFXController(messager);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      setupSceneLighting(view3dFactory);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      robotVisualizer = new JavaFXRobotVisualizer(fullRobotModelFactory);
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(RobotConfigurationData.class),
                                  s -> robotVisualizer.submitNewConfiguration(s.takeNextData()));
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());

      planarRegionsViewer = new JavaFXPlanarRegionsViewer();
      ros2Node.createSubscription(REACommunicationProperties.outputTopic.withTypeName(PlanarRegionsListMessage.class),
                                  s -> planarRegionsViewer.submitPlanarRegions(s.takeNextData()));
      view3dFactory.addNodeToView(planarRegionsViewer.getRootNode());

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

      stepGeneratorJavaFXController = new StepGeneratorJavaFXController(robotName,
                                                                        messager,
                                                                        walkingControllerParameters,
                                                                        ros2Node,
                                                                        robotVisualizer,
                                                                        kickMessenger,
                                                                        punchMessenger,
                                                                        lowLevelMessenger,
                                                                        footPolygons);
      view3dFactory.addNodeToView(stepGeneratorJavaFXController.getRootNode());

      messager.startMessager();
      stepGeneratorParametersPaneController.initialize(messager, walkingControllerParameters, workingDirectoryPath);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setScene(new Scene(mainPane, 800, 600));
      primaryStage.setOnCloseRequest(event -> stop());

      start();
   }

   public void createYoVariableServer(DataServerSettings settings, LogModelProvider modelProvider)
   {
      stepGeneratorJavaFXController.createYoVariableServer(settings, modelProvider);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return robotVisualizer.getFullRobotModel();
   }

   private void setupSceneLighting(View3DFactory view3dFactory)
   {
      // TODO: Replace with View3DFactory.addDefaultLighting() when javafx-toolkit 0.12.8+ i
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

   public void setActiveSecondaryControlOption(SecondaryControlOption activeSecondaryControlOption)
   {
      stepGeneratorJavaFXController.setActiveSecondaryControlOption(activeSecondaryControlOption);
      stepGeneratorParametersPaneController.updateImageLayout(activeSecondaryControlOption);
   }

   public void start() throws IOException
   {
      primaryStage.show();
      robotVisualizer.start();
      stepGeneratorJavaFXController.start();
      cameraTracking.start();
      planarRegionsViewer.start();
   }

   public void stop()
   {
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      xBoxOneJavaFXController.stop();
      robotVisualizer.stop();
      stepGeneratorJavaFXController.stop();
      stepGeneratorParametersPaneController.close();
      cameraTracking.stop();
      planarRegionsViewer.stop();
      ThreadTools.sleep(100); // Give some time to send the message.:
   }
}
