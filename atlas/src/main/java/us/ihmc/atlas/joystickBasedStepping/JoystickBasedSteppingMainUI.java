package us.ihmc.atlas.joystickBasedStepping;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;

public class JoystickBasedSteppingMainUI
{
   private static final String HOST = NetworkParameters.getHost(NetworkParameterKeys.robotController);
   private static final NetworkPorts PORT = NetworkPorts.JOYSTICK_BASED_CONTINUOUS_STEPPING;
   private static final IHMCCommunicationKryoNetClassList NET_CLASS_LIST = new IHMCCommunicationKryoNetClassList();

   private final PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(HOST, PORT, NET_CLASS_LIST);

   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final JavaFXRobotVisualizer javaFXRobotVisualizer;
   private final StepGeneratorJavaFXController stepGeneratorJavaFXController;

   public JoystickBasedSteppingMainUI(Stage primaryStage, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                      WalkingControllerParameters walkingControllerParameters)
         throws IOException
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      javaFXRobotVisualizer = new JavaFXRobotVisualizer(fullRobotModelFactory);
      packetCommunicator.attachListener(RobotConfigurationData.class, javaFXRobotVisualizer::submitNewConfiguration);
      view3dFactory.addNodeToView(javaFXRobotVisualizer.getRootNode());

      stepGeneratorJavaFXController = new StepGeneratorJavaFXController(walkingControllerParameters, packetCommunicator, javaFXRobotVisualizer);
      view3dFactory.addNodeToView(stepGeneratorJavaFXController.getRootNode());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());

      start();
   }

   public void start() throws IOException
   {
      primaryStage.show();
      javaFXRobotVisualizer.start();
      stepGeneratorJavaFXController.start();
      packetCommunicator.connect();
   }

   public void stop()
   {
      javaFXRobotVisualizer.stop();
      stepGeneratorJavaFXController.stop();
      packetCommunicator.closeConnection();
   }
}
