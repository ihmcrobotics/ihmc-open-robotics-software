package us.ihmc.robotEnvironmentAwareness.polygonizer;

import static us.ihmc.robotEnvironmentAwareness.polygonizer.MultiplePointCloudViewer.*;
import static us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.*;
import static us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager.*;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.javafx.JavaFXMessager;

public class PolygonizerVisualizerUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final MultipleConcaveHullViewer multipleConcaveHullViewer;
   private final MultiplePointCloudViewer multiplePointCloudViewer;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   @FXML
   private PolygonizerMenuBarController polygonizerMenuBarController;
   @FXML
   private PolygonizerDisplayOptionTabController polygonizerDisplayOptionTabController;
   @FXML
   private PolygonizerParametersTabController polygonizerParametersTabController;

   public PolygonizerVisualizerUI(JavaFXMessager messager, Stage primaryStage) throws Exception
   {
      this.messager = messager;
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();
      messager.startMessager();

      polygonizerMenuBarController.initialize(messager, executorService, primaryStage);
      polygonizerDisplayOptionTabController.initialize(messager);
      polygonizerParametersTabController.initialize(messager);

      new PolygonizerManager(messager, executorService);
      multipleConcaveHullViewer = new MultipleConcaveHullViewer(messager);
      multiplePointCloudViewer = new MultiplePointCloudViewer(messager, executorService);

      messager.addTopicListener(PolygonizerOutput,
                                     message -> executorService.execute(() -> multipleConcaveHullViewer.submit(MultipleConcaveHullViewer.toConcaveHullViewerInputList(message))));
      messager.addTopicListener(PlanarRegionSemgentationData, data -> messager.submitMessage(PointCloudInput, MultiplePointCloudViewer.toInputList(data)));

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(multipleConcaveHullViewer.getRootNode());
      view3dFactory.addNodeToView(multiplePointCloudViewer.getRootNode());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.addEventHandler(KeyEvent.KEY_PRESSED, event -> {
         if (event.getCode() == KeyCode.F5)
            messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationReload, true);
      });

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws Exception
   {
      primaryStage.show();
      multipleConcaveHullViewer.start();
      multiplePointCloudViewer.start();
   }

   public void stop()
   {
      try
      {
         multipleConcaveHullViewer.stop();
         multiplePointCloudViewer.stop();
         executorService.shutdown();
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private static MessagerAPI messagerAPI;

   public static MessagerAPI getMessagerAPI()
   {
      if (messagerAPI == null)
      {
         MessagerAPIFactory apiFactory = new MessagerAPIFactory();
         apiFactory.createRootCategory(PolygonizerVisualizerUI.class.getSimpleName());
         apiFactory.includeMessagerAPIs(PolygonizerManager.API, MultipleConcaveHullViewer.API, MultiplePointCloudViewer.API);
         messagerAPI = apiFactory.getAPIAndCloseFactory();
      }
      return messagerAPI;
   }
}
