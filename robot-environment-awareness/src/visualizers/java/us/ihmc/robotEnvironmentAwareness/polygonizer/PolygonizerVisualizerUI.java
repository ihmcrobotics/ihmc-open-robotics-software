package us.ihmc.robotEnvironmentAwareness.polygonizer;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

public class PolygonizerVisualizerUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final Polygonizer polygonizer;
   private final MultipleConcaveHullViewer multipleConcaveHullViewer;

   @FXML
   private PolygonizerMenuBarController polygonizerMenuBarController;
   @FXML
   private PolygonizerDisplayOptionTabController polygonizerDisplayOptionTabController;

   public PolygonizerVisualizerUI(JavaFXMessager messager, Stage primaryStage) throws Exception
   {
      this.messager = messager;
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();
      messager.startMessager();

      polygonizerMenuBarController.initialize(messager, primaryStage);
      polygonizerDisplayOptionTabController.initialize(messager);

      polygonizer = new Polygonizer(messager);
      multipleConcaveHullViewer = new MultipleConcaveHullViewer(messager);
      messager.registerTopicListener(Polygonizer.PolygonizerOutput,
                                     message -> multipleConcaveHullViewer.submit(MultipleConcaveHullViewer.toConcaveHullViewerInputs(message)));

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(multipleConcaveHullViewer.getRootNode());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws Exception
   {
      primaryStage.show();
      multipleConcaveHullViewer.start();
   }

   public void stop()
   {
      try
      {
         multipleConcaveHullViewer.stop();
         polygonizer.shutdown();
         polygonizerMenuBarController.shutdown();
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
         apiFactory.includeMessagerAPIs(Polygonizer.API, MultipleConcaveHullViewer.API);
         messagerAPI = apiFactory.getAPIAndCloseFactory();
      }
      return messagerAPI;
   }
}
