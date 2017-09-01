package us.ihmc.modelFileLoaders.assimp;

import java.io.IOException;

import javafx.application.Application;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.graphics.JAssImpJavaFXTools;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class JAssImpExample extends Application
{


   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("Jassimp Example");

      View3DFactory view3DFactory = new View3DFactory(1024, 768);
      view3DFactory.addCameraController();
      view3DFactory.addNodeToView(new AmbientLight(Color.WHITE));

      String meshFileName = "sampleMeshes/cinderblock1Meter.obj";


      MeshView[] meshViews = JAssImpJavaFXTools.getJavaFxMeshes(meshFileName);

      for (int i = 0; i < meshViews.length; i++)
      {
         view3DFactory.addNodeToView(meshViews[i]);
      }

      primaryStage.setScene(view3DFactory.getScene());
      primaryStage.show();
   }


   public static void main(String[] args) throws IOException
   {
      launch(args);
   }
}
