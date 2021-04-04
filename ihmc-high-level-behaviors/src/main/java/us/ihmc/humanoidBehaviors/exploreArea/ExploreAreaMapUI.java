package us.ihmc.humanoidBehaviors.exploreArea;

import javafx.animation.AnimationTimer;
import javafx.scene.Scene;
import javafx.scene.control.ScrollPane;
import javafx.scene.text.Font;
import javafx.scene.text.FontPosture;
import javafx.scene.text.Text;
import javafx.scene.text.TextFlow;
import javafx.stage.Stage;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ExploreAreaMapUI
{
   private final ExploreAreaLatticePlanner exploreAreaLatticePlanner;
   private final TextFlow textFlow = new TextFlow();

   private final AtomicReference<List<String>> stringsToProcess = new AtomicReference<>();

   public ExploreAreaMapUI(ExploreAreaLatticePlanner exploreAreaLatticePlanner)
   {
      this.exploreAreaLatticePlanner = exploreAreaLatticePlanner;
      JavaFXApplicationCreator.buildJavaFXApplication(this::build);

      new AnimationTimer()
      {
         @Override
         public void handle(long l)
         {
            if (stringsToProcess.get() != null)
            {
               textFlow.getChildren().clear();
               List<String> stringList = stringsToProcess.getAndSet(null);
               for (int i = 0; i < stringList.size(); i++)
               {
                  Text text = new Text(stringList.get(i) + "\n");
                  text.setFont(Font.font("Courier", FontPosture.REGULAR, 12));
                  textFlow.getChildren().add(text);
               }
            }
         }
      }.start();
   }

   private void build(Stage primaryStage)
   {
      ScrollPane scrollPane = new ScrollPane();
      scrollPane.setContent(textFlow);

      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(scrollPane, 1200, 1200);

      primaryStage.setScene(mainScene);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.show();
   }

   public void update()
   {
      stringsToProcess.set(exploreAreaLatticePlanner.getStringList());
   }
}
