package us.ihmc.footstepPlanning.ui;

import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

public class StandaloneFootstepPlannerUI
{
   private final JavaFXMessager messager;


   private final FootstepPathCalculator pathCalculator;

   private final FootstepPlannerUI ui;


   public StandaloneFootstepPlannerUI(Stage primaryStage) throws Exception
   {
      ui = new FootstepPlannerUI(primaryStage);
      messager = ui.getMessager();

      this.pathCalculator = new FootstepPathCalculator(messager);
      pathCalculator.start();
   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      ui.show();
   }

   public void stop()
   {
      ui.stop();
      pathCalculator.stop();
   }
}
