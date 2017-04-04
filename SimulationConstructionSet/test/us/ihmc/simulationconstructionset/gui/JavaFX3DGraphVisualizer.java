package us.ihmc.simulationconstructionset.gui;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFX3DGraph;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class JavaFX3DGraphVisualizer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setWidth(1920);
      primaryStage.setHeight(1080);
      
      DataEntryHolder dataEntryHolder = new DataEntryHolder()
      {
         @Override
         public DataEntry getEntry(YoVariable<?> yoVariable)
         {
            return null;
         }
      };
      TimeDataHolder timeDataHolder = new MinimalTimeDataHolder(200);
      SelectedVariableHolder selectedVariableHolder = new SelectedVariableHolder();
      GraphIndicesHolder graphIndicesHolder = new MinimalGraphIndicesHolder();
      
      JavaFX3DGraph javaFX3DGraph = new JavaFX3DGraph(graphIndicesHolder, selectedVariableHolder, dataEntryHolder, timeDataHolder);
      
      primaryStage.setScene(javaFX3DGraph.getPanel().getScene());
      primaryStage.show();
      
      primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>()
      {
         @Override
         public void handle(WindowEvent event)
         {
            System.exit(0);
         }
      });
   }
   
   public static void main(String[] args)
   {
      launch(args);
   }
}
