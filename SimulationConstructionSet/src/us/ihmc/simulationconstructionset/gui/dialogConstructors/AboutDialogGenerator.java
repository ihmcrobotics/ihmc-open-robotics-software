package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.TextArea;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;

public class AboutDialogGenerator implements AboutDialogConstructor
{
   // TODO: remove parentJFrame and all uses of it
   private JFrame parentJFrame;

   public AboutDialogGenerator(JFrame parentJFrame)
   {
      this.parentJFrame = parentJFrame;
   }

   @Override
   public void constructDialog()
   {
      String scsVersionNumber = SimulationConstructionSet.getVersion();

      // Must run on JavaFX application thread - use Platform.runLater()
      // TODO: make prettier :)
      Platform.runLater(() ->{
         Pane pane = new Pane();
         Scene scene = new Scene(pane);
         Stage about = new Stage();
         about.setScene(scene);

         TextArea ta = new TextArea();
         ta.setText(
                 "SimulationConstructionSet.\n\n " +
                         "Originally developed at Yobotics, Inc. from 2000-2010.\n\n " +
                         "Now developed at IHMC from 2002 to the present.\n\n " +
                         "Website: https://www.ihmc.us/simulationconstructionset/"
         );
         ta.setEditable(false);
         ta.setWrapText(true);
         ta.setPrefSize(500,200);

         pane.getChildren().add(ta);
         pane.setPrefSize(500,200);

         about.setResizable(false);
         about.show();
      });
   }

   public void closeAndDispose()
   {
      parentJFrame = null;
   }

}
