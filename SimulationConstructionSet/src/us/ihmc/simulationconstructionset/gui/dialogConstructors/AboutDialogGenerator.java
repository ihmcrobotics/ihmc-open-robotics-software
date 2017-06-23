package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.HPos;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Hyperlink;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.text.Font;
import javafx.stage.Stage;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;

public class AboutDialogGenerator implements AboutDialogConstructor, EventHandler<ActionEvent> {
    // TODO: remove parentJFrame and all uses of it
    private JFrame parentJFrame;

    public AboutDialogGenerator(JFrame parentJFrame) {
        this.parentJFrame = parentJFrame;
    }

    @Override
    public void constructDialog() {
        String scsVersionNumber = SimulationConstructionSet.getVersion();

        // Must run on JavaFX application thread - use Platform.runLater()
        Platform.runLater(() -> {
            VBox wrapper = new VBox();
            /*wrapper.setBackground(new Background(
                    new BackgroundFill(
                            Color.rgb(26,26,26),
                            CornerRadii.EMPTY,
                            null
                    )
            ));*/
            VBox pane = new VBox();
            pane.setAlignment(Pos.CENTER);

            Image banner = new Image(getClass().getClassLoader().getResourceAsStream("Banner-v1-Transparent-Inverted.png"));
            ImageView view = new ImageView(banner);
            view.setPreserveRatio(true);
            view.setFitHeight(200);

            Label version = new Label("Version "+scsVersionNumber);
            //version.setTextFill(Color.WHITE);
            version.setFont(new Font("DTL Nobel", 32));

            Hyperlink github = new Hyperlink("https://ihmcrobotics.github.io/");
            github.addEventHandler(ActionEvent.ACTION, this);
            github.setFont(new Font("DTL Nobel", 30));
            github.setBorder(null);

            Label yobotics = new Label("Originally developed at Yobotics, Inc. from 2000-2010.");
            //yobotics.setTextFill(Color.WHITE);
            yobotics.setFont(new Font("DTL Nobel", 20));
            yobotics.setWrapText(true);
            VBox.setMargin(yobotics, new Insets(20,0,20,0));

            Label ihmc = new Label("Now developed at IHMC from 2002 to the present.");
            //ihmc.setTextFill(Color.WHITE);
            ihmc.setFont(new Font("DTL Nobel", 20));
            ihmc.setWrapText(true);
            VBox.setMargin(ihmc, new Insets(0,0,20,0));

            pane.getChildren().addAll(
                    view,
                    version,
                    github,
                    yobotics,
                    ihmc
            );

            VBox.setMargin(pane, new Insets(0,10,0,10));

            wrapper.getChildren().add(pane);

            Scene scene = new Scene(wrapper);
            Stage about = new Stage();
            about.setScene(scene);
            about.setTitle("About");

            about.setResizable(false);
            about.show();
        });
    }

    public void closeAndDispose() {
        parentJFrame = null;
    }

    @Override
    public void handle(ActionEvent actionEvent) {
        // do hyperlink
    }
}
