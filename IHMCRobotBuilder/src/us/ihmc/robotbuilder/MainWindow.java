package us.ihmc.robotbuilder;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.Pane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javaslang.control.Option;
import us.ihmc.robotbuilder.model.Loader;
import us.ihmc.robotbuilder.util.Util;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

import java.io.File;

/**
 *
 */
public class MainWindow extends Application {
    @FXML
    private Pane view3D;

    @FXML
    private TreeView<String> treeView;

    private Stage stage;

    @Override
    public void start(Stage primaryStage) throws Exception {
        Parent root = FXMLLoader.load(getClass().getResource("/main_window.fxml"));

        Scene scene = new Scene(root, 300, 275);

        this.stage = primaryStage;
        stage.setTitle("IHMC Robot Builder");
        stage.setScene(scene);
        stage.setMaximized(true);
        stage.show();
    }

    /**
     * File -> Open action
     */
    public void onOpen() {
        FileChooser chooser = new FileChooser();
        chooser.setTitle("Open Robot Definition File");
        if (System.getProperty("initial.dir") != null)
            chooser.setInitialDirectory(new File(System.getProperty("initial.dir")));
        File file = chooser.showOpenDialog(stage);
        if (file == null)
            return;

        Loader.loadFile(file, options -> Util.runLaterInUI(() -> {
            if (options.isEmpty())
                return Option.none();
            if (options.length() == 1)
                return Option.of(options.get(0));

            // Let the user choose a model to load
            ChoiceDialog<String> dialog = new ChoiceDialog<>(options.get(0), options.toJavaArray(String.class));
            dialog.setTitle("Model Selection");
            dialog.setHeaderText("Please choose a model to open.");
            dialog.setContentText("Model:");
            return Option.ofOptional(dialog.showAndWait());
        })).flatMap(robotDescription -> Util.runLaterInUI(() -> {
            robotDescription.peek(this::constructTreeView);
            return null;
        })).onFailure(err -> Util.runLaterInUI(() -> {
            Alert alert = new Alert(AlertType.ERROR, "Error loading file: " + err.getMessage(), ButtonType.OK);
            alert.showAndWait();
            return null;
        }));
    }

    private void constructTreeView(RobotDescription description) {
        TreeItem<String> rootItem = new TreeItem<>(description.getName());
        description.getRootJoints().stream()
                .map(this::constructTreeView)
                .forEach(rootItem.getChildren()::add);
        treeView.setRoot(rootItem);
    }

    private TreeItem<String> constructTreeView(JointDescription description) {
        TreeItem<String> node = new TreeItem<>(description.getName());
        description.getChildrenJoints().stream()
                .map(this::constructTreeView)
                .forEach(node.getChildren()::add);
        return node;
    }

    private void populate3DView(RobotDescription description) {

    }

    private void populate3DView(JointDescription description) {

    }
}
