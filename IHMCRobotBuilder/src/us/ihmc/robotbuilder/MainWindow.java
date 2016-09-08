package us.ihmc.robotbuilder;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javaslang.control.Option;
import us.ihmc.javaFXToolkit.cameraControllers.SimpleCameraMouseEventHandler;
import us.ihmc.robotbuilder.model.Loader;
import us.ihmc.robotbuilder.util.Util;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.RobotDescriptionNode;

import javax.vecmath.Vector3d;
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
            robotDescription.peek(description -> {
                treeView.setRoot(constructTreeView(description));
                populate3DView(description);
            });
            return null;
        })).onFailure(err -> Util.runLaterInUI(() -> {
            Alert alert = new Alert(AlertType.ERROR, "Error loading file: " + err.getMessage(), ButtonType.OK);
            alert.showAndWait();
            return null;
        }));
    }

    private TreeItem<String> constructTreeView(RobotDescriptionNode description) {
        TreeItem<String> rootItem = new TreeItem<>(description.getName());
        description.getChildrenJoints().stream()
                .map(this::constructTreeView)
                .forEach(rootItem.getChildren()::add);
        return rootItem;
    }

    private void populate3DView(RobotDescription description) {
        Group group = new Group();
        description.getChildrenJoints().stream().map(this::create3DNodes).forEach(group.getChildren()::add);
        AmbientLight light = new AmbientLight(Color.WHITE);
        group.getChildren().add(light);

        SubScene scene3d = new SubScene(group, view3D.getWidth(), view3D.getHeight(), true, SceneAntialiasing.DISABLED);
        scene3d.setFill(Color.BLACK);
        PerspectiveCamera camera = Util.lookAtNodeFromDirection(group, 60, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        scene3d.setCamera(camera);

        //FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(scene3d.widthProperty(), scene3d.heightProperty(), camera, new Vector3d(0, 1, 0));
        //scene3d.addEventHandler(Event.ANY, cameraController);
        SimpleCameraMouseEventHandler cameraController = new SimpleCameraMouseEventHandler(camera);
        scene3d.addEventHandler(MouseEvent.ANY, cameraController);

        view3D.getChildren().add(scene3d);
    }

    private Node create3DNodes(JointDescription description) {
        Group jointGroup = new Group();
        Vector3d offset = new Vector3d();
        description.getOffsetFromParentJoint(offset);
        jointGroup.getTransforms().add(new Translate(offset.x, offset.y, offset.z));

        final PhongMaterial redMaterial = new PhongMaterial();
        redMaterial.setDiffuseColor(Color.DARKRED);
        redMaterial.setSpecularColor(Color.RED);

        Sphere graphics = new Sphere(0.01);
        graphics.setMaterial(redMaterial);
        jointGroup.getChildren().add(graphics);

        description.getChildrenJoints().stream().map(this::create3DNodes).forEach(jointGroup.getChildren()::add);
        return jointGroup;
    }
}
