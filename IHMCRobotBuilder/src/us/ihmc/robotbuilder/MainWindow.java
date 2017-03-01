package us.ihmc.robotbuilder;

import java.io.File;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ChoiceDialog;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javaslang.control.Option;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.cameraControllers.SimpleCameraKeyboardEventHandler;
import us.ihmc.javaFXToolkit.cameraControllers.SimpleCameraMouseEventHandler;
import us.ihmc.robotbuilder.model.Loader;
import us.ihmc.robotbuilder.util.JavaFX3DInstructionExecutor;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotbuilder.util.TreeAdapter;
import us.ihmc.robotbuilder.util.Util;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.RobotDescriptionNode;

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
                TreeAdapter<RobotDescriptionNode> tree = Tree.of(description, RobotDescriptionNode::getChildrenJoints);
                treeView.setRoot(Tree.map(tree, (node, children) -> {
                    TreeItem<String> item = new TreeItem<>(node.getValue().getName());
                    item.getChildren().addAll(children);
                    return item;
                }));
                populate3DView(description);
            });
            return null;
        })).onFailure(err -> Util.runLaterInUI(() -> {
            Alert alert = new Alert(AlertType.ERROR, "Error loading file: " + err.getMessage(), ButtonType.OK);
            alert.showAndWait();
            return null;
        }));
    }

    private void populate3DView(RobotDescription description) {
        Group group = new Group();
        description.getChildrenJoints().stream().map(this::create3DNodes).forEach(group.getChildren()::add);

        SubScene scene3d = new SubScene(group, view3D.getWidth(), view3D.getHeight(), true, SceneAntialiasing.DISABLED);
        scene3d.setFill(Color.BLACK);
        PerspectiveCamera camera = Util.lookAtNodeFromDirection(group, 60, new Vector3D(1, 0, 0), new Vector3D(0, 1, 0));
        scene3d.setCamera(camera);

        SimpleCameraMouseEventHandler mouseController = new SimpleCameraMouseEventHandler(camera);
        SimpleCameraKeyboardEventHandler keyboardController = new SimpleCameraKeyboardEventHandler(camera);
        scene3d.addEventHandler(MouseEvent.ANY, mouseController);
        scene3d.addEventHandler(KeyEvent.ANY, keyboardController);
        scene3d.setFocusTraversable(true);
        scene3d.requestFocus();

        view3D.getChildren().add(scene3d);
    }

    private Node create3DNodes(JointDescription description) {
        TreeAdapter<JointDescription> tree = Tree.of(description, JointDescription::getChildrenJoints);
        return Tree.map(tree, (node, children) -> {
                Group jointGroup = new Group();
                Vector3D offset = new Vector3D();
                node.getValue().getOffsetFromParentJoint(offset);
                jointGroup.getTransforms().add(new Translate(offset.getX(), offset.getY(), offset.getZ()));

                final PhongMaterial redMaterial = new PhongMaterial();
                redMaterial.setDiffuseColor(Color.DARKRED);
                redMaterial.setSpecularColor(Color.RED);

                Sphere graphics = new Sphere(0.05);
                graphics.setMaterial(redMaterial);
                jointGroup.getChildren().add(graphics);

                JavaFX3DInstructionExecutor executor = new JavaFX3DInstructionExecutor(node.getValue().getLink().getLinkGraphics().getGraphics3DInstructions());
                jointGroup.getChildren().add(executor.getResult());

                jointGroup.getChildren().addAll(children);
                return jointGroup;
        });
    }
}
