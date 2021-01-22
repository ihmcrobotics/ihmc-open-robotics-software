package us.ihmc.humanoidBehaviors.ui.simulation;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;

public class RobotAndMapViewer
{
   private FootstepPlanGraphic footstepPlanGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private PositionGraphic goalGraphic;
   private Group markers = new Group();

   public RobotAndMapViewer(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      Platform.runLater(() ->
      {
         View3DFactory view3dFactory = new View3DFactory(1200, 800);
         FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
         double isoZoomOut = 10.0;
         camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();

         view3dFactory.addNodeToView(new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, false));
//         view3dFactory.addNodeToView(new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.REA, false));
         view3dFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

         footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
         view3dFactory.addNodeToView(footstepPlanGraphic);

         bodyPathPlanGraphic = new BodyPathPlanGraphic();
         view3dFactory.addNodeToView(bodyPathPlanGraphic);

         goalGraphic = new PositionGraphic(Color.RED, 0.07);
         Platform.runLater(() -> goalGraphic.clear());
         view3dFactory.addNodeToView(goalGraphic.getNode());

         view3dFactory.addNodeToView(markers);

         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         primaryStage.setScene(view3dFactory.getScene());

         primaryStage.show();
      });
   }

   public void setGoalLocation(Point3DReadOnly goalLocation)
   {
      Platform.runLater(() -> goalGraphic.setPosition(goalLocation));
   }

   public void addMarker(Point3DReadOnly position)
   {
      Platform.runLater(() ->
      {
         PositionGraphic graphic = new PositionGraphic(Color.GRAY, 0.06);
         graphic.setPosition(position);
         markers.getChildren().add(graphic.getNode());
      });
   }

   public void setFootstepsToVisualize(FootstepPlan footstepPlan)
   {
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getFootstepPose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertPairListToMinimalFoostepList(footstepLocations));
   }

   public void setBodyPathPlanToVisualize(List<? extends Pose3DReadOnly> bodyPath)
   {
      bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPath);
   }
}
