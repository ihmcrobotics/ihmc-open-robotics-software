package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanWithTextGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

import java.util.ArrayList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepRemoteVisualizer
{
   private FootstepPlanWithTextGraphic footstepPlanGraphic;
   private FootstepPlanWithTextGraphic startAndGoalFootPoses;
   private PoseGraphic closestPointAlongPathGraphic;
   private PoseGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private LivePlanarRegionsGraphic planarRegionsGraphic;
   private PoseGraphic goalGraphic;

   private Stage primaryStage;

   public LookAndStepRemoteVisualizer(DRCRobotModel robotModel, Ros2Node ros2Node, Messager behaviorMessager)
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

         view3dFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

         startAndGoalFootPoses = new FootstepPlanWithTextGraphic();
         behaviorMessager.registerTopicListener(StartAndGoalFootPosesForUI, startAndGoalFootPoses::generateMeshesAsynchronously);
         footstepPlanGraphic = new FootstepPlanWithTextGraphic();
         behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

         planarRegionsGraphic = new LivePlanarRegionsGraphic(false);
         behaviorMessager.registerTopicListener(PlanarRegionsForUI, planarRegions -> {
            planarRegionsGraphic.acceptPlanarRegions(planarRegions);
         });

         goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

         closestPointAlongPathGraphic = new PoseGraphic("Closest", Color.BLUE, 0.027);
         behaviorMessager.registerTopicListener(ClosestPointForUI, pose -> Platform.runLater(() -> closestPointAlongPathGraphic.setPose(pose)));
         subGoalGraphic = new PoseGraphic("Sub goal", Color.YELLOW, 0.027);
         behaviorMessager.registerTopicListener(SubGoalForUI, pose -> Platform.runLater(() -> subGoalGraphic.setPose(pose)));

         bodyPathPlanGraphic = new BodyPathPlanGraphic();
         behaviorMessager.registerTopicListener(BodyPathPlanForUI, bodyPathPlan ->
         {
            ArrayList<Point3DReadOnly> bodyPathAsPoints = new ArrayList<>();
            for (Pose3D pose3D : bodyPathPlan)
            {
               bodyPathAsPoints.add(pose3D.getPosition());
            }
            Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathAsPoints));
         });

         view3dFactory.addNodeToView(footstepPlanGraphic);
         view3dFactory.addNodeToView(bodyPathPlanGraphic);
         view3dFactory.addNodeToView(closestPointAlongPathGraphic);
         view3dFactory.addNodeToView(subGoalGraphic);
         view3dFactory.addNodeToView(goalGraphic);
         view3dFactory.addNodeToView(planarRegionsGraphic);
         view3dFactory.addNodeToView(startAndGoalFootPoses);

         primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         primaryStage.setScene(view3dFactory.getScene());

         primaryStage.show();
      });
   }

   public void close()
   {
      primaryStage.close();
   }
}
