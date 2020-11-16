package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;

public class BuildingExplorationFootstepVisualizer
{
   private static final double POLYGON_THICKNESS = 0.01;

   private final Group root = new Group();
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final AtomicReference<FootstepDataListMessage> plannedSteps = new AtomicReference<>();
   private final AnimationTimer animationTimer;

   public BuildingExplorationFootstepVisualizer(ROS2Node ros2Node)
   {
      new IHMCROS2Callback<>(ros2Node, TraverseStairsBehaviorAPI.PLANNED_STEPS, plannedSteps::set);
      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            FootstepDataListMessage footstepList = plannedSteps.getAndSet(null);
            if (footstepList == null)
               return;

            meshBuilder.clear();
            root.getChildren().clear();

            for (int i = 0; i < footstepList.getFootstepDataList().size(); i++)
            {
               FootstepDataMessage footstepData = footstepList.getFootstepDataList().get(i);
               Color color = footstepData.getRobotSide() == 0 ? Color.RED : Color.GREEN;
               RigidBodyTransform transform = new RigidBodyTransform(footstepData.getOrientation(), footstepData.getLocation());

               ConvexPolygon2D foothold = new ConvexPolygon2D();
               footstepData.getPredictedContactPoints2d().forEach(p -> foothold.addVertex(p.getX(), p.getY()));
               foothold.update();

               Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
               for (int j = 0; j < vertices.length; j++)
               {
                  vertices[j] = new Point2D(foothold.getVertex(j));
               }

               transform.appendTranslation(0.0, 0.0, 0.5 * POLYGON_THICKNESS);
               meshBuilder.addMultiLine(transform, vertices, POLYGON_THICKNESS, color, true);
               meshBuilder.addPolygon(transform, foothold, color);

               MeshView meshView = new MeshView();
               meshView.setMesh(meshBuilder.generateMesh());
               meshView.setMaterial(meshBuilder.generateMaterial());
               root.getChildren().add(meshView);
            }
         }
      };
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   public Group getRoot()
   {
      return root;
   }
}
