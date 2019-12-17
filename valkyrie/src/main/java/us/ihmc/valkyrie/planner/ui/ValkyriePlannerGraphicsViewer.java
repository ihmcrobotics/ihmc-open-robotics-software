package us.ihmc.valkyrie.planner.ui;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import javafx.animation.AnimationTimer;
import javafx.beans.property.ObjectProperty;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlannerParameters;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class ValkyriePlannerGraphicsViewer extends AnimationTimer
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final ValkyrieAStarFootstepPlannerParameters parameters;

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final List<Point2D> footPoints;

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());
   private final AtomicBoolean reset = new AtomicBoolean();
   private final MeshHolder startSteps = new MeshHolder();
   private final MeshHolder solutionSteps = new MeshHolder();
   private final GoalGraphic goalGraphic;

   public ValkyriePlannerGraphicsViewer(FootstepNodeSnapperReadOnly snapper, ValkyrieAStarFootstepPlannerParameters parameters)
   {
      this.snapper = snapper;
      this.parameters = parameters;

      ValkyriePhysicalProperties physicalProperties = new ValkyriePhysicalProperties();
      double footLength = physicalProperties.getFootLength();
      double footWidth = physicalProperties.getFootWidth();
      footPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      footPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      footPolygon.update();
      footPoints = footPolygon.getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());

      goalGraphic = new GoalGraphic();
   }

   @Override
   public void handle(long now)
   {
      if (reset.getAndSet(false))
      {
         root.getChildren().clear();
         startSteps.clear();
         solutionSteps.clear();
         root.getChildren().add(goalGraphic);
      }

      startSteps.update();
      solutionSteps.update();
      goalGraphic.update();
   }

   public void initialize(ValkyrieFootstepPlanningRequestPacket planningRequestPacket)
   {
      reset.set(true);

      meshBuilder.clear();
      addFootstep(planningRequestPacket.getStartLeftFootPose().getPosition(), planningRequestPacket.getStartLeftFootPose().getOrientation(), Color.DARKGREEN);
      addFootstep(planningRequestPacket.getStartRightFootPose().getPosition(), planningRequestPacket.getStartRightFootPose().getOrientation(), Color.DARKGREEN);
      startSteps.meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   public void processIterationData(AStarIterationData<FootstepNode> iterationData)
   {
      // TODO
   }

   public void processPlanningStatus(ValkyrieFootstepPlanningStatus planningStatus)
   {
      if (planningStatus.getFootstepDataList().getFootstepDataList().isEmpty())
         return;

      meshBuilder.clear();

      for (int i = 0; i < planningStatus.getFootstepDataList().getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = planningStatus.getFootstepDataList().getFootstepDataList().get(i);
         addFootstep(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation(), Color.GREEN);
      }

      solutionSteps.meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   private void addFootstep(Tuple3DReadOnly translation, Orientation3DReadOnly orientation, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform(orientation, translation);
      transform.appendTranslation(0.0, 0.0, 0.0025);
      meshBuilder.addMultiLine(transform, footPoints, 0.01, color, true);
      meshBuilder.addPolygon(transform, footPolygon, color);
   }

   public void reset()
   {
      reset.set(true);
   }

   private class GoalGraphic extends Group
   {
      private final MeshView meshView = new MeshView();
      ObjectProperty<Double> xProperty;
      ObjectProperty<Double> yProperty;
      ObjectProperty<Double> zProperty;
      ObjectProperty<Double> yawProperty;

      GoalGraphic()
      {
         meshBuilder.clear();
         meshBuilder.addSphere(0.05f, Color.INDIANRED);
         meshBuilder.addCylinder(0.15, 0.01, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.INDIANRED);
         meshBuilder.addCone(0.02, 0.02, new Point3D(0.15, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.INDIANRED);

         addFootstep(new Vector3D(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0), new Quaternion(), Color.INDIANRED);
         addFootstep(new Vector3D(0.0, -0.5 * parameters.getIdealFootstepWidth(), 0.0), new Quaternion(), Color.INDIANRED);

         meshView.setMesh(meshBuilder.generateMesh());
         meshView.setMaterial(meshBuilder.generateMaterial());
         meshView.setMouseTransparent(true);
         getChildren().add(meshView);

         setVisible(false);
         root.getChildren().add(this);
      }

      void update()
      {
         setVisible(true);
         setTranslateX(xProperty.getValue());
         setTranslateY(yProperty.getValue());
         setTranslateZ(zProperty.getValue());
         setRotate(Math.toDegrees(yawProperty.getValue()));
      }
   }

   public void setGoalPoseProperties(ObjectProperty<Double> xProperty,
                                     ObjectProperty<Double> yProperty,
                                     ObjectProperty<Double> zProperty,
                                     ObjectProperty<Double> yawProperty)
   {
      goalGraphic.xProperty = xProperty;
      goalGraphic.yProperty = yProperty;
      goalGraphic.zProperty = zProperty;
      goalGraphic.yawProperty = yawProperty;
   }

   private class MeshHolder
   {
      final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
      final MeshView meshView = new MeshView();
      boolean addedFlag = false;

      void update()
      {
         Pair<Mesh, Material> mesh = meshReference.getAndSet(null);
         if (mesh != null)
         {
            if(!addedFlag)
            {
               root.getChildren().add(meshView);
               addedFlag = true;
            }

            meshView.setMesh(mesh.getKey());
            meshView.setMaterial(mesh.getValue());
         }
      }

      void clear()
      {
         meshView.setMesh(null);
         meshView.setMaterial(null);
         addedFlag = false;
      }
   }

   public Group getRoot()
   {
      return root;
   }
}
