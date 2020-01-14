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
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlannerParameters;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class ValkyriePlannerGraphicsViewer extends AnimationTimer
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final ValkyrieAStarFootstepPlannerParameters parameters;

   private final ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D();
   private final List<Point2D> defaultFootPoints;

   private final AtomicInteger waypointIndex = new AtomicInteger(-1);
   private ObjectProperty<Double> xGoalProperty, yGoalProperty, zGoalProperty, yawGoalProperty;
   private ObjectProperty<Double> xWaypointProperty, yWaypointProperty, zWaypointProperty, yawWaypointProperty;

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());
   private final AtomicBoolean reset = new AtomicBoolean();
   private final MeshHolder startSteps = new MeshHolder();
   private final MeshHolder solutionSteps = new MeshHolder();
   private final MeshHolder debugParentStepGraphic = new MeshHolder();
   private final MeshHolder debugChildStepGraphic = new MeshHolder();
   private final MeshHolder debugIdealStepGraphic = new MeshHolder();
   private final FootstepPairGraphic goalGraphic;
   private final FootstepPairGraphic[] waypointGraphics = new FootstepPairGraphic[20];

   private final AtomicReference<Pair<RigidBodyTransform, ConvexPolygon2D>> debugParentStep;
   private final AtomicReference<Pair<RigidBodyTransform, ConvexPolygon2D>> debugChildStep;
   private final AtomicReference<RigidBodyTransform> debugIdealStep;

   public ValkyriePlannerGraphicsViewer(FootstepNodeSnapperReadOnly snapper, ValkyrieAStarFootstepPlannerParameters parameters, Messager messager)
   {
      this.snapper = snapper;
      this.parameters = parameters;

      debugParentStep = messager.createInput(ValkyriePlannerMessagerAPI.parentDebugStep);
      debugChildStep = messager.createInput(ValkyriePlannerMessagerAPI.childDebugStep);
      debugIdealStep = messager.createInput(ValkyriePlannerMessagerAPI.idealDebugStep);

      ValkyriePhysicalProperties physicalProperties = new ValkyriePhysicalProperties();
      double footLength = physicalProperties.getFootLength();
      double footWidth = physicalProperties.getFootWidth();
      defaultFootPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      defaultFootPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      defaultFootPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      defaultFootPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      defaultFootPolygon.update();
      defaultFootPoints = defaultFootPolygon.getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());

      goalGraphic = new FootstepPairGraphic(Color.INDIANRED);
      for (int i = 0; i < waypointGraphics.length; i++)
      {
         waypointGraphics[i] = new FootstepPairGraphic(Color.GRAY);
      }

      messager.registerTopicListener(ValkyriePlannerMessagerAPI.addWaypoint, addWaypoint -> addWaypoint());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.clearWaypoints, clear -> clearWaypoints());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.dataSetSelected, clear -> reset());
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
         root.getChildren().addAll(waypointGraphics);
      }

      startSteps.update();
      solutionSteps.update();

      goalGraphic.setVisible(true);
      goalGraphic.setTranslateX(xGoalProperty.getValue());
      goalGraphic.setTranslateY(yGoalProperty.getValue());
      goalGraphic.setTranslateZ(zGoalProperty.getValue());
      goalGraphic.setRotate(Math.toDegrees(yawGoalProperty.getValue()));

      for (int i = 0; i < waypointGraphics.length; i++)
      {
         waypointGraphics[i].setVisible(i <= waypointIndex.get());
         if (i == waypointIndex.get())
         {
            waypointGraphics[i].setTranslateX(xWaypointProperty.getValue());
            waypointGraphics[i].setTranslateY(yWaypointProperty.getValue());
            waypointGraphics[i].setTranslateZ(zWaypointProperty.getValue());
            waypointGraphics[i].setRotate(Math.toDegrees(yawWaypointProperty.getValue()));
         }
      }

      Pair<RigidBodyTransform, ConvexPolygon2D> debugParentStep = this.debugParentStep.getAndSet(null);
      Pair<RigidBodyTransform, ConvexPolygon2D> debugChildStep = this.debugChildStep.getAndSet(null);
      RigidBodyTransform debugIdealStep = this.debugIdealStep.getAndSet(null);

      if (debugParentStep != null)
      {
         meshBuilder.clear();
         List<Point2D> footPoints = debugParentStep.getValue().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         if(footPoints.isEmpty())
            addFootstep(debugParentStep.getKey().getTranslation(), debugParentStep.getKey().getRotation(), defaultFootPoints, defaultFootPolygon, Color.GREEN);
         else
            addFootstep(debugParentStep.getKey().getTranslation(), debugParentStep.getKey().getRotation(), footPoints, debugParentStep.getValue(), Color.GREEN);

         this.debugParentStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugParentStepGraphic.update();

         meshBuilder.clear();
         this.debugChildStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugChildStepGraphic.update();
      }

      if (debugChildStep != null)
      {
         meshBuilder.clear();
         List<Point2D> footPolygon = debugChildStep.getValue().getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
         if (footPolygon.isEmpty() || debugChildStep.getValue().containsNaN())
            addFootstep(debugChildStep.getKey().getTranslation(), debugChildStep.getKey().getRotation(), defaultFootPoints, defaultFootPolygon, Color.RED);
         else
            addFootstep(debugChildStep.getKey().getTranslation(), debugChildStep.getKey().getRotation(), footPolygon, debugChildStep.getValue(), Color.RED);
         this.debugChildStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugChildStepGraphic.update();
      }

      if (debugIdealStep != null)
      {
         meshBuilder.clear();
         addFootstep(debugIdealStep.getTranslation(), debugIdealStep.getRotation(), defaultFootPoints, defaultFootPolygon, Color.BLUE);
         this.debugIdealStepGraphic.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
         debugIdealStepGraphic.update();
      }
   }

   public void initialize(ValkyrieFootstepPlanningRequestPacket planningRequestPacket)
   {
      reset.set(true);

      meshBuilder.clear();
      addFootstep(planningRequestPacket.getStartLeftFootPose().getPosition(),
                  planningRequestPacket.getStartLeftFootPose().getOrientation(),
                  defaultFootPoints,
                  defaultFootPolygon,
                  Color.DARKGREEN);
      addFootstep(planningRequestPacket.getStartRightFootPose().getPosition(),
                  planningRequestPacket.getStartRightFootPose().getOrientation(),
                  defaultFootPoints,
                  defaultFootPolygon,
                  Color.DARKGREEN);
      startSteps.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
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
         addFootstep(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation(), defaultFootPoints, defaultFootPolygon, Color.GREEN);
      }

      solutionSteps.meshReference.set(Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   private void addFootstep(Tuple3DReadOnly translation, Orientation3DReadOnly orientation, List<Point2D> footPoints, ConvexPolygon2D footPolygon, Color color)
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

   class FootstepPairGraphic extends Group
   {
      private final MeshView meshView = new MeshView();

      FootstepPairGraphic(Color color)
      {
         meshBuilder.clear();
         meshBuilder.addSphere(0.05f, color);
         meshBuilder.addCylinder(0.15, 0.01, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(0.02, 0.02, new Point3D(0.15, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

         addFootstep(new Vector3D(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0), new Quaternion(), defaultFootPoints, defaultFootPolygon, color);
         addFootstep(new Vector3D(0.0, -0.5 * parameters.getIdealFootstepWidth(), 0.0), new Quaternion(), defaultFootPoints, defaultFootPolygon, color);

         meshView.setMesh(meshBuilder.generateMesh());
         meshView.setMaterial(meshBuilder.generateMaterial());
         meshView.setMouseTransparent(true);
         getChildren().add(meshView);

         setVisible(false);
         root.getChildren().add(this);
      }

      boolean addedToRoot = true;

      void addToRoot(boolean addToRoot)
      {
         if(addToRoot && !addedToRoot)
         {
            root.getChildren().add(this);
         }
         else if(!addToRoot && addedToRoot)
         {
            root.getChildren().remove(this);
         }
      }
   }

   public void setGoalPoseProperties(ObjectProperty<Double> xProperty,
                                     ObjectProperty<Double> yProperty,
                                     ObjectProperty<Double> zProperty,
                                     ObjectProperty<Double> yawProperty)
   {
      this.xGoalProperty = xProperty;
      this.yGoalProperty = yProperty;
      this.zGoalProperty = zProperty;
      this.yawGoalProperty = yawProperty;
   }

   public void setWaypointPoseProperties(ObjectProperty<Double> xProperty, ObjectProperty<Double> yProperty, ObjectProperty<Double> zProperty, ObjectProperty<Double> yawProperty)
   {
      this.xWaypointProperty = xProperty;
      this.yWaypointProperty = yProperty;
      this.zWaypointProperty = zProperty;
      this.yawWaypointProperty = yawProperty;
   }

   public void addWaypoint()
   {
      if (waypointIndex.get() <= waypointGraphics.length - 2)
         waypointIndex.incrementAndGet();
   }

   public void clearWaypoints()
   {
      waypointIndex.set(-1);
   }

   public void packWaypoints(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      if (waypointIndex.get() < 0)
         return;

      int numberOfWaypoints = waypointIndex.get() + 1;
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         Pose3D waypoint = requestPacket.getWaypoints().add();
         waypoint.setX(waypointGraphics[i].getTranslateX());
         waypoint.setY(waypointGraphics[i].getTranslateY());
         waypoint.setZ(waypointGraphics[i].getTranslateZ());
         waypoint.setOrientationYawPitchRoll(Math.toRadians(waypointGraphics[i].getRotate()), 0.0, 0.0);
      }
   }

   void showPath(boolean show)
   {
      startSteps.show(show);
      solutionSteps.show(show);
      goalGraphic.addToRoot(show);

      for (int i = 0; i < waypointGraphics.length; i++)
      {
         waypointGraphics[i].addToRoot(show);
      }
   }

   void showDebugSteps(boolean show)
   {
      debugParentStepGraphic.show(show);
      debugChildStepGraphic.show(show);
   }

   class MeshHolder
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

      void show(boolean show)
      {
         meshView.setVisible(show);
      }
   }

   public Group getRoot()
   {
      return root;
   }
}
