package us.ihmc.footstepPlanning.ui.viewers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.RADIUS;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalOpaqueMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalTransparentMaterial;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class GoalOrientationViewer extends AnimationTimer
{
   private final Group root = new Group();

   private static final double cylinderLength = 5.0 * RADIUS;
   private final ArrowGraphic lowLevelGoalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.DARKRED);
   private final ArrowGraphic goalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.RED);
   private final SideDependentList<FootGraphic> footGraphics = new SideDependentList<>();

   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private SideDependentList<Material> transparentFootMaterial = new SideDependentList<>();
   private SideDependentList<Material> opaqueFootMaterial = new SideDependentList<>();

   private FootstepPlannerParametersReadOnly parameters;
   private final AtomicReference<Boolean> goalRotationEditModeEnabled;
   private final AtomicReference<Boolean> goalPositionEditModeEnabled;

   private final AtomicReference<Point3D> lowLevelGoalPositionReference;
   private final AtomicReference<Quaternion> lowLevelGoalQuaternionReference;

   private final AtomicReference<Point3D> goalMidFootPosition;
   private final AtomicReference<Quaternion> goalMidFootOrientation;
   private final AtomicReference<Pose3DReadOnly> leftFootGoalPose;
   private final AtomicReference<Pose3DReadOnly> rightFootGoalPose;

   public GoalOrientationViewer(Messager messager)
   {
      lowLevelGoalArrow.setMouseTransparent(true);
      goalArrow.setMouseTransparent(true);

      root.getChildren().add(lowLevelGoalArrow);
      root.getChildren().add(goalArrow);

      goalRotationEditModeEnabled = messager.createInput(GoalOrientationEditModeEnabled, false);
      goalPositionEditModeEnabled = messager.createInput(GoalPositionEditModeEnabled, false);

      lowLevelGoalPositionReference = messager.createInput(LowLevelGoalPosition, new Point3D());
      lowLevelGoalQuaternionReference = messager.createInput(LowLevelGoalOrientation, new Quaternion());

      goalMidFootPosition = messager.createInput(GoalMidFootPosition, new Point3D());
      goalMidFootOrientation = messager.createInput(GoalMidFootOrientation, new Quaternion());
      leftFootGoalPose = messager.createInput(LeftFootGoalPose);
      rightFootGoalPose = messager.createInput(RightFootGoalPose);

      messager.addTopicListener(ShowGoal, root::setVisible);
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public void handle(long now)
   {
      if (goalRotationEditModeEnabled.get())
         goalArrow.setMaterial(goalTransparentMaterial);
      else
         goalArrow.setMaterial(goalOpaqueMaterial);

      if (!footGraphics.isEmpty())
      {
         if (goalPositionEditModeEnabled.get() || goalRotationEditModeEnabled.get())
         {
            for (RobotSide robotSide : RobotSide.values)
               footGraphics.get(robotSide).setMaterial(transparentFootMaterial.get(robotSide));
         }
         else
         {
            for (RobotSide robotSide : RobotSide.values)
               footGraphics.get(robotSide).setMaterial(opaqueFootMaterial.get(robotSide));
         }
      }

      Point3D goalPosition = goalMidFootPosition.get();
      if (goalPosition != null)
      {
         setArrowPose(goalArrow, goalPosition, goalMidFootOrientation.get().getYaw());
      }

      Point3D lowLevelGoalPosition = lowLevelGoalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(lowLevelGoalArrow, lowLevelGoalPosition, lowLevelGoalQuaternionReference.get().getYaw());
      }

      if (!footGraphics.isEmpty())
      {
         if (leftFootGoalPose.get() != null)
         {
            setFootPose(footGraphics.get(RobotSide.LEFT), leftFootGoalPose.getAndSet(null));
         }

         if (rightFootGoalPose.get() != null)
         {
            setFootPose(footGraphics.get(RobotSide.RIGHT), rightFootGoalPose.getAndSet(null));
         }
      }
   }

   private static void setArrowPose(ArrowGraphic arrow, Point3D position, double orientationRadians)
   {
      arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * (Math.cos(orientationRadians) - 1.0));
      arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
      arrow.setTranslateZ(position.getZ());
      arrow.setRotate(Math.toDegrees(orientationRadians));
   }

   private static void setFootPose(FootGraphic foot, Pose3DReadOnly footPose)
   {
      foot.setTranslateX(footPose.getX());
      foot.setTranslateY(footPose.getY());
      foot.setTranslateZ(footPose.getZ());
      foot.setRotate(Math.toDegrees(footPose.getYaw()));
   }

   public void setDefaultContactPoints(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         for (int i = 0; i < defaultContactPoints.get(robotSide).size(); i++)
         {
            defaultFoothold.addVertex(defaultContactPoints.get(robotSide).get(i));
         }

         defaultFoothold.update();
         this.defaultContactPoints.put(robotSide, defaultFoothold);


         FootGraphic opaqueFootGraphic = new FootGraphic(defaultFoothold, toTransparentColor(getFootstepColor(robotSide), 0.8));
         FootGraphic transparentFootGraphic = new FootGraphic(defaultFoothold, toTransparentColor(getFootstepColor(robotSide), 0.4));

         transparentFootMaterial.put(robotSide, transparentFootGraphic.getMaterial());
         opaqueFootMaterial.put(robotSide, opaqueFootGraphic.getMaterial());

         footGraphics.put(robotSide, opaqueFootGraphic);
         root.getChildren().add(opaqueFootGraphic);
      }
   }


   private Color getFootstepColor(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? Color.RED : Color.GREEN;
   }

   private class ArrowGraphic extends Group
   {
      private final MeshView arrow;

      public ArrowGraphic(double radius, double length, Color color)
      {
         TextureColorPalette1D colorPalette = new TextureColorPalette1D();
         colorPalette.setHueBased(1.0, 1.0);
         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

         double coneHeight = 0.10 * length;
         double coneRadius = 1.5 * radius;

         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

         this.arrow = new MeshView(meshBuilder.generateMesh());
         arrow.setMaterial(meshBuilder.generateMaterial());
         getChildren().add(arrow);
      }

      public void setMaterial(PhongMaterial material)
      {
         arrow.setMaterial(material);
      }
   }

   private class FootGraphic extends Group
   {
      private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
      private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
      private final MeshView foot = new MeshView();
      private final Material material;

      public FootGraphic(ConvexPolygon2D foot, Color color)
      {
         Point3D[] vertices = new Point3D[foot.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
            vertices[j] = new Point3D(foot.getVertex(j));
         meshBuilder.addMultiLine(vertices, 0.01, color, true);

         material = meshBuilder.generateMaterial();
         this.foot.setMesh(meshBuilder.generateMesh());
         this.foot.setMaterial(material);
         getChildren().add(this.foot);
      }

      public Material getMaterial()
      {
         return material;
      }

      public void setMaterial(Material material)
      {
         foot.setMaterial(material);
      }
   }

   public static Color toTransparentColor(Color opaqueColor, double opacity)
   {
      double red = opaqueColor.getRed();
      double green = opaqueColor.getGreen();
      double blue = opaqueColor.getBlue();
      return new Color(red, green, blue, opacity);
   }

   public Node getRoot()
   {
      return root;
   }
}
