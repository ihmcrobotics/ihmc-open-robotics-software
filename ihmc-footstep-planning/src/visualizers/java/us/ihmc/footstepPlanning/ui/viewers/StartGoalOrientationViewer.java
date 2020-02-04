package us.ihmc.footstepPlanning.ui.viewers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.RADIUS;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalOpaqueMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalTransparentMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.startOpaqueMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.startTransparentMaterial;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class StartGoalOrientationViewer extends AnimationTimer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Group root = new Group();

   private static final double cylinderLength = 5.0 * RADIUS;
   private final ArrowGraphic startArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.GREEN);
   private final ArrowGraphic lowLevelGoalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.DARKRED);
   private final ArrowGraphic goalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.RED);
   private final SideDependentList<FootGraphic> footGraphics = new SideDependentList<>();

   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private SideDependentList<Material> transparentFootMaterial = new SideDependentList<>();
   private SideDependentList<Material> opaqueFootMaterial = new SideDependentList<>();

   private FootstepPlannerParametersReadOnly parameters;
   private final AtomicReference<Boolean> startRotationEditModeEnabled;
   private final AtomicReference<Boolean> goalRotationEditModeEnabled;
   private final AtomicReference<Boolean> goalPositionEditModeEnabled;

   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> lowLevelGoalPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicReference<Quaternion> startQuaternionReference;
   private final AtomicReference<Quaternion> lowLevelGoalQuaternionReference;
   private final AtomicReference<Quaternion> goalQuaternionReference;

   private final PoseReferenceFrame goalFrame = new PoseReferenceFrame("goalFrame", ReferenceFrame.getWorldFrame());

   public StartGoalOrientationViewer(Messager messager)
   {
      startArrow.setMouseTransparent(true);
      lowLevelGoalArrow.setMouseTransparent(true);
      goalArrow.setMouseTransparent(true);

      root.getChildren().add(startArrow);
      root.getChildren().add(lowLevelGoalArrow);
      root.getChildren().add(goalArrow);

      startRotationEditModeEnabled = messager.createInput(StartOrientationEditModeEnabled, false);
      goalRotationEditModeEnabled = messager.createInput(GoalOrientationEditModeEnabled, false);
      goalPositionEditModeEnabled = messager.createInput(GoalPositionEditModeEnabled, false);

      startPositionReference = messager.createInput(StartPosition, new Point3D());
      lowLevelGoalPositionReference = messager.createInput(LowLevelGoalPosition, new Point3D());
      goalPositionReference = messager.createInput(GoalPosition, new Point3D());

      startQuaternionReference = messager.createInput(StartOrientation, new Quaternion());
      lowLevelGoalQuaternionReference = messager.createInput(LowLevelGoalOrientation, new Quaternion());
      goalQuaternionReference = messager.createInput(GoalOrientation, new Quaternion());
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.parameters = parameters;
   }

   private void updateGoalFrame(Point3D goalPosition)
   {
      goalFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, goalPosition));
      goalFrame.setOrientationAndUpdate(goalQuaternionReference.get());

      if (footGraphics.isEmpty())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         double width = parameters.getIdealFootstepWidth() / 2.0;
         FramePoint3D footPose = new FramePoint3D(goalFrame, 0.0, robotSide.negateIfRightSide(width), 0.0);
         footPose.changeFrame(worldFrame);
         setFootPose(footGraphics.get(robotSide), footPose, goalQuaternionReference.get().getYaw());
      }
   }

   @Override
   public void handle(long now)
   {
      if (startRotationEditModeEnabled.get())
         startArrow.setMaterial(startTransparentMaterial);
      else
         startArrow.setMaterial(startOpaqueMaterial);

      Point3D startPosition = startPositionReference.get();
      if (startPosition != null)
      {
         setArrowPose(startArrow, startPosition, startQuaternionReference.get().getYaw());
      }

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

      Point3D goalPosition = goalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(goalArrow, goalPosition, goalQuaternionReference.get().getYaw());
      }

      Point3D lowLevelGoalPosition = lowLevelGoalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(lowLevelGoalArrow, lowLevelGoalPosition, lowLevelGoalQuaternionReference.get().getYaw());
      }

      if (goalPosition != null)
      {
         updateGoalFrame(goalPosition);
      }
   }

   private static void setArrowPose(ArrowGraphic arrow, Point3D position, double orientationRadians)
   {
      arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * (Math.cos(orientationRadians) - 1.0));
      arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
      arrow.setTranslateZ(position.getZ());
      arrow.setRotate(Math.toDegrees(orientationRadians));
   }

   private static void setFootPose(FootGraphic foot, Point3DReadOnly position, double orienationInRadians)
   {
      foot.setTranslateX(position.getX());
      foot.setTranslateY(position.getY());
      foot.setTranslateZ(position.getZ());
      foot.setRotate(Math.toDegrees(orienationInRadians));
   }

   public void setDefaultContactPoints(RobotContactPointParameters<RobotSide> defaultContactPointParameters)
   {
      SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints = defaultContactPointParameters.getControllerFootGroundContactPoints();
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         ArrayList<Point2D> defaultContactPoints = controllerFootGroundContactPoints.get(robotSide);
         for (int i = 0; i < defaultContactPoints.size(); i++)
         {
            defaultFoothold.addVertex(defaultContactPoints.get(i));
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
