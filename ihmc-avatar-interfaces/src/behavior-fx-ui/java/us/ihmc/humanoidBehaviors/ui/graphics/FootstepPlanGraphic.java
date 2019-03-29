package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.animation.AnimationTimer;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class FootstepPlanGraphic extends AnimationTimer
{
   private final MeshView meshView = new MeshView();
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;

   public FootstepPlanGraphic(DRCRobotModel robotModel)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         robotModel.getContactPointParameters().getControllerFootGroundContactPoints().get(robotSide).forEach(point2D -> defaultFoothold.addVertex(point2D));
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   public void generateMeshes(ArrayList<Pair<RobotSide, Pose3D>> message)
   {
      meshBuilder.clear();

      FramePose3D footPose = new FramePose3D();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      FootstepPlan plan = new FootstepPlan();
      message.forEach(pair -> plan.addFootstep(pair.getLeft(), new FramePose3D(pair.getRight())));

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         plan.getFootstep(i).setFoothold(defaultContactPoints.get(plan.getFootstep(i).getRobotSide()));
      }

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         Color regionColor = footstep.getRobotSide() == RobotSide.LEFT ? Color.RED : Color.GREEN;

         footstep.getSoleFramePose(footPose);
         footPose.get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if (footstep.hasFoothold())
            footstep.getFoothold(foothold);
         else
            foothold.set(defaultContactPoints.get(plan.getFootstep(i).getRobotSide()));

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);
      }

      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      synchronized (this)
      {
         this.mesh = mesh;
         this.material = material;
      }
   }

   @Override
   public void handle(long now)
   {
      synchronized (this)
      {
         meshView.setMesh(mesh);
         meshView.setMaterial(material);
      }
   }

   public MeshView getNode()
   {
      return meshView;
   }
}
