package us.ihmc.javaFXVisualizers;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepMeshManager
{
   private static final double ADDITIONAL_HEIGHT = 0.01;

   private final JavaFXMultiColorMeshBuilder meshBuilder;
   
   private final AtomicBoolean selected = new AtomicBoolean(false);
   private final AtomicBoolean ignorePartialFootholds = new AtomicBoolean(false);
   
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   
   private final int index;
   private final MeshHolder meshHolder;
   private final AtomicReference<FootstepDataMessage> footstepDataMessage = new AtomicReference<>();

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   public FootstepMeshManager(Group root, JavaFXMultiColorMeshBuilder meshBuilder, int index)
   {
      meshHolder = new MeshHolder(root);
      this.meshBuilder = meshBuilder;
      this.index = index;
   }

   /**
    * Call from any thread
    */
   public void computeMesh()
   {
      if (footstepDataMessage.get() == null)
      {
         return;
      }

      meshBuilder.clear();
      foothold.clear();

      FootstepDataMessage footstepDataMessage = this.footstepDataMessage.get();

      Color footColor = getFootstepColor(footstepDataMessage, index);
      RobotSide robotSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());

      transformToWorld.set(footstepDataMessage.getOrientation(), footstepDataMessage.getLocation());
      transformToWorld.appendTranslation(0.0, 0.0, ADDITIONAL_HEIGHT);

      if (ignorePartialFootholds.get() || footstepDataMessage.getPredictedContactPoints2d().isEmpty())
      {
         foothold.set(defaultContactPoints.get(robotSide));
      }
      else
      {
         footstepDataMessage.getPredictedContactPoints2d().forEach(p -> foothold.addVertex(p.getX(), p.getY()));
         foothold.update();
      }

      Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
      for (int j = 0; j < vertices.length; j++)
      {
         vertices[j] = new Point2D(foothold.getVertex(j));
      }

      meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, footColor, true);
      meshBuilder.addPolygon(transformToWorld, foothold, footColor);

      Pair<Mesh, Material> meshMaterialPair = Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial());
      meshHolder.setMeshReference(meshMaterialPair);
   }
   
   public void setIgnorePartialFootHolds(boolean doIgnore)
   {
      ignorePartialFootholds.set(doIgnore);
   }
   
   public void setSelected(boolean selected)
   {
      this.selected.set(selected);
   }
   
   public void setFoothold(SideDependentList<List<Point2D>> defaultContactPoints)
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
      }
   }
   
   private Color getFootstepColor(FootstepDataMessage footstepDataMessage, int index)
   {
      if (selected.get())
      {
         return Color.YELLOW.darker();
      }
      else
      {
         return footstepDataMessage.getRobotSide() == 0 ? Color.RED : Color.GREEN;
      }
   }
   
   public void setFootstepDataMessage(FootstepDataMessage message)
   {
      footstepDataMessage.set(message);
   }
   
   public MeshHolder getMeshHolder()
   {
      return meshHolder;
   }

   /**
    * Call from AnimationTimer.handle
    */
   public void updateMesh()
   {
      meshHolder.update();
   }

   /**
    * Call from AnimationTimer.handle
    */
   public void clear()
   {
      footstepDataMessage.set(null);
      meshHolder.remove();
   }
}
