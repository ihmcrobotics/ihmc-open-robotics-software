package us.ihmc.javaFXVisualizers;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class FootstepMeshManager
{
   private static final double POLYGON_THICKNESS = 0.01;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final Function<RobotSide, ConvexPolygon2D> defaultContactPointSupplier;
   private final BooleanSupplier isSelected;
   private final BooleanSupplier ignorePartialFootholds;

   private final MeshHolder meshHolder;
   private final AtomicReference<FootstepDataMessage> footstepDataMessage = new AtomicReference<>();

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   public FootstepMeshManager(Group root,
                              JavaFXMultiColorMeshBuilder meshBuilder,
                              Function<RobotSide, ConvexPolygon2D> defaultContactPointSupplier,
                              BooleanSupplier isSelected,
                              BooleanSupplier ignorePartialFootholds)
   {
      meshHolder = new MeshHolder(root);
      this.meshBuilder = meshBuilder;
      this.defaultContactPointSupplier = defaultContactPointSupplier;
      this.isSelected = isSelected;
      this.ignorePartialFootholds = ignorePartialFootholds;
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

      Color footColor = getFootstepColor(footstepDataMessage);
      RobotSide robotSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());

      transformToWorld.set(footstepDataMessage.getOrientation(), footstepDataMessage.getLocation());

      if (ignorePartialFootholds.getAsBoolean() || footstepDataMessage.getPredictedContactPoints2d().isEmpty())
      {
         foothold.set(defaultContactPointSupplier.apply(robotSide));
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

      // Shift halfway so that bottom of perimeter line is flush with ground
      transformToWorld.appendTranslation(0.0, 0.0, 0.5 * POLYGON_THICKNESS);
      meshBuilder.addMultiLine(transformToWorld, vertices, POLYGON_THICKNESS, footColor, true);
      meshBuilder.addPolygon(transformToWorld, foothold, footColor);

      Pair<Mesh, Material> meshMaterialPair = Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial());
      meshHolder.setMeshReference(meshMaterialPair);
   }

   private Color getFootstepColor(FootstepDataMessage footstepDataMessage)
   {
      if (isSelected.getAsBoolean())
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
