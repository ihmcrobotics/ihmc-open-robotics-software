package us.ihmc.rdx.simulation.environment.object.objects.terrain;

import com.badlogic.gdx.graphics.Color;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

/**
 * Flat ground at z = 0 that fills the area around the other procedural terrain, like the border in
 * IsaacLab's terrain generators. The footprint of every procedural terrain object is cut out of the
 * ground, so pits (stairs down, slopes down) open into it rather than being covered.
 * The cut is exact; see {@link ProceduralTerrainGeometry#groundPieces}.
 */
public class RDXProceduralGroundObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "Procedural Ground";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXProceduralGroundObject.class);

   private final ImDouble sizeX = new ImDouble(40.0);
   private final ImDouble sizeY = new ImDouble(40.0);
   /** RGB. Applied as a material tint over a white mesh, since the mesh palette can't reproduce arbitrary colors. */
   private final float[] color = {0.824f, 0.706f, 0.549f};
   /** Footprints of the other terrain in this object's frame, counter-clockwise. */
   private final ArrayList<List<Point2D>> holes = new ArrayList<>();
   private double[] holesSignature = new double[0];
   private final Point3D tempPoint = new Point3D();

   public RDXProceduralGroundObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   /**
    * Cuts the footprints of the given terrain out of the ground, rebuilding only if they changed.
    */
   public void updateHoles(List<RDXProceduralTerrainObject> terrainObjects)
   {
      ArrayList<List<Point2D>> newHoles = new ArrayList<>();
      for (RDXProceduralTerrainObject terrainObject : terrainObjects)
      {
         ArrayList<Point2D> hole = new ArrayList<>();
         for (Point3D corner : terrainObject.getFootprintInWorld())
         {
            getObjectTransform().inverseTransform(corner);
            hole.add(new Point2D(corner.getX(), corner.getY()));
         }
         newHoles.add(hole);
      }

      double[] signature = newHoles.stream().flatMap(List::stream).flatMapToDouble(point -> Arrays.stream(new double[] {point.getX(), point.getY()})).toArray();
      if (!Arrays.equals(signature, holesSignature))
      {
         holesSignature = signature;
         holes.clear();
         holes.addAll(newHoles);
         rebuild();
      }
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      List<List<Point2D>> pieces = ProceduralTerrainGeometry.groundPieces(sizeX.get(), sizeY.get(), holes);
      meshBuilders.add(meshBuilder ->
      {
         for (List<Point2D> piece : pieces)
         {
            meshBuilder.addPolygon(piece.stream().map(point -> new Point3D(point.getX(), point.getY(), 0.0)).toList(), Color.WHITE);
         }
      });
   }

   /** Lets clicks through the holes reach the terrain below ground level. */
   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      if (!super.intersect(pickRay, intersectionToPack))
         return false;
      tempPoint.set(intersectionToPack);
      getObjectTransform().inverseTransform(tempPoint);
      for (List<Point2D> hole : holes)
      {
         if (ProceduralTerrainGeometry.isInside(hole, tempPoint.getX(), tempPoint.getY()))
            return false;
      }
      return true;
   }

   @Override
   public void rebuild()
   {
      super.rebuild();
      applyColor();
   }

   private void applyColor()
   {
      getRealisticModelInstance().materials.get(0).set(PBRColorAttribute.createBaseColorFactor(new Color(color[0], color[1], color[2], 1.0f)));
   }

   @Override
   protected double[] computeBounds()
   {
      return new double[] {sizeX.get(), sizeY.get(), -BASE_THICKNESS, 0.0};
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      boolean changed = false;
      changed |= inputMeters("Size X (m)", sizeX, 1.0, 200.0);
      changed |= inputMeters("Size Y (m)", sizeY, 1.0, 200.0);
      if (ImGui.colorEdit3(labels.get("Color"), color))
         applyColor();
      ImGui.text("Cut out around %d terrain objects".formatted(holes.size()));
      return changed;
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      objectNode.put("sizeX", sizeX.get());
      objectNode.put("sizeY", sizeY.get());
      objectNode.putArray("color").add(color[0]).add(color[1]).add(color[2]);
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      sizeX.set(objectNode.path("sizeX").asDouble(sizeX.get()));
      sizeY.set(objectNode.path("sizeY").asDouble(sizeY.get()));
      JsonNode colorNode = objectNode.path("color");
      for (int i = 0; i < color.length && i < colorNode.size(); i++)
         color[i] = (float) colorNode.get(i).asDouble();
      rebuild();
   }
}
