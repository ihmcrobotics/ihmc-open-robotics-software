package us.ihmc.rdx.simulation.environment.object.objects.terrain;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainSurface;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

/**
 * Base class for terrain that is generated from a few dimensions instead of loaded from a mesh file,
 * modeled after the IsaacLab terrain generators (isaaclab.terrains).
 * <p>
 * The object frame is at the center of the terrain footprint, with z = 0 at the level of the
 * surrounding ground. Terrain that goes below ground (stairs down, slopes down) extends to negative z.
 * </p>
 */
public abstract class RDXProceduralTerrainObject extends RDXEnvironmentObject
{
   /** Solid material added under the lowest surface so the terrain is never paper thin. */
   protected static final double BASE_THICKNESS = ProceduralTerrainGeometry.BASE_THICKNESS;
   /** Keeps each mesh under the 16-bit index limit of the mesh interpreter. */
   private static final int MAX_BOXES_PER_MESH = 2000;

   protected final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<Model> models = new ArrayList<>();
   private final ArrayList<RDXModelInstance> additionalModelInstances = new ArrayList<>();
   private final Point3D firstIntersection = new Point3D();
   private final Point3D secondIntersection = new Point3D();
   private Box3D boundingBox;

   public RDXProceduralTerrainObject(String titleCasedName, RDXEnvironmentObjectFactory factory)
   {
      super(titleCasedName, factory);
      setMass(10000.0f);
   }

   /** Adds one mesh builder per model to generate. Use {@link #addBoxesToMeshBuilders} for many boxes. */
   protected abstract void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders);

   /** @return { sizeX, sizeY, minZ, maxZ } of the generated geometry in the object frame. */
   protected abstract double[] computeBounds();

   /**
    * @return the corners of the rectangular footprint in world, counter-clockwise seen from above.
    *       Roll and pitch are ignored; terrain is assumed to sit level on the ground.
    */
   public List<Point3D> getFootprintInWorld()
   {
      double[] bounds = computeBounds();
      double halfX = 0.5 * bounds[0];
      double halfY = 0.5 * bounds[1];
      List<Point3D> corners = List.of(new Point3D(-halfX, -halfY, 0.0),
                                      new Point3D(halfX, -halfY, 0.0),
                                      new Point3D(halfX, halfY, 0.0),
                                      new Point3D(-halfX, halfY, 0.0));
      for (Point3D corner : corners)
         getObjectTransform().transform(corner);
      return corners;
   }

   /** Renders the dimension widgets. @return whether a parameter changed. */
   protected abstract boolean renderParameterWidgets();

   public abstract void saveParameters(ObjectNode objectNode);

   public abstract void loadParameters(JsonNode objectNode);

   /**
    * Regenerates the meshes and collision box from the current parameters, keeping the current pose.
    */
   public void rebuild()
   {
      for (Model model : models)
         model.dispose();
      models.clear();
      additionalModelInstances.clear();

      ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders = new ArrayList<>();
      buildMeshes(meshBuilders);
      for (int i = 0; i < meshBuilders.size(); i++)
      {
         Model model = RDXModelBuilder.buildModel(meshBuilders.get(i), getPascalCasedName() + getObjectIndex() + "Chunk" + i);
         models.add(model);
         if (i == 0)
            setRealisticModel(model);
         else
            additionalModelInstances.add(new RDXModelInstance(model));
      }

      double[] bounds = computeBounds();
      double sizeX = bounds[0];
      double sizeY = bounds[1];
      double sizeZ = bounds[3] - bounds[2];
      boundingBox = new Box3D(sizeX, sizeY, sizeZ);
      getCollisionShapeOffset().setToZero();
      getCollisionShapeOffset().getTranslation().setZ(0.5 * (bounds[2] + bounds[3]));
      getBoundingSphere().setRadius(0.5 * Math.sqrt(sizeX * sizeX + sizeY * sizeY) + Math.max(Math.abs(bounds[2]), Math.abs(bounds[3])));

      Box3D collisionBox = boundingBox;
      setCollisionModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox(sizeX, sizeY, sizeZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color);
      });
      models.add(collisionMesh);
      setCollisionGeometryObject(collisionBox);

      updateRenderablesPoses();
   }

   private static final Color[] SHADES = {Color.LIGHT_GRAY, Color.GRAY, Color.DARK_GRAY};

   /**
    * Splits boxes into mesh builders of at most {@link #MAX_BOXES_PER_MESH} boxes each.
    */
   protected static void addBoxesToMeshBuilders(List<TerrainBox> boxes, ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      for (int start = 0; start < boxes.size(); start += MAX_BOXES_PER_MESH)
      {
         List<TerrainBox> chunk = boxes.subList(start, Math.min(boxes.size(), start + MAX_BOXES_PER_MESH));
         meshBuilders.add(meshBuilder ->
         {
            Point3D offset = new Point3D();
            for (TerrainBox box : chunk)
            {
               offset.set(box.x(), box.y(), box.z());
               meshBuilder.addBox(box.sizeX(), box.sizeY(), box.sizeZ(), offset, SHADES[box.shade()]);
            }
         });
      }
   }

   /**
    * Adds each surface as a solid: its top, the vertical walls down to its bottom, and the bottom.
    * Walls shared by neighboring surfaces are hidden inside the terrain.
    */
   protected static void addSurfacesToMeshBuilder(List<TerrainSurface> surfaces, ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      meshBuilders.add(meshBuilder ->
      {
         for (TerrainSurface surface : surfaces)
         {
            List<Point3D> top = surface.vertices();
            List<Point3D> bottom = top.stream().map(vertex -> new Point3D(vertex.getX(), vertex.getY(), surface.bottomZ())).toList();
            meshBuilder.addPolygon(top, SHADES[surface.shade()]);
            for (int i = 0; i < top.size(); i++)
            {
               int next = (i + 1) % top.size();
               // Counter-clockwise seen from outside
               meshBuilder.addPolygon(List.of(bottom.get(i), bottom.get(next), top.get(next), top.get(i)), Color.GRAY);
            }
            // Counter-clockwise seen from below
            ArrayList<Point3D> bottomSeenFromBelow = new ArrayList<>(bottom);
            Collections.reverse(bottomSeenFromBelow);
            meshBuilder.addPolygon(bottomSeenFromBelow, Color.GRAY);
         }
      });
   }

   /** Renders the parameter widgets and rebuilds the terrain if one changed. */
   public void renderImGuiWidgets()
   {
      ImGui.text(getTitleCasedName() + " dimensions:");
      if (renderParameterWidgets())
         rebuild();
   }

   protected boolean inputMeters(String label, ImDouble value, double min, double max)
   {
      ImGui.pushItemWidth(150.0f);
      boolean changed = ImGui.inputDouble(labels.get(label), value, 0.01, 0.1, "%.3f");
      ImGui.popItemWidth();
      if (changed)
         value.set(Math.max(min, Math.min(max, value.get())));
      return changed;
   }

   @Override
   public void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();
      for (RDXModelInstance modelInstance : additionalModelInstances)
         modelInstance.transform.set(realisticModelInstance.transform);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRealRenderables(renderables, pool);
      for (RDXModelInstance modelInstance : additionalModelInstances)
         modelInstance.getRenderables(renderables, pool);
   }

   /** Exact ray-box test; the default sampling test misses thin, wide terrain. */
   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      if (boundingBox == null || boundingBox.intersectionWith(pickRay, firstIntersection, secondIntersection) == 0)
         return false;
      intersectionToPack.set(firstIntersection);
      return true;
   }
}
