package us.ihmc.rdx.simulation.environment.object.objects.terrain;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry;

import java.util.ArrayList;
import java.util.Random;
import java.util.function.Consumer;

/**
 * Square grid of square tiles at random heights, after IsaacLab's {@code MeshRandomGridTerrainCfg}.
 * The geometry is in {@link ProceduralTerrainGeometry#unevenTiles}; the seed is saved so a reloaded
 * environment gets the same heights.
 */
public class RDXUnevenTilesObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "Uneven Tiles";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXUnevenTilesObject.class);

   private final ImDouble tileSize = new ImDouble(0.45);
   private final ImDouble maxHeightDelta = new ImDouble(0.1);
   private final ImDouble gridSize = new ImDouble(6.0);
   private long seed = new Random().nextLong();

   public RDXUnevenTilesObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   public int getTilesPerSide()
   {
      return ProceduralTerrainGeometry.unevenTilesPerSide(tileSize.get(), gridSize.get());
   }

   private double getBottomZ()
   {
      return -BASE_THICKNESS;
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      addBoxesToMeshBuilders(ProceduralTerrainGeometry.unevenTiles(tileSize.get(), maxHeightDelta.get(), gridSize.get(), seed), meshBuilders);
   }

   @Override
   protected double[] computeBounds()
   {
      double width = getTilesPerSide() * tileSize.get();
      return new double[] {width, width, getBottomZ(), maxHeightDelta.get()};
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      boolean changed = false;
      changed |= inputMeters("Tile size (m)", tileSize, 0.05, 5.0);
      changed |= inputMeters("Max height delta (m)", maxHeightDelta, 0.0, 1.0);
      changed |= inputMeters("Grid size (m)", gridSize, 0.05, 50.0);
      if (ImGui.button(labels.get("Randomize heights")))
      {
         seed = new Random().nextLong();
         changed = true;
      }
      int tilesPerSide = getTilesPerSide();
      ImGui.text("Tiles: %d x %d (%.2f m)".formatted(tilesPerSide, tilesPerSide, tilesPerSide * tileSize.get()));
      if (tilesPerSide == ProceduralTerrainGeometry.MAX_TILES_PER_SIDE)
         ImGui.text("Limited to %d tiles per side".formatted(ProceduralTerrainGeometry.MAX_TILES_PER_SIDE));
      return changed;
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      objectNode.put("tileSize", tileSize.get());
      objectNode.put("maxHeightDelta", maxHeightDelta.get());
      objectNode.put("gridSize", gridSize.get());
      objectNode.put("seed", seed);
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      tileSize.set(objectNode.path("tileSize").asDouble(tileSize.get()));
      maxHeightDelta.set(objectNode.path("maxHeightDelta").asDouble(maxHeightDelta.get()));
      gridSize.set(objectNode.path("gridSize").asDouble(gridSize.get()));
      seed = objectNode.path("seed").asLong(seed);
      rebuild();
   }
}
