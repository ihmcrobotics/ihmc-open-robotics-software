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
 * Bumpy ground of random heights, after IsaacLab's {@code HfRandomUniformTerrainCfg}, that meets the
 * surrounding ground at its edges. The geometry is in {@link ProceduralTerrainGeometry#randomHeightField}.
 * The seed is saved, so the simulation environment gets the same heights.
 */
public class RDXRandomHeightFieldObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "Random Height Field";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXRandomHeightFieldObject.class);

   private final ImDouble cellSize = new ImDouble(0.25);
   private final ImDouble maxHeightDelta = new ImDouble(0.08);
   private final ImDouble gridSize = new ImDouble(6.0);
   private long seed = new Random().nextLong();

   public RDXRandomHeightFieldObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   public int getCellsPerSide()
   {
      return ProceduralTerrainGeometry.randomHeightFieldCellsPerSide(cellSize.get(), gridSize.get());
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      addSurfacesToMeshBuilder(ProceduralTerrainGeometry.randomHeightField(cellSize.get(), maxHeightDelta.get(), gridSize.get(), seed), meshBuilders);
   }

   @Override
   protected double[] computeBounds()
   {
      double width = getCellsPerSide() * cellSize.get();
      return new double[] {width, width, -BASE_THICKNESS, maxHeightDelta.get()};
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      boolean changed = false;
      changed |= inputMeters("Cell size (m)", cellSize, 0.05, 5.0);
      changed |= inputMeters("Max height delta (m)", maxHeightDelta, 0.0, 1.0);
      changed |= inputMeters("Grid size (m)", gridSize, 0.05, 50.0);
      if (ImGui.button(labels.get("Randomize heights")))
      {
         seed = new Random().nextLong();
         changed = true;
      }
      int cellsPerSide = getCellsPerSide();
      ImGui.text("Cells: %d x %d (%.2f m)".formatted(cellsPerSide, cellsPerSide, cellsPerSide * cellSize.get()));
      if (cellsPerSide == ProceduralTerrainGeometry.MAX_HEIGHT_FIELD_CELLS_PER_SIDE)
         ImGui.text("Limited to %d cells per side".formatted(ProceduralTerrainGeometry.MAX_HEIGHT_FIELD_CELLS_PER_SIDE));
      return changed;
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      objectNode.put("cellSize", cellSize.get());
      objectNode.put("maxHeightDelta", maxHeightDelta.get());
      objectNode.put("gridSize", gridSize.get());
      objectNode.put("seed", seed);
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      cellSize.set(objectNode.path("cellSize").asDouble(cellSize.get()));
      maxHeightDelta.set(objectNode.path("maxHeightDelta").asDouble(maxHeightDelta.get()));
      gridSize.set(objectNode.path("gridSize").asDouble(gridSize.get()));
      seed = objectNode.path("seed").asLong(seed);
      rebuild();
   }
}
