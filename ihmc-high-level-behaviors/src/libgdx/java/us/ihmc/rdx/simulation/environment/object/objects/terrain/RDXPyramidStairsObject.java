package us.ihmc.rdx.simulation.environment.object.objects.terrain;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * Square pyramid of stairs that trims to a flat platform at the center, after IsaacLab's
 * {@code MeshPyramidStairsTerrainCfg} / {@code MeshInvertedPyramidStairsTerrainCfg}.
 * The geometry is in {@link ProceduralTerrainGeometry#pyramidStairs}.
 */
public class RDXPyramidStairsObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "Pyramid Stairs";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPyramidStairsObject.class);

   private final ImDouble platformSize = new ImDouble(2.0);
   private final ImDouble baseSize = new ImDouble(6.0);
   private final ImDouble stepRise = new ImDouble(0.15);
   private final ImDouble stepRun = new ImDouble(0.3);

   public RDXPyramidStairsObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   public int getNumberOfSteps()
   {
      return ProceduralTerrainGeometry.pyramidStairsNumberOfSteps(baseSize.get(), platformSize.get(), stepRun.get());
   }

   private double getCenterPlatformHeight()
   {
      return ProceduralTerrainGeometry.pyramidStairsPlatformHeight(baseSize.get(), platformSize.get(), stepRise.get(), stepRun.get());
   }

   private double getBottomZ()
   {
      return ProceduralTerrainGeometry.pyramidStairsBottomZ(baseSize.get(), platformSize.get(), stepRise.get(), stepRun.get());
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      addBoxesToMeshBuilders(ProceduralTerrainGeometry.pyramidStairs(baseSize.get(), platformSize.get(), stepRise.get(), stepRun.get()), meshBuilders);
   }

   @Override
   protected double[] computeBounds()
   {
      return new double[] {baseSize.get(), baseSize.get(), getBottomZ(), Math.max(0.0, getCenterPlatformHeight())};
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      boolean changed = false;
      changed |= inputMeters("Base size (m)", baseSize, 0.1, 50.0);
      changed |= inputMeters("Platform size (m)", platformSize, 0.0, baseSize.get());
      changed |= inputMeters("Step rise (m)", stepRise, -1.0, 1.0);
      changed |= inputMeters("Step run (m)", stepRun, 0.05, 5.0);
      if (changed)
         platformSize.set(Math.min(platformSize.get(), baseSize.get()));
      ImGui.text("Steps: %d, total height: %.3f m".formatted(getNumberOfSteps(), getCenterPlatformHeight()));
      return changed;
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      objectNode.put("platformSize", platformSize.get());
      objectNode.put("baseSize", baseSize.get());
      objectNode.put("stepRise", stepRise.get());
      objectNode.put("stepRun", stepRun.get());
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      platformSize.set(objectNode.path("platformSize").asDouble(platformSize.get()));
      baseSize.set(objectNode.path("baseSize").asDouble(baseSize.get()));
      stepRise.set(objectNode.path("stepRise").asDouble(stepRise.get()));
      stepRun.set(objectNode.path("stepRun").asDouble(stepRun.get()));
      rebuild();
   }
}
