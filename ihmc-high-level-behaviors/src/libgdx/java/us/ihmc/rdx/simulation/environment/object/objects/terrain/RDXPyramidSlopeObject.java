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
 * Hill shaped as a square pyramid that trims to a flat platform at the center, after IsaacLab's
 * {@code HfPyramidSlopedTerrainCfg} / {@code HfInvertedPyramidSlopedTerrainCfg}. A positive slope builds a
 * hill, a negative slope a pit. The geometry is in {@link ProceduralTerrainGeometry#pyramidSlope}.
 */
public class RDXPyramidSlopeObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "Hill";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPyramidSlopeObject.class);

   private final ImDouble platformSize = new ImDouble(2.0);
   private final ImDouble baseSize = new ImDouble(6.0);
   private final ImDouble slopeDegrees = new ImDouble(15.0);

   public RDXPyramidSlopeObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   private double getPlatformHeight()
   {
      return ProceduralTerrainGeometry.pyramidSlopePlatformHeight(baseSize.get(), platformSize.get(), slopeDegrees.get());
   }

   private double getBottomZ()
   {
      return ProceduralTerrainGeometry.pyramidSlopeBottomZ(baseSize.get(), platformSize.get(), slopeDegrees.get());
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      addSurfacesToMeshBuilder(ProceduralTerrainGeometry.pyramidSlope(baseSize.get(), platformSize.get(), slopeDegrees.get()), meshBuilders);
   }

   @Override
   protected double[] computeBounds()
   {
      return new double[] {baseSize.get(), baseSize.get(), getBottomZ(), Math.max(0.0, getPlatformHeight())};
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      boolean changed = false;
      changed |= inputMeters("Base size (m)", baseSize, 0.1, 50.0);
      changed |= inputMeters("Platform size (m)", platformSize, 0.0, baseSize.get());
      ImGui.pushItemWidth(150.0f);
      if (ImGui.inputDouble(labels.get("Slope (deg)"), slopeDegrees, 1.0, 5.0, "%.1f"))
      {
         slopeDegrees.set(Math.max(-60.0, Math.min(60.0, slopeDegrees.get())));
         changed = true;
      }
      ImGui.popItemWidth();
      if (changed)
         platformSize.set(Math.min(platformSize.get(), baseSize.get()));
      ImGui.text("Platform height: %.3f m".formatted(getPlatformHeight()));
      return changed;
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      objectNode.put("platformSize", platformSize.get());
      objectNode.put("baseSize", baseSize.get());
      objectNode.put("slopeDegrees", slopeDegrees.get());
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      platformSize.set(objectNode.path("platformSize").asDouble(platformSize.get()));
      baseSize.set(objectNode.path("baseSize").asDouble(baseSize.get()));
      slopeDegrees.set(objectNode.path("slopeDegrees").asDouble(slopeDegrees.get()));
      rebuild();
   }
}
