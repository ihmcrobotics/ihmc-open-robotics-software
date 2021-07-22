package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public class GDXSimulatedTerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private DynamicGDXModel model;
   private ModelInstance modelInstance;

   public GDXSimulatedTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      model = GDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions());
      modelInstance = model.getOrCreateModelInstance();
      GDXTools.toGDX(model.getLocalTransform(), modelInstance.transform);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
