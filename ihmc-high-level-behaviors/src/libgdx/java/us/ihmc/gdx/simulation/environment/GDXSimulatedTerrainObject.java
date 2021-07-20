package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

public class GDXSimulatedTerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private ModelInstance modelInstance;

   public GDXSimulatedTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      modelInstance = GDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions());
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
