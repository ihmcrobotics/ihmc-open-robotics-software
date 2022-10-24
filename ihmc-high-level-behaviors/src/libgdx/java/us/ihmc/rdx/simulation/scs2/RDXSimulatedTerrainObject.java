package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

import java.util.ArrayList;

public class RDXSimulatedTerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private final ArrayList<ModelInstance> visualModelInstances = new ArrayList<>();
   private final ArrayList<ModelInstance> collisionModelInstances = new ArrayList<>();

   public RDXSimulatedTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      Gdx.app.postRunnable(() ->
      {
         for (RDXVisualModelInstance terrainModelInstance : RDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions()))
         {
            visualModelInstances.add(terrainModelInstance);
            LibGDXTools.toLibGDX(terrainModelInstance.getLocalTransform(), terrainModelInstance.transform);
         }
         for (RDXVisualModelInstance terrainCollisionModelInstance : RDXVisualTools.collectCollisionNodes(terrainObjectDefinition.getCollisionShapeDefinitions()))
         {
            collisionModelInstances.add(terrainCollisionModelInstance);
            LibGDXTools.toLibGDX(terrainCollisionModelInstance.getLocalTransform(), terrainCollisionModelInstance.transform);
         }
      });
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance modelInstance : visualModelInstances)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void getCollisionRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance modelInstance : collisionModelInstances)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }
}
