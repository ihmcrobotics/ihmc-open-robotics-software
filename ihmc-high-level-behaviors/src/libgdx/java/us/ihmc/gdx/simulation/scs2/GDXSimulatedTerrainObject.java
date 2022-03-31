package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;

import java.util.ArrayList;

public class GDXSimulatedTerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private final ArrayList<ModelInstance> visualModelInstances = new ArrayList<>();
   private final ArrayList<ModelInstance> collisionModelInstances = new ArrayList<>();

   public GDXSimulatedTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      Gdx.app.postRunnable(() ->
      {
         for (DynamicGDXModel terrainModelPart : GDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions()))
         {
            ModelInstance modelInstance = terrainModelPart.getOrCreateModelInstance();
            visualModelInstances.add(modelInstance);
            GDXTools.toGDX(terrainModelPart.getLocalTransform(), modelInstance.transform);
         }
         for (DynamicGDXModel terrainCollisionModelPart : GDXVisualTools.collectCollisionNodes(terrainObjectDefinition.getCollisionShapeDefinitions()))
         {
            ModelInstance modelInstance = terrainCollisionModelPart.getOrCreateModelInstance();
            collisionModelInstances.add(modelInstance);
            GDXTools.toGDX(terrainCollisionModelPart.getLocalTransform(), modelInstance.transform);
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
