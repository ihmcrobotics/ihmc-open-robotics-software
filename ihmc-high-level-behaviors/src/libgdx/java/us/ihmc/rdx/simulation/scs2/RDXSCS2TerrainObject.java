package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTerrainObject;

public class RDXSCS2TerrainObject
{
   private final TerrainObjectDefinition terrainObjectDefinition;
   private RDXFrameGraphicsNode visualFrameGraphicsNode;
   private RDXFrameGraphicsNode collisionFrameGraphicsNode;
   private final MutableReferenceFrame centerOfMassFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
   private BulletTerrainObject bulletTerrainObject;

   public RDXSCS2TerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition = terrainObjectDefinition;
   }

   public void create()
   {
      visualFrameGraphicsNode = new RDXFrameGraphicsNode(centerOfMassFrame.getReferenceFrame());
      collisionFrameGraphicsNode = new RDXFrameGraphicsNode(centerOfMassFrame.getReferenceFrame());

      for (RDXVisualModelInstance terrainModelInstance : RDXVisualTools.collectNodes(terrainObjectDefinition.getVisualDefinitions()))
      {
         visualFrameGraphicsNode.addModelPart(terrainModelInstance);
      }
      for (RDXVisualModelInstance terrainCollisionModelInstance : RDXVisualTools.collectCollisionNodes(terrainObjectDefinition.getCollisionShapeDefinitions()))
      {
         collisionFrameGraphicsNode.addModelPart(terrainCollisionModelInstance);
      }
   }

   public void update()
   {
      if (bulletTerrainObject != null)
      {
         centerOfMassFrame.update(transformToParent ->
         {
            transformToParent.set(bulletTerrainObject.getTransformToWorld());
         });
      }

      visualFrameGraphicsNode.update();
      collisionFrameGraphicsNode.update();
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      visualFrameGraphicsNode.getRenderables(renderables, pool);
   }

   public void getCollisionRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      collisionFrameGraphicsNode.getRenderables(renderables, pool);
   }

   /**
    * Support moving terrain objects when using Bullet physics
    */
   public void setBulletTerrainObject(BulletTerrainObject bulletTerrainObject)
   {
      this.bulletTerrainObject = bulletTerrainObject;
   }
}
