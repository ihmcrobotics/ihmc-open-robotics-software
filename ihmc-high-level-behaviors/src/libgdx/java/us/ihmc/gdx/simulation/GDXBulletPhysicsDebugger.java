package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btDiscreteDynamicsWorld;
import com.badlogic.gdx.physics.bullet.linearmath.*;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw.DebugDrawModes;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.log.LogTools;

public class GDXBulletPhysicsDebugger
{
   private final btIDebugDraw btIDebugDraw;
   private int debugMode = DebugDrawModes.DBG_DrawWireframe; // TODO: Provide options in combo box
   private btDiscreteDynamicsWorld discreteDynamicsWorld;
   private GDXMultiColorMeshBuilder meshBuilder;
   private ModelInstance debugModelInstance;
   private ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean drawDebug = new ImBoolean(false);

   public GDXBulletPhysicsDebugger(btDiscreteDynamicsWorld discreteDynamicsWorld)
   {
      this.discreteDynamicsWorld = discreteDynamicsWorld;

      btIDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(Vector3 from, Vector3 to, Vector3 color)
         {
            meshBuilder.addLine(from.x, from.y, from.z, to.x, to.y, to.z, 0.001f, new Color(color.x, color.y, color.z, 1.0f));
         }

         @Override
         public void drawContactPoint(Vector3 PointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color)
         {

         }

         @Override
         public void drawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 color, float alpha)
         {

         }

         @Override
         public void reportErrorWarning(String warningString)
         {
            LogTools.error("Bullet: {}", warningString);
         }

         @Override
         public void draw3dText(Vector3 location, String textString)
         {

         }

         @Override
         public void clearLines()
         {

         }

         @Override
         public void setDebugMode(int debugMode)
         {
            GDXBulletPhysicsDebugger.this.debugMode = debugMode;
         }

         @Override
         public int getDebugMode()
         {
            return debugMode;
         }
      };
      discreteDynamicsWorld.setDebugDrawer(btIDebugDraw);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Draw debug wireframes"), drawDebug);
   }

   public void update()
   {
      if (drawDebug.get())
      {
         debugModelInstance = GDXModelPrimitives.buildModelInstance(meshBuilder ->
         {
            this.meshBuilder = meshBuilder;
            discreteDynamicsWorld.debugDrawWorld();
         }, "BulletDebug");
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (debugModelInstance != null)
      {
         debugModelInstance.getRenderables(renderables, pool);
      }
   }
}
