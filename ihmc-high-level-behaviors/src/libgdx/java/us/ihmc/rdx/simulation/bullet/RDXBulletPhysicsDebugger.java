package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.linearmath.*;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw.DebugDrawModes;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;

public class RDXBulletPhysicsDebugger
{
   private final btIDebugDraw btIDebugDraw;
   private int debugMode = DebugDrawModes.DBG_DrawWireframe; // TODO: Provide options in combo box
   private final btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final RecyclingArrayList<RDXBulletPhysicsDebuggerModel> models = new RecyclingArrayList<>(RDXBulletPhysicsDebuggerModel::new);
   private RDXBulletPhysicsDebuggerModel currentModel;
   private int lineDraws;
   private final int maxLineDrawsPerModel = 100;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean updateDebugDrawings = new ImBoolean(false);
   private final ImBoolean showDebugDrawings = new ImBoolean(true);
   private final Timer autoDisableTimer = new Timer();

   public RDXBulletPhysicsDebugger(btMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      btIDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(Vector3 from, Vector3 to, Vector3 color)
         {
            if (lineDraws >= maxLineDrawsPerModel)
            {
               lineDraws = 0;
               currentModel.end();
               nextModel();
            }

            currentModel.addLineGDX(from, to, new Color(color.x, color.y, color.z, 1.0f));

            ++lineDraws;
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
         public void setDebugMode(int debugMode)
         {
            RDXBulletPhysicsDebugger.this.debugMode = debugMode;
         }

         @Override
         public int getDebugMode()
         {
            return debugMode;
         }
      };
      multiBodyDynamicsWorld.setDebugDrawer(btIDebugDraw);
   }

   public void renderImGuiWidgets()
   {
      if (autoDisableTimer.isExpired(3.0))
      {
         updateDebugDrawings.set(false);
      }

      // FIXME: There's a native memory leak I think. Or maybe just need to fix the mesh drawing above.
      if (ImGui.checkbox(labels.get("Update Bullet debug drawings (Crashes after a while)"), updateDebugDrawings) && updateDebugDrawings.get())
      {
         autoDisableTimer.reset();
         if (updateDebugDrawings.get())
         {
            showDebugDrawings.set(true);
         }
      }
      ImGui.checkbox(labels.get("Show Bullet debug drawings"), showDebugDrawings);
   }

   public void update()
   {
      if (updateDebugDrawings.get())
      {
         models.clear();
         lineDraws = 0;
         nextModel();
         multiBodyDynamicsWorld.debugDrawWorld();
         currentModel.end();
      }
   }

   private void nextModel()
   {
      currentModel = models.add();
      currentModel.begin();
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showDebugDrawings.get())
      {
         for (RDXBulletPhysicsDebuggerModel model : models)
         {
            ModelInstance modelInstance = model.getModelInstance();
            if (modelInstance != null)
            {
               modelInstance.getRenderables(renderables, pool);
            }
         }
      }
   }
}
