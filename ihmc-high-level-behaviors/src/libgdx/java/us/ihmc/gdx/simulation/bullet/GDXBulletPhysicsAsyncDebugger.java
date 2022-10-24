package us.ihmc.gdx.simulation.bullet;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw.DebugDrawModes;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyDynamicsWorld;

public class GDXBulletPhysicsAsyncDebugger
{
   private final btIDebugDraw btIDebugDraw;
   private int debugMode = DebugDrawModes.DBG_DrawWireframe; // TODO: Provide options in combo box
   private final BulletMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final RecyclingArrayList<GDXBulletPhysicsDebuggerModel> models = new RecyclingArrayList<>(GDXBulletPhysicsDebuggerModel::new);
   private final RecyclingArrayList<GDXBulletPhysicsDebuggerLineSegment> lineSegmentsToDraw = new RecyclingArrayList<>(GDXBulletPhysicsDebuggerLineSegment::new);
   private GDXBulletPhysicsDebuggerModel currentModel;
   private int lineDraws;
   private final int maxLineDrawsPerModel = 100;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean updateDebugDrawings = new ImBoolean(false);
   private final ImBoolean showDebugDrawings = new ImBoolean(true);
   private final Notification newDebugDrawingsAvailable = new Notification();

   public GDXBulletPhysicsAsyncDebugger(BulletMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      btIDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(Vector3 from, Vector3 to, Vector3 color)
         {
            GDXBulletPhysicsDebuggerLineSegment lineSegment = lineSegmentsToDraw.add();
            GDXTools.toEuclid(from, lineSegment.getLineSegment().getFirstEndpoint());
            GDXTools.toEuclid(to, lineSegment.getLineSegment().getSecondEndpoint());
            GDXTools.toGDX(color, lineSegment.getColor());
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
            GDXBulletPhysicsAsyncDebugger.this.debugMode = debugMode;
         }

         @Override
         public int getDebugMode()
         {
            return debugMode;
         }
      };
      multiBodyDynamicsWorld.setBtDebugDrawer(btIDebugDraw);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Update Bullet debug drawings"), updateDebugDrawings) && updateDebugDrawings.get())
      {
         showDebugDrawings.set(true);
      }
      ImGui.checkbox(labels.get("Show Bullet debug drawings"), showDebugDrawings);
   }

   public void drawBulletDebugDrawings()
   {
      synchronized (this)
      {
         if (updateDebugDrawings.get())
         {
            lineSegmentsToDraw.clear();
            multiBodyDynamicsWorld.debugDrawWorld();
            newDebugDrawingsAvailable.set();
         }
      }
   }

   public void update()
   {
      if (updateDebugDrawings.get() && newDebugDrawingsAvailable.poll())
      {
         models.clear();
         lineDraws = 0;
         nextModel();

         synchronized (this)
         {
            for (GDXBulletPhysicsDebuggerLineSegment lineSegment : lineSegmentsToDraw)
            {
               if (lineDraws >= maxLineDrawsPerModel)
               {
                  lineDraws = 0;
                  currentModel.end();
                  nextModel();
               }

               currentModel.addLineEuclid(lineSegment.getLineSegment().getFirstEndpoint(), lineSegment.getLineSegment().getSecondEndpoint(), lineSegment.getColor());

               ++lineDraws;
            }
         }

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
         for (GDXBulletPhysicsDebuggerModel model : models)
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
