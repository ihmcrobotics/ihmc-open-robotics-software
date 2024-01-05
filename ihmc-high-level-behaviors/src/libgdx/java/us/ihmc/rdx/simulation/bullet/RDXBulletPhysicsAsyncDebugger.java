package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import org.bytedeco.bullet.LinearMath.btIDebugDraw;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyDynamicsWorld;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTools;

public class RDXBulletPhysicsAsyncDebugger
{
   private final btIDebugDraw btDebugDraw;
   private int debugMode = btIDebugDraw.DBG_DrawWireframe; // TODO: Provide options in combo box
   private final BulletMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final RecyclingArrayList<RDXBulletPhysicsDebuggerModel> models = new RecyclingArrayList<>(RDXBulletPhysicsDebuggerModel::new);
   private final RecyclingArrayList<RDXBulletPhysicsDebuggerLineSegment> lineSegmentsToDraw = new RecyclingArrayList<>(RDXBulletPhysicsDebuggerLineSegment::new);
   private RDXBulletPhysicsDebuggerModel currentModel;
   private int lineDraws;
   private final int maxLineDrawsPerModel = 100;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean updateDebugDrawings = new ImBoolean(false);
   private final ImBoolean showDebugDrawings = new ImBoolean(true);
   private final Notification newDebugDrawingsAvailable = new Notification();

   public RDXBulletPhysicsAsyncDebugger(BulletMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      btDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(btVector3 from, btVector3 to, btVector3 color)
         {
            RDXBulletPhysicsDebuggerLineSegment lineSegment = lineSegmentsToDraw.add();
            BulletTools.toEuclid(from, lineSegment.getLineSegment().getFirstEndpoint());
            BulletTools.toEuclid(to, lineSegment.getLineSegment().getSecondEndpoint());
            BulletLibGDXTools.toLibGDX(color, lineSegment.getColor());
         }

         @Override
         public void drawContactPoint(btVector3 PointOnB, btVector3 normalOnB, double distance, int lifeTime, btVector3 color)
         {

         }

         @Override
         public void drawTriangle(btVector3 v0, btVector3 v1, btVector3 v2, btVector3 color, double alpha)
         {

         }

         @Override
         public void reportErrorWarning(BytePointer warningString)
         {
            LogTools.error("Bullet: {}", warningString.getString());
         }

         @Override
         public void draw3dText(btVector3 location, BytePointer textString)
         {

         }

         @Override
         public void setDebugMode(int debugMode)
         {
            RDXBulletPhysicsAsyncDebugger.this.debugMode = debugMode;
         }

         @Override
         public int getDebugMode()
         {
            return debugMode;
         }
      };
      multiBodyDynamicsWorld.setBtDebugDrawer(btDebugDraw);
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
            for (RDXBulletPhysicsDebuggerLineSegment lineSegment : lineSegmentsToDraw)
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

   public void setUpdateDebugDrawings(boolean updateDebugDrawings)
   {
      this.updateDebugDrawings.set(updateDebugDrawings);
   }
}
