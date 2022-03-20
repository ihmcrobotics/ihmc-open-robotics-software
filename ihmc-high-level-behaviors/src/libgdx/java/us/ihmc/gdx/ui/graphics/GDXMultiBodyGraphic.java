package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.simulation.scs2.GDXMultiBodySystemFactories;
import us.ihmc.gdx.simulation.scs2.GDXRigidBody;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.thread.Activator;

public class GDXMultiBodyGraphic extends ImGuiGDXVisualizer implements RenderableProvider
{
   private GDXRigidBody multiBody;
   private final Activator robotLoadedActivator = new Activator();

   public GDXMultiBodyGraphic(String title)
   {
      super(title);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics rootBody)
   {
      if (multiBody != null)
         multiBody.destroy();

      ThreadTools.startAsDaemon(() ->
      {
         multiBody = GDXMultiBodySystemFactories.toGDXMultiBodySystem(rootBody, ReferenceFrame.getWorldFrame(), robotDefinition, Gdx.app::postRunnable);
         robotLoadedActivator.activate();
      }, getClass().getSimpleName() + "Loading");
   }

   @Override
   public void update()
   {
      super.update();
      if (robotLoadedActivator.poll())
      {
         // multiBody.updateFramesRecursively(); // It is expected that these are updated before this; but TODO find out for sure
         multiBody.updateSubtreeGraphics();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive() && robotLoadedActivator.poll())
      {
         multiBody.getVisualRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      multiBody.destroy();
   }

   public boolean isRobotLoaded()
   {
      return robotLoadedActivator.poll();
   }
}
