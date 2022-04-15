package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.simulation.scs2.GDXMultiBodySystemFactories;
import us.ihmc.gdx.simulation.scs2.GDXRigidBody;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
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

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody)
   {
      if (multiBody != null)
         multiBody.destroy();

      ThreadTools.startAsDaemon(() ->
      {
         multiBody = loadRigidBody(originalRootBody, robotDefinition);
         robotLoadedActivator.activate();
      }, getClass().getSimpleName() + "Loading");
   }

   private GDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition)
   {
      GDXRigidBody gdxRigidBody = GDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
                                                                             robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                             Gdx.app::postRunnable,
                                                                             robotDefinition.getResourceClassLoader());

      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         childrenJoint.setSuccessor(loadRigidBody(childrenJoint.getSuccessor(), robotDefinition));
      }

      return gdxRigidBody;
   }

   @Override
   public void update()
   {
      super.update();
      if (robotLoadedActivator.poll())
      {
         // multiBody.updateFramesRecursively(); // It is expected that these are updated before this
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
