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
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.thread.Activator;

import java.util.concurrent.Executor;

public class GDXMultiBodyGraphic extends ImGuiGDXVisualizer implements RenderableProvider
{
   protected GDXRigidBody multiBody;
   private final Activator robotLoadedActivator = new Activator();

   public GDXMultiBodyGraphic(String title)
   {
      super(title);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody)
   {
      loadRobotModelAndGraphics(robotDefinition, originalRootBody, false);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody, boolean scale)
   {
      if (multiBody != null)
         multiBody.destroy();

      ThreadTools.startAsDaemon(() ->
      {
         multiBody = loadRigidBody(originalRootBody, robotDefinition, scale);
         robotLoadedActivator.activate();
      }, getClass().getSimpleName() + "Loading");
   }

   private GDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition)
   {
      return loadRigidBody(rigidBody, robotDefinition, false);
   }

   private GDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition, boolean scale)
   {
      GDXRigidBody gdxRigidBody;
      Executor executorToRunLaterOnThreadWithGraphicsContext = Gdx.app::postRunnable;
      if (scale)
      {
         gdxRigidBody = GDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
                                                                   robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                   executorToRunLaterOnThreadWithGraphicsContext,
                                                                   robotDefinition.getResourceClassLoader(),
                                                                   1.1f, 1.1f, 1.1f);
      }

      else
      {
         gdxRigidBody = GDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
                                                                   robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                   executorToRunLaterOnThreadWithGraphicsContext,
                                                                   robotDefinition.getResourceClassLoader());
      }

      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         if (childrenJoint instanceof CrossFourBarJointReadOnly)
         {
            CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
            CrossFourBarJointDefinition fourBarJointDefinition = (CrossFourBarJointDefinition) robotDefinition.getJointDefinition(fourBarJoint.getName());

            fourBarJoint.getJointA().setSuccessor(GDXMultiBodySystemFactories.toGDXRigidBody(fourBarJoint.getBodyDA(),
                                                                                             fourBarJointDefinition.getBodyDA(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             robotDefinition.getResourceClassLoader(),
                                                                                             1.1f, 1.1f, 1.1f));
            fourBarJoint.getJointB().setSuccessor(GDXMultiBodySystemFactories.toGDXRigidBody(fourBarJoint.getBodyBC(),
                                                                                             fourBarJointDefinition.getBodyBC(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             robotDefinition.getResourceClassLoader(),
                                                                                             0.0f, 0.0f, 0.0f));
         }

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

   public Activator getRobotLoadedActivator()
   {
      return robotLoadedActivator;
   }

   public GDXRigidBody getMultiBody()
   {
      return multiBody;
   }
}
