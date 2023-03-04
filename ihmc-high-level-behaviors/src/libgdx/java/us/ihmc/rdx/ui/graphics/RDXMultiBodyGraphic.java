package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.thread.Activator;

import java.util.Set;
import java.util.concurrent.Executor;

public class RDXMultiBodyGraphic extends RDXVisualizer
{
   protected RDXRigidBody multiBody;
   private final Activator robotLoadedActivator = new Activator();

   public RDXMultiBodyGraphic(String title)
   {
      super(title);
      setSceneLevels(RDXSceneLevel.GROUND_TRUTH, RDXSceneLevel.MODEL);
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

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition)
   {
      return loadRigidBody(rigidBody, robotDefinition, false);
   }

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition, boolean scale)
   {
      RDXRigidBody RDXRigidBody;
      Executor executorToRunLaterOnThreadWithGraphicsContext = Gdx.app::postRunnable;
      if (scale)
      {
         RDXRigidBody = RDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
                                                                   robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                   executorToRunLaterOnThreadWithGraphicsContext,
                                                                   robotDefinition.getResourceClassLoader(),
                                                                   1.1f, 1.1f, 1.1f);
      }

      else
      {
         RDXRigidBody = RDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
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

            fourBarJoint.getJointA().setSuccessor(RDXMultiBodySystemFactories.toGDXRigidBody(fourBarJoint.getBodyDA(),
                                                                                             fourBarJointDefinition.getBodyDA(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             robotDefinition.getResourceClassLoader(),
                                                                                             1.1f, 1.1f, 1.1f));
            fourBarJoint.getJointB().setSuccessor(RDXMultiBodySystemFactories.toGDXRigidBody(fourBarJoint.getBodyBC(),
                                                                                             fourBarJointDefinition.getBodyBC(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             robotDefinition.getResourceClassLoader(),
                                                                                             0.0f, 0.0f, 0.0f));
         }

         childrenJoint.setSuccessor(loadRigidBody(childrenJoint.getSuccessor(), robotDefinition));
      }

      return RDXRigidBody;
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && robotLoadedActivator.poll() && sceneLevelCheck(sceneLevels))
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

   public RDXRigidBody getMultiBody()
   {
      return multiBody;
   }
}
