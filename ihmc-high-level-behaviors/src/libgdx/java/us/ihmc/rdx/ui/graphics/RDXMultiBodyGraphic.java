package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Vector3D;
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

import java.util.concurrent.Executor;

public class RDXMultiBodyGraphic extends RDXVisualizer implements RenderableProvider
{
   protected RDXRigidBody multiBody;
   private final Activator robotLoadedActivator = new Activator();

   public RDXMultiBodyGraphic(String title)
   {
      super(title);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody)
   {
      loadRobotModelAndGraphics(robotDefinition, originalRootBody, false);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody, boolean scaleALittleBigger)
   {
      if (multiBody != null)
         multiBody.destroy();

      ThreadTools.startAsDaemon(() ->
      {
         multiBody = loadRigidBody(originalRootBody, robotDefinition, scaleALittleBigger);
         robotLoadedActivator.activate();
      }, getClass().getSimpleName() + "Loading");
   }

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition)
   {
      return loadRigidBody(rigidBody, robotDefinition, false);
   }

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition, boolean scaleALittleBigger)
   {
      RDXRigidBody RDXRigidBody;
      Executor executorToRunLaterOnThreadWithGraphicsContext = Gdx.app::postRunnable;
      Vector3D scaleVector = scaleALittleBigger ? new Vector3D(1.1, 1.1, 1.1) : new Vector3D(1.0, 1.0, 1.0);
      RDXRigidBody = RDXMultiBodySystemFactories.toGDXRigidBody(rigidBody,
                                                                robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                executorToRunLaterOnThreadWithGraphicsContext,
                                                                robotDefinition.getResourceClassLoader(),
                                                                scaleVector);

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
                                                                                             scaleVector));
            fourBarJoint.getJointB().setSuccessor(RDXMultiBodySystemFactories.toGDXRigidBody(fourBarJoint.getBodyBC(),
                                                                                             fourBarJointDefinition.getBodyBC(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             robotDefinition.getResourceClassLoader(),
                                                                                             scaleVector));
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

   public RDXRigidBody getMultiBody()
   {
      return multiBody;
   }
}
