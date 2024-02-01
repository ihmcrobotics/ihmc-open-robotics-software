package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.*;
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
      setSceneLevels(RDXSceneLevel.GROUND_TRUTH, RDXSceneLevel.VIRTUAL);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics originalRootBody)
   {
      loadRobotModelAndGraphics(robotDefinition, originalRootBody, RDXVisualTools.NO_SCALING, false);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition,
                                         RigidBodyBasics originalRootBody,
                                         double scaleFactor,
                                         boolean createReferenceFrameGraphics)
   {
      ThreadTools.startAsDaemon(() ->
      {
         multiBody = loadRigidBody(originalRootBody, robotDefinition, scaleFactor, createReferenceFrameGraphics);
         robotLoadedActivator.activate();
      }, getClass().getSimpleName() + "Loading");
   }

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition)
   {
      return loadRigidBody(rigidBody, robotDefinition, RDXVisualTools.NO_SCALING, false);
   }

   private RDXRigidBody loadRigidBody(RigidBodyBasics rigidBody, RobotDefinition robotDefinition, double scaleFactor, boolean createReferenceFrameGraphics)
   {
      RDXRigidBody rdxRigidBody;
      Executor executorToRunLaterOnThreadWithGraphicsContext = Gdx.app::postRunnable;
      rdxRigidBody = RDXMultiBodySystemFactories.toRDXRigidBody(rigidBody,
                                                                robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                                                executorToRunLaterOnThreadWithGraphicsContext,
                                                                scaleFactor,
                                                                createReferenceFrameGraphics);

      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         if (childrenJoint instanceof CrossFourBarJointReadOnly)
         {
            CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
            CrossFourBarJointDefinition fourBarJointDefinition = (CrossFourBarJointDefinition) robotDefinition.getJointDefinition(fourBarJoint.getName());

            fourBarJoint.getJointA().setSuccessor(RDXMultiBodySystemFactories.toRDXRigidBody(fourBarJoint.getBodyDA(),
                                                                                             fourBarJointDefinition.getBodyDA(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             scaleFactor,
                                                                                             createReferenceFrameGraphics));
            fourBarJoint.getJointB().setSuccessor(RDXMultiBodySystemFactories.toRDXRigidBody(fourBarJoint.getBodyBC(),
                                                                                             fourBarJointDefinition.getBodyBC(),
                                                                                             executorToRunLaterOnThreadWithGraphicsContext,
                                                                                             scaleFactor,
                                                                                             createReferenceFrameGraphics));
         }

         childrenJoint.setSuccessor(loadRigidBody(childrenJoint.getSuccessor(), robotDefinition, scaleFactor, createReferenceFrameGraphics));
      }

      return rdxRigidBody;
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

   public void getVisualReferenceFrameRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && robotLoadedActivator.poll() && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         multiBody.getVisualReferenceFrameRenderables(renderables, pool);
      }
   }

   public void destroy()
   {

   }

   public void setOpacity(float opacity)
   {
      for (RDXRigidBody rigidBody : multiBody.subtreeIterable())
      {
         RDXVisualTools.collectRDXRigidBodiesIncludingPossibleFourBars(rigidBody, rdxRigidBody ->
         {
            RDXFrameGraphicsNode visualGraphicsNode = rdxRigidBody.getVisualGraphicsNode();
            if (visualGraphicsNode != null) // This is null for the elevator
            {
               for (RDXFrameNodePart part : visualGraphicsNode.getParts())
               {
                  part.getModelInstance().setOpacity(opacity);
               }
            }
         });
      }
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
