package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

import java.util.List;
import java.util.TreeSet;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Stream;

public class RDXRigidBody implements RigidBodyBasics
{
   private final RigidBodyBasics rigidBody;
   private RDXFrameGraphicsNode visualGraphicsNode;
   private RDXFrameGraphicsNode collisionGraphicsNode;
   private TreeSet<String> rigidBodiesToHide = new TreeSet<>();

   // Store method references to avoid allocation in performance critical code
   // This is done to avoid large duplicated chunks and error-prone sections of code
   private final Function<RDXRigidBody, RDXFrameGraphicsNode> visualGraphicsNodeSelector = RDXRigidBody::getVisualGraphicsNode;
   private final Function<RDXRigidBody, RDXFrameGraphicsNode> collisionGraphicsNodeSelector = RDXRigidBody::getCollisionGraphicsNode;
   private final Consumer<RDXRigidBody> rigidBodyRenderableProvider = this::getRigidBodyRenderables;
   private final Consumer<RDXRigidBody> referenceFrameRenderableProvider = this::getReferenceFrameRenderables;
   // Temporary variables for the above
   private Function<RDXRigidBody, RDXFrameGraphicsNode> nodeSelector;
   private Array<Renderable> renderables;
   private Pool<Renderable> pool;

   public RDXRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
      if (!rigidBody.isRootBody())
         rigidBody.getParentJoint().setSuccessor(this);
   }

   /** Call each time after modifiying joint data before renderering. */
   public void update()
   {
      updateFramesRecursively();
      updateSubtreeGraphics();
   }

   public void updateSubtreeGraphics()
   {
      updateGraphics();
      subtreeStream().forEach(RDXRigidBody::updateGraphics);
   }

   public void updateGraphics()
   {
      if (visualGraphicsNode != null)
         visualGraphicsNode.update();
      if (collisionGraphicsNode != null)
         collisionGraphicsNode.update();

      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         if (childrenJoint instanceof CrossFourBarJointReadOnly)
         {
            CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
            RDXRigidBody bodyDA = (RDXRigidBody) fourBarJoint.getJointA().getSuccessor();
            bodyDA.updateGraphics();
            RDXRigidBody bodyBC = (RDXRigidBody) fourBarJoint.getJointB().getSuccessor();
            bodyBC.updateGraphics();
         }
      }
   }

   public void setVisualGraphics(RDXFrameGraphicsNode visualGraphicsNode)
   {
      this.visualGraphicsNode = visualGraphicsNode;
      // TODO: Set node ids to reflect the name of this rigid-body
   }

   public void setCollisionGraphics(RDXFrameGraphicsNode collisionGraphicsNode)
   {
      this.collisionGraphicsNode = collisionGraphicsNode;
   }

   public void setDiffuseColor(Color color)
   {
      if (visualGraphicsNode != null)
      {
         for (RDXFrameNodePart part : visualGraphicsNode.getParts())
         {
            part.getModelInstance().setDiffuseColor(color);
         }
      }
   }

   public void setOpacity(float opacity)
   {
      if (visualGraphicsNode != null)
      {
         for (RDXFrameNodePart part : visualGraphicsNode.getParts())
         {
            part.getModelInstance().setOpacity(opacity);
         }
      }
   }

   public void setDiffuseColorRecursive(Color color)
   {
      for (RDXRigidBody rigidBody : subtreeIterable())
      {
         rigidBody.setDiffuseColor(color);
         for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
         {
            if (childrenJoint instanceof CrossFourBarJoint fourBarJoint)
            {
               ((RDXRigidBody) fourBarJoint.getJointA().getSuccessor()).setDiffuseColor(color);
               ((RDXRigidBody) fourBarJoint.getJointB().getSuccessor()).setDiffuseColor(color);
            }
         }
      }
   }

   public void setDiffuseColorForBody(String name, Color color)
   {
      for (RDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBody.getName().equals(name))
         {
            rigidBody.setDiffuseColor(color);
            return;
         }
      }
   }

   public void setOpacityForBody(String name, double opacity)
   {
      for (RDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBody.getName().equals(name))
         {
            rigidBody.setOpacity((float) opacity);
            return;
         }
      }
   }

   public void setOpacityRecursive(float opacity)
   {
      for (RDXRigidBody rigidBody : subtreeIterable())
      {
         rigidBody.setOpacity(opacity);
         for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
         {
            if (childrenJoint instanceof CrossFourBarJoint fourBarJoint)
            {
               ((RDXRigidBody) fourBarJoint.getJointA().getSuccessor()).setOpacity(opacity);
               ((RDXRigidBody) fourBarJoint.getJointB().getSuccessor()).setOpacity(opacity);
            }
         }
      }
   }

   public RDXFrameGraphicsNode getVisualGraphicsNode()
   {
      return visualGraphicsNode;
   }

   public RDXFrameGraphicsNode getCollisionGraphicsNode()
   {
      return collisionGraphicsNode;
   }

   public void getVisualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      getRenderables(visualGraphicsNodeSelector, renderables, pool, rigidBodyRenderableProvider);
   }

   public void getVisualReferenceFrameRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      getRenderables(visualGraphicsNodeSelector, renderables, pool, referenceFrameRenderableProvider);
   }

   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      getRenderables(collisionGraphicsNodeSelector, renderables, pool, rigidBodyRenderableProvider);
   }

   public void getCollisionMeshReferenceFrameRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      getRenderables(collisionGraphicsNodeSelector, renderables, pool, referenceFrameRenderableProvider);
   }

   private void getRenderables(Function<RDXRigidBody, RDXFrameGraphicsNode> nodeSelector,
                               Array<Renderable> renderables,
                               Pool<Renderable> pool,
                               Consumer<RDXRigidBody> renderableProvider)
   {
      this.nodeSelector = nodeSelector;
      this.renderables = renderables;
      this.pool = pool;

      for (RDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBodiesToHide != null  && (rigidBodiesToHide.isEmpty() || !rigidBodiesToHide.contains(rigidBody.getName())))
         {
            RDXVisualTools.collectRDXRigidBodiesIncludingPossibleFourBars(rigidBody, renderableProvider);
         }
      }
   }

   private void getRigidBodyRenderables(RDXRigidBody rigidBody)
   {
      RDXFrameGraphicsNode frameGraphicsNode = nodeSelector.apply(rigidBody);
      if (frameGraphicsNode != null)
      {
         frameGraphicsNode.getRenderables(renderables, pool);
      }
   }

   private void getReferenceFrameRenderables(RDXRigidBody rigidBody)
   {
      RDXFrameGraphicsNode frameGraphicsNode = nodeSelector.apply(rigidBody);
      if (frameGraphicsNode != null)
      {
         frameGraphicsNode.getReferenceFrameRenderables(renderables, pool);
      }
   }

   @Override
   public SpatialInertiaBasics getInertia()
   {
      return rigidBody.getInertia();
   }

   @Override
   public MovingReferenceFrame getBodyFixedFrame()
   {
      return rigidBody.getBodyFixedFrame();
   }

   @Override
   public JointBasics getParentJoint()
   {
      return rigidBody.getParentJoint();
   }

   @Override
   public void addChildJoint(JointBasics joint)
   {
      rigidBody.addChildJoint(joint);
   }

   @Override
   public List<JointBasics> getChildrenJoints()
   {
      return rigidBody.getChildrenJoints();
   }

   @Override
   public String toString()
   {
      return rigidBody.toString();
   }

   @Override
   public String getName()
   {
      return rigidBody.getName();
   }

   @Override
   public String getNameId()
   {
      return rigidBody.getNameId();
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public Iterable<? extends RDXRigidBody> subtreeIterable()
   {
      return new RigidBodyIterable<>(RDXRigidBody.class, null, null, this);
   }

   @Override
   public Stream<? extends RDXRigidBody> subtreeStream()
   {
      return SubtreeStreams.from(RDXRigidBody.class, this);
   }

   public TreeSet<String> getRigidBodiesToHide()
   {
      if (rigidBodiesToHide == null)
         rigidBodiesToHide = new TreeSet<>();
      return rigidBodiesToHide;
   }

   /** Convenient for the user to move the graphic around. */
   public SixDoFJoint getFloatingJoint()
   {
      if (getChildrenJoints().get(0) instanceof SixDoFJoint sixDoFJoint)
         return sixDoFJoint;
      return null;
   }
}
