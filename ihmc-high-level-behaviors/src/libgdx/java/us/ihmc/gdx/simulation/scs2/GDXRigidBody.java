package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

import java.util.List;
import java.util.TreeSet;
import java.util.stream.Stream;

public class GDXRigidBody implements RigidBodyBasics
{
   private final RigidBodyBasics rigidBody;
   private FrameGDXGraphicsNode visualGraphicsNode;
   private FrameGDXGraphicsNode collisionGraphicsNode;
   private TreeSet<String> rigidBodiesToHide = new TreeSet<>();

   public GDXRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
      if (!rigidBody.isRootBody())
         rigidBody.getParentJoint().setSuccessor(this);
   }

   public void updateSubtreeGraphics()
   {
      updateGraphics();
      subtreeStream().forEach(GDXRigidBody::updateGraphics);
   }

   public void updateGraphics()
   {
      if (visualGraphicsNode != null)
         visualGraphicsNode.updatePose();
      if (collisionGraphicsNode != null)
         collisionGraphicsNode.updatePose();

      for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
      {
         if (childrenJoint instanceof CrossFourBarJointReadOnly)
         {
            CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
            GDXRigidBody bodyDA = (GDXRigidBody) fourBarJoint.getJointA().getSuccessor();
            bodyDA.updateGraphics();
            GDXRigidBody bodyBC = (GDXRigidBody) fourBarJoint.getJointB().getSuccessor();
            bodyBC.updateGraphics();
         }
      }
   }

   public void setVisualGraphics(FrameGDXGraphicsNode visualGraphicsNode)
   {
      this.visualGraphicsNode = visualGraphicsNode;
      // TODO: Set node ids to reflect the name of this rigid-body
   }

   public void setCollisionGraphics(FrameGDXGraphicsNode collisionGraphicsNode)
   {
      this.collisionGraphicsNode = collisionGraphicsNode;
   }

   public FrameGDXGraphicsNode getVisualGraphicsNode()
   {
      return visualGraphicsNode;
   }

   public FrameGDXGraphicsNode getCollisionGraphicsNode()
   {
      return collisionGraphicsNode;
   }

   public void getVisualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBodiesToHide != null  && (rigidBodiesToHide.isEmpty() || !rigidBodiesToHide.contains(rigidBody.getName())))
         {
            if (rigidBody.getVisualGraphicsNode() != null)
            {
               rigidBody.getVisualGraphicsNode().getRenderables(renderables, pool);
            }
            for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
            {
               if (childrenJoint instanceof CrossFourBarJointReadOnly)
               {
                  CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
                  GDXRigidBody bodyDA = (GDXRigidBody) fourBarJoint.getJointA().getSuccessor();
                  if (bodyDA.getVisualGraphicsNode() != null)
                  {
                     bodyDA.getVisualGraphicsNode().getRenderables(renderables, pool);
                  }
                  GDXRigidBody bodyBC = (GDXRigidBody) fourBarJoint.getJointB().getSuccessor();
                  if (bodyBC.getVisualGraphicsNode() != null)
                  {
                     bodyBC.getVisualGraphicsNode().getRenderables(renderables, pool);
                  }
               }
            }
         }
      }
   }

   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBody.getCollisionGraphicsNode() != null)
         {
            rigidBody.getCollisionGraphicsNode().getRenderables(renderables, pool);
         }
         for (JointBasics childrenJoint : rigidBody.getChildrenJoints())
         {
            if (childrenJoint instanceof CrossFourBarJointReadOnly)
            {
               CrossFourBarJoint fourBarJoint = (CrossFourBarJoint) childrenJoint;
               GDXRigidBody bodyDA = (GDXRigidBody) fourBarJoint.getJointA().getSuccessor();
               if (bodyDA.getCollisionGraphicsNode() != null)
               {
                  bodyDA.getCollisionGraphicsNode().getRenderables(renderables, pool);
               }
               GDXRigidBody bodyBC = (GDXRigidBody) fourBarJoint.getJointB().getSuccessor();
               if (bodyBC.getCollisionGraphicsNode() != null)
               {
                  bodyBC.getCollisionGraphicsNode().getRenderables(renderables, pool);
               }
            }
         }
      }
   }

   public void destroy()
   {
      for (GDXRigidBody rigidBody : subtreeIterable())
      {
         if (rigidBody.getVisualGraphicsNode() != null)
         {
            rigidBody.getVisualGraphicsNode().dispose();
         }
         if (rigidBody.getCollisionGraphicsNode() != null)
         {
            rigidBody.getCollisionGraphicsNode().dispose();
         }
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

   @Override
   public Iterable<? extends GDXRigidBody> subtreeIterable()
   {
      return new RigidBodyIterable<>(GDXRigidBody.class, null, this);
   }

   @Override
   public Stream<? extends GDXRigidBody> subtreeStream()
   {
      return SubtreeStreams.from(GDXRigidBody.class, this);
   }

   public void scale(float x, float y, float z)
   {
      if (visualGraphicsNode != null)
         visualGraphicsNode.scale(x, y, z);
      if (collisionGraphicsNode != null)
         collisionGraphicsNode.scale(x, y, z);
   }

   public TreeSet<String> getRigidBodiesToHide()
   {
      if (rigidBodiesToHide == null)
         rigidBodiesToHide = new TreeSet<>();
      return rigidBodiesToHide;
   }
}
