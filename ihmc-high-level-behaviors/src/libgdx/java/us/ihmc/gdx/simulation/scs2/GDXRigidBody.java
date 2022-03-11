package us.ihmc.gdx.simulation.scs2;

import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

import java.util.List;
import java.util.stream.Stream;

public class GDXRigidBody implements RigidBodyBasics
{
   private final RigidBodyBasics rigidBody;
   private FrameGDXGraphicsNode visualGraphicsNode;
   private FrameGDXGraphicsNode collisionGraphicsNode;

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
}
