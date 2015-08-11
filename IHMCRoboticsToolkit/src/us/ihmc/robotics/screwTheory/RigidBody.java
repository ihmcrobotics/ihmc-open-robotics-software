package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RigidBody
{
   private final RigidBodyInertia inertia;
   private final ReferenceFrame bodyFixedFrame;
   private final InverseDynamicsJoint parentJoint;
   private final FrameVector comOffset;
   private final ArrayList<InverseDynamicsJoint> childrenJoints = new ArrayList<InverseDynamicsJoint>();
   private final List<InverseDynamicsJoint> childrenJointsReadOnly = Collections.unmodifiableList(childrenJoints);
   private final String name;

   public RigidBody(String name, ReferenceFrame rootBodyFrame)    // root body constructor
   {
      this.name = name;
      this.inertia = null;
      this.bodyFixedFrame = rootBodyFrame;
      this.parentJoint = null;
      this.comOffset = null;
   }

   public RigidBody(String name, RigidBodyInertia inertia, InverseDynamicsJoint parentJoint)
   {
      inertia.getBodyFrame().checkReferenceFrameMatch(inertia.getExpressedInFrame());    // inertia should be expressed in body frame, otherwise it will change
      this.name = name;
      this.inertia = inertia;
      this.bodyFixedFrame = inertia.getBodyFrame();
      this.parentJoint = parentJoint;
      this.parentJoint.setSuccessor(this);

      this.comOffset = new FrameVector(parentJoint.getFrameAfterJoint());
      RigidBodyTransform comTransform = inertia.getBodyFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
      comTransform.get(comOffset.getVector());
      
      
   }

   public RigidBodyInertia getInertia()
   {
      return inertia;
   }

   public RigidBodyInertia getInertiaCopy()
   {
      return new RigidBodyInertia(getInertia());
   }

   public ReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   public InverseDynamicsJoint getParentJoint()
   {
      return parentJoint;
   }

   public void addChildJoint(InverseDynamicsJoint joint)
   {
      this.childrenJoints.add(joint);
   }

   public List<InverseDynamicsJoint> getChildrenJoints()
   {
      return childrenJointsReadOnly;
   }

   public boolean hasChildrenJoints()
   {
      return !childrenJoints.isEmpty();
   }

   public boolean isRootBody()
   {
      return parentJoint == null;
   }

   public String getName()
   {
      return name;
   }

   public void packCoMOffset(FramePoint comOffsetToPack)
   {
      comOffsetToPack.setIncludingFrame(comOffset);
   }
   
   public void setCoMOffset(FramePoint comOffset)
   {
      this.comOffset.set(comOffset);
   }

   public void updateFramesRecursively()
   {
      this.bodyFixedFrame.update();

//      for (InverseDynamicsJoint joint : childrenJoints)
      for(int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
      {
         childrenJoints.get(childIndex).updateFramesRecursively();
      }
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append(name);
//      builder.append(name + "\n");
//      builder.append("Root body: " + isRootBody() + "\n");

//      builder.append("Children joints: ");
//
//      if (childrenJoints.isEmpty())
//      {
//         builder.append("none");
//      }
//      else
//      {
//         Iterator<InverseDynamicsJoint> iterator = childrenJoints.iterator();
//         while (iterator.hasNext())
//         {
//            InverseDynamicsJoint joint = iterator.next();
//            builder.append(joint.getName());
//
//            if (iterator.hasNext())
//            {
//               builder.append(", ");
//            }
//         }
//      }

      return builder.toString();
   }
}
