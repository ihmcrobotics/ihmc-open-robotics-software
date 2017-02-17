package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

public abstract class AbstractInverseDynamicsCopier
{  
   private final ArrayList<ImmutablePair<InverseDynamicsJoint, InverseDynamicsJoint>> jointPairs = new ArrayList<ImmutablePair<InverseDynamicsJoint,InverseDynamicsJoint>>();
   
   
   public AbstractInverseDynamicsCopier(RigidBody originalBody, RigidBody targetBody)
   {
      setRigidBodies(originalBody, targetBody);
   }

   public void setRigidBodies(RigidBody originalBody, RigidBody targetBody)
   {
      jointPairs.clear();
      InverseDynamicsJoint[] originalJoints = ScrewTools.computeSubtreeJoints(originalBody);
      InverseDynamicsJoint[] targetJoints = ScrewTools.computeSubtreeJoints(targetBody);
      
      for(int i = 0; i < originalJoints.length; i++)
      {      
         InverseDynamicsJoint originalJoint = originalJoints[i];
         InverseDynamicsJoint targetJoint = targetJoints[i];
         
         areJointsTheSame(originalJoint, targetJoint);
         
         ImmutablePair<InverseDynamicsJoint, InverseDynamicsJoint> jointPair = new ImmutablePair<InverseDynamicsJoint, InverseDynamicsJoint>(originalJoint, targetJoint);
         
         jointPairs.add(jointPair);
      }
   }
   
   
   public final void copy()
   {
      for(int i = 0; i <  jointPairs.size(); i++)
      {
         ImmutablePair<InverseDynamicsJoint,InverseDynamicsJoint> jointPair = jointPairs.get(i);
         InverseDynamicsJoint originalJoint = jointPair.getLeft();
         InverseDynamicsJoint targetJoint = jointPair.getRight();
         
         copyJoint(originalJoint, targetJoint);
         
      }
   }
   
   protected abstract void copyJoint(InverseDynamicsJoint originalJoint, InverseDynamicsJoint targetJoint);
   
   
   private static final void areJointsTheSame(InverseDynamicsJoint originalJoint, InverseDynamicsJoint targetJoint)
   {
      if(!(originalJoint.getClass().equals(targetJoint.getClass()) && 
            originalJoint.getName().equals(targetJoint.getName())))
      {
         throw new RuntimeException(originalJoint.getName() + " differs from " + targetJoint);
      }
      
   }
}
