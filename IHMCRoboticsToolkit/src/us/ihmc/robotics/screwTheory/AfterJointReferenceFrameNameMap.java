package us.ihmc.robotics.screwTheory;

import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class AfterJointReferenceFrameNameMap
{
   private final LinkedHashMap<String, ReferenceFrame> afterJointReferenceFrames = new LinkedHashMap<String, ReferenceFrame>();
   
   public AfterJointReferenceFrameNameMap(RigidBody base)
   {
      InverseDynamicsJoint[] joints = ScrewTools.computeSubtreeJoints(base);
      
      for(InverseDynamicsJoint joint : joints)
      {
         afterJointReferenceFrames.put(joint.getFrameAfterJoint().getName(), joint.getFrameAfterJoint());
      }
   }
   
   public AfterJointReferenceFrameNameMap(List<ReferenceFrame> frames)
   {
      for(ReferenceFrame frame : frames)
      {
         afterJointReferenceFrames.put(frame.getName(), frame);
      }
   }

   public ReferenceFrame getFrameByName(String name)
   {
      final ReferenceFrame referenceFrame = afterJointReferenceFrames.get(name);
      if(referenceFrame == null)
      {
         throw new RuntimeException("Unknown frame " + name);
      }
      return referenceFrame;
   }
}
