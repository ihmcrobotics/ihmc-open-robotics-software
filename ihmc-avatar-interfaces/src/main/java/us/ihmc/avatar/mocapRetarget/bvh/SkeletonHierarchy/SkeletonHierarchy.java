package us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

public class SkeletonHierarchy {
   Map<String, JointInfo> joints = new LinkedHashMap<>();
   String rootName;

   public void addJoint(JointInfo joint) {
      joints.put(joint.name(), joint);
   }

   public JointInfo getJoint(String jointName){
      return joints.get(jointName);
   }

   public Set<String> getJointNames(){
      return joints.keySet();
   }

   public int getTotalChannels() {
      int channelCount = 0;
      for (Map.Entry<String, JointInfo> entry : joints.entrySet()) {
         channelCount += entry.getValue().channelCount();
      }
      return channelCount;
   }

   public void setRootName(String name) {
      rootName = name;
   }
}

