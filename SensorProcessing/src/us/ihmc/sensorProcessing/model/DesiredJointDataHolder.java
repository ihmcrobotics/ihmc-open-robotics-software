package us.ihmc.sensorProcessing.model;

import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class DesiredJointDataHolder
{
   private final OneDoFJoint[] joints;
   private final ArrayList<ImmutablePair<OneDoFJoint, DesiredJointData>> desiredJointDataList = new ArrayList<>();
   private final HashMap<OneDoFJoint, DesiredJointData> desiredJointData = new HashMap<>();
   
   public DesiredJointDataHolder(OneDoFJoint[] joints)
   {
      this.joints = joints;
      for (int i = 0; i < joints.length; i++)
      {
         DesiredJointData value = new DesiredJointData();
         desiredJointDataList.add(new ImmutablePair<OneDoFJoint, DesiredJointDataHolder.DesiredJointData>(joints[i], value));
         desiredJointData.put(joints[i], value);
      }
   }

   public OneDoFJoint[] getJoints()
   {
      return joints;
   }
   
   public void updateFromModel()
   {
      
      for (int i = 0; i < desiredJointDataList.size(); i++)
      {
         ImmutablePair<OneDoFJoint, DesiredJointData> desiredJointData = desiredJointDataList.get(i);
         OneDoFJoint joint = desiredJointData.getLeft();
         DesiredJointData data = desiredJointData.getRight();
         data.setQddDesired(joint.getQddDesired());
         data.setTauDesired(joint.getTau());
         data.setPositionDesired(joint.getqDesired());
      }
   }

   public DesiredJointData get(OneDoFJoint joint)
   {
      return desiredJointData.get(joint);
   }
   
   public DesiredJointData get(int i)
   {
      return desiredJointDataList.get(i).getRight();
   }

   public static class DesiredJointData
   {
      private double qddDesired;
      private double tauDesired;
      private double positionDesired;

      public double getQddDesired()
      {
         return qddDesired;
      }

      public void setQddDesired(double qddDesired)
      {
         this.qddDesired = qddDesired;
      }

      public double getTauDesired()
      {
         return tauDesired;
      }

      public void setTauDesired(double tauDesired)
      {
         this.tauDesired = tauDesired;
      }

      public double getPositionDesired()
      {
         return positionDesired;
      }

      public void setPositionDesired(double positionDesired)
      {
         this.positionDesired = positionDesired;
      }

      public void set(DesiredJointData desiredJointData)
      {
         setTauDesired(desiredJointData.getTauDesired());
         setQddDesired(desiredJointData.getQddDesired());
         setPositionDesired(desiredJointData.getPositionDesired());
      }

   }
}
