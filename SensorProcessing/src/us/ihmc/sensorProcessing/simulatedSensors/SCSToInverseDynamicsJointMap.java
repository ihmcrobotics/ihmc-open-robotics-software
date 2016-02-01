package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class SCSToInverseDynamicsJointMap
{
   private final LinkedHashMap<SixDoFJoint, FloatingJoint> sixDofToFloatingJointMap = new LinkedHashMap<SixDoFJoint, FloatingJoint>();
   private final LinkedHashMap<FloatingJoint, SixDoFJoint> floatingToSixDofToJointMap = new LinkedHashMap<FloatingJoint, SixDoFJoint>();

   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> oneDoFToSCSJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint> scsToOneDoFJointMap = new LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint>();

   public void addLinkedJoints(OneDegreeOfFreedomJoint currentJoint, OneDoFJoint currentIDJoint)
   {
      oneDoFToSCSJointMap.put(currentIDJoint, currentJoint);
      scsToOneDoFJointMap.put(currentJoint, currentIDJoint);
   }

   public void addLinkedJoints(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      sixDofToFloatingJointMap.put(sixDoFJoint, floatingJoint);
      floatingToSixDofToJointMap.put(floatingJoint, sixDoFJoint);
   }


   public OneDoFJoint getInverseDynamicsOneDoFJoint(OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      return scsToOneDoFJointMap.get(oneDegreeOfFreedomJoint);
   }

   public Collection<OneDegreeOfFreedomJoint> getSCSOneDegreeOfFreedomJoints()
   {
      return scsToOneDoFJointMap.keySet();
   }

   public SixDoFJoint getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
   {
      return floatingToSixDofToJointMap.get(floatingJoint);
   }

   public Collection<? extends FloatingJoint> getFloatingJoints()
   {
      return floatingToSixDofToJointMap.keySet();
   }

   public RigidBody getRigidBody(Joint joint)
   {
      if (joint instanceof FloatingJoint)
      {
         SixDoFJoint parentSixDoFJoint = floatingToSixDofToJointMap.get(joint);

         return parentSixDoFJoint.getSuccessor();
      }
      else if (joint instanceof OneDegreeOfFreedomJoint)
      {
         OneDoFJoint parentOneDoFJoint = scsToOneDoFJointMap.get(joint);

         return parentOneDoFJoint.getSuccessor();
      }
      else
      {
         throw new RuntimeException();
      }
   }

   public static SCSToInverseDynamicsJointMap createByName(FloatingJoint floatingRootJoint, SixDoFJoint sixDoFRootJoint)
   {
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();
      
      
      InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.computeSubtreeJoints(sixDoFRootJoint.getSuccessor());
      LinkedHashMap<String, OneDoFJoint> inverseDynamicsJointsByName = new LinkedHashMap<String, OneDoFJoint>();
      
      for(InverseDynamicsJoint inverseDynamicsJoint : inverseDynamicsJoints)
      {
         if(inverseDynamicsJoint instanceof OneDoFJoint)
         {
            inverseDynamicsJointsByName.put(inverseDynamicsJoint.getName(), (OneDoFJoint) inverseDynamicsJoint);
         }
         else
         {
            throw new RuntimeException(inverseDynamicsJoint.getName() + " is not an OneDoFJoint");
         }
      }
      
      
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      floatingRootJoint.recursiveGetOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);
      
      

      if (inverseDynamicsJointsByName.size() < oneDegreeOfFreedomJoints.size())
      {
         throw new RuntimeException("oneDoFJoints.length < oneDegreeOfFreedomJoints.size()");
      }

      scsToInverseDynamicsJointMap.addLinkedJoints(floatingRootJoint, sixDoFRootJoint);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         String name = oneDegreeOfFreedomJoint.getName();
         OneDoFJoint oneDoFJoint = inverseDynamicsJointsByName.get(name);

         scsToInverseDynamicsJointMap.addLinkedJoints(oneDegreeOfFreedomJoint, oneDoFJoint);
      }

      return scsToInverseDynamicsJointMap;
   }

}
