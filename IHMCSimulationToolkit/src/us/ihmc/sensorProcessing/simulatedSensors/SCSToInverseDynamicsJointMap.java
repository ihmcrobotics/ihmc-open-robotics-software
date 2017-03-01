package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SCSToInverseDynamicsJointMap
{
   private final LinkedHashMap<FloatingInverseDynamicsJoint, FloatingJoint> sixDofToFloatingJointMap = new LinkedHashMap<FloatingInverseDynamicsJoint, FloatingJoint>();
   private final LinkedHashMap<FloatingJoint, FloatingInverseDynamicsJoint> floatingToSixDofToJointMap = new LinkedHashMap<FloatingJoint, FloatingInverseDynamicsJoint>();

   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> oneDoFToSCSJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint> scsToOneDoFJointMap = new LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJoint>();

   public void addLinkedJoints(OneDegreeOfFreedomJoint currentJoint, OneDoFJoint currentIDJoint)
   {
      oneDoFToSCSJointMap.put(currentIDJoint, currentJoint);
      scsToOneDoFJointMap.put(currentJoint, currentIDJoint);
   }

   public void addLinkedJoints(FloatingJoint floatingJoint, FloatingInverseDynamicsJoint sixDoFJoint)
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

   public FloatingInverseDynamicsJoint getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
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
         FloatingInverseDynamicsJoint parentSixDoFJoint = floatingToSixDofToJointMap.get(joint);

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

   public static SCSToInverseDynamicsJointMap createByName(FloatingJoint floatingRootJoint, FloatingInverseDynamicsJoint sixDoFRootJoint)
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
      
      

//      if (inverseDynamicsJointsByName.size() < oneDegreeOfFreedomJoints.size())
//      {
//         throw new RuntimeException("oneDoFJoints.length < oneDegreeOfFreedomJoints.size()");
//      }

      scsToInverseDynamicsJointMap.addLinkedJoints(floatingRootJoint, sixDoFRootJoint);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         String name = oneDegreeOfFreedomJoint.getName();
         if(inverseDynamicsJointsByName.containsKey(name))
         {
            OneDoFJoint oneDoFJoint = inverseDynamicsJointsByName.get(name);
   
            scsToInverseDynamicsJointMap.addLinkedJoints(oneDegreeOfFreedomJoint, oneDoFJoint);
         }
      }

      return scsToInverseDynamicsJointMap;
   }

}
