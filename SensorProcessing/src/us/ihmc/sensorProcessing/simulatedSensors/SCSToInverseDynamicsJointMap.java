package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;

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

   public static SCSToInverseDynamicsJointMap createByName(FloatingJoint floatingRootJoint, SixDoFJoint sixDoFRootJoint,
           Collection<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints, OneDoFJoint[] oneDoFJoints)
   {
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();

      if (oneDoFJoints.length != oneDegreeOfFreedomJoints.size())
      {
         throw new RuntimeException("oneDoFJoints.length != oneDegreeOfFreedomJoints.size()");
      }

      scsToInverseDynamicsJointMap.addLinkedJoints(floatingRootJoint, sixDoFRootJoint);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         String name = oneDegreeOfFreedomJoint.getName();
         OneDoFJoint oneDoFJoint = findJointWithName(name, oneDoFJoints);

         scsToInverseDynamicsJointMap.addLinkedJoints(oneDegreeOfFreedomJoint, oneDoFJoint);
      }

      return scsToInverseDynamicsJointMap;
   }

   private static OneDoFJoint findJointWithName(String name, OneDoFJoint[] oneDoFJoints)
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (oneDoFJoint.getName().equals(name))
         {
            return oneDoFJoint;
         }
      }

      throw new RuntimeException("Could not find OneDoFJoint with name : " + name);
   }
}
