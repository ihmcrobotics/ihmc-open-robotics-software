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

/**
 * Convenient mapping between the joints of a simulated robot and a equivalent inverse dynamics
 * model using {@code InverseDynamicsJoint}s.
 * 
 */
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

   /**
    * Retrieves the corresponding inverse dynamics joint given a simulated joint.
    * 
    * @param oneDegreeOfFreedomJoint the simulated joint. Not modified.
    * @return the corresponding inverse dynamics joint.
    */
   public OneDoFJoint getInverseDynamicsOneDoFJoint(OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      return scsToOneDoFJointMap.get(oneDegreeOfFreedomJoint);
   }

   /**
    * Gets all the simulated joints as a collection.
    * 
    * @return the collection of the registered simulated joints.
    */
   public Collection<OneDegreeOfFreedomJoint> getSCSOneDegreeOfFreedomJoints()
   {
      return scsToOneDoFJointMap.keySet();
   }

   /**
    * Gets all the inverse dynamics joints as a collection.
    * 
    * @return the collection of the registered inverse dynamics joints.
    */
   public Collection<OneDoFJoint> getInverseDynamicsOneDoFJoints()
   {
      return scsToOneDoFJointMap.values();
   }

   /**
    * Retrieves the corresponding inverse dynamics floating joint given the simulated joint.
    * 
    * @param floatingJoint the simulated floating joint.
    * @return the corresponding inverse dynamics floating joint.
    */
   public FloatingInverseDynamicsJoint getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
   {
      return floatingToSixDofToJointMap.get(floatingJoint);
   }

   /**
    * Gets all the simulated floating joints as a collection.
    * 
    * @return the collection of the registered simulated floating joints.
    */
   public Collection<? extends FloatingJoint> getFloatingJoints()
   {
      return floatingToSixDofToJointMap.keySet();
   }

   /**
    * Retrieves the child or successor {@code RigidBody} from a simulated joint.
    * <p>
    * This method first retrieves the corresponding inverse dynamics joint and then returns its
    * successor.
    * </p>
    * 
    * @param joint the simulated joint.
    * @return the child/successor rigid-body.
    */
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

   /**
    * Retrieves the corresponding simulated joint given an inverse dynamics joint.
    * 
    * @param the inverse dynamics joint. Not modified.
    * @return the corresponding simulated joint.
    */
   public OneDegreeOfFreedomJoint getSimulatedOneDegreeOfFreedomJoint(OneDoFJoint inverseDynamicsJoint)
   {
      return oneDoFToSCSJointMap.get(inverseDynamicsJoint);
   }

   /**
    * Given a simulated and inverse dynamics robot models, this method creates map associating
    * simulated joints with inverse dynamics by name.
    * 
    * @param floatingRootJoint the simulated floating joint. Not modified.
    * @param sixDoFRootJoint the inverse dynamics floating joint. Not modified.
    * @return
    */
   public static SCSToInverseDynamicsJointMap createByName(FloatingJoint floatingRootJoint, FloatingInverseDynamicsJoint sixDoFRootJoint)
   {
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();

      InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.computeSubtreeJoints(sixDoFRootJoint.getSuccessor());
      LinkedHashMap<String, OneDoFJoint> inverseDynamicsJointsByName = new LinkedHashMap<String, OneDoFJoint>();

      for (InverseDynamicsJoint inverseDynamicsJoint : inverseDynamicsJoints)
      {
         if (inverseDynamicsJoint instanceof OneDoFJoint)
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
         if (inverseDynamicsJointsByName.containsKey(name))
         {
            OneDoFJoint oneDoFJoint = inverseDynamicsJointsByName.get(name);

            scsToInverseDynamicsJointMap.addLinkedJoints(oneDegreeOfFreedomJoint, oneDoFJoint);
         }
      }

      return scsToInverseDynamicsJointMap;
   }
}
