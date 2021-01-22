package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
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
   private final LinkedHashMap<FloatingJointBasics, FloatingJoint> sixDofToFloatingJointMap = new LinkedHashMap<FloatingJointBasics, FloatingJoint>();
   private final LinkedHashMap<FloatingJoint, FloatingJointBasics> floatingToSixDofToJointMap = new LinkedHashMap<FloatingJoint, FloatingJointBasics>();

   private final LinkedHashMap<OneDoFJointBasics, OneDegreeOfFreedomJoint> oneDoFToSCSJointMap = new LinkedHashMap<OneDoFJointBasics, OneDegreeOfFreedomJoint>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJointBasics> scsToOneDoFJointMap = new LinkedHashMap<OneDegreeOfFreedomJoint, OneDoFJointBasics>();

   public void addLinkedJoints(OneDegreeOfFreedomJoint currentJoint, OneDoFJointBasics currentIDJoint)
   {
      oneDoFToSCSJointMap.put(currentIDJoint, currentJoint);
      scsToOneDoFJointMap.put(currentJoint, currentIDJoint);
   }

   public void addLinkedJoints(FloatingJoint floatingJoint, FloatingJointBasics sixDoFJoint)
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
   public OneDoFJointBasics getInverseDynamicsOneDoFJoint(OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
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
   public Collection<OneDoFJointBasics> getInverseDynamicsOneDoFJoints()
   {
      return scsToOneDoFJointMap.values();
   }

   /**
    * Retrieves the corresponding inverse dynamics floating joint given the simulated joint.
    * 
    * @param floatingJoint the simulated floating joint.
    * @return the corresponding inverse dynamics floating joint.
    */
   public FloatingJointBasics getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
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
   public RigidBodyBasics getRigidBody(Joint joint)
   {
      if (joint instanceof FloatingJoint)
      {
         FloatingJointBasics parentSixDoFJoint = floatingToSixDofToJointMap.get(joint);

         return parentSixDoFJoint.getSuccessor();
      }
      else if (joint instanceof OneDegreeOfFreedomJoint)
      {
         OneDoFJointBasics parentOneDoFJoint = scsToOneDoFJointMap.get(joint);

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
   public OneDegreeOfFreedomJoint getSimulatedOneDegreeOfFreedomJoint(OneDoFJointBasics inverseDynamicsJoint)
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
   public static SCSToInverseDynamicsJointMap createByName(FloatingJoint floatingRootJoint, FloatingJointBasics sixDoFRootJoint)
   {
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();

      JointBasics[] inverseDynamicsJoints = MultiBodySystemTools.collectSubtreeJoints(sixDoFRootJoint.getSuccessor());
      LinkedHashMap<String, OneDoFJointBasics> inverseDynamicsJointsByName = new LinkedHashMap<String, OneDoFJointBasics>();

      for (JointBasics inverseDynamicsJoint : inverseDynamicsJoints)
      {
         if (inverseDynamicsJoint instanceof InvertedFourBarJoint)
         {
            RevoluteJointBasics jointA = ((InvertedFourBarJoint) inverseDynamicsJoint).getJointA();
            RevoluteJointBasics jointB = ((InvertedFourBarJoint) inverseDynamicsJoint).getJointB();
            RevoluteJointBasics jointC = ((InvertedFourBarJoint) inverseDynamicsJoint).getJointC();
            RevoluteJointBasics jointD = ((InvertedFourBarJoint) inverseDynamicsJoint).getJointD();
            inverseDynamicsJointsByName.put(jointA.getName(), jointA);
            inverseDynamicsJointsByName.put(jointB.getName(), jointB);
            inverseDynamicsJointsByName.put(jointC.getName(), jointC);
            inverseDynamicsJointsByName.put(jointD.getName(), jointD);
         }
         else if (inverseDynamicsJoint instanceof OneDoFJointBasics)
         {
            inverseDynamicsJointsByName.put(inverseDynamicsJoint.getName(), (OneDoFJointBasics) inverseDynamicsJoint);
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
            OneDoFJointBasics oneDoFJoint = inverseDynamicsJointsByName.get(name);

            scsToInverseDynamicsJointMap.addLinkedJoints(oneDegreeOfFreedomJoint, oneDoFJoint);
         }
      }

      return scsToInverseDynamicsJointMap;
   }
}
