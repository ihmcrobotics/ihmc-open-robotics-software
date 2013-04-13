package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Collection;
import java.util.LinkedHashMap;

import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.PinJoint;

public class SCSToInverseDynamicsJointMap
{
   private final LinkedHashMap<SixDoFJoint, FloatingJoint> sixDofToFloatingJointMap = new LinkedHashMap<SixDoFJoint, FloatingJoint>();
   private final LinkedHashMap<FloatingJoint, SixDoFJoint> floatingToSixDofToJointMap = new LinkedHashMap<FloatingJoint, SixDoFJoint>();
   
   private final LinkedHashMap<RevoluteJoint, PinJoint> revoluteToPinJointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
   private final LinkedHashMap<PinJoint, RevoluteJoint> pinToRevoluteJointMap = new LinkedHashMap<PinJoint, RevoluteJoint>();

   public void addLinkedJoints(PinJoint currentJoint, RevoluteJoint currentIDJoint)
   {
      revoluteToPinJointMap.put(currentIDJoint, currentJoint);
      pinToRevoluteJointMap.put(currentJoint, currentIDJoint);
   }

   public void addLinkedJoints(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      sixDofToFloatingJointMap.put(sixDoFJoint, floatingJoint);
      floatingToSixDofToJointMap.put(floatingJoint, sixDoFJoint);
   }
   
   
   public RevoluteJoint getInverseDynamicsRevoluteJoint(PinJoint pinJoint)
   {
      return pinToRevoluteJointMap.get(pinJoint);      
   }

   public Collection<PinJoint> getPinJoints()
   {
      return pinToRevoluteJointMap.keySet();
   }
   
   public SixDoFJoint getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
   {
      return floatingToSixDofToJointMap.get(floatingJoint);
   }

   public Collection<? extends FloatingJoint> getFloatingJoints()
   {
      return floatingToSixDofToJointMap.keySet();
   }


   public OneDoFJoint getOneDoFJoint(OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (oneDegreeOfFreedomJoint instanceof PinJoint)
      {
         return getInverseDynamicsRevoluteJoint((PinJoint) oneDegreeOfFreedomJoint);
      }
      
      else
      {
         throw new RuntimeException("Not implemented for SliderJoints yet!");
      }
      
   }
   
   public RigidBody getRigidBody(Joint joint)
   {      
      if (joint instanceof FloatingJoint)
      {
         SixDoFJoint parentSixDoFJoint = floatingToSixDofToJointMap.get(joint);
         return parentSixDoFJoint.getSuccessor();
      }
      else if (joint instanceof PinJoint)
      {
         RevoluteJoint parentRevoluteJoint = pinToRevoluteJointMap.get(joint);
         return parentRevoluteJoint.getSuccessor();
      }
      else
      {
         throw new RuntimeException();
      }   
   }
}
