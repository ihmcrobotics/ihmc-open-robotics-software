package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;

public class JointPDHighLevelHumanoidController extends HighLevelBehavior
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable gainScaling = new DoubleYoVariable("hl_gainScaling", registry);
   
   private final ArrayList<OneDoFJoint> oneDoFJoints; 
   
   private final HashMap<OneDoFJoint, DoubleYoVariable> kpJoints, kdJoints, q_dJoints, desiredTorques;
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> qd_filtered_Joints;
   private final HashMap<OneDoFJoint, YoFunctionGenerator> functionGenerators;
   private final DoubleYoVariable alphaQD;
   
   public final static HighLevelState controllerState = HighLevelState.JOINT_PD_CONTROL;

   public JointPDHighLevelHumanoidController(DoubleYoVariable timeYoVariable, Map<OneDoFJoint, Double> initialKpGains, Map<OneDoFJoint, Double> initialKdGains)
   {
      super(controllerState);
      
      kpJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      kdJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      q_dJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      qd_filtered_Joints = new HashMap<OneDoFJoint, AlphaFilteredYoVariable>();
      alphaQD = new DoubleYoVariable("alphaQD", registry);
      alphaQD.set(0.8);
      
      desiredTorques = new HashMap<OneDoFJoint, DoubleYoVariable>();
      
      functionGenerators = new HashMap<OneDoFJoint, YoFunctionGenerator>();
      
      if (initialKpGains == null)
      {
         this.oneDoFJoints = new ArrayList<>();
         return; 
      }
      
      this.oneDoFJoints = new ArrayList<>(initialKpGains.keySet());
      
      for (int i = 0; i < this.oneDoFJoints.size(); i++)
      {
         OneDoFJoint joint = oneDoFJoints.get(i);
         DoubleYoVariable kp = new DoubleYoVariable("hl_" + joint.getName()+ CommonNames.k_q_p, registry);
         kp.set(initialKpGains.get(joint));
         kpJoints.put(joint, kp);
         
         DoubleYoVariable kd = new DoubleYoVariable("hl_" + joint.getName() + CommonNames.k_qd_p, registry);
         kd.set(initialKdGains.get(joint));
         kdJoints.put(joint, kd);
         
         DoubleYoVariable q_d = new DoubleYoVariable("hl_" + joint.getName() + CommonNames.q_d, registry);
         q_dJoints.put(joint, q_d);
         
         AlphaFilteredYoVariable qd_filtered = new AlphaFilteredYoVariable("hl_" + joint.getName() + "_qd_filt", registry, alphaQD);
         qd_filtered_Joints.put(joint, qd_filtered );
         
         DoubleYoVariable desiredTorque = new DoubleYoVariable("hl_" + joint.getName() + "_tau_d", registry);
         desiredTorques.put(joint, desiredTorque);
         
         YoFunctionGenerator functionGenerator = new YoFunctionGenerator("hl_" + joint.getName(), timeYoVariable, registry);
         functionGenerators.put(joint, functionGenerator);
      }
      
      gainScaling.set(1.0);
      alphaQD.set(0.8);
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < this.oneDoFJoints.size(); i++)
      {
         OneDoFJoint joint = oneDoFJoints.get(i);      
         DoubleYoVariable desiredPosition = q_dJoints.get(joint);

//         YoFunctionGenerator functionGenerator = functionGenerators.get(joint);
         //desiredPosition.set(functionGenerator.getValue());
         
         double kp = kpJoints.get(joint).getDoubleValue() * gainScaling.getDoubleValue();
         double kd = kdJoints.get(joint).getDoubleValue() * gainScaling.getDoubleValue();
         double q_d = desiredPosition.getDoubleValue();
   
         double qd = joint.getQd();
         qd_filtered_Joints.get(joint).update(qd);
         
         double torque = kp * (q_d - joint.getQ()) - kd * qd_filtered_Joints.get(joint).getDoubleValue();
         DoubleYoVariable desiredTorque = desiredTorques.get(joint);
         desiredTorque.set(torque);
         
         joint.setTau(torque);
      } 
   }

   @Override
   public void doTransitionIntoAction()
   {      
      for (int i = 0; i < this.oneDoFJoints.size(); i++)
      {
         OneDoFJoint joint = oneDoFJoints.get(i); 
         YoFunctionGenerator functionGenerator = functionGenerators.get(joint);
         functionGenerator.setOffset(joint.getQ());
         q_dJoints.get(joint).set(joint.getQ());
      } 
   }

   @Override
   public void doTransitionOutOfAction()
   {      
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

}
