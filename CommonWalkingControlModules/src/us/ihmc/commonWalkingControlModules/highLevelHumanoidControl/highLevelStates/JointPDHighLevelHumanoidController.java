package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;

public class JointPDHighLevelHumanoidController extends State<HighLevelState>
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final Set<OneDoFJoint> oneDoFJoints; 
   
   private final HashMap<OneDoFJoint, DoubleYoVariable> kpJoints, kdJoints, q_dJoints;
   
   public final static HighLevelState controllerState = HighLevelState.JOINT_PD_CONTROL;

   public JointPDHighLevelHumanoidController(Map<OneDoFJoint, Double> initialKpGains, Map<OneDoFJoint, Double> initialKdGains)
   {
      super(controllerState);
      
      kpJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      kdJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      q_dJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      
      if (initialKpGains == null)
      {
         oneDoFJoints = new HashSet<OneDoFJoint>();
         return; 
      }
      
      this.oneDoFJoints = initialKpGains.keySet();
      
      for (OneDoFJoint joint : oneDoFJoints)
      {
         DoubleYoVariable kp = new DoubleYoVariable("hl_kp_" + joint.getName(), registry);
         kp.set(initialKpGains.get(joint));
         kpJoints.put(joint, kp);
         
         DoubleYoVariable kd = new DoubleYoVariable("hl_kd_" + joint.getName(), registry);
         kd.set(initialKdGains.get(joint));
         kdJoints.put(joint, kd);
         
         DoubleYoVariable q_d = new DoubleYoVariable("hl_q_d_" + joint.getName(), registry);
         q_dJoints.put(joint, q_d);
      }
   }

   @Override
   public void doAction()
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {         
         double kp = kpJoints.get(joint).getDoubleValue();
         double kd = kdJoints.get(joint).getDoubleValue();
         double q_d = q_dJoints.get(joint).getDoubleValue();
   
         double torque = kp * (q_d - joint.getQ()) - kd * joint.getQd();
         
         joint.setTau(torque);
      } 
   }

   @Override
   public void doTransitionIntoAction()
   {      
      for (OneDoFJoint joint : oneDoFJoints)
      {  
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
