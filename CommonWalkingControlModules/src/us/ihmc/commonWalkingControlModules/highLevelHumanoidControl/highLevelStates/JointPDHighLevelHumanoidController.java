package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.HashSet;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class JointPDHighLevelHumanoidController extends HighLevelBehavior
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable gainScaling = new DoubleYoVariable("hl_gainScaling", registry);

   private final HashMap<OneDoFJoint, DoubleYoVariable>  q_dJoints, q_Joints, desiredTorques;
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> qd_filtered_Joints;
   
   
   private final DoubleYoVariable kpJoints, kdJoints;
   
   private final HashMap<OneDoFJoint, YoFunctionGenerator> functionGenerators;
   private final HashSet<OneDoFJoint> jointsBeenControlled = new HashSet<OneDoFJoint>();
   private final DoubleYoVariable alphaQD;
   private final FullRobotModel fullRobotModel;

   public final static HighLevelState controllerState = HighLevelState.JOINT_PD_CONTROL;

   public JointPDHighLevelHumanoidController(MomentumBasedController momentumBasedController, double initialKpGains, double initialKdGains)
   {
      super(controllerState);

      fullRobotModel = momentumBasedController.getFullRobotModel();

      kpJoints = new DoubleYoVariable("hl_AllJoints" + CommonNames.k_q_p, registry);
      kpJoints.set( initialKpGains );
      
      kdJoints = new DoubleYoVariable("hl_AllJoints" + CommonNames.k_qd_p, registry);
      kdJoints.set( initialKdGains );
      
      q_dJoints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      q_Joints = new HashMap<OneDoFJoint, DoubleYoVariable>();
      qd_filtered_Joints = new HashMap<OneDoFJoint, AlphaFilteredYoVariable>();
      alphaQD = new DoubleYoVariable("alphaQD", registry);
      alphaQD.set(0.8);

      desiredTorques = new HashMap<OneDoFJoint, DoubleYoVariable>();    
      functionGenerators = new HashMap<OneDoFJoint, YoFunctionGenerator>();


      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         String joinName = joint.getName();

         if( joinName.contains("finger") ) continue;
         if( joinName.contains("hokuyo") ) continue;
         if( joinName.contains("neck") ) continue;
         if( joinName.contains("arm") ) continue;
         
         jointsBeenControlled.add(joint);

         /*DoubleYoVariable kp = new DoubleYoVariable("hl_" + joint.getName()+ CommonNames.k_q_p, registry);
         kp.set( initialKpGains );
         kpJoints.put(joint, kp);

         DoubleYoVariable kd = new DoubleYoVariable("hl_" + joint.getName() + CommonNames.k_qd_p, registry);
         kd.set( initialKdGains );
         kdJoints.put(joint, kd);*/

         DoubleYoVariable q_d = new DoubleYoVariable("hl_" + joint.getName() + "_q_desired", registry);
         q_d.set( joint.getQ() );
         q_dJoints.put(joint, q_d);

         DoubleYoVariable q = new DoubleYoVariable("hl_" + joint.getName() + "_q", registry);
         q.set( joint.getQ() );
         q_Joints.put(joint, q);
         
         AlphaFilteredYoVariable qd_filtered = new AlphaFilteredYoVariable("hl_" + joint.getName() + "_qd_filt", registry, alphaQD);
         qd_filtered_Joints.put(joint, qd_filtered );

         DoubleYoVariable desiredTorque = new DoubleYoVariable("hl_" + joint.getName() + "_tau_d", registry);
         desiredTorques.put(joint, desiredTorque);

         YoFunctionGenerator functionGenerator = new YoFunctionGenerator("hl_" + joint.getName(), momentumBasedController.getYoTime(), registry);
         functionGenerators.put(joint, functionGenerator);
      }

      gainScaling.set(1.0);
      alphaQD.set(0.8);
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         fullRobotModel.getOneDoFJoints()[i].setTau( 0.0 );
      }
      
      System.out.println("--------------------");
      for (OneDoFJoint joint: jointsBeenControlled)
      {      
         DoubleYoVariable desiredPosition = q_dJoints.get(joint);

         //         YoFunctionGenerator functionGenerator = functionGenerators.get(joint);
         //desiredPosition.set(functionGenerator.getValue());

         double kp = kpJoints.getDoubleValue() * gainScaling.getDoubleValue();
         double kd = kdJoints.getDoubleValue() * gainScaling.getDoubleValue();
         double q_desired = desiredPosition.getDoubleValue();

         double qd = joint.getQd();
         qd_filtered_Joints.get(joint).update(qd);
         q_Joints.get(joint).set(joint.getQ());
         double torque = kp * (q_desired - joint.getQ()) /*- kd * qd_filtered_Joints.get(joint).getDoubleValue()*/;
         DoubleYoVariable desiredTorque = desiredTorques.get(joint);
         

         if(torque > 400)  torque =  400;
         if(torque < -400) torque = -400;

         joint.setTau(torque);
         desiredTorque.set(torque);
        // System.out.format("%s\t %.1f * ( %.3f - %.3f ) - ( %.1f * %.3f ) = %.3f\n", joint.getName(), kp, q_desired, joint.getQ(), kd , qd_filtered_Joints.get(joint).getDoubleValue(), torque );
         System.out.format("%s\t %.1f * ( %.3f - %.3f ) = %.3f\n", joint.getName(), kp, q_desired, joint.getQ(), torque );
      } 
   }

   @Override
   public void doTransitionIntoAction()
   {      
      for (OneDoFJoint joint: jointsBeenControlled)
      {
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
