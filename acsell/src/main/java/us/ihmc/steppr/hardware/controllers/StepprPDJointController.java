package us.ihmc.steppr.hardware.controllers;

import java.util.ArrayList;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.steppr.parameters.BonoRobotModel;

public class StepprPDJointController implements StepprController
{
   BonoRobotModel robotModel = new BonoRobotModel(true, true);
   protected final YoVariableRegistry registry = new YoVariableRegistry("StepprPDJointController");

   protected final ArrayList<OneDoFJoint> joints = new ArrayList<>();
   protected final ArrayList<PDController> controllers = new ArrayList<>();
   protected final ArrayList<YoDouble> desiredPositions = new ArrayList<>();
   protected final ArrayList<YoDouble> desiredVelocities = new ArrayList<>();
   protected final ArrayList<YoDouble> tauFFs = new ArrayList<>();
   protected final ArrayList<YoDouble> damping = new ArrayList<>();


   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      for(OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joints.add(joint);
         controllers.add(new PDController(joint.getName(), registry));
         desiredPositions.add(new YoDouble(joint.getName() + "_q_d", registry));
         desiredVelocities.add(new YoDouble(joint.getName() + "_qd_d", registry));
         tauFFs.add(new YoDouble(joint.getName() + "_tau_ff", registry));
         damping.add(new YoDouble(joint.getName() + "_damping", registry));
      }

   }

   @Override
   public void initialize(long timestamp)
   {
      for(int i = 0; i < controllers.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         desiredPositions.get(i).set(joint.getQ());;
         desiredVelocities.get(i).set(0);
      }
   }

   @Override
   public void doControl(long timestamp)
   {
      for(int i = 0; i < controllers.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         PDController controller = controllers.get(i);
         double tauFF = tauFFs.get(i).getDoubleValue();
         double q_d = desiredPositions.get(i).getDoubleValue();
         double qd_d = desiredVelocities.get(i).getDoubleValue();

         double tau = controller.compute(joint.getQ(), q_d, joint.getQd(), qd_d) + tauFF;

         joint.setTau(tau);
         joint.setKd(damping.get(i).getDoubleValue());
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }


   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprPDJointController());
   }

   @Override
   public boolean turnOutputOn()
   {
      return false;
   }
}
