package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.ArrayList;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to stand prep
 */
public class QuadrupedForceBasedDoNothingController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SDFFullQuadrupedRobotModel fullRobotModel;

   private final ArrayList<DoubleYoVariable> desiredDoNothingTorques = new ArrayList<>();
   private final ArrayList<OneDoFJoint> legJoints = new ArrayList<>();

   public QuadrupedForceBasedDoNothingController(QuadrupedRuntimeEnvironment environment, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            legJoints.add(joint);
            desiredDoNothingTorques.add(new DoubleYoVariable(joint.getName() + "DoNothingTorque", registry));
         }
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      for(int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint joint = legJoints.get(i);
         joint.setUnderPositionControl(false);
         joint.setTau(0.0);
      }
   }

   @Override
   public ControllerEvent process()
   {
      for(int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint joint = legJoints.get(i);
         joint.setUnderPositionControl(false);
         joint.setTau(desiredDoNothingTorques.get(i).getDoubleValue());
      }
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

