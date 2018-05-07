package us.ihmc.quadrupedRobotics.output;

import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

public class JointIntegratorComponent implements OutputProcessorComponent
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final JointDesiredOutputList jointDesiredOutputList;
   private final List<QuadrupedJointIntegrator> quadrupedJoints = new ArrayList<>();

   private final double controlDT;

   public JointIntegratorComponent(QuadrupedRuntimeEnvironment runtimeEnvironment, YoVariableRegistry parentRegistry)
   {
      this.jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();
      controlDT = runtimeEnvironment.getControlDT();

      parentRegistry.addChild(registry);
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      OneDoFJoint[] controllerJoints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJoint controllerJoint : controllerJoints)
      {
         quadrupedJoints.add(new QuadrupedJointIntegrator(controllerJoint, jointDesiredOutputList.getJointDesiredOutput(controllerJoint), controlDT, registry));
      }

   }

   public void initialize()
   {

   }

   public void update()
   {
      for (int i = 0; i < quadrupedJoints.size(); i++)
      {
         quadrupedJoints.get(i).computeDesiredStateFromJointController();
      }
   }
}
