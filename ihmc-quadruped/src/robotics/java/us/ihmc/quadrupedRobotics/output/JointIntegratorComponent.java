package us.ihmc.quadrupedRobotics.output;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointIntegratorComponent implements OutputProcessorComponent
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final JointDesiredOutputList jointDesiredOutputList;
   private final List<QuadrupedJointIntegrator> quadrupedJoints = new ArrayList<>();

   private final double controlDT;

   public JointIntegratorComponent(QuadrupedRuntimeEnvironment runtimeEnvironment, YoRegistry parentRegistry)
   {
      this.jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();
      controlDT = runtimeEnvironment.getControlDT();

      parentRegistry.addChild(registry);
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      OneDoFJointBasics[] controllerJoints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJointBasics controllerJoint : controllerJoints)
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
