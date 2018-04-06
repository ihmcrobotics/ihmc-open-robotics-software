package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

import java.util.ArrayList;
import java.util.List;

public class JointControlComponent implements OutputProcessorComponent
{
   private final JointDesiredOutputList jointDesiredOutputList;
   private final List<QuadrupedJoint> quadrupedJoints = new ArrayList();

   public JointControlComponent(JointDesiredOutputList jointDesiredOutputList)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      OneDoFJoint[] controllerJoints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJoint controllerJoint : controllerJoints)
      {
//         quadrupedJoints.add(new QuadrupedJoint())
      }

   }

   public void initialize()
   {

   }

   public void update()
   {

   }
}
