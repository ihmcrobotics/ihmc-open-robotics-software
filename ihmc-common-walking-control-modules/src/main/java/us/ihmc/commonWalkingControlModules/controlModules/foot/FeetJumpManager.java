package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class FeetJumpManager
{
   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private FullHumanoidRobotModel fullRobotModel;
   
   private JointspaceAccelerationCommand jointAccelerationCommand = new JointspaceAccelerationCommand();
   private DenseMatrix64F jointAcceleration = new DenseMatrix64F(6,1);
   
   public FeetJumpManager(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
   }
   
   public void compute()
   {
      InverseDynamicsJoint joint = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH);
      jointAcceleration.reshape(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0, -0.24);
      jointAccelerationCommand.addJoint(joint, jointAcceleration);
      jointAccelerationCommand.setWeight(10.0);
   }
   
   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide side)
   {
      return jointAccelerationCommand;
   }
}
