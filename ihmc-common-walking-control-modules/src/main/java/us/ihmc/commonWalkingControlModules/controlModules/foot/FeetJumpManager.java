package us.ihmc.commonWalkingControlModules.controlModules.foot;

public class FeetJumpManager
{
   /*
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
      InverseDynamicsJoint joint = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      jointAcceleration.reshape(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0,  -10.0);
      jointAccelerationCommand.clear();
      jointAccelerationCommand.addJoint(joint, jointAcceleration, 100.0);
   }
   
   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide side)
   {
      return jointAccelerationCommand;
   }
   */
}
