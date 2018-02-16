package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.RootJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Manages the gravitational compensation from the {@code InverseDynamicsCalculator} through the 
 * {@code RootJointAccelerationCommand}
 * <p>
 * Plays an important role in loading / unloading the robot joints during take off and landing while 
 * jumping
 * </p>
 * @author Apoorv Shrivastava
 *
 */
public class GravityCompensationManager
{
   private final RootJointAccelerationCommand rootJointAccelerationCommand;
   private final FrameVector3D rootAcceleration;
   
   private JumpStateEnum jumpState; 
   
   public GravityCompensationManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters controllerParameters, YoVariableRegistry registry)
   {
      ReferenceFrame controlFrame = ReferenceFrame.getWorldFrame();
      RigidBody someRigidBody = controllerToolbox.getControlledJoints()[0].getSuccessor();
      ReferenceFrame rootFrame = ScrewTools.getRootBody(someRigidBody).getBodyFixedFrame();
      rootJointAccelerationCommand = new RootJointAccelerationCommand(rootFrame, controlFrame, rootFrame);
      rootAcceleration = new FrameVector3D(rootFrame);
   }
   
   public void updateState(JumpStateEnum jumpStateEnum)
   {
      this.jumpState = jumpStateEnum;
   }
   
   public void compute()
   {
      rootAcceleration.setToZero();
      rootJointAccelerationCommand.setRootJointLinearAcceleration(rootAcceleration);
   }
   
   public RootJointAccelerationCommand getRootJointAccelerationCommand()
   {
      return rootJointAccelerationCommand;
   }
}
