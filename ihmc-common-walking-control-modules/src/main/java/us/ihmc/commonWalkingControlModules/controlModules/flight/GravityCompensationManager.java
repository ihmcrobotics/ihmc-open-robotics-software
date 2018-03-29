package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.RootJointAccelerationCommand;
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
public class GravityCompensationManager implements JumpControlManagerInterface
{
   private final double gravityZ;
   private final JumpControllerParameters jumpControllerParameters;

   private final RootJointAccelerationCommand rootJointAccelerationCommand;
   private final FrameVector3D rootAcceleration;

   public GravityCompensationManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControlParameters,
                                     YoVariableRegistry registry)
   {
      this.gravityZ = controllerToolbox.getGravityZ();
      this.jumpControllerParameters = jumpControlParameters;

      ReferenceFrame controlFrame = ReferenceFrame.getWorldFrame();
      RigidBody someRigidBody = controllerToolbox.getControlledJoints()[0].getSuccessor();
      ReferenceFrame rootFrame = ScrewTools.getRootBody(someRigidBody).getBodyFixedFrame();
      rootJointAccelerationCommand = new RootJointAccelerationCommand(rootFrame, controlFrame, rootFrame);
      rootAcceleration = new FrameVector3D(rootFrame);
   }
   
   public void setRootJointAccelerationForFreeFall()
   {
      rootAcceleration.set(0.0, 0.0, 0.0);
      rootJointAccelerationCommand.setRootJointLinearAcceleration(rootAcceleration);
   }

   public void setRootJointAccelerationForStandardGravitationalForce()
   {
      rootAcceleration.set(0.0, 0.0, gravityZ);
      rootJointAccelerationCommand.setRootJointLinearAcceleration(rootAcceleration);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return rootJointAccelerationCommand;
   }
   
   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
