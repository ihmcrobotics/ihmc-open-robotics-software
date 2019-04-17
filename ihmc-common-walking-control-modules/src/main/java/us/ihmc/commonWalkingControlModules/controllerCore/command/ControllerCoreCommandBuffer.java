package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;

/**
 * This class is not for general user, it is used for performing cross-robot command conversion in a
 * garbage free manner.
 * <p>
 * This class should only be used with {@link CrossRobotCommandResolver} to resolve a
 * {@link ControllerCoreCommand}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class ControllerCoreCommandBuffer implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandBuffer inverseDynamicsCommandBuffer = new InverseDynamicsCommandBuffer();
   private final InverseKinematicsCommandBuffer inverseKinematicsCommandBuffer = new InverseKinematicsCommandBuffer();
   private final VirtualModelControlCommandBuffer virtualModelControlCommandBuffer = new VirtualModelControlCommandBuffer();
   private final FeedbackControlCommandBuffer feedbackControlCommandBuffer = new FeedbackControlCommandBuffer();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private WholeBodyControllerCoreMode controllerCoreMode;
   private boolean reinitialize = false;

   public ControllerCoreCommandBuffer()
   {
   }

   @Override
   public void clear()
   {
      inverseDynamicsCommandBuffer.clear();
      inverseKinematicsCommandBuffer.clear();
      virtualModelControlCommandBuffer.clear();
      feedbackControlCommandBuffer.clear();
      lowLevelOneDoFJointDesiredDataHolder.clear();
      reinitialize = false;
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      if (this.controllerCoreMode != controllerCoreMode)
      {
         clear();
         this.controllerCoreMode = controllerCoreMode;
      }
   }

   public void requestReinitialization()
   {
      reinitialize = true;
   }

   @Override
   public boolean isReinitializationRequested()
   {
      return reinitialize;
   }

   @Override
   public WholeBodyControllerCoreMode getControllerCoreMode()
   {
      return controllerCoreMode;
   }

   @Override
   public InverseDynamicsCommandBuffer getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandBuffer;
   }

   @Override
   public InverseKinematicsCommandBuffer getInverseKinematicsCommandList()
   {
      return inverseKinematicsCommandBuffer;
   }

   @Override
   public VirtualModelControlCommandBuffer getVirtualModelControlCommandList()
   {
      return virtualModelControlCommandBuffer;
   }

   @Override
   public FeedbackControlCommandBuffer getFeedbackControlCommandList()
   {
      return feedbackControlCommandBuffer;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ControllerCoreCommandInterface)
         return ControllerCoreCommandInterface.super.equals((ControllerCoreCommandInterface) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      String ret = "Request control mode: " + controllerCoreMode + "\n";
      ret += "ID commands: " + inverseDynamicsCommandBuffer + "\n";
      ret += "IK commands: " + inverseKinematicsCommandBuffer + "\n";
      ret += "VMC commands: " + virtualModelControlCommandBuffer + "\n";
      ret += "Feedback commands: " + feedbackControlCommandBuffer + "\n";
      ret += "Low-level command: " + lowLevelOneDoFJointDesiredDataHolder;
      return ret;
   }
}
