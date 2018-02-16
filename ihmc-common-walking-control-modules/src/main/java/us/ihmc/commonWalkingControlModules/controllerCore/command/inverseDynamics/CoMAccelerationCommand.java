package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

/**
 * This command is used to inform the {@code WholeBodyControllerCore} of the CoMs estimated acceleration
 * This is used to update the root body acceleration for the {@code InverseDynamicsQPSolver} and the 
 * {@code InverseDynamicsOptimizationControlModule}
 * 
 * @author Apoorv Shrivastava
 */

public class CoMAccelerationCommand implements InverseDynamicsCommand<CoMAccelerationCommand>
{
   private SpatialAccelerationVector comAcceleration;
   
   public CoMAccelerationCommand(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      comAcceleration = new SpatialAccelerationVector(bodyFrame, baseFrame, expressedInFrame);
   }
   
   public SpatialAccelerationVector getCoMSpatialAcceleration()
   {
      return comAcceleration;
   }
   
   public void getCoMSpatialAcceleration(SpatialAccelerationVector comAcceleration)
   {
      comAcceleration.set(this.comAcceleration);
   }
   
   public void getCoMLinearAcceleration(FrameVector3D comLinearAccelerationToSet)
   {
      comAcceleration.getLinearPart(comLinearAccelerationToSet);
   }
   
   public void getCoMAngularAcceleration(FrameVector3D comAngularAccelerationToSet)
   {
      comAcceleration.getAngularPart(comAngularAccelerationToSet);
   }
   
   public void setCoMSpatialAccleration(SpatialAccelerationVector spatialAccelerationVectorToCopy)
   {
      comAcceleration.set(spatialAccelerationVectorToCopy);
   }
   
   public void setCoMLinearAcceleration(FrameVector3D linearAccelerationToCopy)
   {
      comAcceleration.setLinearPart(linearAccelerationToCopy);
   }

   public void setCoMAngularAcceleration(FrameVector3D angularAccelerationToCopy)
   {
      comAcceleration.setAngularPart(angularAccelerationToCopy);
   }

   @Override
   public void set(CoMAccelerationCommand other)
   {
      this.comAcceleration.set(other.comAcceleration);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COM_ACCELERATION_COMMAND;
   }
}
