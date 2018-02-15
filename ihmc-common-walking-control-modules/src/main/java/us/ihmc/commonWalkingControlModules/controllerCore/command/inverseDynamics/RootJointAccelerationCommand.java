package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

/**
 * The {@code RootJointAccelerationCommand} is designed to be submitted to the {@code WholeBodyControllerCore} via 
 * a {@code ControllerCoreCommand} to properly set up the root joint accelerations in case the root joint is accelerating
 * 
 * <p>
 * The {@code RootJointAccelerationCommand} must be provided the {@code #rootJointAcceleration} via the 
 * {@code #setRootJointAccelerationInInertialFrame}. This information is used to calculate the 
 * <li> gravitational wrench in the {@code InverseDynamicsOptimizationControlModule} for the wrench equilibrium constraint </li>
 * <li> root acceleration for the {@code SpatialAccelerationCalculator} for computing accelerations of other rigid bodies
 * </p>
 * 
 * @author Apoorv Shriavstava
 */
public class RootJointAccelerationCommand implements InverseDynamicsCommand<RootJointAccelerationCommand>
{
   private SpatialAccelerationVector rootJointAcceleration = new SpatialAccelerationVector();
   
   public void getRootJointSpatialAcceleration(SpatialAccelerationVector rootJointSpatialAccelerationToGet)
   {
      rootJointSpatialAccelerationToGet.set(this.rootJointAcceleration);
   }

   public SpatialAccelerationVector getRootJointSpatialAcceleration()
   {
      return rootJointAcceleration;
   }

   public void getRootJointLinearAcceleration(FrameVector3D rootJointLinearAccelerationToGet)
   {
      this.rootJointAcceleration.getLinearPart(rootJointLinearAccelerationToGet);
   }
   
   public void getRootJointAngularAcceleration(FrameVector3D rootJointAngularAccelerationToGet)
   {
      this.rootJointAcceleration.getAngularPart(rootJointAngularAccelerationToGet);
   }
   
   public void setRootJointSpatialAcceleration(SpatialAccelerationVector rootJointSpatialAcceleration)
   {
      this.rootJointAcceleration.set(rootJointSpatialAcceleration);
   }

   public void setRootJointLinearAcceleration(FrameVector3D rootJointLinearAcceleration)
   {
      this.rootJointAcceleration.setLinearPart(rootJointLinearAcceleration);
   }
   
   public void setRootJointAngularAcceleration(FrameVector3D rootJointAngularAcceleration)
   {
      this.rootJointAcceleration.setAngularPart(rootJointAngularAcceleration);
   }
   
   @Override
   public void set(RootJointAccelerationCommand other)
   {
      this.rootJointAcceleration.set(other.rootJointAcceleration);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.ROOT_JOINT_ACCELERATION_COMMAND;
   }
}
