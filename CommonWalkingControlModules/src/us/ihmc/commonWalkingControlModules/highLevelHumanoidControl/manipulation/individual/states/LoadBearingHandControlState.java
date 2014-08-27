package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/**
 * @author twan
 *         Date: 5/30/13
 */
public abstract class LoadBearingHandControlState extends TaskspaceHandControlState
{
   protected final DoubleYoVariable coefficientOfFriction;
   protected final SpatialAccelerationVector handAcceleration;

   public LoadBearingHandControlState(String namePrefix, HandControlState stateEnum, MomentumBasedController momentumBasedController, int jacobianId,
         RigidBody elevator, RigidBody endEffector, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, elevator, endEffector, parentRegistry);
      
      coefficientOfFriction = new DoubleYoVariable(name + "CoefficientOfFriction", registry);
      handAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(), endEffector.getBodyFixedFrame());
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   protected SpatialAccelerationVector computeDesiredSpatialAcceleration()
   {
      return handAcceleration;
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}
