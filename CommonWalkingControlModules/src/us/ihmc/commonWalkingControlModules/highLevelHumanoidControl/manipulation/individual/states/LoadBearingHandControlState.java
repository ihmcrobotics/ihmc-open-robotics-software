package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

/**
 * @author twan
 *         Date: 5/30/13
 */
public abstract class LoadBearingHandControlState extends TaskspaceHandControlState
{
   protected final DoubleYoVariable coefficientOfFriction;
   protected final SpatialAccelerationVector handAcceleration;

   public LoadBearingHandControlState(IndividualHandControlState stateEnum, MomentumBasedController momentumBasedController, GeometricJacobian jacobian, RigidBody elevator, YoVariableRegistry parentRegistry)
   {
      super(stateEnum, momentumBasedController, jacobian, parentRegistry);
      coefficientOfFriction = new DoubleYoVariable(name + "CoefficientOfFriction", registry);
      handAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevator.getBodyFixedFrame(), jacobian.getEndEffectorFrame());
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
