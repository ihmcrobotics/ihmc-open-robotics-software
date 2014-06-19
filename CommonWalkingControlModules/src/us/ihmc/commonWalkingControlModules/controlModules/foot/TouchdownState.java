package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;

public class TouchdownState extends AbstractOnEdgeState
{
   public TouchdownState(ConstraintType constraintType,
         DoubleTrajectoryGenerator pitchTouchdownTrajectoryGenerator,
         
         YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody,
         EnumYoVariable<ConstraintType> requestedState, int jacobianId, 
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, BooleanYoVariable forceFootAccelerateIntoGround,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule,
         RobotSide robotSide, YoVariableRegistry registry)
   {
      super(constraintType, pitchTouchdownTrajectoryGenerator,
            
            yoDesiredPosition, yoDesiredLinearVelocity,
            yoDesiredLinearAcceleration, accelerationControlModule, momentumBasedController,
            contactableBody, requestedState, jacobianId, nullspaceMultiplier,
            jacobianDeterminantInRange, doSingularityEscape,
            forceFootAccelerateIntoGround, legSingularityAndKneeCollapseAvoidanceControlModule,
            robotSide, registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      setTouchdownOnEdgeGains();
   }
}
