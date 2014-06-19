package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class OnToesState extends AbstractOnEdgeState
{
   private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
   private final List<FramePoint> originalContactPointPositions;

   private final FramePoint2d singleToeContactPoint;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap;

   double alphaShrinkFootSizeForToeOff = 0.0;
   double kneeToeOffGain = 0.0;
   double kneeToeOffQDesired = 0.7;
   double kneeToeOffDamp = 0.0;
   double controlledKneeToeOffQdd;

   public OnToesState(DoubleProvider maximumTakeoffAngle, YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity,
         YoFrameVector yoDesiredLinearAcceleration, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule, RobotSide robotSide,
         YoVariableRegistry registry, EnumMap<ConstraintType, boolean[]> contactStatesMap)
   {
      super(ConstraintType.TOES, maximumTakeoffAngle,

      yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration, accelerationControlModule, momentumBasedController, contactableBody,
            requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape,
            legSingularityAndKneeCollapseAvoidanceControlModule, robotSide, registry);

      this.contactStatesMap = contactStatesMap;

      singleToeContactPoint = new FramePoint2d(edgeContactPoints.get(0).getReferenceFrame());
      singleToeContactPoint.interpolate(edgeContactPoints.get(0), edgeContactPoints.get(1), 0.5);

      originalContactPointPositions = new ArrayList<FramePoint>(contactPoints.size());
      copyOriginalContactPointPositions();
   }

   public void doSpecificAction()
   {
      super.doSpecificAction();

      shrinkFootSizeToMidToe();

      // TODO Hack to do singularity escape since the motion constraint handler doesn't handle it with the given selection matrix 
      controlledKneeToeOffQdd = kneeToeOffGain * Math.max(0.0, kneeToeOffQDesired - kneeJoint.getQ()) - kneeToeOffDamp * kneeJoint.getQd();
      momentumBasedController.setOneDoFJointAcceleration(kneeJoint, controlledKneeToeOffQdd);
   }

   private void copyOriginalContactPointPositions()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         originalContactPointPositions.add(new FramePoint(contactPoints.get(i).getPosition()));
      }
   }

   private void resetContactPointPositions()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).getPosition().set(originalContactPointPositions.get(i));
      }
   }

   private void shrinkFootSizeToMidToe()
   {
      double alphaShrink = alphaShrinkFootSizeForToeOff;
      for (int i = 0; i < contactPoints.size(); i++)
      {
         FramePoint position = contactPoints.get(i).getPosition();
         position.setX(alphaShrink * position.getX() + (1.0 - alphaShrink) * singleToeContactPoint.getX());
         position.setY(alphaShrink * position.getY() + (1.0 - alphaShrink) * singleToeContactPoint.getY());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      if (useMaximumPitchAngle)
      {
         setOnToesFreeMotionGains();
      }
      else
      {
         setGains(0.0, 500.0, 1.0);
      }

      //alphaShrinkFootSizeForToeOff

      momentumBasedController.setPlaneContactState(contactableBody, contactStatesMap.get(ConstraintType.TOES));
   }

   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      resetContactPointPositions();

      controlledKneeToeOffQdd = 0.0;
   }

   private void setOnToesFreeMotionGains()
   {
      double kPosition = 0.0;
      double dPosition = GainCalculator.computeDerivativeGain(0.0, toeOffZeta);
      double dxOrientation = GainCalculator.computeDerivativeGain(toeOffKpRoll, toeOffZeta);
      double dyOrientation = GainCalculator.computeDerivativeGain(toeOffKpPitch, toeOffZeta);
      double dzOrientation = GainCalculator.computeDerivativeGain(toeOffKpYaw, toeOffZeta);

      accelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      accelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      accelerationControlModule.setOrientationProportionalGains(toeOffKpRoll, toeOffKpPitch, toeOffKpYaw);
      accelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }
}
