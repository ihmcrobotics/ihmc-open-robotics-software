package us.ihmc.avatar.obstacleCourseTests;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SlipOnNextStepPerturber implements Controller
{
   private final YoRegistry registry;
   private final List<Controller> subControllers = new ArrayList<Controller>();
   private final GroundContactPointsSlipper groundContactPointsSlipper;
   private final List<GroundContactPoint> groundContactPoints;

   private final Robot robot;

   private final YoEnum<SlipState> slipState;
   private final YoBoolean slipNextStep;
   private final YoDouble slipAfterTimeDelta, touchdownTimeForSlip;
   private final YoFrameVector3D amountToSlipNextStep;
   private final YoFrameYawPitchRoll rotationToSlipNextStep;
   private final DoubleProvider time;

   public SlipOnNextStepPerturber(DoubleProvider time, Robot robot, RobotSide robotSide, String footName)
   {
      this.time = time;
      registry = new YoRegistry(robotSide.getCamelCaseNameForStartOfExpression() + "SlipOnEachStepPerturber");

      String sideString = robotSide.getCamelCaseNameForStartOfExpression();
      this.robot = robot;
      this.touchdownTimeForSlip = new YoDouble(sideString + "TouchdownTimeForSlip", registry);
      this.slipAfterTimeDelta = new YoDouble(sideString + "SlipAfterTimeDelta", registry);
      this.slipNextStep = new YoBoolean(sideString + "SlipNextStep", registry);

      amountToSlipNextStep = new YoFrameVector3D(sideString + "AmountToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      rotationToSlipNextStep = new YoFrameYawPitchRoll(sideString + "RotationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      slipState = new YoEnum<SlipState>(sideString + "SlipState", registry, SlipState.class);
      slipState.set(SlipState.NOT_SLIPPING);

      groundContactPoints = robot.getRigidBody(footName).getParentJoint().getAuxialiryData().getGroundContactPoints();

      groundContactPointsSlipper = new GroundContactPointsSlipper(robotSide.getLowerCaseName());
      groundContactPointsSlipper.addGroundContactPoints(groundContactPoints);
      groundContactPointsSlipper.setPercentToSlipPerTick(0.05);

      subControllers.add(groundContactPointsSlipper);
   }

   public void setPercentToSlipPerTick(double percentToSlipPerTick)
   {
      groundContactPointsSlipper.setPercentToSlipPerTick(percentToSlipPerTick);
   }

   public void setSlipAfterStepTimeDelta(double slipAfterTimeDelta)
   {
      this.slipAfterTimeDelta.set(slipAfterTimeDelta);
   }

   public void setSlipNextStep(boolean slipNextStep)
   {
      this.slipNextStep.set(slipNextStep);
   }

   public void setAmountToSlipNextStep(Vector3D amountToSlipNextStep)
   {
      this.amountToSlipNextStep.set(amountToSlipNextStep);
   }

   public void setRotationToSlipNextStep(double yaw, double pitch, double roll)
   {
      rotationToSlipNextStep.setYawPitchRoll(yaw, pitch, roll);
   }

   public void doControl()
   {
      subControllers.forEach(Controller::doControl);

      switch (slipState.getEnumValue())
      {
         case NOT_SLIPPING:
         {
            if (footTouchedDown())
            {
               if (slipNextStep.getBooleanValue())
               {
                  slipState.set(SlipState.TOUCHED_DOWN);
                  touchdownTimeForSlip.set(time.getValue());
               }
               else // Wait till foot lift back up before allowing a slip.
               {
                  slipState.set(SlipState.DONE_SLIPPING);
               }
            }

            break;
         }

         case TOUCHED_DOWN:
         {
            if (time.getValue() > touchdownTimeForSlip.getDoubleValue() + slipAfterTimeDelta.getDoubleValue())
            {
               slipState.set(SlipState.SLIPPING);
               startSlipping(amountToSlipNextStep, rotationToSlipNextStep.getYawPitchRoll());
            }

            break;
         }

         case SLIPPING:
         {
            if (groundContactPointsSlipper.isDoneSlipping())
            {
               slipState.set(SlipState.DONE_SLIPPING);
            }

            break;
         }

         case DONE_SLIPPING:
         {
            if (footLiftedUp())
            {
               slipState.set(SlipState.NOT_SLIPPING);
            }

            break;
         }
      }
   }

   private void startSlipping(Vector3DReadOnly slipAmount, double[] yawPitchRoll)
   {
      groundContactPointsSlipper.setDoSlip(true);
      groundContactPointsSlipper.setPercentToSlipPerTick(0.01);
      groundContactPointsSlipper.setSlipTranslation(slipAmount);
      groundContactPointsSlipper.setSlipRotationYawPitchRoll(yawPitchRoll);
   }

   private boolean footTouchedDown()
   {
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         if (groundContactPoint.getInContact().getValue())
            return true;
      }

      return false;
   }

   private boolean footLiftedUp()
   {
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         if (groundContactPoint.getInContact().getValue())
            return false;
      }

      return true;
   }

   private enum SlipState
   {
      NOT_SLIPPING, TOUCHED_DOWN, SLIPPING, DONE_SLIPPING
   }
}
