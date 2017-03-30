package us.ihmc.avatar.obstacleCourseTests;

import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;

public class SlipOnNextStepPerturber extends ModularRobotController
{
   private final GroundContactPointsSlipper groundContactPointsSlipper;
   private final List<GroundContactPoint> groundContactPoints;

   private final FloatingRootJointRobot robot;

   private final EnumYoVariable<SlipState> slipState;
   private final BooleanYoVariable slipNextStep;
   private final DoubleYoVariable slipAfterTimeDelta, touchdownTimeForSlip;
   private final YoFrameVector amountToSlipNextStep;
   private final YoFrameOrientation rotationToSlipNextStep;

   public SlipOnNextStepPerturber(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
   {
      super(robotSide.getCamelCaseNameForStartOfExpression() + "SlipOnEachStepPerturber");

      String sideString = robotSide.getCamelCaseNameForStartOfExpression();
      this.robot = robot;
      this.touchdownTimeForSlip = new DoubleYoVariable(sideString + "TouchdownTimeForSlip", registry);
      this.slipAfterTimeDelta = new DoubleYoVariable(sideString + "SlipAfterTimeDelta", registry);
      this.slipNextStep = new BooleanYoVariable(sideString + "SlipNextStep", registry);

      amountToSlipNextStep = new YoFrameVector(sideString + "AmountToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      rotationToSlipNextStep = new YoFrameOrientation(sideString + "RotationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      slipState = new EnumYoVariable<SlipState>(sideString + "SlipState", registry, SlipState.class);
      slipState.set(SlipState.NOT_SLIPPING);

      groundContactPoints = robot.getFootGroundContactPoints(robotSide);
      
      groundContactPointsSlipper = new GroundContactPointsSlipper(robotSide.getLowerCaseName());
      groundContactPointsSlipper.addGroundContactPoints(groundContactPoints);
      groundContactPointsSlipper.setPercentToSlipPerTick(0.05);

      this.addRobotController(groundContactPointsSlipper);
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
      super.doControl();

      switch (slipState.getEnumValue())
      {
         case NOT_SLIPPING :
         {
            if (footTouchedDown())
            {
               if (slipNextStep.getBooleanValue())
               {
                  slipState.set(SlipState.TOUCHED_DOWN);
                  touchdownTimeForSlip.set(robot.getTime());
               }
               else // Wait till foot lift back up before allowing a slip.
               {
                  slipState.set(SlipState.DONE_SLIPPING);
               }
            }

            break;
         }

         case TOUCHED_DOWN :
         {
            if (robot.getTime() > touchdownTimeForSlip.getDoubleValue() + slipAfterTimeDelta.getDoubleValue())
            {
               slipState.set(SlipState.SLIPPING);
               startSlipping(amountToSlipNextStep.getVector3dCopy(), rotationToSlipNextStep.getYawPitchRoll());
            }

            break;
         }

         case SLIPPING :
         {
            if (groundContactPointsSlipper.isDoneSlipping())
            {
               slipState.set(SlipState.DONE_SLIPPING);
            }

            break;
         }

         case DONE_SLIPPING :
         {
            if (footLiftedUp())
            {
               slipState.set(SlipState.NOT_SLIPPING);
            }

            break;
         }
      }
   }
   
   private void startSlipping(Vector3D slipAmount, double[] yawPitchRoll)
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
         if (groundContactPoint.isInContact())
            return true;
      }

      return false;
   }

   private boolean footLiftedUp()
   {
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         if (groundContactPoint.isInContact())
            return false;
      }

      return true;
   }

   private enum SlipState {NOT_SLIPPING, TOUCHED_DOWN, SLIPPING, DONE_SLIPPING}
}
