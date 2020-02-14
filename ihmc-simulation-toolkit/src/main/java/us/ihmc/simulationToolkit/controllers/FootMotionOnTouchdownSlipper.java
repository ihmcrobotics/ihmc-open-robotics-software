package us.ihmc.simulationToolkit.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

import java.util.List;

public class FootMotionOnTouchdownSlipper extends ModularRobotController
{
   private final SideDependentList<GroundContactPointsSlipper> groundContactPointsSlippers;
   private final SideDependentList<List<GroundContactPoint>> groundContactPointsMap = new SideDependentList<>();

   private final YoDouble time = new YoDouble("time", registry);
   private final SideDependentList<YoDouble> timesAtContact = new SideDependentList<>();
   private final SideDependentList<YoBoolean> footIsInContacts = new SideDependentList<>();

   private final YoDouble slipDuration;
   private final YoFrameVector3D translationMagnitudes;
   private final YoFrameYawPitchRoll rotationMagnitudesYawPitchRoll;

   private final double deltaT;

   public FootMotionOnTouchdownSlipper(HumanoidFloatingRootJointRobot robot, double deltaT)
   {
      super("FootSlipperPerturber");
      String name = "FootSlip";

      this.deltaT = deltaT;

      for (RobotSide robotSide : RobotSide.values)
      {
         groundContactPointsMap.put(robotSide, robot.getFootGroundContactPoints(robotSide));
         YoDouble timeAtContact = new YoDouble(robotSide.getCamelCaseName() + "TimeAtContact", registry);
         YoBoolean footIsInContact = new YoBoolean(robotSide.getCamelCaseName() + "FootIsInContact", registry);
         footIsInContact.addVariableChangedListener((v) -> {
            if (footIsInContact.getBooleanValue())
               timeAtContact.set(time.getDoubleValue());
         });

         timesAtContact.put(robotSide, timeAtContact);
         footIsInContacts.put(robotSide, footIsInContact);
      }

      translationMagnitudes = new YoFrameVector3D(name + "TranslationMagnitude", ReferenceFrame.getWorldFrame(), registry);
      slipDuration = new YoDouble(name + "SilpDuration", registry);
      rotationMagnitudesYawPitchRoll = new YoFrameYawPitchRoll(name + "RotationRate", ReferenceFrame.getWorldFrame(), registry);

      GroundContactPointsSlipper leftSlipper = new GroundContactPointsSlipper("left");
      GroundContactPointsSlipper rightSlipper = new GroundContactPointsSlipper("right");

      groundContactPointsSlippers = new SideDependentList<>(leftSlipper, rightSlipper);

      this.addRobotController(leftSlipper);
      this.addRobotController(rightSlipper);
   }

   public void setTranslationMagnitudes(double[] translation)
   {
      this.translationMagnitudes.set(translation[0], translation[1], translation[2]);
   }

   public void setRotationMagnitudesYawPitchRoll(double[] rotationYawPitchRoll)
   {
      this.rotationMagnitudesYawPitchRoll.setYawPitchRoll(rotationYawPitchRoll[0], rotationYawPitchRoll[1], rotationYawPitchRoll[2]);
   }

   public void setSlipDuration(double duration)
   {
      slipDuration.set(duration);
   }

   @Override
   public void doControl()
   {
      super.doControl();
      time.add(deltaT);

      for (RobotSide robotSide : RobotSide.values())
      {
         updateFootContactState(robotSide);
         updateSlipping(robotSide);
      }
   }

   private void updateSlipping(RobotSide robotSide)
   {
      GroundContactPointsSlipper slipper = groundContactPointsSlippers.get(robotSide);
      if (!footIsInContacts.get(robotSide).getBooleanValue())
      {
         slipper.setDoSlip(false);
      }
      else
      {
         double timeInState = time.getDoubleValue() - timesAtContact.get(robotSide).getDoubleValue();
         if (timeInState > slipDuration.getDoubleValue())
         {
            slipper.setDoSlip(false);
         }
         else
         {
            double slips = slipDuration.getDoubleValue() / deltaT;
            slipper.setGroundContactPoints(groundContactPointsMap.get(robotSide));

            Vector3D translationIncrement = new Vector3D(translationMagnitudes);
            Vector3D rotationIncrement = new Vector3D(rotationMagnitudesYawPitchRoll.getYaw(), rotationMagnitudesYawPitchRoll.getPitch(),
                                                      rotationMagnitudesYawPitchRoll.getRoll());
            translationIncrement.scale(1.0 / slips);
            rotationIncrement.scale(1.0 / slips);

            slipper.setDoSlip(true);
            slipper.setSlipTranslation(translationIncrement);
            slipper.setSlipRotationEulerAngles(rotationIncrement);
            slipper.setPercentToSlipPerTick(1.0);
         }
      }
   }

   private void updateFootContactState(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.isInContact())
         {
            footIsInContacts.get(robotSide).set(true);
            return;
         }
      }

      footIsInContacts.get(robotSide).set(false);
   }
}

