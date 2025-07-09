package us.ihmc.simulationToolkit.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class SlipFeetPerturber implements Controller
{
   private final YoRegistry registry = new YoRegistry("NoisilyShakeFeetPerturber");
   private final List<Controller> subControllers = new ArrayList<>();
   private final SideDependentList<GroundContactPointsSlipper> groundContactPointsSlippers;
   private final SideDependentList<List<GroundContactPoint>> groundContactPointsMap = new SideDependentList<List<GroundContactPoint>>();

   private final YoFrameVector3D translationMagnitude;
   private final YoFrameYawPitchRoll rotationMagnitude;

   private final YoFrameVector3D totalTranslation;
   private final YoFrameVector3D totalRotation;

   private final SideDependentList<double[]> translationDurations = new SideDependentList<double[]>();
   private final SideDependentList<double[]> rotationDurationsYawPitchRoll = new SideDependentList<double[]>();

   private final double deltaT;

   public SlipFeetPerturber(Robot robot, SideDependentList<String> footNames, double deltaT)
   {
      String name = "ShakeFeet";

      this.deltaT = deltaT;

      for (RobotSide robotSide : RobotSide.values())
      {
         groundContactPointsMap.put(robotSide, robot.getRigidBody(footNames.get(robotSide)).getParentJoint().getAuxialiryData().getGroundContactPoints());

         double[] translationalFreqHz = new double[3];
         double[] rotationFreqHzYawPitchRoll = new double[3];

         translationDurations.put(robotSide, translationalFreqHz);
         rotationDurationsYawPitchRoll.put(robotSide, rotationFreqHzYawPitchRoll);
      }

      translationMagnitude = new YoFrameVector3D(name + "TranslationMagnitude", ReferenceFrame.getWorldFrame(), registry);
      totalTranslation = new YoFrameVector3D(name + "TotalTranslation", ReferenceFrame.getWorldFrame(), registry);
      rotationMagnitude = new YoFrameYawPitchRoll(name + "RotationMagnitude", ReferenceFrame.getWorldFrame(), registry);
      totalRotation = new YoFrameVector3D(name + "TotalRotation", ReferenceFrame.getWorldFrame(), registry);

      GroundContactPointsSlipper leftSlipper = new GroundContactPointsSlipper("left");
      GroundContactPointsSlipper rightSlipper = new GroundContactPointsSlipper("right");

      groundContactPointsSlippers = new SideDependentList<>(leftSlipper, rightSlipper);

      subControllers.add(leftSlipper);
      subControllers.add(rightSlipper);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void setTranslationMagnitude(double[] translation)
   {
      this.translationMagnitude.set(translation[0], translation[1], translation[2]);
   }

   public void setRotationMagnitudeYawPitchRoll(double[] rotationYawPitchRoll)
   {
      this.rotationMagnitude.set(rotationYawPitchRoll);
   }

   public void setTranslationDuration(RobotSide robotSide, double[] freqHz)
   {
      translationDurations.put(robotSide, freqHz);
   }

   public void setRotationDurationYawPitchRoll(RobotSide robotSide, double[] freqHz)
   {
      rotationDurationsYawPitchRoll.put(robotSide, freqHz);
   }

   @Override
   public void doControl()
   {
      subControllers.forEach(Controller::doControl);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (footTouchedDown(robotSide))
         {
            startSlipping(robotSide);
         }
         else
         {
            stopSlipping(robotSide);
         }
      }

   }

   private void startSlipping(RobotSide robotSide)
   {
      double[] translationDurations = this.translationDurations.get(robotSide);
      double[] rotationDurationsYawPitchRoll = this.rotationDurationsYawPitchRoll.get(robotSide);

      GroundContactPointsSlipper groundContactPointsSlipper = groundContactPointsSlippers.get(robotSide);

      groundContactPointsSlipper.setGroundContactPoints(groundContactPointsMap.get(robotSide));
      groundContactPointsSlipper.setPercentToSlipPerTick(1.0); //nextSlipPercentSlipPerTick.getDoubleValue());
      groundContactPointsSlipper.setDoSlip(true);


      Vector3D maxAllowableTranslation = new Vector3D();
      maxAllowableTranslation.sub(translationMagnitude, totalTranslation);

      Vector3D nextTranslationToSlip = new Vector3D();

      nextTranslationToSlip.setX(MathTools.clamp(deltaT * translationMagnitude.getX() / translationDurations[0], maxAllowableTranslation.getX()));
      nextTranslationToSlip.setY(MathTools.clamp(deltaT * translationMagnitude.getY() / translationDurations[1], maxAllowableTranslation.getY()));
      nextTranslationToSlip.setZ(MathTools.clamp(deltaT * translationMagnitude.getZ() / translationDurations[2], maxAllowableTranslation.getZ()));

      totalTranslation.add(nextTranslationToSlip);

      Vector3D maxAllowableRotationEuler = new Vector3D();

      Vector3D nextRotationToSlipEulerAngles = new Vector3D();
      rotationMagnitude.getEuler(nextRotationToSlipEulerAngles);
      maxAllowableRotationEuler.sub(nextRotationToSlipEulerAngles, totalRotation);

      if (rotationDurationsYawPitchRoll[2] > 0.0)
         nextRotationToSlipEulerAngles.setX(MathTools.clamp(deltaT * nextRotationToSlipEulerAngles.getX() / rotationDurationsYawPitchRoll[2], maxAllowableRotationEuler.getX()));
      else
         nextRotationToSlipEulerAngles.setX(0.0);
      if (rotationDurationsYawPitchRoll[1] > 0.0)
         nextRotationToSlipEulerAngles.setY(MathTools.clamp(deltaT * nextRotationToSlipEulerAngles.getY() / rotationDurationsYawPitchRoll[1], maxAllowableRotationEuler.getY()));
      else
         nextRotationToSlipEulerAngles.setY(0.0);
      if (rotationDurationsYawPitchRoll[0] > 0.0)
         nextRotationToSlipEulerAngles.setZ(MathTools.clamp(deltaT * nextRotationToSlipEulerAngles.getZ() / rotationDurationsYawPitchRoll[0], maxAllowableRotationEuler.getZ()));
      else
         nextRotationToSlipEulerAngles.setZ(0.0);

      totalRotation.add(nextRotationToSlipEulerAngles);

      groundContactPointsSlipper.setSlipTranslation(nextTranslationToSlip);
      groundContactPointsSlipper.setSlipRotationEulerAngles(nextRotationToSlipEulerAngles);
   }

   private void stopSlipping(RobotSide robotSide)
   {
      totalRotation.setToZero();
      totalTranslation.setToZero();
   }

   private boolean footTouchedDown(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.getInContact().getValue())
            return true;
      }

      return false;
   }
}
