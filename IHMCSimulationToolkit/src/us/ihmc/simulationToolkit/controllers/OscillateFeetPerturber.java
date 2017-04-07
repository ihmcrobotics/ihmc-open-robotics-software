package us.ihmc.simulationToolkit.controllers;

import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;

public class OscillateFeetPerturber extends ModularRobotController
{
   private final SideDependentList<GroundContactPointsSlipper> groundContactPointsSlippers;
   private final SideDependentList<List<GroundContactPoint>> groundContactPointsMap = new SideDependentList<List<GroundContactPoint>>();

   private final YoFrameVector translationMagnitude;
   private final YoFrameOrientation rotationMagnitude;

   private final SideDependentList<YoFrameVector> translationPhases = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> rotationPhasesEuler = new SideDependentList<YoFrameVector>();
   
   private final SideDependentList<double[]> translationFrequenciesHz = new SideDependentList<double[]>();
   private final SideDependentList<double[]> rotationFrequenciesHzYawPitchRoll = new SideDependentList<double[]>();

   private final double deltaT;
   
   public OscillateFeetPerturber(HumanoidFloatingRootJointRobot robot, double deltaT)
   {
      super("NoisilyShakeFeetPerturber");
      String name = "ShakeFeet";

      this.deltaT = deltaT;
      
      for (RobotSide robotSide : RobotSide.values())
      {
         groundContactPointsMap.put(robotSide, robot.getFootGroundContactPoints(robotSide));
         
         YoFrameVector translationPhase = new YoFrameVector(robotSide.getLowerCaseName() + "TranslationPhase", null, registry);
         YoFrameVector rotationPhaseEuler = new YoFrameVector(robotSide.getLowerCaseName() + "RotationPhase", null, registry);
         
         double[] translationalFreqHz = new double[3];
         double[] rotationFreqHzYawPitchRoll = new double[3];
         
         translationPhases.put(robotSide,  translationPhase);
         rotationPhasesEuler.put(robotSide,  rotationPhaseEuler);
         
         translationFrequenciesHz.put(robotSide,  translationalFreqHz);
         rotationFrequenciesHzYawPitchRoll.put(robotSide,  rotationFreqHzYawPitchRoll);
      }

      translationMagnitude = new YoFrameVector(name + "TranslationMagnitude", ReferenceFrame.getWorldFrame(), registry);
      rotationMagnitude = new YoFrameOrientation(name + "TotationMagnitude", ReferenceFrame.getWorldFrame(), registry);

      GroundContactPointsSlipper leftSlipper = new GroundContactPointsSlipper("left");
      GroundContactPointsSlipper rightSlipper = new GroundContactPointsSlipper("right");
      
      groundContactPointsSlippers = new SideDependentList<GroundContactPointsSlipper>(leftSlipper, rightSlipper);
      
      this.addRobotController(leftSlipper);
      this.addRobotController(rightSlipper);
   }

   public void setTranslationMagnitude(double[] translation)
   {
      this.translationMagnitude.set(translation[0], translation[1], translation[2]);
   }

   public void setRotationMagnitudeYawPitchRoll(double[] rotationYawPitchRoll)
   {
      this.rotationMagnitude.setYawPitchRoll(rotationYawPitchRoll);
   }
   
   public void setTranslationFrequencyHz(RobotSide robotSide, double[] freqHz)
   {
      translationFrequenciesHz.put(robotSide, freqHz);
   }
   
   public void setRotationFrequencyHzYawPitchRoll(RobotSide robotSide, double[] freqHz)
   {
      rotationFrequenciesHzYawPitchRoll.put(robotSide, freqHz);
   }
   
   @Override
   public void doControl()
   {
      super.doControl();

      for (RobotSide robotSide : RobotSide.values())
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
      YoFrameVector translationPhase = translationPhases.get(robotSide);
      YoFrameVector rotationPhaseEuler = rotationPhasesEuler.get(robotSide);
      
      double[] translationFreqHz = translationFrequenciesHz.get(robotSide);
      double[] rotationFreqHzYawPitchRoll = rotationFrequenciesHzYawPitchRoll.get(robotSide);
      
      GroundContactPointsSlipper groundContactPointsSlipper = groundContactPointsSlippers.get(robotSide);

      groundContactPointsSlipper.setGroundContactPoints(groundContactPointsMap.get(robotSide));
      groundContactPointsSlipper.setPercentToSlipPerTick(1.0); //nextSlipPercentSlipPerTick.getDoubleValue());
      groundContactPointsSlipper.setDoSlip(true);
      
      translationPhase.setX(translationPhase.getX() + 2.0  * Math.PI * translationFreqHz[0] * deltaT);
      translationPhase.setY(translationPhase.getY() + 2.0  * Math.PI * translationFreqHz[1] * deltaT);
      translationPhase.setZ(translationPhase.getZ() + 2.0  * Math.PI * translationFreqHz[2] * deltaT);

      rotationPhaseEuler.setX(rotationPhaseEuler.getX() + 2.0  * Math.PI * rotationFreqHzYawPitchRoll[2] * deltaT);
      rotationPhaseEuler.setY(rotationPhaseEuler.getY() + 2.0  * Math.PI * rotationFreqHzYawPitchRoll[1] * deltaT);
      rotationPhaseEuler.setZ(rotationPhaseEuler.getZ() + 2.0  * Math.PI * rotationFreqHzYawPitchRoll[0] * deltaT);

      
      Vector3D nextTranslationToSlip = translationMagnitude.getVector3dCopy();
      nextTranslationToSlip.setX(nextTranslationToSlip.getX() * (2.0 * Math.PI * translationFreqHz[0] * Math.sin(translationPhase.getX()) * deltaT));
      nextTranslationToSlip.setY(nextTranslationToSlip.getY() * (2.0 * Math.PI * translationFreqHz[1] * Math.sin(translationPhase.getY()) * deltaT));
      nextTranslationToSlip.setZ(nextTranslationToSlip.getZ() * (2.0 * Math.PI * translationFreqHz[2] * Math.sin(translationPhase.getZ()) * deltaT));
      
      Vector3D nextRotationToSlipEulerAngles = new Vector3D();
      rotationMagnitude.getEulerAngles(nextRotationToSlipEulerAngles);
      nextRotationToSlipEulerAngles.setX(nextRotationToSlipEulerAngles.getX() * (2.0 * Math.PI * rotationFreqHzYawPitchRoll[2] * Math.sin(rotationPhaseEuler.getX()) * deltaT));
      nextRotationToSlipEulerAngles.setY(nextRotationToSlipEulerAngles.getY() * (2.0 * Math.PI * rotationFreqHzYawPitchRoll[1] * Math.sin(rotationPhaseEuler.getY()) * deltaT));
      nextRotationToSlipEulerAngles.setZ(nextRotationToSlipEulerAngles.getZ() * (2.0 * Math.PI * rotationFreqHzYawPitchRoll[0] * Math.sin(rotationPhaseEuler.getZ()) * deltaT));
      
      groundContactPointsSlipper.setSlipTranslation(nextTranslationToSlip);
      groundContactPointsSlipper.setSlipRotationEulerAngles(nextRotationToSlipEulerAngles);
   }

   private void stopSlipping(RobotSide robotSide)
   {
       YoFrameVector translationPhase = translationPhases.get(robotSide);
       translationPhase.setToZero();
       
       YoFrameVector rotationPhaseEuler = rotationPhasesEuler.get(robotSide);
       rotationPhaseEuler.setToZero();
   }
   
   
   private boolean footTouchedDown(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.isInContact())
            return true;
      }

      return false;
   }
}

