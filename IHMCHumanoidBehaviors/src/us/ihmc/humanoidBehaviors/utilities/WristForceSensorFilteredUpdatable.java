package us.ihmc.humanoidBehaviors.utilities;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVectorInMultipleFrames;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final RobotSide robotSide;

   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;

   private final ForceSensorData forceSensorData;
   private final Wrench wristSensorWrench;

   private final DoubleYoVariable yoWristSensorForceMagnitude;
   private final FirstOrderFilteredYoVariable yoWristSensorForceMagnitudeBias;
   private final FirstOrderBandPassFilteredYoVariable yoWristSensorForceMagnitudeBandPassFiltered;

   private final BooleanYoVariable yoImpactDetected;
   private final DoubleYoVariable yoImpactTime;

   private final CenterOfMassCalculator handMassCalc;
   private final FrameVector handWeightInWorld;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame sensorFrame;
   private final FramePose sensorPose;
   private final YoFramePoint yoSensorPosition;

   private final DoubleYoVariable handMass;

   private final YoFrameVectorInMultipleFrames yoWristSensorForce;
   private final YoFrameVectorInMultipleFrames yoWristSensorTorque;
   private final YoFrameVectorInMultipleFrames yoWristSensorForceDueToHandMass;
   private final YoFrameVectorInMultipleFrames yoSensorToHandCoMvector;
   private final YoFrameVectorInMultipleFrames yoWristSensorTorqueDueToHandMass;

   private final YoFrameVector yoWristSensorForceHandMassCompensated;
   private final YoFrameVector yoWristSensorTorqueHandMassCompensated;

   private final DoubleYoVariable yoImpactStiffnessThreshold_NperM;
   private final DoubleYoVariable yoImpactForceThreshold_N;
   private final BooleanYoVariable stopArmMotionIfImpactDetected;
   private final BooleanYoVariable addSimulatedSensorNoise;

   private final TurboEncabulatorDingus stiffnessCalc;

   private final ObjectCommunicator controllerCommunicator;

   public WristForceSensorFilteredUpdatable(RobotSide robotSide, FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInfo,
         ForceSensorDataHolder forceSensorDataHolder, double DT, ObjectCommunicator controllerCommunicator, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;

      String sidePrefix = robotSide.getLowerCaseName();
      String forceSensorName = sensorInfo.getWristForceSensorNames().get(robotSide);

      ForceSensorDefinition wristSensorDefinition = null;
      List<ForceSensorDefinition> forceSensorDefinitions = forceSensorDataHolder.getForceSensorDefinitions();
      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         if (forceSensorDefinition.getSensorName().equals(forceSensorName))
         {
            wristSensorDefinition = forceSensorDefinition;
         }
      }
      if (wristSensorDefinition == null)
      {
         System.out.println(this.toString() + ": No Wrist Sensor Definition Found!  Make sure that forceSensorName is properly set.");
      }
      this.forceSensorData = forceSensorDataHolder.getByName(forceSensorName);

      InverseDynamicsJoint wristJoint = forceSensorData.getMeasurementLink().getParentJoint();
      wristSensorWrench = new Wrench();
      forceSensorData.packWrench(wristSensorWrench);

      this.sensorFrame = wristSensorDefinition.getSensorFrame();
      this.sensorPose = new FramePose(world);
      this.yoSensorPosition = new YoFramePoint(sidePrefix + "WristSensorPosition", world, registry);

      handMass = new DoubleYoVariable(robotSide.getLowerCaseName() + "HandMass", registry);
      handMassCalc = new CenterOfMassCalculator(ScrewTools.computeRigidBodiesAfterThisJoint(wristJoint), sensorFrame);
      handWeightInWorld = new FrameVector(world);

      ReferenceFrame[] worldAndSensorFrames = new ReferenceFrame[] { world, sensorFrame };

      yoWristSensorForce = new YoFrameVectorInMultipleFrames(sidePrefix + "WristForce", registry, worldAndSensorFrames);
      yoWristSensorTorque = new YoFrameVectorInMultipleFrames(sidePrefix + "WristTorque", registry, worldAndSensorFrames);

      yoWristSensorForceDueToHandMass = new YoFrameVectorInMultipleFrames(sidePrefix + "WristForceDueToHandMass", registry, worldAndSensorFrames);
      yoSensorToHandCoMvector = new YoFrameVectorInMultipleFrames(sidePrefix + "SensorToHandCoM", registry, worldAndSensorFrames);
      yoWristSensorTorqueDueToHandMass = new YoFrameVectorInMultipleFrames(sidePrefix + "WristTorqueDueToHandMass", registry, worldAndSensorFrames);

      yoWristSensorForceHandMassCompensated = new YoFrameVector(sidePrefix + "WristForceMassCompensated", sensorFrame, registry);
      yoWristSensorTorqueHandMassCompensated = new YoFrameVector(sidePrefix + "WristTorqueMassCompensated", sensorFrame, registry);

      yoWristSensorForceMagnitude = new DoubleYoVariable(sidePrefix + "WristForceMag", registry);
      yoWristSensorForceMagnitudeBias = new FirstOrderFilteredYoVariable(sidePrefix + "WristForceBias", "", 0.0001, DT, FirstOrderFilterType.LOW_PASS, registry);
      yoWristSensorForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(sidePrefix + "WristForceMagFiltered", "",
            forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);

      yoImpactStiffnessThreshold_NperM = new DoubleYoVariable(sidePrefix + "ImpactStiffnessThreshold_NperM", registry);
      yoImpactForceThreshold_N = new DoubleYoVariable(sidePrefix + "ImpactForceThreshold_N", registry);
      yoImpactDetected = new BooleanYoVariable(sidePrefix + "WristImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new DoubleYoVariable(sidePrefix + "WristImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(sidePrefix + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
      //            YoAppearance.OrangeRed());

      addSimulatedSensorNoise = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "AddSimulatedNoise", registry);
      stopArmMotionIfImpactDetected = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "stopArmMotionIfImpactDetected", registry);

      stiffnessCalc = new TurboEncabulatorDingus(sidePrefix, DT, registry);

      this.controllerCommunicator = controllerCommunicator;

      initialize();
   }

   private void initialize()
   {
      addSimulatedSensorNoise.set(false);
      stopArmMotionIfImpactDetected.set(false);

      handMassCalc.compute();
      handMass.set(handMassCalc.getTotalMass());
      
      yoImpactForceThreshold_N.set(20.0);
      yoImpactStiffnessThreshold_NperM.set(20000.0);
   }

   public void setStopMotionIfCollision(boolean stop)
   {
      stopArmMotionIfImpactDetected.set(stop);
   }

   public double getHandMass()
   {
      return handMass.getDoubleValue();
   }

   public FrameVector getWristForceMassCompensated()
   {
      return yoWristSensorForceHandMassCompensated.getFrameTuple();
   }

   public DoubleYoVariable getWristForceMagnitude()
   {
      return yoWristSensorForceMagnitude;
   }

   public DoubleYoVariable getWristForceBandPassFiltered()
   {
      return yoWristSensorForceMagnitudeBandPassFiltered;
   }

   public Boolean getIsHandCurrentlyColliding()
   {
      return this.yoImpactDetected.getBooleanValue();
   }

   public double getHandImpactTime()
   {
      return this.yoImpactTime.getDoubleValue();
   }

   public void updateWristSensorValuesFromRobot()
   {
      forceSensorData.packWrench(wristSensorWrench);

      ReferenceFrame wristWrenchFrame = wristSensorWrench.getExpressedInFrame();

      yoWristSensorForce.set(wristWrenchFrame, wristSensorWrench.getLinearPartX(), wristSensorWrench.getLinearPartY(), wristSensorWrench.getLinearPartZ());
      yoWristSensorTorque.set(wristWrenchFrame, wristSensorWrench.getAngularPartX(), wristSensorWrench.getAngularPartY(), wristSensorWrench.getAngularPartZ());

      yoWristSensorForceMagnitude.set(yoWristSensorForce.length());

      if (addSimulatedSensorNoise.getBooleanValue())
      {
         yoWristSensorForceMagnitude.add(0.1 * 2.0 * (Math.random() - 0.5) + 0.25);
      }
   }

   private final FramePoint temp = new FramePoint();

   public void update(double time)
   {
      sensorFrame.update();

      handMassCalc.compute();
      handMass.set(handMassCalc.getTotalMass());
      handWeightInWorld.set(0.0, 0.0, 9.81 * handMass.getDoubleValue());

      yoSensorToHandCoMvector.changeFrame(handMassCalc.getDesiredFrame());
      handMassCalc.getCenterOfMass(temp);
      yoSensorToHandCoMvector.set(temp.getReferenceFrame(), temp.getX(), temp.getY(), temp.getZ());

      updateWristSensorValuesFromRobot();

      computeSensorForceDueToWeightOfHand();

      computeSensorTorqueDueToWeightOfHand();
      
      estimateStiffnessOfConstraintsActingUponWrist();

      yoWristSensorForceMagnitudeBias.update(yoWristSensorForceMagnitude.getDoubleValue());
      yoWristSensorForceMagnitudeBandPassFiltered.update(yoWristSensorForceMagnitude.getDoubleValue());

//      if (Math.abs(yoWristForceHandMassCompensated.length()) > yoImpactForceThreshold_N.getDoubleValue())
      if( stiffnessCalc.getStiffnessAlongDirectionOfMotion() > yoImpactStiffnessThreshold_NperM.getDoubleValue() )
      {
         if (!yoImpactDetected.getBooleanValue() && time > 1.0)
         {
            yoImpactDetected.set(true);
            yoImpactTime.set(time);

            if (stopArmMotionIfImpactDetected.getBooleanValue())
            {
               StopArmMotionPacket pausePacket = new StopArmMotionPacket(robotSide);
               pausePacket.setDestination(PacketDestination.CONTROLLER);
               controllerCommunicator.consumeObject(pausePacket);
            }
         }
      }
   }

   private void computeSensorForceDueToWeightOfHand()
   {
      yoWristSensorForceDueToHandMass.changeFrame(world);
      yoWristSensorForceDueToHandMass.set(world, 0.0, 0.0, 9.81 * handMass.getDoubleValue()); // USE +9.81 since measured force is equal and opposite

      yoWristSensorForce.changeFrame(sensorFrame);
      yoWristSensorForceDueToHandMass.changeFrame(sensorFrame);

      yoWristSensorForceHandMassCompensated.sub(yoWristSensorForce, yoWristSensorForceDueToHandMass);
   }

   private void computeSensorTorqueDueToWeightOfHand()
   {
      yoSensorToHandCoMvector.changeFrame(handWeightInWorld.getReferenceFrame());
      yoWristSensorTorqueDueToHandMass.changeFrame(handWeightInWorld.getReferenceFrame());

      FrameVector wristTorqueDueToHandMass = yoWristSensorTorqueDueToHandMass.getFrameTuple();
      wristTorqueDueToHandMass.cross(yoSensorToHandCoMvector.getFrameTuple(), handWeightInWorld);

      yoWristSensorTorqueDueToHandMass.set(wristTorqueDueToHandMass.getReferenceFrame(), wristTorqueDueToHandMass.getX(), wristTorqueDueToHandMass.getY(),
            wristTorqueDueToHandMass.getZ());

      yoWristSensorTorque.changeFrame(sensorFrame);
      yoWristSensorTorqueDueToHandMass.changeFrame(sensorFrame);

      yoWristSensorTorqueHandMassCompensated.sub(yoWristSensorTorque, yoWristSensorTorqueDueToHandMass);
   }
   
   private void estimateStiffnessOfConstraintsActingUponWrist()
   {
      sensorPose.setPose(sensorFrame.getTransformToDesiredFrame(world));

      sensorPose.getPositionIncludingFrame(temp);
      yoSensorPosition.set(temp.getReferenceFrame(), temp.getX(), temp.getY(), temp.getZ());

      ReferenceFrame changeBackToThisFrame = yoWristSensorForce.getReferenceFrame();
      yoWristSensorForce.changeFrame(world);

      stiffnessCalc.update(yoSensorPosition.getFrameTuple(), yoWristSensorForce.getFrameTuple());
      yoWristSensorForce.changeFrame(changeBackToThisFrame);
   }

}
