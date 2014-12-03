package us.ihmc.humanoidBehaviors.utilities;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;
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
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVectorInMultipleFrames;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final RobotSide robotSide;

   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;

   private final ForceSensorData forceSensorData;
   private final Wrench wristWrench;

   private final DoubleYoVariable yoWristForceMagnitude;
   private final FirstOrderFilteredYoVariable yoWristForceMagnitudeBias;
   private final FirstOrderBandPassFilteredYoVariable yoWristForceMagnitudeBandPassFiltered;

   private final BooleanYoVariable yoImpactDetected;
   private final DoubleYoVariable yoImpactTime;

   private final CenterOfMassCalculator handMassCalc;
   private final FrameVector handWeightInWorld;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame sensorFrame;

   private final DoubleYoVariable handMass;

   private final YoFrameVectorInMultipleFrames yoWristSensorForce;
   private final YoFrameVectorInMultipleFrames yoWristSensorTorque;
   private final YoFrameVectorInMultipleFrames yoWristForceDueToHandMass;
   private final YoFrameVectorInMultipleFrames yoSensorToHandCoMvector;
   private final YoFrameVectorInMultipleFrames yoWristTorqueDueToHandMass;

   private final YoFrameVector yoWristForceHandMassCompensated;
   private final YoFrameVector yoWristTorqueHandMassCompensated;

   private final DoubleYoVariable yoImpactForceThreshold_N;
   private final BooleanYoVariable stopArmMotionIfImpactDetected;
   private final BooleanYoVariable addSimulatedSensorNoise;

   private final ObjectCommunicator controllerCommunicator;

   public WristForceSensorFilteredUpdatable(RobotSide robotSide, FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder, double DT,
         ObjectCommunicator controllerCommunicator, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;

      String forceSensorName = robotSide.getShortLowerCaseName() + "_arm_wrx"; //TODO: Fix this so that it works with robots other than Atlas, by using DRCRobotSensorInformation.getWristForceSensorNames()

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
      wristWrench = new Wrench();
      forceSensorData.packWrench(wristWrench);

      this.sensorFrame = wristSensorDefinition.getSensorFrame();

      handMass = new DoubleYoVariable(robotSide.getLowerCaseName() + "HandMass", registry);
      handMassCalc = new CenterOfMassCalculator(ScrewTools.computeRigidBodiesAfterThisJoint(wristJoint), sensorFrame);
      handWeightInWorld = new FrameVector(world);

      ReferenceFrame[] worldAndSensorFrames = new ReferenceFrame[] { world, sensorFrame };

      yoWristSensorForce = new YoFrameVectorInMultipleFrames(robotSide.getLowerCaseName() + "WristForce", registry, worldAndSensorFrames);
      yoWristSensorTorque = new YoFrameVectorInMultipleFrames(robotSide.getLowerCaseName() + "WristTorque", registry, worldAndSensorFrames);

      yoWristForceDueToHandMass = new YoFrameVectorInMultipleFrames(robotSide.getLowerCaseName() + "WristForceDueToHandMass", registry, worldAndSensorFrames);
      yoSensorToHandCoMvector = new YoFrameVectorInMultipleFrames(robotSide.getLowerCaseName() + "SensorToHandCoM", registry, worldAndSensorFrames);
      yoWristTorqueDueToHandMass = new YoFrameVectorInMultipleFrames(robotSide.getLowerCaseName() + "WristTorqueDueToHandMass", registry, worldAndSensorFrames);

      yoWristForceHandMassCompensated = new YoFrameVector(robotSide.getLowerCaseName() + "WristForceMassCompensated", sensorFrame, registry);
      yoWristTorqueHandMassCompensated = new YoFrameVector(robotSide.getLowerCaseName() + "WristTorqueMassCompensated", sensorFrame, registry);

      yoWristForceMagnitude = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "WristForceMag", registry);
      yoWristForceMagnitudeBias = new FirstOrderFilteredYoVariable(robotSide.getShortLowerCaseName() + "WristForceBias", "", 0.0001, DT,
            FirstOrderFilterType.LOW_PASS, registry);
      yoWristForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(robotSide.getShortLowerCaseName() + "WristForceMagFiltered", "",
            forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);

      yoImpactForceThreshold_N = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "ImpactForceThreshold_N", registry);
      yoImpactDetected = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "WristImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "WristImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(robotSide.getLowerCaseName() + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
      //            YoAppearance.OrangeRed());

      addSimulatedSensorNoise = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "AddSimulatedNoise", registry);

      stopArmMotionIfImpactDetected = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "stopArmMotionIfImpactDetected", registry);

      this.controllerCommunicator = controllerCommunicator;
      
      initialize();
   }

   private void initialize()
   {
      addSimulatedSensorNoise.set(false);
      stopArmMotionIfImpactDetected.set(false);
      
      handMassCalc.compute();
      handMass.set(handMassCalc.getTotalMass());
      yoImpactForceThreshold_N.set(  20.0 );
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
      return yoWristForceHandMassCompensated.getFrameTuple();
   }

   public DoubleYoVariable getWristForceMagnitude()
   {
      return yoWristForceMagnitude;
   }

   public DoubleYoVariable getWristForceBandPassFiltered()
   {
      return yoWristForceMagnitudeBandPassFiltered;
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
      forceSensorData.packWrench(wristWrench);

      ReferenceFrame wristWrenchFrame = wristWrench.getExpressedInFrame();

      yoWristSensorForce.set(wristWrenchFrame, wristWrench.getLinearPartX(), wristWrench.getLinearPartY(), wristWrench.getLinearPartZ());
      yoWristSensorTorque.set(wristWrenchFrame, wristWrench.getAngularPartX(), wristWrench.getAngularPartY(), wristWrench.getAngularPartZ());

      yoWristForceMagnitude.set(yoWristSensorForce.length());

      if (addSimulatedSensorNoise.getBooleanValue())
      {
         yoWristForceMagnitude.add(0.1 * 2.0 * (Math.random() - 0.5) + 0.25);
      }

   }

   FramePoint temp = new FramePoint();

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

      yoWristForceMagnitudeBias.update(yoWristForceMagnitude.getDoubleValue());
      yoWristForceMagnitudeBandPassFiltered.update(yoWristForceMagnitude.getDoubleValue());

      
      if (Math.abs(yoWristForceHandMassCompensated.length()) > yoImpactForceThreshold_N.getDoubleValue())
      {
         if (!yoImpactDetected.getBooleanValue() && time > 1.0)
         {
            yoImpactTime.set(time);

            if (stopArmMotionIfImpactDetected.getBooleanValue())
            {
               StopArmMotionPacket pausePacket = new StopArmMotionPacket(robotSide);
               pausePacket.setDestination(PacketDestination.CONTROLLER);
               controllerCommunicator.consumeObject(pausePacket);
            }
         }
         yoImpactDetected.set(true);

      }
      else
      {
         yoImpactDetected.set(false);
      }
   }

   private void computeSensorForceDueToWeightOfHand()
   {
      yoWristForceDueToHandMass.changeFrame(world);
      yoWristForceDueToHandMass.set(world, 0.0, 0.0, 9.81 * handMass.getDoubleValue()); // USE +9.81 since measured force is equal and opposite

      yoWristSensorForce.changeFrame(sensorFrame);
      yoWristForceDueToHandMass.changeFrame(sensorFrame);

      yoWristForceHandMassCompensated.sub(yoWristSensorForce, yoWristForceDueToHandMass);
   }

   private void computeSensorTorqueDueToWeightOfHand()
   {
      yoSensorToHandCoMvector.changeFrame(handWeightInWorld.getReferenceFrame());
      yoWristTorqueDueToHandMass.changeFrame(handWeightInWorld.getReferenceFrame());

      FrameVector wristTorqueDueToHandMass = yoWristTorqueDueToHandMass.getFrameTuple();
      wristTorqueDueToHandMass.cross(yoSensorToHandCoMvector.getFrameTuple(), handWeightInWorld);

      yoWristTorqueDueToHandMass.set(wristTorqueDueToHandMass.getReferenceFrame(), wristTorqueDueToHandMass.getX(), wristTorqueDueToHandMass.getY(),
            wristTorqueDueToHandMass.getZ());

      yoWristSensorTorque.changeFrame(sensorFrame);
      yoWristTorqueDueToHandMass.changeFrame(sensorFrame);

      yoWristTorqueHandMassCompensated.sub(yoWristSensorTorque, yoWristTorqueDueToHandMass);
   }

}
