package us.ihmc.humanoidBehaviors.utilities;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.StopArmMotionPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

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

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame sensorFrame;
   private final FramePose sensorPose;
   private final YoFramePoint yoSensorPosition;
   
   private final ForceSensorDistalMassCompensator sensorMassCompensator;

   private final TaskSpaceStiffnessCalculator taskspaceStiffnessCalc;

   private final DoubleYoVariable yoImpactStiffnessThreshold_NperM;
   private final DoubleYoVariable yoImpactForceThreshold_N;
   private final BooleanYoVariable stopArmMotionIfImpactDetected;
   private final BooleanYoVariable addSimulatedSensorNoise;

   private final PacketCommunicator controllerCommunicator;
   

   public WristForceSensorFilteredUpdatable(RobotSide robotSide, FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInfo,
         ForceSensorDataHolder forceSensorDataHolder, double DT, PacketCommunicator controllerCommunicator, YoVariableRegistry registry)
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

      wristSensorWrench = new Wrench();
      forceSensorData.packWrench(wristSensorWrench);

      this.sensorFrame = wristSensorDefinition.getSensorFrame();
      this.sensorPose = new FramePose(world);
      this.yoSensorPosition = new YoFramePoint(sidePrefix + "WristSensorPosition", world, registry);

      this.sensorMassCompensator = new ForceSensorDistalMassCompensator(wristSensorDefinition, registry);

      yoWristSensorForceMagnitude = new DoubleYoVariable(sidePrefix + "WristForceMag", registry);
      yoWristSensorForceMagnitudeBias = new FirstOrderFilteredYoVariable(sidePrefix + "WristForceBias", "", 0.0001, DT, FirstOrderFilterType.LOW_PASS, registry);
      yoWristSensorForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(sidePrefix + "WristForceMagFiltered", "",
            forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);

      taskspaceStiffnessCalc = new TaskSpaceStiffnessCalculator(sidePrefix, DT, registry);

      yoImpactStiffnessThreshold_NperM = new DoubleYoVariable(sidePrefix + "ImpactStiffnessThreshold_NperM", registry);
      yoImpactForceThreshold_N = new DoubleYoVariable(sidePrefix + "ImpactForceThreshold_N", registry);
      yoImpactDetected = new BooleanYoVariable(sidePrefix + "WristImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new DoubleYoVariable(sidePrefix + "WristImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(sidePrefix + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
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

      yoImpactForceThreshold_N.set(20.0);
      yoImpactStiffnessThreshold_NperM.set(20000.0);
   }

   public void setStopMotionIfCollision(boolean stop)
   {
      stopArmMotionIfImpactDetected.set(stop);
   }
   
   public double getHandMass()
   {
      return sensorMassCompensator.getDistalMass();
   }

   public FrameVector getWristForceMassCompensated()
   {
      return sensorMassCompensator.getSensorForceMassCompensated();
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

   public void update(double time)
   {
      updateSensorValuesFromRobot();

      stopArmIfHandCollisionIsDetected(time);
   }

   private void updateSensorValuesFromRobot()
   {
      forceSensorData.packWrench(wristSensorWrench);
      sensorMassCompensator.update(forceSensorData);

      yoWristSensorForceMagnitude.set(sensorMassCompensator.getSensorForceRaw().length());

      if (addSimulatedSensorNoise.getBooleanValue())
      {
         yoWristSensorForceMagnitude.add(0.1 * 2.0 * (Math.random() - 0.5) + 0.25);
      }
   }
   
   private void stopArmIfHandCollisionIsDetected(double time)
   {
      estimateStiffnessOfConstraintsActingUponWrist();

      yoWristSensorForceMagnitudeBias.update(yoWristSensorForceMagnitude.getDoubleValue());
      yoWristSensorForceMagnitudeBandPassFiltered.update(yoWristSensorForceMagnitude.getDoubleValue());

      //      if (Math.abs(yoWristForceHandMassCompensated.length()) > yoImpactForceThreshold_N.getDoubleValue())
      if (taskspaceStiffnessCalc.getStiffnessAlongDirectionOfMotion() > yoImpactStiffnessThreshold_NperM.getDoubleValue())
      {
         if (!yoImpactDetected.getBooleanValue() && time > 1.0)
         {
            yoImpactDetected.set(true);
            yoImpactTime.set(time);

            if (stopArmMotionIfImpactDetected.getBooleanValue())
            {
               StopArmMotionPacket pausePacket = new StopArmMotionPacket(robotSide);
               pausePacket.setDestination(PacketDestination.CONTROLLER);
               controllerCommunicator.send(pausePacket);
            }
         }
      }
   }

   private final FramePoint temp = new FramePoint();
   
   private void estimateStiffnessOfConstraintsActingUponWrist()
   {
      sensorPose.setPose(sensorFrame.getTransformToDesiredFrame(world));
      sensorPose.getPositionIncludingFrame(temp);
      yoSensorPosition.set(temp.getReferenceFrame(), temp.getX(), temp.getY(), temp.getZ());

      FrameVector wristSensorForce = sensorMassCompensator.getSensorForceRaw();
      ReferenceFrame changeBackToThisFrame = wristSensorForce.getReferenceFrame();
      wristSensorForce.changeFrame(world);

      taskspaceStiffnessCalc.update(yoSensorPosition.getFrameTuple(), wristSensorForce);
      
      wristSensorForce.changeFrame(changeBackToThisFrame);
   }
}
