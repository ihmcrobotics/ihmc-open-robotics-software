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
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final double DT;
   private final RobotSide robotSide;

   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;

   private final ForceSensorData forceSensorData;
   private final Wrench wristSensorWrench;

   private final DoubleYoVariable yoWristSensorForceMagnitude;
   private final FirstOrderFilteredYoVariable yoWristSensorForceMagnitudeBias;
   private final FirstOrderBandPassFilteredYoVariable yoWristSensorForceMagnitudeBandPassFiltered;

   private final BooleanYoVariable yoForceLimitExceeded;
   private final BooleanYoVariable yoStiffnessLimitExceeded;
   private final BooleanYoVariable yoImpactDetected;
   private final DoubleYoVariable yoImpactTime;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final ForceSensorDistalMassCompensator sensorMassCompensator;

   private final TaskSpaceStiffnessCalculator taskspaceStiffnessCalc;

   private final DoubleYoVariable yoImpactStiffnessThreshold_NperM;
   private final DoubleYoVariable yoImpactForceThreshold_N;
   private final BooleanYoVariable stopArmMotionIfImpactDetected;

   private final PacketCommunicator controllerCommunicator;

   public WristForceSensorFilteredUpdatable(RobotSide robotSide, FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInfo,
         ForceSensorDataHolder forceSensorDataHolder, double DT, PacketCommunicator controllerCommunicator, YoVariableRegistry registry)
   {
      this.DT = DT;
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

      this.sensorMassCompensator = new ForceSensorDistalMassCompensator(wristSensorDefinition, DT, registry);

      yoWristSensorForceMagnitude = new DoubleYoVariable(forceSensorName + "ForceMag", registry);
      yoWristSensorForceMagnitudeBias = new FirstOrderFilteredYoVariable(forceSensorName + "ForceBias", "", 0.0001, DT, FirstOrderFilterType.LOW_PASS, registry);
      yoWristSensorForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(forceSensorName + "ForceMagFiltered", "",
            forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);

      taskspaceStiffnessCalc = new TaskSpaceStiffnessCalculator(sidePrefix, DT, registry);

      yoImpactStiffnessThreshold_NperM = new DoubleYoVariable(forceSensorName + "ImpactStiffnessThreshold_NperM", registry);
      yoImpactForceThreshold_N = new DoubleYoVariable(forceSensorName + "ImpactForceThreshold_N", registry);

      yoForceLimitExceeded = new BooleanYoVariable(forceSensorName + "forceLimitExceeded", registry);
      yoStiffnessLimitExceeded = new BooleanYoVariable(forceSensorName + "stiffnessLimitExceeded", registry);
      yoImpactDetected = new BooleanYoVariable(forceSensorName + "ImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new DoubleYoVariable(forceSensorName + "ImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(sidePrefix + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
      //            YoAppearance.OrangeRed());

      stopArmMotionIfImpactDetected = new BooleanYoVariable(forceSensorName + "stopArmMotionIfImpactDetected", registry);

      this.controllerCommunicator = controllerCommunicator;

      initialize();
   }

   private void initialize()
   {
      stopArmMotionIfImpactDetected.set(false);

      yoImpactForceThreshold_N.set(100.0);
      yoImpactStiffnessThreshold_NperM.set(10000.0);
   }

   public double getDT()
   {
      return DT;
   }

   public void setStopMotionIfCollision(boolean stop)
   {
      stopArmMotionIfImpactDetected.set(stop);
   }

   public double getHandMass()
   {
      return sensorMassCompensator.getDistalMass();
   }
   
   public FramePoint getWristPositionInWorld()
   {
      return sensorMassCompensator.getSensorPosition();
   }

   public FrameVector getWristForceRawInWorld()
   {
      return sensorMassCompensator.getSensorForceRaw(world);
   }

   public FrameVector getWristForceMassCompensatedInWorld()
   {
      return sensorMassCompensator.getSensorForceMassCompensated(world);
   }

   public DoubleYoVariable getWristForceMagnitude()
   {
      return yoWristSensorForceMagnitude;
   }

   public DoubleYoVariable getWristForceBandPassFiltered()
   {
      return yoWristSensorForceMagnitudeBandPassFiltered;
   }

   public double getSensorZForceLowPassFilteredInWorld()
   {
      return sensorMassCompensator.getSensorZForceLowPassFilteredInWorld();
   }
   
   public double getForceRateOfChangeAlongDirectionOfMotion()
   {
      return taskspaceStiffnessCalc.getForceRateOfChangeAlongDirectionOfMotion();
   }

   public Boolean hasHandCollidedWithSomething()
   {
      return this.yoImpactDetected.getBooleanValue();
   }

   public void resetHandCollisionDetector()
   {
      yoImpactDetected.set(false);
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

      FrameVector sensorForceRawInWorld = sensorMassCompensator.getSensorForceRaw(world);
      yoWristSensorForceMagnitude.set(sensorForceRawInWorld.length());
   }

   private void stopArmIfHandCollisionIsDetected(double time)
   {
      estimateStiffnessOfConstraintsActingUponWrist();

      yoWristSensorForceMagnitudeBias.update(yoWristSensorForceMagnitude.getDoubleValue());
      yoWristSensorForceMagnitudeBandPassFiltered.update(yoWristSensorForceMagnitude.getDoubleValue());

      yoForceLimitExceeded.set( taskspaceStiffnessCalc.getForceAlongDirectionOfMotion() > yoImpactForceThreshold_N.getDoubleValue() );
      yoStiffnessLimitExceeded.set( taskspaceStiffnessCalc.getStiffnessAlongDirectionOfMotion() > yoImpactStiffnessThreshold_NperM.getDoubleValue() );

      if ( yoForceLimitExceeded.getBooleanValue() || yoStiffnessLimitExceeded.getBooleanValue() )
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

   private final FrameVector wristSensorForceInWorld = new FrameVector();

   private void estimateStiffnessOfConstraintsActingUponWrist()
   {
      FramePoint sensorPositionInWorld = sensorMassCompensator.getSensorPosition();

      FrameVector forceInWorldFrame = sensorMassCompensator.getSensorForceMassCompensated(world);
      wristSensorForceInWorld.setIncludingFrame(forceInWorldFrame);

      taskspaceStiffnessCalc.update(sensorPositionInWorld, wristSensorForceInWorld);
   }
}
