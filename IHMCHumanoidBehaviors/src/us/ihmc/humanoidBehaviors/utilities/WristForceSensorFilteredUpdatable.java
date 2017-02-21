package us.ihmc.humanoidBehaviors.utilities;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.robotics.math.filters.FirstOrderFilteredYoVariable;
import us.ihmc.robotics.math.filters.FirstOrderFilteredYoVariable.FirstOrderFilterType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.tools.io.printing.PrintTools;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final boolean DEBUG = false;

   private final double DT;
   private final RobotSide robotSide;

   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;

   private final ForceSensorData forceSensorData;
   private final Wrench wristSensorWrench;

   private final DoubleYoVariable yoWristSensorForceMagnitude;
   private final FirstOrderFilteredYoVariable yoWristSensorForceMagnitudeBias;
   private final FirstOrderBandPassFilteredYoVariable yoWristSensorForceMagnitudeBandPassFiltered;

   private final IntegerYoVariable yoCollisionSeverityLevelOneToThree;
   private final BooleanYoVariable yoForceLimitExceeded;
   private final BooleanYoVariable yoStiffnessLimitExceeded;
   private final BooleanYoVariable yoImpactDetected;
   private final DoubleYoVariable yoImpactTime;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final ForceSensorDistalMassCompensator sensorMassCompensator;

   private final TaskSpaceStiffnessCalculator taskspaceStiffnessCalc;

   private final DoubleYoVariable yoImpactStiffnessThreshold_NperM;
   private final DoubleYoVariable yoImpactForceThreshold_N;

   private final PacketCommunicator controllerCommunicator;

   public WristForceSensorFilteredUpdatable(RobotSide robotSide, FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInfo,
         ForceSensorDataHolder forceSensorDataHolder, double DT, PacketCommunicator behaviorPacketCommunicator, YoVariableRegistry registry)
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
         throw new RuntimeException("No Wrist Sensor Definition Found!  Make sure that forceSensorName is properly set.");
      }
      this.forceSensorData = forceSensorDataHolder.getByName(forceSensorName);

      wristSensorWrench = new Wrench();
      forceSensorData.getWrench(wristSensorWrench);

      this.sensorMassCompensator = new ForceSensorDistalMassCompensator(wristSensorDefinition, DT, registry);

      yoWristSensorForceMagnitude = new DoubleYoVariable(forceSensorName + "ForceMag", registry);
      yoWristSensorForceMagnitudeBias = new FirstOrderFilteredYoVariable(forceSensorName + "ForceBias", "", 0.0001, DT, FirstOrderFilterType.LOW_PASS, registry);
      yoWristSensorForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoVariable(forceSensorName + "ForceMagFiltered", "",
            forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, DT, registry);

      taskspaceStiffnessCalc = new TaskSpaceStiffnessCalculator(sidePrefix, DT, registry);

      yoImpactStiffnessThreshold_NperM = new DoubleYoVariable(forceSensorName + "ImpactStiffnessThreshold_NperM", registry);
      yoImpactForceThreshold_N = new DoubleYoVariable(forceSensorName + "ImpactForceThreshold_N", registry);

      yoCollisionSeverityLevelOneToThree = new IntegerYoVariable(forceSensorName + "CollisionSeverity", "", registry, 1, 3);
      yoForceLimitExceeded = new BooleanYoVariable(forceSensorName + "forceLimitExceeded", registry);
      yoStiffnessLimitExceeded = new BooleanYoVariable(forceSensorName + "stiffnessLimitExceeded", registry);
      yoImpactDetected = new BooleanYoVariable(forceSensorName + "ImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new DoubleYoVariable(forceSensorName + "ImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(sidePrefix + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
      //            YoAppearance.OrangeRed());

      this.controllerCommunicator = behaviorPacketCommunicator;

      initialize();
   }

   private void initialize()
   {
      yoImpactForceThreshold_N.set(100.0);
      yoImpactStiffnessThreshold_NperM.set(10000.0 * 10000.0);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getDT()
   {
      return DT;
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
      forceSensorData.getWrench(wristSensorWrench);
      sensorMassCompensator.update(forceSensorData);

      FrameVector sensorForceRawInWorld = sensorMassCompensator.getSensorForceRaw(world);
      yoWristSensorForceMagnitude.set(sensorForceRawInWorld.length());
   }

   private double maxFilteredForce;

   private void stopArmIfHandCollisionIsDetected(double time)
   {
      estimateStiffnessOfConstraintsActingUponWrist();

      yoWristSensorForceMagnitudeBias.update(yoWristSensorForceMagnitude.getDoubleValue());
      yoWristSensorForceMagnitudeBandPassFiltered.update(yoWristSensorForceMagnitude.getDoubleValue());

      yoForceLimitExceeded.set(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() > yoImpactForceThreshold_N.getDoubleValue());

      double forceToForceLimitRatio = yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() / yoImpactForceThreshold_N.getDoubleValue();
      yoCollisionSeverityLevelOneToThree.set(MathTools.clipToMinMax((int) Math.round(forceToForceLimitRatio), 1, 3));

      //            yoForceLimitExceeded.set( taskspaceStiffnessCalc.getForceAlongDirectionOfMotion() > yoImpactForceThreshold_N.getDoubleValue() );
      //      yoStiffnessLimitExceeded.set(taskspaceStiffnessCalc.getStiffnessAlongDirectionOfMotion() > yoImpactStiffnessThreshold_NperM.getDoubleValue());

      if (yoForceLimitExceeded.getBooleanValue())
      {
         if (!yoImpactDetected.getBooleanValue() && time > 1.0 || yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() > maxFilteredForce)
         {
            yoImpactDetected.set(true);
            yoImpactTime.set(time);

            controllerCommunicator.send(new HandCollisionDetectedPacket(robotSide, yoCollisionSeverityLevelOneToThree.getIntegerValue()));
            if (DEBUG)
               PrintTools.debug(this, "Sending Collision Detected Packet.  FilteredForce = " + yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue());

         }
      }

      if (Math.abs(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue()) > maxFilteredForce)
      {
         maxFilteredForce = Math.abs(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue());
         if (DEBUG)
            PrintTools.debug(this, "maxFilteredForce = " + maxFilteredForce);
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
