package us.ihmc.humanoidBehaviors.utilities;

import java.util.List;

import controller_msgs.msg.dds.HandCollisionDetectedPacket;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.filters.AlphaBasedOnBreakFrequencyProvider;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.filters.FirstOrderBandPassFilteredYoDouble;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class WristForceSensorFilteredUpdatable implements Updatable
{
   private final boolean DEBUG = false;

   private final double DT;
   private final RobotSide robotSide;

   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;

   private final ForceSensorData forceSensorData;

   private final YoDouble yoWristSensorForceMagnitude;
   private final AlphaFilteredYoVariable yoWristSensorForceMagnitudeBias;
   private final FirstOrderBandPassFilteredYoDouble yoWristSensorForceMagnitudeBandPassFiltered;

   private final YoInteger yoCollisionSeverityLevelOneToThree;
   private final YoBoolean yoForceLimitExceeded;
   private final YoBoolean yoStiffnessLimitExceeded;
   private final YoBoolean yoImpactDetected;
   private final YoDouble yoImpactTime;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final ForceSensorDistalMassCompensator sensorMassCompensator;

   private final TaskSpaceStiffnessCalculator taskspaceStiffnessCalc;

   private final YoDouble yoImpactStiffnessThreshold_NperM;
   private final YoDouble yoImpactForceThreshold_N;

   private final ROS2PublisherBasics<HandCollisionDetectedPacket> publisher;

   public WristForceSensorFilteredUpdatable(String robotName,
                                            RobotSide robotSide,
                                            FullRobotModel fullRobotModel,
                                            HumanoidRobotSensorInformation sensorInfo,
                                            ForceSensorDataHolder forceSensorDataHolder,
                                            double DT,
                                            ROS2Node ros2Node,
                                            YoRegistry registry)
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
      this.forceSensorData = forceSensorDataHolder.getData(forceSensorName);

      this.sensorMassCompensator = new ForceSensorDistalMassCompensator(wristSensorDefinition, DT, registry);

      yoWristSensorForceMagnitude = new YoDouble(forceSensorName + "ForceMag", registry);
      yoWristSensorForceMagnitudeBias = new AlphaFilteredYoVariable(forceSensorName + "ForceBias",
                                                                    registry,
                                                                    new AlphaBasedOnBreakFrequencyProvider(() -> 0.0001, DT));
      yoWristSensorForceMagnitudeBandPassFiltered = new FirstOrderBandPassFilteredYoDouble(forceSensorName + "ForceMagFiltered",
                                                                                           "",
                                                                                           forceSensorMinPassThroughFreq_Hz,
                                                                                           forceSensorMaxPassThroughFreq_Hz,
                                                                                           DT,
                                                                                           FirstOrderBandPassFilteredYoDouble.FirstOrderFilterType.BAND,
                                                                                           registry);

      taskspaceStiffnessCalc = new TaskSpaceStiffnessCalculator(sidePrefix, DT, registry);

      yoImpactStiffnessThreshold_NperM = new YoDouble(forceSensorName + "ImpactStiffnessThreshold_NperM", registry);
      yoImpactForceThreshold_N = new YoDouble(forceSensorName + "ImpactForceThreshold_N", registry);

      yoCollisionSeverityLevelOneToThree = new YoInteger(forceSensorName + "CollisionSeverity", "", registry);
      yoCollisionSeverityLevelOneToThree.setVariableBounds(1, 3);
      yoForceLimitExceeded = new YoBoolean(forceSensorName + "forceLimitExceeded", registry);
      yoStiffnessLimitExceeded = new YoBoolean(forceSensorName + "stiffnessLimitExceeded", registry);
      yoImpactDetected = new YoBoolean(forceSensorName + "ImpactDetected", registry);
      yoImpactDetected.set(false);

      yoImpactTime = new YoDouble(forceSensorName + "ImpactTime", registry);

      //      YoGraphicVector wristForceViz = new YoGraphicVector(sidePrefix + "Wrist Force", yoWristSensorPoint, yoWristSensorForce,
      //            YoAppearance.OrangeRed());

      publisher = ros2Node.createPublisher(IHMCHumanoidBehaviorManager.getOutputTopic(robotName).withTypeName(HandCollisionDetectedPacket.class));

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

   public FramePoint3DReadOnly getWristPositionInWorld()
   {
      return sensorMassCompensator.getSensorPosition();
   }

   public FrameVector3DReadOnly getWristForceRaw()
   {
      return sensorMassCompensator.getSensorForceRaw();
   }

   public FrameVector3DReadOnly getWristForceMassCompensated()
   {
      return sensorMassCompensator.getSensorForceMassCompensated();
   }

   public YoDouble getWristForceMagnitude()
   {
      return yoWristSensorForceMagnitude;
   }

   public YoDouble getWristForceBandPassFiltered()
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
      sensorMassCompensator.update(forceSensorData.getWrench());

      yoWristSensorForceMagnitude.set(sensorMassCompensator.getSensorForceRaw().norm());
   }

   private double maxFilteredForce;

   private void stopArmIfHandCollisionIsDetected(double time)
   {
      estimateStiffnessOfConstraintsActingUponWrist();

      yoWristSensorForceMagnitudeBias.update(yoWristSensorForceMagnitude.getDoubleValue());
      yoWristSensorForceMagnitudeBandPassFiltered.update(yoWristSensorForceMagnitude.getDoubleValue());

      yoForceLimitExceeded.set(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() > yoImpactForceThreshold_N.getDoubleValue());

      double forceToForceLimitRatio = yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() / yoImpactForceThreshold_N.getDoubleValue();
      yoCollisionSeverityLevelOneToThree.set(MathTools.clamp((int) Math.round(forceToForceLimitRatio), 1, 3));

      //            yoForceLimitExceeded.set( taskspaceStiffnessCalc.getForceAlongDirectionOfMotion() > yoImpactForceThreshold_N.getDoubleValue() );
      //      yoStiffnessLimitExceeded.set(taskspaceStiffnessCalc.getStiffnessAlongDirectionOfMotion() > yoImpactStiffnessThreshold_NperM.getDoubleValue());

      if (yoForceLimitExceeded.getBooleanValue())
      {
         if (!yoImpactDetected.getBooleanValue() && time > 1.0 || yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue() > maxFilteredForce)
         {
            yoImpactDetected.set(true);
            yoImpactTime.set(time);

            publisher.publish(HumanoidMessageTools.createHandCollisionDetectedPacket(robotSide, yoCollisionSeverityLevelOneToThree.getIntegerValue()));
            if (DEBUG)
               LogTools.debug("Sending Collision Detected Packet.  FilteredForce = " + yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue());

         }
      }

      if (Math.abs(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue()) > maxFilteredForce)
      {
         maxFilteredForce = Math.abs(yoWristSensorForceMagnitudeBandPassFiltered.getDoubleValue());
         if (DEBUG)
            LogTools.debug("maxFilteredForce = " + maxFilteredForce);
      }
   }

   private final FrameVector3D wristSensorForceInWorld = new FrameVector3D();

   private void estimateStiffnessOfConstraintsActingUponWrist()
   {
      FramePoint3DReadOnly sensorPositionInWorld = sensorMassCompensator.getSensorPosition();

      wristSensorForceInWorld.setIncludingFrame(sensorMassCompensator.getSensorForceMassCompensated());
      wristSensorForceInWorld.changeFrame(world);

      taskspaceStiffnessCalc.update(sensorPositionInWorld, wristSensorForceInWorld);
   }
}
