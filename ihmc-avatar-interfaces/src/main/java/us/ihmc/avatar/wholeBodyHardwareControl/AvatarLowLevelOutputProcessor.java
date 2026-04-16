package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointControlBlender;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.LinkedHashMap;

/**
 * This class is responsible for applying a master gain to all low-level desired outputs.
 * It also provides a 'servo' and 'unservo' functionality which interpolates that master
 * gain between 0 and 1 over a period of time. This is designed to be the last layer of
 * control before these desired values get sent to the hardware drivers/managers.
 *
 * @author Stefan Fasano
 */
public class AvatarLowLevelOutputProcessor
{
   private static final boolean INCLUDE_INTERPOLATION_VARS = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry interpolationRegistry = new YoRegistry(getClass().getSimpleName() + "_interpolation");

   private static final double DEFAULT_SERVO_DURATION = 2.0; // In units of seconds
   private static final double LOW_MASTER_GAIN = 0.0;
   private static final double HIGH_MASTER_GAIN = 1.00;
   private final static boolean DEFAULT_OFFSET_DESIREDS_BY_FILTERS = false;

   private final double updateDt;
   private DoubleProvider yoTime;

   private final YoBoolean servo = new YoBoolean("servoRobot", registry);
   private final YoBoolean unservoQuickly = new YoBoolean("unservoQuickly", registry);
   private final YoBoolean isServoing = new YoBoolean("isServoing", registry);
   private final YoBoolean isUnservoing = new YoBoolean("isUnservoing", registry);
   private final YoBoolean isServod = new YoBoolean("isRobotServod", registry);

   private final YoDouble servoDuration = new YoDouble("servoDuration", registry);
   private final YoDouble servoTime = new YoDouble("servoTime", registry);
   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoBoolean interpolateDesireds = new YoBoolean("interpolateDesireds", interpolationRegistry);
   private final YoDouble interpolateDuration = new YoDouble("interpolateDuration", interpolationRegistry);
   private final YoDouble interpolationStartTime = new YoDouble("interpolationTime", interpolationRegistry);
   private final YoDouble interpolationRatio = new YoDouble("interpolationRatio", interpolationRegistry);
   private final YoBoolean offsetsDesiredsByFilters = new YoBoolean("offsetsDesiredsByFilters", registry);

   private final YoDouble timeFromEstimator = new YoDouble("timeFromEstimator", registry);
   private TimestampProvider monotonicTime;

   private final LinkedHashMap<OneDoFJointReadOnly, YoDouble> jointPositionOffsets = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointReadOnly, YoDouble> jointVelocityOffsets = new LinkedHashMap<>();

   private final JointControlBlender[] jointControlBlenders;

   private final OneDoFJointReadOnly[] controlledJoints;

   private final YoLowLevelOneDoFJointDesiredDataHolder unprocessedDesireds;
   private final YoLowLevelOneDoFJointDesiredDataHolder previousDesireds;
   private final YoLowLevelOneDoFJointDesiredDataHolder processedDesireds;

   private double servoStartGain = 0.0;
   private double unservoStartGain = 0.0;

   public AvatarLowLevelOutputProcessor(String robotName, OneDoFJointReadOnly[] controlledJoints, double updateDt, YoRegistry parentRegistry)
   {
      this(robotName, controlledJoints, updateDt, null, parentRegistry);
   }

   public AvatarLowLevelOutputProcessor(String robotName, OneDoFJointReadOnly[] controlledJoints, double updateDt, TimestampProvider monotonicTime, YoRegistry parentRegistry)
   {
      this(robotName, controlledJoints, updateDt, null, monotonicTime, parentRegistry);
      yoTime = timeFromEstimator;
      this.monotonicTime = monotonicTime;
   }

   public AvatarLowLevelOutputProcessor(String robotName, OneDoFJointReadOnly[] controlledJoints, double updateDt, DoubleProvider yoTime, TimestampProvider monotonicTime, YoRegistry parentRegistry)
   {
      this.controlledJoints = controlledJoints;
      this.updateDt = updateDt;
      this.yoTime = yoTime;

      unprocessedDesireds = new YoLowLevelOneDoFJointDesiredDataHolder(robotName, controlledJoints, registry);
      previousDesireds = new YoLowLevelOneDoFJointDesiredDataHolder(robotName + "Previous", controlledJoints, interpolationRegistry);
      processedDesireds = new YoLowLevelOneDoFJointDesiredDataHolder(robotName + "Processed", controlledJoints, registry);

      jointControlBlenders = new JointControlBlender[controlledJoints.length];
      interpolateDuration.set(2 * updateDt);
      interpolationStartTime.set(0.0);

      for (int i = 0; i < controlledJoints.length; i++)
         jointControlBlenders[i] = new JointControlBlender("LowLevelOutputInterpolator", controlledJoints[i], registry);

      servoDuration.set(DEFAULT_SERVO_DURATION);
      offsetsDesiredsByFilters.set(DEFAULT_OFFSET_DESIREDS_BY_FILTERS);

      addServoListener(change ->
                       {
                          if (servo.getBooleanValue())
                          {
                             servoStartGain = masterGain.getDoubleValue();
                             servoTime.set(0.0);
                             isServoing.set(true);
                             isUnservoing.set(false);
                          }
                          else
                          {
                             unservoStartGain = masterGain.getDoubleValue();
                             servoTime.set(0.0);
                             isUnservoing.set(true);
                             isServoing.set(false);
                          }
                       });

      unservoQuickly.addListener(change ->
                                 {
                                    if (unservoQuickly.getBooleanValue())
                                    {
                                       servo.set(false);
                                       isServoing.set(false);
                                       isUnservoing.set(false);
                                       masterGain.set(0.0);
                                    }
                                 });

      masterGain.addListener(value ->
                             {
                                if (masterGain.getDoubleValue() < 1.0)
                                   isServod.set(false);
                                else
                                   isServod.set(true);
                             });

      interpolateDuration.addListener(change ->
                                      {
                                         if(interpolateDuration.getDoubleValue() < 0.0)
                                            interpolateDuration.set(updateDt);
                                      });

      parentRegistry.addChild(registry);
      if (INCLUDE_INTERPOLATION_VARS)
         parentRegistry.addChild(interpolationRegistry);
   }

   public void update(JointDesiredOutputListReadOnly unprocessedDesireds)
   {
      if (monotonicTime != null)
         timeFromEstimator.set(Conversions.nanosecondsToSeconds(monotonicTime.getTimestamp()));
      this.unprocessedDesireds.overwriteWith(unprocessedDesireds);

      if (interpolateDesireds.getBooleanValue())
         interpolate();
      else
         processedDesireds.overwriteWith(unprocessedDesireds);


      if (isServoing.getBooleanValue())
         computeMasterGainForServo();

      if (isUnservoing.getBooleanValue())
         computeMasterGainForUnservo();

      for (OneDoFJointReadOnly controlledJoint : controlledJoints)
      {
         JointDesiredOutputBasics jointDesiredOutput = processedDesireds.getJointDesiredOutput(controlledJoint);
         jointDesiredOutput.setMasterGain(masterGain.getDoubleValue());
         jointDesiredOutput.setDesiredPosition(jointDesiredOutput.getDesiredPosition() + getJointPositionOffset(controlledJoint));
         jointDesiredOutput.setDesiredVelocity(jointDesiredOutput.getDesiredVelocity() + getJointVelocityOffset(controlledJoint));
      }
      if (unservoQuickly.getBooleanValue())
         unservoQuickly.set(false, false);
   }

   public void enableInterpolation(boolean enable)
   {
      interpolateDesireds.set(enable);
   }

   public void startDesiredsInterpolation()
   {
      interpolationStartTime.set(yoTime.getValue());
      previousDesireds.overwriteWith(processedDesireds);
   }

   private void interpolate()
   {
      //TODO Figure out how to make the interp work without having to reduce duration
      double ratio = (yoTime.getValue() - interpolationStartTime.getValueAsDouble()) / (interpolateDuration.getValueAsDouble() - updateDt);
      ratio = MathTools.clamp(ratio, 0.0, 1.0);
      interpolationRatio.set(ratio);
      if (ratio <= 1.0)
      {
         for (int i = 0; i < processedDesireds.getNumberOfJointsWithDesiredOutput(); i++)
         {
            JointDesiredOutputReadOnly previousJointDesireds = previousDesireds.getJointDesiredOutput(i);
            JointDesiredOutputReadOnly currentJointDesireds = unprocessedDesireds.getJointDesiredOutput(i);

            jointControlBlenders[i].computeAndUpdateJointControl(processedDesireds.getJointDesiredOutput(i),
                                                                 previousJointDesireds,
                                                                 currentJointDesireds,
                                                                 interpolationRatio.getDoubleValue());
         }
      }

   }

   private void computeMasterGainForServo()
   {
      if (servoTime.getDoubleValue() < servoDuration.getDoubleValue())
      {
         servoTime.add(updateDt);
         masterGain.set(computeMasterGain(servoTime.getDoubleValue(), servoDuration.getDoubleValue(), servoStartGain, HIGH_MASTER_GAIN));
      }
      else
      {
         isServod.set(true);
         isServoing.set(false);
         servoTime.set(0.0);
      }
   }

   private void computeMasterGainForUnservo()
   {
      if (servoTime.getDoubleValue() < servoDuration.getDoubleValue())
      {
         servoTime.add(updateDt);
         masterGain.set(computeMasterGain(servoTime.getDoubleValue(), servoDuration.getDoubleValue(), unservoStartGain, LOW_MASTER_GAIN));
      }
      else
      {
         isServod.set(false);
         isUnservoing.set(false);
         servoTime.set(0.0);
      }
   }

   public boolean getIsRobotServod()
   {
      return isServod.getBooleanValue();
   }

   public void setServoDuration(double duration)
   {
      servoDuration.set(duration);
   }

   public void servoRobot()
   {
      servo.set(true);
   }

   public void unservoRobot()
   {
      servo.set(false);
   }

   public void unservoRobotQuickly()
   {
      unservoQuickly.set(true);
   }

   public void addServoListener(YoVariableChangedListener listener)
   {
      servo.addListener(listener);
   }

   public void addUnservoQuicklyListener(YoVariableChangedListener listener)
   {
      unservoQuickly.addListener(listener);
   }

   public void addMasterGainListener(YoVariableChangedListener listener)
   {
      masterGain.addListener(listener);
   }

   private static double computeMasterGain(double servoTime, double servoDuration, double startGain, double endGain)
   {
      double alpha = servoTime / servoDuration;
      double masterGain = InterpolationTools.linearInterpolate(startGain, endGain, alpha);
      return MathTools.clamp(masterGain, LOW_MASTER_GAIN, HIGH_MASTER_GAIN);
   }

   public void setJointPositionOffset(OneDoFJointReadOnly joint, double positionOffset)
   {
      getYoJointPositionOffset(joint).set(positionOffset);
   }

   public void setJointVelocityOffset(OneDoFJointReadOnly joint, double velocityOffset)
   {
      getYoJointVelocityOffset(joint).set(velocityOffset);
   }

   public double getJointPositionOffset(OneDoFJointReadOnly joint)
   {
      return getYoJointPositionOffset(joint).getDoubleValue();
   }

   public double getJointVelocityOffset(OneDoFJointReadOnly joint)
   {
      return getYoJointVelocityOffset(joint).getDoubleValue();
   }

   public YoDouble getYoJointPositionOffset(OneDoFJointReadOnly joint)
   {
      if (!jointPositionOffsets.containsKey(joint))
         jointPositionOffsets.put(joint, new YoDouble(joint.getName() + "_PositionOffset", registry));

      return jointPositionOffsets.get(joint);
   }

   public YoDouble getYoJointVelocityOffset(OneDoFJointReadOnly joint)
   {
      if (!jointVelocityOffsets.containsKey(joint))
         jointVelocityOffsets.put(joint, new YoDouble(joint.getName() + "_VelocityOffset", registry));

      return jointVelocityOffsets.get(joint);
   }

   public boolean getOffsetDesiredsByFilters()
   {
      return offsetsDesiredsByFilters.getBooleanValue();
   }

   public JointDesiredOutputListBasics getProcessedDesiredOutput()
   {
      return processedDesireds;
   }

   public DoubleProvider getMasterGain()
   {
      return masterGain;
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain.set(masterGain);
   }

   public void setInterpolationDuration(double duration)
   {
      this.interpolateDuration.set(duration);
   }

   public void setYoTime(DoubleProvider yoTime)
   {
      this.yoTime = yoTime;
   }
}
