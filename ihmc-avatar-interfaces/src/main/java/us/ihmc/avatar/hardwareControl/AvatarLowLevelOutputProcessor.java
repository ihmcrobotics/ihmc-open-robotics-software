package us.ihmc.avatar.hardwareControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AvatarLowLevelOutputProcessor
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final double RAMP_UP_DOWN_DURATION = 6.0;
   private static final double LOW_MASTER_GAIN = 0.0;
   private static final double HIGH_MASTER_GAIN = 1.00;

   private final double updateDt;

   private final YoBoolean servo = new YoBoolean("servoRobot", registry);
   private final YoBoolean unservoQuickly = new YoBoolean("unservoQuickly", registry);
   private final YoBoolean isServoing = new YoBoolean("isServoing", registry);
   private final YoBoolean isUnservoing = new YoBoolean("isUnservoing", registry);
   private final YoBoolean isServod = new YoBoolean("isRobotServod", registry);

   private final YoDouble servoTime = new YoDouble("servoTime", registry);
   private final YoDouble masterGain = new YoDouble("masterGain", registry);

   private final YoLowLevelOneDoFJointDesiredDataHolder unprocessedDesireds;
   private final YoLowLevelOneDoFJointDesiredDataHolder processedDesireds;

   private double servoStartGain = 0.0;
   private double unservoStartGain = 0.0;

   public AvatarLowLevelOutputProcessor(OneDoFJointBasics[] controlledJoints, double updateDt, YoRegistry parentRegistry)
   {
      this.updateDt = updateDt;

      unprocessedDesireds = new YoLowLevelOneDoFJointDesiredDataHolder("h1", controlledJoints, registry);
      processedDesireds = new YoLowLevelOneDoFJointDesiredDataHolder("h1Processed", controlledJoints, registry);

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
                                       unservoQuickly.set(false, false);
                                    }
                                 });

      parentRegistry.addChild(registry);
   }

   public void update(JointDesiredOutputListReadOnly unprocessedDesireds)
   {
      this.unprocessedDesireds.overwriteWith(unprocessedDesireds);
      processedDesireds.overwriteWith(unprocessedDesireds);

      if (isServoing.getBooleanValue())
         computeMasterGainForServo();

      if (isUnservoing.getBooleanValue())
         computeMasterGainForUnservo();

      for (int i = 0; i < processedDesireds.getNumberOfJointsWithDesiredOutput(); i++)
         processedDesireds.getJointDesiredOutput(i).setMasterGain(masterGain.getDoubleValue());
   }

   private void computeMasterGainForServo()
   {
      if (servoTime.getDoubleValue() < RAMP_UP_DOWN_DURATION)
      {
         servoTime.add(updateDt);
         masterGain.set(computeMasterGain(servoTime.getDoubleValue(), servoStartGain, HIGH_MASTER_GAIN));
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
      if (servoTime.getDoubleValue() < RAMP_UP_DOWN_DURATION)
      {
         servoTime.add(updateDt);
         masterGain.set(computeMasterGain(servoTime.getDoubleValue(), unservoStartGain, LOW_MASTER_GAIN));
      }
      else
      {
         isServod.set(false);
         isUnservoing.set(false);
         servoTime.set(0.0);
      }
   }

   public void servoRobot()
   {
      servo.set(true);
   }

   public void unservoRobot()
   {
      servo.set(false);
   }

   public void addServoListener(YoVariableChangedListener listener)
   {
      servo.addListener(listener);
   }

   private static double computeMasterGain(double servoTime, double startGain, double endGain)
   {
      double alpha = servoTime / RAMP_UP_DOWN_DURATION;
      double masterGain = InterpolationTools.linearInterpolate(startGain, endGain, alpha);
      return MathTools.clamp(masterGain, LOW_MASTER_GAIN, HIGH_MASTER_GAIN);
   }

   public JointDesiredOutputListReadOnly getProcessedDesiredOutput()
   {
      return processedDesireds;
   }
}
