package us.ihmc.quadrupedRobotics.input.mode;

import net.java.games.input.Event;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedBodyPoseTeleopManager;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Map;

public class QuadrupedStepTeleopMode
{
   private static final double DT = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter yawScaleParameter = new DoubleParameter("yawScale", registry, 0.15);
   private final DoubleParameter pitchScaleParameter = new DoubleParameter("pitchScale", registry, 0.15);
   private final DoubleParameter zdotScaleParameter = new DoubleParameter("zdotScale", registry, 0.25);
   private final DoubleParameter xStrideMax = new DoubleParameter("xStrideMax", registry, 0.4);
   private final DoubleParameter yStrideMax = new DoubleParameter("yStrideMax", registry, 0.25);
   private final DoubleParameter yawRateScale = new DoubleParameter("yawRateScale", registry, 0.25);

   // xgait step parameters
   private final DoubleParameter[] xGaitStepDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndDoubleSupportDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndPhaseShift = new DoubleParameter[2];

   private final DoubleParameter xGaitBodyOrientationShiftTime = new DoubleParameter("xGaitBodyOrientationShiftTime", registry, 0.1);;

   private final QuadrupedStepTeleopManager stepTeleopManager;
   private final QuadrupedBodyPoseTeleopManager bodyPoseTeleopManager;
   private InputValueIntegrator comZ;

   public QuadrupedStepTeleopMode(PacketCommunicator packetCommunicator, QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                  QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.stepTeleopManager = new QuadrupedStepTeleopManager(packetCommunicator, defaultXGaitSettings, referenceFrames, registry);
      this.bodyPoseTeleopManager = new QuadrupedBodyPoseTeleopManager(physicalProperties.getNominalCoMHeight(), packetCommunicator);
      this.comZ = new InputValueIntegrator(DT, physicalProperties.getNominalCoMHeight());

      xGaitStepDuration[0] = new DoubleParameter("xGaitStepDurationMode0", registry, 0.5);
      xGaitStepDuration[1] = new DoubleParameter("xGaitStepDurationMode1", registry, 0.33);
      xGaitEndDoubleSupportDuration[0] = new DoubleParameter("xGaitEndDoubleSupportDurationMode0", registry, 1.0);
      xGaitEndDoubleSupportDuration[1] = new DoubleParameter("xGaitEndDoubleSupportDurationMode1", registry, 0.05);
      xGaitEndPhaseShift[0] = new DoubleParameter("xGaitEndPhaseShiftMode0", registry, 90);
      xGaitEndPhaseShift[1] = new DoubleParameter("xGaitEndPhaseShiftMode1", registry, 180);

      parentRegistry.addChild(registry);
   }

   public void update(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates)
   {
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * pitchScaleParameter.getValue();
      double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawScaleParameter.getValue();
      bodyPoseTeleopManager.setDesiredBodyOrientation(bodyYaw, bodyPitch, bodyRoll, xGaitBodyOrientationShiftTime.getValue());

      double comZdot = 0.0;
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         comZdot += zdotScaleParameter.getValue();
      }
      if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         comZdot -= zdotScaleParameter.getValue();
      }
      bodyPoseTeleopManager.setDesiredCoMHeight(comZ.update(comZdot));
      bodyPoseTeleopManager.update();

      YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
      double xVelocityMax = 0.5 * xStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yVelocityMax = 0.5 * yStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double yawRateMax = yawRateScale.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
      double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
      double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
      double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
      stepTeleopManager.setDesiredVelocity(xVelocity, yVelocity, yawRate);
      stepTeleopManager.update();
   }

   public void onInputEvent(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimates estimates, Event event)
   {
      if (event.getValue() < 0.5)
         return;

      if(XBoxOneMapping.getMapping(event) == XBoxOneMapping.A)
      {
         stepTeleopManager.requestSteppingState();
         if(stepTeleopManager.isWalking())
            stepTeleopManager.requestStanding();
      }

      if(XBoxOneMapping.getMapping(event) == XBoxOneMapping.X)
      {
         stepTeleopManager.requestXGait();
      }

      YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
      switch (XBoxOneMapping.getMapping(event))
      {
      case RIGHT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[0].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[0].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[0].getValue());
         break;
      case LEFT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[1].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[1].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[1].getValue());
         break;
      }
   }
}
