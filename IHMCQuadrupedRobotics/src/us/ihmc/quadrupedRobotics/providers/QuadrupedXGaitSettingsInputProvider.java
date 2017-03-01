package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class QuadrupedXGaitSettingsInputProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter defaultStanceLengthParameter = parameterFactory.createDouble("defaultStanceLength", 1.1);
   private final DoubleParameter defaultStanceWidthParameter = parameterFactory.createDouble("defaultStanceWidth", 0.2);
   private final DoubleParameter defaultStepGroundClearanceParameter = parameterFactory.createDouble("defaultStepGroundClearance", 0.1);
   private final DoubleParameter defaultStepDurationParameter = parameterFactory.createDouble("defaultStepDurationParameter", 0.33);
   private final DoubleParameter defaultEndDoubleSupportDurationParameter = parameterFactory.createDouble("defaultEndDoubleSupportDuration", 0.0);
   private final DoubleParameter defaultEndPhaseShiftParameter = parameterFactory.createDouble("defaultEndPhaseShift", 180);
   private final DoubleParameter stanceLengthLowerLimitParameter = parameterFactory.createDouble("stanceLengthLowerLimit", 0.8);
   private final DoubleParameter stanceLengthUpperLimitParameter = parameterFactory.createDouble("stanceLengthUpperLimit", 1.4);
   private final DoubleParameter stanceWidthLowerLimitParameter = parameterFactory.createDouble("stanceWidthLowerLimit", 0.1);
   private final DoubleParameter stanceWidthUpperLimitParameter = parameterFactory.createDouble("stanceWidthUpperLimit", 0.6);
   private final DoubleParameter stepGroundClearanceLowerLimitParameter = parameterFactory.createDouble("stepGroundClearanceLowerLimit", 0.0);
   private final DoubleParameter stepGroundClearanceUpperLimitParameter = parameterFactory.createDouble("stepGroundClearanceUpperLimit", 0.25);
   private final DoubleParameter stepDurationLowerLimitParameter = parameterFactory.createDouble("stepDurationLowerLimit", 0.15);
   private final DoubleParameter stepDurationUpperLimitParameter = parameterFactory.createDouble("stepDurationUpperLimit", 0.6);
   private final DoubleParameter endDoubleSupportDurationLowerLimitParameter = parameterFactory.createDouble("endDoubleSupportDurationLowerLimit", 0.0);
   private final DoubleParameter endDoubleSupportDurationUpperLimitParameter = parameterFactory.createDouble("endDoubleSupportDurationUpperLimit", Double.MAX_VALUE);
   private final DoubleParameter endPhaseShiftLowerLimitParameter = parameterFactory.createDouble("endPhaseShiftLowerLimit", 0);
   private final DoubleParameter endPhaseShiftUpperLimitParameter = parameterFactory.createDouble("endPhaseShiftUpperLimit", 359);

   private final DoubleYoVariable yoStanceLength;
   private final DoubleYoVariable yoStanceWidth;
   private final DoubleYoVariable yoStepGroundClearance;
   private final DoubleYoVariable yoStepDuration;
   private final DoubleYoVariable yoEndDoubleSupportDuration;
   private final DoubleYoVariable yoEndPhaseShift;

   public QuadrupedXGaitSettingsInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      yoStanceLength = new DoubleYoVariable("stanceLengthInput", registry);
      yoStanceWidth = new DoubleYoVariable("stanceWidthInput", registry);
      yoStepGroundClearance = new DoubleYoVariable("stepGroundClearanceInput", registry);
      yoStepDuration = new DoubleYoVariable("stepDurationInput", registry);
      yoEndDoubleSupportDuration = new DoubleYoVariable("endDoubleSupportDurationInput", registry);
      yoEndPhaseShift = new DoubleYoVariable("endPhaseShiftInput", registry);

      yoStanceLength.set(defaultStanceLengthParameter.get());
      yoStanceWidth.set(defaultStanceWidthParameter.get());
      yoStepGroundClearance.set(defaultStepGroundClearanceParameter.get());
      yoStepDuration.set(defaultStepDurationParameter.get());
      yoEndDoubleSupportDuration.set(defaultEndDoubleSupportDurationParameter.get());
      yoEndPhaseShift.set(defaultEndPhaseShiftParameter.get());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedXGaitSettingsPacket.class, new PacketConsumer<QuadrupedXGaitSettingsPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedXGaitSettingsPacket xGaitSettingsPacket)
            {
               QuadrupedXGaitSettings xGaitSettings = xGaitSettingsPacket.get();
               yoStanceLength.set(MathTools.clamp(xGaitSettings.getStanceLength(), stanceLengthLowerLimitParameter.get(), stanceLengthUpperLimitParameter.get()));
               yoStanceWidth.set(MathTools.clamp(xGaitSettings.getStanceWidth(), stanceWidthLowerLimitParameter.get(), stanceWidthUpperLimitParameter.get()));
               yoStepGroundClearance.set(MathTools.clamp(xGaitSettings.getStepGroundClearance(), stepGroundClearanceLowerLimitParameter.get(), stepGroundClearanceUpperLimitParameter.get()));
               yoStepDuration.set(MathTools.clamp(xGaitSettings.getStepDuration(), stepDurationLowerLimitParameter.get(), stepDurationUpperLimitParameter.get()));
               yoEndDoubleSupportDuration.set(MathTools.clamp(xGaitSettings.getEndDoubleSupportDuration(), endDoubleSupportDurationLowerLimitParameter.get(), endDoubleSupportDurationUpperLimitParameter.get()));
               yoEndPhaseShift.set(MathTools.clamp(xGaitSettings.getEndPhaseShift(), endPhaseShiftLowerLimitParameter.get(), endPhaseShiftUpperLimitParameter.get()));
            }
         });
      }

      parentRegistry.addChild(registry);
   }

   public void getSettings(QuadrupedXGaitSettings xGaitSettings)
   {
      xGaitSettings.setStanceLength(yoStanceLength.getDoubleValue());
      xGaitSettings.setStanceWidth(yoStanceWidth.getDoubleValue());
      xGaitSettings.setStepGroundClearance(yoStepGroundClearance.getDoubleValue());
      xGaitSettings.setStepDuration(yoStepDuration.getDoubleValue());
      xGaitSettings.setEndDoubleSupportDuration(yoEndDoubleSupportDuration.getDoubleValue());
      xGaitSettings.setEndPhaseShift(yoEndPhaseShift.getDoubleValue());
   }
}
