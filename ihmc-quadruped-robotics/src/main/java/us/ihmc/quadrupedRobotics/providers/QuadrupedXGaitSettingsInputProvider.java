package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedXGaitSettingsInputProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double defaultStanceLength = 1.1;
   private static final double defaultStanceWidth = 0.2;
   private static final double defaultStepGroundClearance = 0.1;
   private static final double defaultStepDuration = 0.33;
   private static final double defaultEndDoubleSupportDuration = 0.0;
   private static final double defaultEndPhaseShift = 180;

   private final DoubleParameter stanceLengthLowerLimitParameter = new DoubleParameter("stanceLengthLowerLimit", registry, 0.8);
   private final DoubleParameter stanceLengthUpperLimitParameter = new DoubleParameter("stanceLengthUpperLimit", registry, 1.4);
   private final DoubleParameter stanceWidthLowerLimitParameter = new DoubleParameter("stanceWidthLowerLimit", registry, 0.1);
   private final DoubleParameter stanceWidthUpperLimitParameter = new DoubleParameter("stanceWidthUpperLimit", registry, 0.6);
   private final DoubleParameter stepGroundClearanceLowerLimitParameter = new DoubleParameter("stepGroundClearanceLowerLimit", registry, 0.0);
   private final DoubleParameter stepGroundClearanceUpperLimitParameter = new DoubleParameter("stepGroundClearanceUpperLimit", registry, 0.25);
   private final DoubleParameter stepDurationLowerLimitParameter = new DoubleParameter("stepDurationLowerLimit", registry, 0.15);
   private final DoubleParameter stepDurationUpperLimitParameter = new DoubleParameter("stepDurationUpperLimit", registry, 0.6);
   private final DoubleParameter endDoubleSupportDurationLowerLimitParameter = new DoubleParameter("endDoubleSupportDurationLowerLimit", registry, 0.0);
   private final DoubleParameter endDoubleSupportDurationUpperLimitParameter = new DoubleParameter("endDoubleSupportDurationUpperLimit", registry, Double.MAX_VALUE);
   private final DoubleParameter endPhaseShiftLowerLimitParameter = new DoubleParameter("endPhaseShiftLowerLimit", registry, 0);
   private final DoubleParameter endPhaseShiftUpperLimitParameter = new DoubleParameter("endPhaseShiftUpperLimit", registry, 359);

   private final YoDouble yoStanceLength;
   private final YoDouble yoStanceWidth;
   private final YoDouble yoStepGroundClearance;
   private final YoDouble yoStepDuration;
   private final YoDouble yoEndDoubleSupportDuration;
   private final YoDouble yoEndPhaseShift;

   public QuadrupedXGaitSettingsInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      yoStanceLength = new YoDouble("stanceLengthInput", registry);
      yoStanceWidth = new YoDouble("stanceWidthInput", registry);
      yoStepGroundClearance = new YoDouble("stepGroundClearanceInput", registry);
      yoStepDuration = new YoDouble("stepDurationInput", registry);
      yoEndDoubleSupportDuration = new YoDouble("endDoubleSupportDurationInput", registry);
      yoEndPhaseShift = new YoDouble("endPhaseShiftInput", registry);

      yoStanceLength.set(defaultStanceLength);
      yoStanceWidth.set(defaultStanceWidth);
      yoStepGroundClearance.set(defaultStepGroundClearance);
      yoStepDuration.set(defaultStepDuration);
      yoEndDoubleSupportDuration.set(defaultEndDoubleSupportDuration);
      yoEndPhaseShift.set(defaultEndPhaseShift);

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedXGaitSettingsPacket.class, new PacketConsumer<QuadrupedXGaitSettingsPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedXGaitSettingsPacket xGaitSettingsPacket)
            {
               QuadrupedXGaitSettings xGaitSettings = xGaitSettingsPacket.get();
               yoStanceLength.set(MathTools.clamp(xGaitSettings.getStanceLength(), stanceLengthLowerLimitParameter.getValue(), stanceLengthUpperLimitParameter.getValue()));
               yoStanceWidth.set(MathTools.clamp(xGaitSettings.getStanceWidth(), stanceWidthLowerLimitParameter.getValue(), stanceWidthUpperLimitParameter.getValue()));
               yoStepGroundClearance.set(MathTools.clamp(xGaitSettings.getStepGroundClearance(), stepGroundClearanceLowerLimitParameter.getValue(), stepGroundClearanceUpperLimitParameter.getValue()));
               yoStepDuration.set(MathTools.clamp(xGaitSettings.getStepDuration(), stepDurationLowerLimitParameter.getValue(), stepDurationUpperLimitParameter.getValue()));
               yoEndDoubleSupportDuration.set(MathTools.clamp(xGaitSettings.getEndDoubleSupportDuration(), endDoubleSupportDurationLowerLimitParameter.getValue(), endDoubleSupportDurationUpperLimitParameter.getValue()));
               yoEndPhaseShift.set(MathTools.clamp(xGaitSettings.getEndPhaseShift(), endPhaseShiftLowerLimitParameter.getValue(), endPhaseShiftUpperLimitParameter.getValue()));
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

   public static void main(String[] args)
   {
      System.out.println(Double.MAX_VALUE);
   }
}
