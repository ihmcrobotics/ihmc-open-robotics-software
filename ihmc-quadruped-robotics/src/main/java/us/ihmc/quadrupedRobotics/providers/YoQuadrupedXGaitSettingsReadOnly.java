package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoQuadrupedXGaitSettingsReadOnly implements QuadrupedXGaitSettingsReadOnly
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

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

   private final YoDouble yoStanceLength = new YoDouble("stanceLengthInput", registry);
   private final YoDouble yoStanceWidth = new YoDouble("stanceWidthInput", registry);
   private final YoDouble yoStepGroundClearance = new YoDouble("stepGroundClearanceInput", registry);
   private final YoDouble yoStepDuration = new YoDouble("stepDurationInput", registry);
   private final YoDouble yoEndDoubleSupportDuration = new YoDouble("endDoubleSupportDurationInput", registry);
   private final YoDouble yoEndPhaseShift = new YoDouble("endPhaseShiftInput", registry);

   public YoQuadrupedXGaitSettingsReadOnly(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      yoStanceLength.set(defaultXGaitSettings.getStanceLength());
      yoStanceWidth.set(defaultXGaitSettings.getStanceWidth());
      yoStepGroundClearance.set(defaultXGaitSettings.getStepGroundClearance());
      yoStepDuration.set(defaultXGaitSettings.getStepDuration());
      yoEndDoubleSupportDuration.set(defaultXGaitSettings.getEndDoubleSupportDuration());
      yoEndPhaseShift.set(defaultXGaitSettings.getEndPhaseShift());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedXGaitSettingsPacket.class, new PacketConsumer<QuadrupedXGaitSettingsPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedXGaitSettingsPacket xGaitSettingsPacket)
            {
               QuadrupedXGaitSettingsReadOnly xGaitSettings = xGaitSettingsPacket.get();
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

   @Override
   public double getStanceLength()
   {
      return yoStanceLength.getDoubleValue();
   }

   @Override
   public double getStanceWidth()
   {
      return yoStanceWidth.getDoubleValue();
   }

   @Override
   public double getStepGroundClearance()
   {
      return yoStepGroundClearance.getDoubleValue();
   }

   @Override
   public double getStepDuration()
   {
      return yoStepDuration.getDoubleValue();
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return yoEndDoubleSupportDuration.getDoubleValue();
   }

   @Override
   public double getEndPhaseShift()
   {
      return yoEndPhaseShift.getDoubleValue();
   }



   public void setStanceLength(double stanceLength)
   {
      yoStanceLength.set(stanceLength);
   }

   public void setStanceWidth(double stanceWidth)
   {
      yoStanceWidth.set(stanceWidth);
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      yoStepGroundClearance.set(stepGroundClearance);
   }

   public void setStepDuration(double stepDuration)
   {
      yoStepDuration.set(stepDuration);
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      yoEndDoubleSupportDuration.set(endDoubleSupportDuration);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      yoEndPhaseShift.set(endPhaseShift);
   }

}
