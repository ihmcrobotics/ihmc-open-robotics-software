package us.ihmc.quadrupedPlanning;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;

public class YoQuadrupedXGaitSettings implements QuadrupedXGaitSettingsReadOnly
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter stanceLengthLowerLimitParameter = new DoubleParameter("stanceLengthLowerLimit", registry, 0.4);
   private final DoubleParameter stanceLengthUpperLimitParameter = new DoubleParameter("stanceLengthUpperLimit", registry, 1.4);
   private final DoubleParameter stanceWidthLowerLimitParameter = new DoubleParameter("stanceWidthLowerLimit", registry, 0.1);
   private final DoubleParameter stanceWidthUpperLimitParameter = new DoubleParameter("stanceWidthUpperLimit", registry, 0.6);
   private final DoubleParameter stepGroundClearanceLowerLimitParameter = new DoubleParameter("stepGroundClearanceLowerLimit", registry, 0.0);
   private final DoubleParameter stepGroundClearanceUpperLimitParameter = new DoubleParameter("stepGroundClearanceUpperLimit", registry, 0.25);
   private final DoubleParameter stepDurationLowerLimitParameter = new DoubleParameter("stepDurationLowerLimit", registry, 0.15);
   private final DoubleParameter stepDurationUpperLimitParameter = new DoubleParameter("stepDurationUpperLimit", registry, 0.6);
   private final DoubleParameter endDoubleSupportDurationLowerLimitParameter = new DoubleParameter("endDoubleSupportDurationLowerLimit", registry, 0.0);
   private final DoubleParameter endDoubleSupportDurationUpperLimitParameter = new DoubleParameter("endDoubleSupportDurationUpperLimit", registry,
                                                                                                   Double.MAX_VALUE);
   private final DoubleParameter endPhaseShiftLowerLimitParameter = new DoubleParameter("endPhaseShiftLowerLimit", registry, 0);
   private final DoubleParameter endPhaseShiftUpperLimitParameter = new DoubleParameter("endPhaseShiftUpperLimit", registry, 359);

   private final YoDouble yoStanceLength = new YoDouble("stanceLengthInput", registry);
   private final YoDouble yoStanceWidth = new YoDouble("stanceWidthInput", registry);
   private final YoDouble yoStepGroundClearance = new YoDouble("stepGroundClearanceInput", registry);
   private final YoDouble yoStepDuration = new YoDouble("stepDurationInput", registry);
   private final YoDouble yoEndDoubleSupportDuration = new YoDouble("endDoubleSupportDurationInput", registry);
   private final YoDouble yoEndPhaseShift = new YoDouble("endPhaseShiftInput", registry);

   public YoQuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, GlobalDataProducer globalDataProducer,
                                   YoVariableRegistry parentRegistry)
   {
      yoStanceLength.set(defaultXGaitSettings.getStanceLength());
      yoStanceWidth.set(defaultXGaitSettings.getStanceWidth());
      yoStepGroundClearance.set(defaultXGaitSettings.getStepGroundClearance());
      yoStepDuration.set(defaultXGaitSettings.getStepDuration());
      yoEndDoubleSupportDuration.set(defaultXGaitSettings.getEndDoubleSupportDuration());
      yoEndPhaseShift.set(defaultXGaitSettings.getEndPhaseShift());

      parentRegistry.addChild(registry);
   }

   public void addVariableChangedListener(VariableChangedListener listener)
   {
      yoEndDoubleSupportDuration.addVariableChangedListener(listener);
      yoEndPhaseShift.addVariableChangedListener(listener);
      yoStanceLength.addVariableChangedListener(listener);
      yoStanceWidth.addVariableChangedListener(listener);
      yoStepGroundClearance.addVariableChangedListener(listener);
      yoStepDuration.addVariableChangedListener(listener);
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
      yoStanceLength.set(MathTools.clamp(stanceLength, stanceLengthLowerLimitParameter.getValue(), stanceLengthUpperLimitParameter.getValue()));
   }

   public void setStanceWidth(double stanceWidth)
   {
      yoStanceWidth.set(MathTools.clamp(stanceWidth, stanceWidthLowerLimitParameter.getValue(), stanceWidthUpperLimitParameter.getValue()));
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      yoStepGroundClearance
            .set(MathTools.clamp(stepGroundClearance, stepGroundClearanceLowerLimitParameter.getValue(), stepGroundClearanceUpperLimitParameter.getValue()));
   }

   public void setStepDuration(double stepDuration)
   {
      yoStepDuration.set(MathTools.clamp(stepDuration, stepDurationLowerLimitParameter.getValue(), stepDurationUpperLimitParameter.getValue()));
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      yoEndDoubleSupportDuration.set(MathTools.clamp(endDoubleSupportDuration, endDoubleSupportDurationLowerLimitParameter.getValue(),
                                                     endDoubleSupportDurationUpperLimitParameter.getValue()));
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      yoEndPhaseShift.set(MathTools.clamp(endPhaseShift, endPhaseShiftLowerLimitParameter.getValue(), endPhaseShiftUpperLimitParameter.getValue()));
   }

   public void set(QuadrupedXGaitSettingsReadOnly other)
   {
      setStanceLength(other.getStanceLength());
      setStanceWidth(other.getStanceWidth());
      setStepGroundClearance(other.getStepGroundClearance());
      setStepDuration(other.getStepDuration());
      setEndDoubleSupportDuration(other.getEndDoubleSupportDuration());
      setEndPhaseShift(other.getEndPhaseShift());
   }

   public void set(QuadrupedXGaitSettingsPacket packet)
   {
      if (packet.getStanceLength() != -1.0)
         setStanceLength(packet.getStanceLength());
      if (packet.getStanceWidth() != -1.0)
         setStanceWidth(packet.getStanceWidth());
      if (packet.getStepGroundClearance() != -1.0)
         setStepGroundClearance(packet.getStepGroundClearance());
      if (packet.getStepDuration() != -1.0)
         setStepDuration(packet.getStepDuration());
      if (packet.getEndDoubleSupportDuration() != -1.0)
         setEndDoubleSupportDuration(packet.getEndDoubleSupportDuration());
      if (packet.getEndPhaseShift() != -1.0)
         setEndPhaseShift(packet.getEndPhaseShift());
   }
}
