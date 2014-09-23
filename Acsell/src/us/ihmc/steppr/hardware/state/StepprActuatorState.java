package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.steppr.hardware.state.slowSensors.BusVoltage;
import us.ihmc.steppr.hardware.state.slowSensors.MotorTemperature;
import us.ihmc.steppr.hardware.state.slowSensors.StepprSlowSensor;
import us.ihmc.steppr.hardware.state.slowSensors.mcbTemperature1;
import us.ihmc.steppr.hardware.state.slowSensors.mcbTemperature2;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class StepprActuatorState
{
   private final StepprSlowSensor[] slowSensors = new StepprSlowSensor[4];

   private final YoVariableRegistry registry;

   private final double motorKt;
   
   private final LongYoVariable microControllerTime;

   private final IntegerYoVariable rawEncoder;
   private final IntegerYoVariable rawPhaseACurrent;
   private final IntegerYoVariable rawPhaseBCurrent;
   private final IntegerYoVariable rawPhaseCCurrent;

   private final DoubleYoVariable inphaseCompositeStatorCurrent;
   private final DoubleYoVariable quadratureCompositeStatorCurrent;

   private final DoubleYoVariable controlTarget;
   private final DoubleYoVariable inphaseControlEffort;
   private final DoubleYoVariable quadratureControlEffort;

   private final DoubleYoVariable motorEncoder;

   private final DoubleYoVariable estimatedTorqueStrainGauge0;
   private final DoubleYoVariable estimatedTorqueStrainGauge1;

   private final DoubleYoVariable motorVelocityEstimate;

   private final LongYoVariable lastReceivedControlID;
   private final IntegerYoVariable halSensors;
   private final IntegerYoVariable controlMode;

   public StepprActuatorState(String name, double motorKt, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.motorKt = motorKt;
      this.microControllerTime = new LongYoVariable(name + "MicroControllerTime", registry);

      this.rawEncoder = new IntegerYoVariable(name + "RawEncoder", registry);
      this.rawPhaseACurrent = new IntegerYoVariable(name + "RawPhaseACurrent", registry);
      this.rawPhaseBCurrent = new IntegerYoVariable(name + "RawPhaseBCurrent", registry);
      this.rawPhaseCCurrent = new IntegerYoVariable(name + "RawPhaseCCurrent", registry);

      this.inphaseCompositeStatorCurrent = new DoubleYoVariable(name + "InphaseCompositeStatorCurrent", registry);
      this.quadratureCompositeStatorCurrent = new DoubleYoVariable(name + "QuadtratureCompositeStatorCurrent", registry);
      this.controlTarget = new DoubleYoVariable(name + "ControlTarget", registry);
      this.inphaseControlEffort = new DoubleYoVariable(name + "InphaseControlEffort", registry);
      this.quadratureControlEffort = new DoubleYoVariable(name + "QuadratureControlEffort", registry);
      this.motorEncoder = new DoubleYoVariable(name + "MotorEncoder", registry);
      this.estimatedTorqueStrainGauge0 = new DoubleYoVariable(name + "EstimatedTorqueStrainGauge0", registry);
      this.estimatedTorqueStrainGauge1 = new DoubleYoVariable(name + "EstimatedTorqueStrainGauge1", registry);

      this.motorVelocityEstimate = new DoubleYoVariable(name + "MotorVelocityEstimate", registry);

      this.lastReceivedControlID = new LongYoVariable(name + "LastReceivedControlID", registry);

      this.halSensors = new IntegerYoVariable(name + "HalSensors", registry);
      this.controlMode = new IntegerYoVariable(name + "ControlMode", registry);

      createSlowSensors(name);

      parentRegistry.addChild(registry);
   }

   private void createSlowSensors(String name)
   {
      slowSensors[0] = new MotorTemperature(name, registry);
      slowSensors[1] = new mcbTemperature1(name, registry);
      slowSensors[2] = new BusVoltage(name, registry);
      slowSensors[3] = new mcbTemperature2(name, registry);
   }

   public void update(ByteBuffer buffer) throws IOException
   {
      // Values are of no use to us
      @SuppressWarnings("unused")
      int frameFormat = buffer.get() & 0xFF;
      @SuppressWarnings("unused")
      int stateDelay = buffer.getShort() & 0xFFFF;
      @SuppressWarnings("unused")
      int length = buffer.get() & 0xFF;

      buffer.mark();
      
      // Only one format is defined for now
      @SuppressWarnings("unused")
      int stateFormat = buffer.get() & 0xFF;
      
      int slowSensor = buffer.get() & 0xFF;

      slowSensors[slowSensor].update(buffer.getShort() & 0xFFFF);

      microControllerTime.set(buffer.getInt() & 0xFFFFFFFFl);

      rawEncoder.set(buffer.getShort() & 0xFFFF);
      rawPhaseACurrent.set(buffer.getShort());

      rawPhaseBCurrent.set(buffer.getShort());
      rawPhaseCCurrent.set(buffer.getShort());

      inphaseCompositeStatorCurrent.set(buffer.getFloat());
      
      quadratureCompositeStatorCurrent.set(buffer.getFloat());
      
      controlTarget.set(buffer.getFloat());
      
      inphaseControlEffort.set(buffer.getFloat());
      
      quadratureControlEffort.set(buffer.getFloat());
      
      motorEncoder.set(buffer.getFloat());
      
      estimatedTorqueStrainGauge0.set(buffer.getFloat());
      
      estimatedTorqueStrainGauge1.set(buffer.getFloat());
      
      motorVelocityEstimate.set(buffer.getFloat());
      
      lastReceivedControlID.set(buffer.getInt() & 0xFFFFFFFFl);
      
      halSensors.set(buffer.get() & 0xFF);
      controlMode.set(buffer.get() & 0xFF);

      int checksumOffset = buffer.position();
      buffer.reset();
      checksumOffset -= buffer.position();
      
      int calculatedChecksum = 0;
      for(int i = 0; i < checksumOffset; i++)
      {
         calculatedChecksum = (calculatedChecksum + (buffer.get() & 0xFF)) & 0xFF;
      }
      
      int checksum = buffer.get() & 0xFF;
      @SuppressWarnings("unused")
      int unused = buffer.get();
      
      if(calculatedChecksum != checksum)
      {
         throw new IOException("Checksum failure. Frame size: " + checksumOffset + ". Expected: " + calculatedChecksum + ", received " + checksum);
      }
      
   }

   public double getMotorPosition()
   {
      return motorEncoder.getDoubleValue();
   }

   public double getMotorVelocity()
   {
      return motorVelocityEstimate.getDoubleValue();
   }

   public double getMotorTorque()
   {
      return quadratureCompositeStatorCurrent.getDoubleValue() / motorKt;
   }
}
