package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.steppr.hardware.state.slowSensors.MotorTemperature;
import us.ihmc.steppr.hardware.state.slowSensors.StepprSlowSensor;
import us.ihmc.steppr.hardware.state.slowSensors.mcbTemperature1;
import us.ihmc.steppr.hardware.state.slowSensors.mcbTemperature2;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class StepprActuatorState
{
   private final StepprSlowSensor[] slowSensors = new StepprSlowSensor[27];

   private final YoVariableRegistry registry;

   private final double motorKt;
   
   private final LongYoVariable microControllerTime;

   
   private final DoubleYoVariable inphaseCompositeStatorCurrent;
   private final DoubleYoVariable quadratureCompositeStatorCurrent;

   private final DoubleYoVariable controlTarget;

   private final DoubleYoVariable motorEncoderPosition;
   private final DoubleYoVariable motorVelocityEstimate;
   
   private final DoubleYoVariable jointEncoderPosition;
   private final DoubleYoVariable jointEncoderVelocity;

   private final LongYoVariable lastReceivedControlID;

   private final int[] slowSensorSlotIDs = new int[5];

   public StepprActuatorState(String name, double motorKt, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.motorKt = motorKt;
      this.microControllerTime = new LongYoVariable(name + "MicroControllerTime", registry);


      this.inphaseCompositeStatorCurrent = new DoubleYoVariable(name + "InphaseCompositeStatorCurrent", registry);
      this.quadratureCompositeStatorCurrent = new DoubleYoVariable(name + "QuadtratureCompositeStatorCurrent", registry);
      this.controlTarget = new DoubleYoVariable(name + "ControlTarget", registry);

      this.motorEncoderPosition = new DoubleYoVariable(name + "MotorEncoderPosition", registry);
      this.motorVelocityEstimate = new DoubleYoVariable(name + "MotorVelocityEstimate", registry);
      
      this.jointEncoderPosition = new DoubleYoVariable(name + "JointEncoderPosition", registry);
      this.jointEncoderVelocity = new DoubleYoVariable(name + "JointEncoderVelocity", registry);
      

      this.lastReceivedControlID = new LongYoVariable(name + "LastReceivedControlID", registry);


      createSlowSensors(name);

      parentRegistry.addChild(registry);
   }

   private void createSlowSensors(String name)
   {
      slowSensors[6] = new MotorTemperature(name, registry);
      slowSensors[7] = new mcbTemperature1(name, registry);
      slowSensors[8] = new mcbTemperature2(name, registry);
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
      
      for(int i = 0; i < slowSensorSlotIDs.length; i++)
      {
         slowSensorSlotIDs[i] = buffer.get() & 0xFF;
      }
      

      microControllerTime.set(buffer.getInt() & 0xFFFFFFFFl);
      inphaseCompositeStatorCurrent.set(buffer.getFloat());
      quadratureCompositeStatorCurrent.set(buffer.getFloat());
      controlTarget.set(buffer.getFloat());
      motorEncoderPosition.set(buffer.getFloat());
      motorVelocityEstimate.set(buffer.getFloat());

      jointEncoderPosition.set(buffer.getFloat());
      jointEncoderVelocity.set(buffer.getFloat());
      
      lastReceivedControlID.set(buffer.getInt() & 0xFFFFFFFFl);
      
      
      
      for(int i = 0; i < slowSensorSlotIDs.length; i++)
      {
         int id = slowSensorSlotIDs[i];
         if(slowSensors[id] != null)
         {
            slowSensors[id].update(buffer.getShort() & 0xFFFF);            
         }
      }

      int checksumOffset = buffer.position();
      buffer.reset();
      checksumOffset -= buffer.position();
      
      int calculatedChecksum = 0;
      for(int i = 0; i < checksumOffset; i++)
      {
         calculatedChecksum = (calculatedChecksum + (buffer.get() & 0xFF)) & 0xFF;
      }
      
      @SuppressWarnings("unused")
      int unused = buffer.get();

      int checksum = buffer.get() & 0xFF;
      
      if(calculatedChecksum != checksum)
      {
         throw new IOException("Checksum failure. Frame size: " + checksumOffset + ". Expected: " + calculatedChecksum + ", received " + checksum);
      }
      
   }

   public double getMotorPosition()
   {
      return motorEncoderPosition.getDoubleValue();
   }

   public double getMotorVelocity()
   {
      return motorVelocityEstimate.getDoubleValue();
   }

   public double getJointPosition()
   {
      return jointEncoderPosition.getDoubleValue();
   }
   
   public double getJointVelocity()
   {
      return jointEncoderVelocity.getDoubleValue();
   }
   
   public double getMotorTorque()
   {
      return quadratureCompositeStatorCurrent.getDoubleValue() / motorKt;
   }
}
