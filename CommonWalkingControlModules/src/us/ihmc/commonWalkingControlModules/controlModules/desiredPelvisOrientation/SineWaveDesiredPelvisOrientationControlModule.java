package us.ihmc.commonWalkingControlModules.controlModules.desiredPelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;


public class SineWaveDesiredPelvisOrientationControlModule implements DesiredPelvisOrientationControlModule
{
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("SineWaveOrientation");
   
   private final DoubleYoVariable yawAmp = new DoubleYoVariable("yawAmp", registry);
   private final DoubleYoVariable yawOffset = new DoubleYoVariable("yawOffset", registry);
   private final DoubleYoVariable yawFreq = new DoubleYoVariable("yawFreq", registry);
  
   private final DoubleYoVariable pitchAmp = new DoubleYoVariable("pitchAmp", registry);
   private final DoubleYoVariable pitchOffset = new DoubleYoVariable("pitchOffset", registry);
   private final DoubleYoVariable pitchFreq = new DoubleYoVariable("pitchFreq", registry);
   
   private final DoubleYoVariable rollAmp = new DoubleYoVariable("rollAmp", registry);
   private final DoubleYoVariable rollOffset = new DoubleYoVariable("rollOffset", registry);
   private final DoubleYoVariable rollFreq = new DoubleYoVariable("rollFreq", registry);
   private final DoubleYoVariable alphaDesiredOrientation;
      
   private final AlphaFilteredYoFrameVector orientationVector;
   
   public SineWaveDesiredPelvisOrientationControlModule(CommonHumanoidReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;
      
      alphaDesiredOrientation = new DoubleYoVariable("alphaDesiredOrientation", registry);
      orientationVector = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filtDesOrientation", "", registry, alphaDesiredOrientation, referenceFrames.getMidFeetZUpFrame());
      
      parentRegistry.addChild(registry);
   }
   
   public FrameOrientation getDesiredPelvisOrientationSingleSupportCopy(RobotSide robotSide)
   {
      return new FrameOrientation(referenceFrames.getPelvisFrame());
   }
   
   public void setAlphaForFilterFreqInHz(double filterFreqInHz, double dt)
   {
      alphaDesiredOrientation.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterFreqInHz, dt));
   }

   public FrameOrientation getDesiredPelvisOrientationDoubleSupportCopy()
   {
      double time = processedSensors.getTime();
      
      double yaw = yawOffset.getDoubleValue() + yawAmp.getDoubleValue() * Math.sin(2.0 * Math.PI * yawFreq.getDoubleValue() * time);
      double pitch = pitchOffset.getDoubleValue() + pitchAmp.getDoubleValue() * Math.sin(2.0 * Math.PI * pitchFreq.getDoubleValue() * time);
      double roll = rollOffset.getDoubleValue() + rollAmp.getDoubleValue() * Math.sin(2.0 * Math.PI * rollFreq.getDoubleValue() * time);
      
      orientationVector.update(roll, pitch, yaw);
     
      return new FrameOrientation(referenceFrames.getMidFeetZUpFrame(), orientationVector.getZ(), orientationVector.getY(), orientationVector.getX());
   }

   public void setParametersForM2V2()
   {
      yawFreq.set(0.5);
      pitchFreq.set(0.5);
      rollFreq.set(0.5);
   }

   public void setDesiredPelvisOrientation(FrameOrientation orientation)
   {
      // empty
   }

   public FrameOrientation getEstimatedOrientationAtEndOfStepCopy(RobotSide robotSide, FramePoint desiredFootLocation)
   {
      return new FrameOrientation(referenceFrames.getMidFeetZUpFrame());
   }

   public void useTwistScale(boolean useTwistScale)
   {
   }
}
