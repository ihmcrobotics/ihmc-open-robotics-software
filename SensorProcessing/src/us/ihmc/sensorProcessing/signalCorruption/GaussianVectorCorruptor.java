package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class GaussianVectorCorruptor implements SignalCorruptor<FrameVector>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final FrameVector noise = new FrameVector(ReferenceFrame.getWorldFrame());
   private final DoubleYoVariable standardDeviation;

   public GaussianVectorCorruptor(long seed, String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", parentRegistry);
      
      parentRegistry.addChild(registry);
   }

   public void corrupt(FrameVector signal)
   {
      double std = standardDeviation.getDoubleValue();
      double noiseX = std * random.nextGaussian();
      double noiseY = std * random.nextGaussian();
      double noiseZ = std * random.nextGaussian();
      noise.set(signal.getReferenceFrame(), noiseX, noiseY, noiseZ);
      signal.add(noise);
   }

}
