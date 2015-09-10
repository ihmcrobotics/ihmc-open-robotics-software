package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class RandomWalkBiasVectorCorruptor implements SignalCorruptor<Tuple3d>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final Vector3d biasVector = new Vector3d();
   private final DoubleYoVariable standardDeviation;
   private final YoFrameVector biasYoFrameVector;
   private final double squareRootOfUpdateDT;
   
   public RandomWalkBiasVectorCorruptor(long seed, String namePrefix, double updateDT, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", registry);
      this.biasYoFrameVector = new YoFrameVector(namePrefix + "Bias", ReferenceFrame.getWorldFrame(), registry);

      this.squareRootOfUpdateDT = Math.sqrt(updateDT);
      
      parentRegistry.addChild(registry);
   }

   public void corrupt(Tuple3d signal)
   {
      double std = standardDeviation.getDoubleValue();
      double biasUpdateX = std * random.nextGaussian() * squareRootOfUpdateDT;
      double biasUpdateY = std * random.nextGaussian() * squareRootOfUpdateDT;
      double biasUpdateZ = std * random.nextGaussian() * squareRootOfUpdateDT;

      biasYoFrameVector.add(biasUpdateX, biasUpdateY, biasUpdateZ);
      biasYoFrameVector.get(biasVector);

      signal.add(biasVector);
   }

   public void setStandardDeviation(double standardDeviation)
   {
      this.standardDeviation.set(standardDeviation);
   }
   
   public void setBias(Vector3d bias)
   {
      this.biasYoFrameVector.set(bias);
   }
}
