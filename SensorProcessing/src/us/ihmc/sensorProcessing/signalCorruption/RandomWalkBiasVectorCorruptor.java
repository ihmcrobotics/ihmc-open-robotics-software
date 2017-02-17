package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class RandomWalkBiasVectorCorruptor implements SignalCorruptor<Tuple3DBasics>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final Vector3D biasVector = new Vector3D();
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

   public void corrupt(Tuple3DBasics signal)
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
   
   public void setBias(Vector3D bias)
   {
      this.biasYoFrameVector.set(bias);
   }
}
