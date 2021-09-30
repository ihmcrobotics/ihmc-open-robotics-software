package us.ihmc.robotics.math.functionGenerator;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;

public class WhiteNoiseFunctionGenerator extends BaseFunctionGenerator
{
   private final Random random;

   public WhiteNoiseFunctionGenerator()
   {
      this(1776L);
   }

   public WhiteNoiseFunctionGenerator(long seed)
   {
      random = new Random(seed);
   }

   @Override
   protected double computeValue()
   {
      return getOffset() + RandomNumbers.nextDouble(random, getAmplitude());
   }

   @Override
   protected double computeValueDot()
   {
      return 0;
   }

   @Override
   protected double computeValueDDot()
   {
      return 0;
   }
}
