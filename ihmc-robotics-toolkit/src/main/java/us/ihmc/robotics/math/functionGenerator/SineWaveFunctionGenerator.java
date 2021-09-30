package us.ihmc.robotics.math.functionGenerator;

public class SineWaveFunctionGenerator extends BaseFunctionGenerator
{
   public SineWaveFunctionGenerator()
   {
   }

   @Override
   protected double computeValue()
   {
      double offset = getOffset();
      double amplitude = getAmplitude();
      double angle = getAngle();
      double phase = getPhase();
      return offset + amplitude * Math.sin(angle + phase);
   }

   @Override
   protected double computeValueDot()
   {
      double angleDot = getAngleDot();
      double amplitude = getAmplitude();
      double angle = getAngle();
      double phase = getPhase();
      return angleDot * amplitude * Math.cos(angle + phase);
   }

   @Override
   protected double computeValueDDot()
   {
      double angleDot = getAngleDot();
      double amplitude = getAmplitude();
      double angle = getAngle();
      double phase = getPhase();
      return -angleDot * angleDot * amplitude * Math.sin(angle + phase);
   }
}
