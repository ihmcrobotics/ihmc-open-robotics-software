package us.ihmc.robotics.kinematics.fourbar;

public class InvertedFourBarVertex extends FourBarVertex
{
   public InvertedFourBarVertex(String name)
   {
      super(name);
   }

   @Override
   protected void updateLimits()
   {
      InvertedFourBarTools.updateLimits(this);
   }
}
