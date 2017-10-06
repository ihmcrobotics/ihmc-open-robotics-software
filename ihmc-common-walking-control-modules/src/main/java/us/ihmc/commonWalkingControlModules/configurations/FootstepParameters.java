package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface FootstepParameters
{
   public abstract double getFootForwardOffset();

   public abstract double getFootBackwardOffset();

   public abstract double getFootWidth();

   public abstract double getToeWidth();

   public abstract double getFootLength();

   public abstract double getActualFootWidth();

   public abstract double getActualFootLength();

   public default double getFootstepArea()
   {
      return (getToeWidth() + getFootWidth()) * getFootLength() / 2.0;
   }
}
