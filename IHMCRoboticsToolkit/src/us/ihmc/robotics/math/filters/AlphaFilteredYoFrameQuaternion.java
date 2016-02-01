package us.ihmc.robotics.math.filters;

import javax.vecmath.Quat4d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;


public class AlphaFilteredYoFrameQuaternion extends YoFrameQuaternion implements ProcessingYoVariable
{
   private final YoFrameQuaternion unfilteredQuaternion;
   private final DoubleYoVariable alpha;
   private final BooleanYoVariable hasBeenCalled;
   private final ThreadLocal<Quat4d> measuredQuaternion = createThreadLocalQuaternion();
   private final ThreadLocal<Quat4d> previousFilteredQuaternion = createThreadLocalQuaternion();
   private final ThreadLocal<Quat4d> newFilteredQuaternion = createThreadLocalQuaternion();

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion,
           double alpha, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredQuaternion, new DoubleYoVariable(namePrefix + "Alpha", registry), registry);
      this.setAlpha(alpha);
   }

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion,
           DoubleYoVariable alpha, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, unfilteredQuaternion.getReferenceFrame(), registry);
      this.unfilteredQuaternion = unfilteredQuaternion;
      this.alpha = alpha;
      this.hasBeenCalled = new BooleanYoVariable(namePrefix + nameSuffix + "HasBeenCalled", registry);
   }

   public void update()
   {
      if (hasBeenCalled.getBooleanValue())
      {
         Quat4d qMeasured = measuredQuaternion.get();
         unfilteredQuaternion.get(qMeasured);

         Quat4d qPreviousFiltered = previousFilteredQuaternion.get();
         get(qPreviousFiltered);

         Quat4d qNewFiltered = newFilteredQuaternion.get();
         qNewFiltered.interpolate(qMeasured, qPreviousFiltered, alpha.getDoubleValue());    // qPreviousFiltered 'gets multiplied by alpha'
         set(qNewFiltered);
      }
      else
      {
         Quat4d qMeasured = measuredQuaternion.get();
         unfilteredQuaternion.get(qMeasured);
         set(qMeasured);
         hasBeenCalled.set(true);
      }
   }

   public void setAlpha(double alpha)
   {
      this.alpha.set(alpha);
   }

   public void setBreakFrequency(double breakFrequencyHertz, double dt)
   {
      this.alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyHertz, dt));
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public YoFrameQuaternion getUnfilteredQuaternion()
   {
      return unfilteredQuaternion;
   }

   private static ThreadLocal<Quat4d> createThreadLocalQuaternion()
   {
      return new ThreadLocal<Quat4d>()
      {
         protected Quat4d initialValue()
         {
            return new Quat4d();
         }
         ;
      };
   }
}
