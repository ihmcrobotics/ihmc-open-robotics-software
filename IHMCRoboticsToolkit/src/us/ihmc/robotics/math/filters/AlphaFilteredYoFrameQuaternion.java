package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class AlphaFilteredYoFrameQuaternion extends YoFrameQuaternion implements ProcessingYoVariable
{
   private final YoFrameQuaternion unfilteredQuaternion;
   private final DoubleYoVariable alpha;
   private final BooleanYoVariable hasBeenCalled;
   private final Quaternion qMeasured = new Quaternion();
   private final Quaternion qPreviousFiltered = new Quaternion();
   private final Quaternion qNewFiltered = new Quaternion();

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion, double alpha,
         YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredQuaternion, new DoubleYoVariable(namePrefix + "Alpha", registry), registry);
      this.setAlpha(alpha);
   }

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, DoubleYoVariable alpha, ReferenceFrame referenceFrame,
         YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, null, alpha, referenceFrame, registry);
   }

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion, DoubleYoVariable alpha,
         YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredQuaternion, alpha, unfilteredQuaternion.getReferenceFrame(), registry);
   }

   private AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion, DoubleYoVariable alpha,
         ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
      this.unfilteredQuaternion = unfilteredQuaternion;
      this.alpha = alpha;
      this.hasBeenCalled = new BooleanYoVariable(namePrefix + nameSuffix + "HasBeenCalled", registry);
   }

   @Override
   public void update()
   {
      if (unfilteredQuaternion == null)
      {
         throw new NullPointerException("AlphaFilteredYoFrameQuaternion must be constructed with a non null "
               + "quaternion variable to call update(), otherwise use update(Quat4d)");
      }

      unfilteredQuaternion.get(qMeasured);
      update(qMeasured);
   }

   public void update(Quaternion qMeasured)
   {
      if (hasBeenCalled.getBooleanValue())
      {
         get(qPreviousFiltered);

         qNewFiltered.interpolate(qMeasured, qPreviousFiltered, alpha.getDoubleValue()); // qPreviousFiltered 'gets multiplied by alpha'
         set(qNewFiltered);
      }
      else
      {
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
}
