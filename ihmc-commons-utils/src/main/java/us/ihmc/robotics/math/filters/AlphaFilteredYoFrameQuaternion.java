package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredYoFrameQuaternion extends YoFrameQuaternion implements ProcessingYoVariable
{
   private final YoFrameQuaternion unfilteredQuaternion;
   private final DoubleProvider alpha;
   private final YoBoolean hasBeenCalled;
   private final Quaternion qMeasured = new Quaternion();
   private final Quaternion qPreviousFiltered = new Quaternion();
   private final Quaternion qNewFiltered = new Quaternion();

   public AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion, DoubleProvider alpha,
         YoRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredQuaternion, alpha, unfilteredQuaternion.getReferenceFrame(), registry);
   }

   private AlphaFilteredYoFrameQuaternion(String namePrefix, String nameSuffix, YoFrameQuaternion unfilteredQuaternion, DoubleProvider alpha,
         ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
      this.unfilteredQuaternion = unfilteredQuaternion;

      if (alpha == null)
         alpha = VariableTools.createAlphaYoDouble(namePrefix, "", 0.0, registry);
      this.alpha = alpha;

      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
   }

   @Override
   public void update()
   {
      if (unfilteredQuaternion == null)
      {
         throw new NullPointerException("AlphaFilteredYoFrameQuaternion must be constructed with a non null "
               + "quaternion variable to call update(), otherwise use update(Quat4d)");
      }

      qMeasured.set(unfilteredQuaternion);
      update(qMeasured);
   }

   public void update(FrameOrientation3DReadOnly rawOrientation)
   {
      checkReferenceFrameMatch(rawOrientation);
      qMeasured.set(rawOrientation);
      update(qMeasured);
   }

   public void update(Orientation3DReadOnly rawOrientation)
   {
      qMeasured.set(rawOrientation);
      update(qMeasured);
   }

   private void update(QuaternionReadOnly qMeasured)
   {
      if (hasBeenCalled.getBooleanValue())
      {
         qPreviousFiltered.set(this);

         qNewFiltered.interpolate(qMeasured, qPreviousFiltered, alpha.getValue()); // qPreviousFiltered 'gets multiplied by alpha'
         set(qNewFiltered);
      }
      else
      {
         set(qMeasured);
         hasBeenCalled.set(true);
      }
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public YoFrameQuaternion getUnfilteredQuaternion()
   {
      return unfilteredQuaternion;
   }
}
