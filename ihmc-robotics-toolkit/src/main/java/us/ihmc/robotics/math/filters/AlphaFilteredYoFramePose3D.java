package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredYoFramePose3D extends YoFramePose3D implements ProcessingYoVariable
{
   private final DoubleProvider alpha;
   private final FramePose3DReadOnly unfilteredPose;
   private final YoBoolean hasBeenCalled;

   private final Pose3D poseMeasured = new Pose3D();
   private final Pose3D posePreviousFiltered = new Pose3D();

   private static DoubleProvider createAlphaYoDouble(String namePrefix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "AlphaVariable", registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public AlphaFilteredYoFramePose3D(String namePrefix, String nameSuffix, FramePose3DReadOnly unfilteredPose, double alpha, YoRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredPose, createAlphaYoDouble(namePrefix, alpha, registry), registry);
   }

   public AlphaFilteredYoFramePose3D(String namePrefix, String nameSuffix, DoubleProvider alpha, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, nameSuffix, null, alpha, referenceFrame, registry);
   }

   public AlphaFilteredYoFramePose3D(String namePrefix, String nameSuffix, FramePose3DReadOnly unfilteredPose, DoubleProvider alpha, YoRegistry registry)
   {
      this(namePrefix, nameSuffix, unfilteredPose, alpha, unfilteredPose.getReferenceFrame(), registry);
   }

   private AlphaFilteredYoFramePose3D(String namePrefix,
                                      String nameSuffix,
                                      FramePose3DReadOnly unfilteredPose,
                                      DoubleProvider alpha,
                                      ReferenceFrame referenceFrame,
                                      YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
      this.unfilteredPose = unfilteredPose;

      if (alpha == null)
         alpha = createAlphaYoDouble(namePrefix, 0.0, registry);
      this.alpha = alpha;

      this.hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
   }

   @Override
   public void update()
   {
      if (unfilteredPose == null)
      {
         throw new NullPointerException("AlphaFilteredYoFramePose3D must be constructed with a non null "
                                        + "pose variable to call update(), otherwise use update(Pose3DReadOnly)");
      }

      poseMeasured.set(unfilteredPose);
      update(poseMeasured);
   }

   public void update(FramePose3DReadOnly rawPose)
   {
      checkReferenceFrameMatch(rawPose);
      poseMeasured.set(rawPose);
      update(poseMeasured);
   }

   public void update(Pose3DReadOnly rawPose)
   {
      if (hasBeenCalled.getBooleanValue())
      {
         posePreviousFiltered.set(this);

         interpolate(poseMeasured, posePreviousFiltered, alpha.getValue()); // qPreviousFiltered 'gets multiplied by alpha'
      }
      else
      {
         set(poseMeasured);
         hasBeenCalled.set(true);
      }
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public FramePose3DReadOnly getUnfilteredPose()
   {
      return unfilteredPose;
   }
}
