package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

public class AlphaFilteredYoMutableFrameVector3D extends YoMutableFrameVector3D implements ProcessingYoVariable
{
   private final DoubleProvider alphaProvider;

   private final FrameTuple3DReadOnly position;
   private final YoBoolean hasBeenCalled;

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, AlphaFilteredYoVariable.createAlphaYoDouble(namePrefix + nameSuffix, alpha, registry), referenceFrame);
   }

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha,
                                              ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
                                              FrameTuple3DReadOnly unfilteredFrameTuple3D)
   {
      this(namePrefix, nameSuffix, registry, AlphaFilteredYoVariable.createAlphaYoDouble(namePrefix + nameSuffix, alpha, registry),
           unfilteredFrameTuple3D.getReferenceFrame(), unfilteredFrameTuple3D);
   }

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha,
                                              FrameTuple3DReadOnly unfilteredFrameTuple3D)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredFrameTuple3D.getReferenceFrame(), unfilteredFrameTuple3D);
   }

   private AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha,
                                               ReferenceFrame referenceFrame, FrameTuple3DReadOnly unfilteredFrameTuple3D)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);

      alphaProvider = alpha;

      position = unfilteredFrameTuple3D;
      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   @Override
   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position);
   }

   public void update(FrameTuple3DReadOnly unfilteredFrameTuple3D)
   {
      checkReferenceFrameMatch(unfilteredFrameTuple3D);
      update((Tuple3DReadOnly) unfilteredFrameTuple3D);
   }

   public void update(Tuple3DReadOnly unfilteredTuple3D)
   {
      update(unfilteredTuple3D.getX(), unfilteredTuple3D.getY(), unfilteredTuple3D.getZ());
   }

   private final Vector3D unfilteredVector3D = new Vector3D();

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      if (!hasBeenCalled.getValue())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered, zUnfiltered);
      }
      else
      {
         unfilteredVector3D.set(xUnfiltered, yUnfiltered, zUnfiltered);
         interpolate(unfilteredVector3D, this, alphaProvider.getValue());
      }
   }
}
