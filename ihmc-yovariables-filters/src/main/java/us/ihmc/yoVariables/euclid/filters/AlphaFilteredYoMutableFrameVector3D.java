package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.filters.VariableTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AlphaFilteredYoMutableFrameVector3D extends YoMutableFrameVector3D implements ProcessingYoVariable
{
   private final DoubleProvider alpha;

   private final FrameTuple3DReadOnly unfilteredFrameVector;
   private final YoBoolean hasBeenCalled;

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                              ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                              FrameTuple3DReadOnly unfilteredFrameVector)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredFrameVector.getReferenceFrame(), unfilteredFrameVector);
   }

   private AlphaFilteredYoMutableFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                               ReferenceFrame referenceFrame, FrameTuple3DReadOnly unfilteredFrameVector)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);

      this.alpha = alpha;
      this.unfilteredFrameVector = unfilteredFrameVector;

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
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
      if (unfilteredFrameVector == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(unfilteredFrameVector);
   }

   public void update(FrameTuple3DReadOnly unfilteredFrameTuple)
   {
      checkReferenceFrameMatch(unfilteredFrameTuple);
      update((Tuple3DReadOnly) unfilteredFrameTuple);
   }

   public void update(Tuple3DReadOnly unfilteredTuple)
   {
      update(unfilteredTuple.getX(), unfilteredTuple.getY(), unfilteredTuple.getZ());
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
         interpolate(unfilteredVector3D, this, alpha.getValue());
      }
   }
}
