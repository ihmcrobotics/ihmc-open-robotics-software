package us.ihmc.mecano.yoVariables.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AlphaFilteredYoSpatialVector extends YoFixedFrameSpatialVector
{
   private final AlphaFilteredYoFrameVector3D alphaFilteredAngularPart;
   private final AlphaFilteredYoFrameVector3D alphaFilteredLinearPart;

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      super(new AlphaFilteredYoFrameVector3D(namePrefix + "AngularPart", nameSuffix, registry, alphaAngularPart, rawAngularPart),
            new AlphaFilteredYoFrameVector3D(namePrefix + "LinearPart", nameSuffix, registry, alphaLinearPart, rawLinearPart));
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector3D) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector3D) getLinearPart();
   }

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, YoFixedFrameSpatialVector rawSpatialVector)
   {
      super(new AlphaFilteredYoFrameVector3D(namePrefix + "AngularPart", nameSuffix, registry, alphaAngularPart, rawSpatialVector.getAngularPart()),
            new AlphaFilteredYoFrameVector3D(namePrefix + "LinearPart", nameSuffix, registry, alphaLinearPart, rawSpatialVector.getLinearPart()));
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector3D) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector3D) getLinearPart();
   }

   public AlphaFilteredYoSpatialVector(AlphaFilteredYoFrameVector3D yoAngularPart, AlphaFilteredYoFrameVector3D yoLinearPart)
   {
      super(yoAngularPart, yoLinearPart);
      this.alphaFilteredAngularPart = yoAngularPart;
      this.alphaFilteredLinearPart = yoLinearPart;
   }

   public void reset()
   {
      alphaFilteredAngularPart.reset();
      alphaFilteredLinearPart.reset();
   }

   public void update()
   {
      alphaFilteredAngularPart.update();
      alphaFilteredLinearPart.update();
   }

   public void update(FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      alphaFilteredAngularPart.update(rawAngularPart);
      alphaFilteredLinearPart.update(rawLinearPart);
   }

   public void update(Vector3DReadOnly rawAngularPart, Vector3DReadOnly rawLinearPart)
   {
      alphaFilteredAngularPart.update(rawAngularPart);
      alphaFilteredLinearPart.update(rawLinearPart);
   }

   public void update(double rawAngularX, double rawAngularY, double rawAngularZ, double rawLinearX, double rawLinearY, double rawLinearZ)
   {
      alphaFilteredAngularPart.update(rawAngularX, rawAngularY, rawAngularZ);
      alphaFilteredLinearPart.update(rawLinearX, rawLinearY, rawLinearZ);
   }
}
