package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AlphaFilteredYoSpatialVector extends YoFixedFrameSpatialVector
{
   private final AlphaFilteredYoFrameVector alphaFilteredAngularPart;
   private final AlphaFilteredYoFrameVector alphaFilteredLinearPart;

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      super(new AlphaFilteredYoFrameVector(namePrefix + "AngularPart", nameSuffix, registry, alphaAngularPart, rawAngularPart),
            new AlphaFilteredYoFrameVector(namePrefix + "LinearPart", nameSuffix, registry, alphaLinearPart, rawLinearPart));
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector) getLinearPart();
   }

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, YoFixedFrameSpatialVector rawSpatialVector)
   {
      super(new AlphaFilteredYoFrameVector(namePrefix + "AngularPart", nameSuffix, registry, alphaAngularPart, rawSpatialVector.getAngularPart()),
            new AlphaFilteredYoFrameVector(namePrefix + "LinearPart", nameSuffix, registry, alphaLinearPart, rawSpatialVector.getLinearPart()));
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector) getLinearPart();
   }

   public AlphaFilteredYoSpatialVector(AlphaFilteredYoFrameVector yoAngularPart, AlphaFilteredYoFrameVector yoLinearPart)
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
