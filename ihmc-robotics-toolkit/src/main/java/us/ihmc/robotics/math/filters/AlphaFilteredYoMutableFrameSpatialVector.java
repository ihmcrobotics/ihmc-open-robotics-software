package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.dataStructures.YoMutableFrameSpatialVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AlphaFilteredYoMutableFrameSpatialVector extends YoMutableFrameSpatialVector
{
   private final AlphaFilteredYoMutableFrameVector3D alphaFilteredAngularPart;
   private final AlphaFilteredYoMutableFrameVector3D alphaFilteredLinearPart;

   public AlphaFilteredYoMutableFrameSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      super(new AlphaFilteredYoMutableFrameVector3D(namePrefix, nameSuffix, registry, alphaAngularPart, rawAngularPart),
            new AlphaFilteredYoMutableFrameVector3D(namePrefix, nameSuffix, registry, alphaLinearPart, rawLinearPart));
      this.alphaFilteredAngularPart = (AlphaFilteredYoMutableFrameVector3D) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoMutableFrameVector3D) getLinearPart();
   }

   public AlphaFilteredYoMutableFrameSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaAngularPart,
                                       DoubleProvider alphaLinearPart, YoFixedFrameSpatialVector rawSpatialVector)
   {
      super(new AlphaFilteredYoMutableFrameVector3D(namePrefix, nameSuffix, registry, alphaAngularPart, rawSpatialVector.getAngularPart()),
            new AlphaFilteredYoMutableFrameVector3D(namePrefix, nameSuffix, registry, alphaLinearPart, rawSpatialVector.getLinearPart()));
      this.alphaFilteredAngularPart = (AlphaFilteredYoMutableFrameVector3D) getAngularPart();
      this.alphaFilteredLinearPart = (AlphaFilteredYoMutableFrameVector3D) getLinearPart();
   }

   public AlphaFilteredYoMutableFrameSpatialVector(AlphaFilteredYoMutableFrameVector3D yoAngularPart, AlphaFilteredYoMutableFrameVector3D yoLinearPart)
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
