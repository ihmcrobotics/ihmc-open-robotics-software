package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoSpatialVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AlphaFilteredYoSpatialVector extends YoSpatialVector
{
   private final AlphaFilteredYoFrameVector alphaFilteredLinearPart;
   private final AlphaFilteredYoFrameVector alphaFilteredAngularPart;

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaLinearPart,
                                       DoubleProvider alphaAngularPart, FrameVector3DReadOnly rawLinearPart, FrameVector3DReadOnly rawAngularPart)
   {
      super(new AlphaFilteredYoFrameVector(namePrefix, nameSuffix, registry, alphaLinearPart, rawLinearPart),
            new AlphaFilteredYoFrameVector(namePrefix, nameSuffix, registry, alphaAngularPart, rawAngularPart));
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector) linearPart;
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector) angularPart;
   }

   public AlphaFilteredYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alphaLinearPart,
                                       DoubleProvider alphaAngularPart, YoSpatialVector rawSpatialVector)
   {
      super(new AlphaFilteredYoFrameVector(namePrefix, nameSuffix, registry, alphaLinearPart, rawSpatialVector.getYoLinearPart()),
            new AlphaFilteredYoFrameVector(namePrefix, nameSuffix, registry, alphaAngularPart, rawSpatialVector.getYoAngularPart()));
      this.alphaFilteredLinearPart = (AlphaFilteredYoFrameVector) linearPart;
      this.alphaFilteredAngularPart = (AlphaFilteredYoFrameVector) angularPart;
   }

   public AlphaFilteredYoSpatialVector(AlphaFilteredYoFrameVector yoLinearPart, AlphaFilteredYoFrameVector yoAngularPart)
   {
      super(yoLinearPart, yoAngularPart);
      this.alphaFilteredLinearPart = yoLinearPart;
      this.alphaFilteredAngularPart = yoAngularPart;
   }

   public void reset()
   {
      alphaFilteredLinearPart.reset();
      alphaFilteredAngularPart.reset();
   }

   public void update()
   {
      alphaFilteredLinearPart.update();
      alphaFilteredAngularPart.update();
   }

   public void update(FrameVector3DReadOnly rawLinearPart, FrameVector3DReadOnly rawAngularPart)
   {
      alphaFilteredLinearPart.update(rawLinearPart);
      alphaFilteredAngularPart.update(rawAngularPart);
   }

   public void update(Vector3DReadOnly rawLinearPart, Vector3DReadOnly rawAngularPart)
   {
      alphaFilteredLinearPart.update(rawLinearPart);
      alphaFilteredAngularPart.update(rawAngularPart);
   }

   public void update(double rawLinearX, double rawLinearY, double rawLinearZ, double rawAngularX, double rawAngularY, double rawAngularZ)
   {
      alphaFilteredLinearPart.update(rawLinearX, rawLinearY, rawLinearZ);
      alphaFilteredAngularPart.update(rawAngularX, rawAngularY, rawAngularZ);
   }
}
