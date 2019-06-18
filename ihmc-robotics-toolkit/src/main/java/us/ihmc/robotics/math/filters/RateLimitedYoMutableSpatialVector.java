package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.dataStructures.YoMutableFrameSpatialVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RateLimitedYoMutableSpatialVector extends YoMutableFrameSpatialVector
{
   private final RateLimitedYoMutableFrameVector3D rateLimitedAngularPart;
   private final RateLimitedYoMutableFrameVector3D rateLimitedLinearPart;

   public RateLimitedYoMutableSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maximumAngularRate,
                                            DoubleProvider maximumLinearRate, double dt, FrameVector3DReadOnly rawAngularPart,
                                            FrameVector3DReadOnly rawLinearPart)
   {
      super(new RateLimitedYoMutableFrameVector3D(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawAngularPart),
            new RateLimitedYoMutableFrameVector3D(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawLinearPart));
      this.rateLimitedAngularPart = (RateLimitedYoMutableFrameVector3D) getAngularPart();
      this.rateLimitedLinearPart = (RateLimitedYoMutableFrameVector3D) getLinearPart();
   }

   public RateLimitedYoMutableSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider maximumAngularRate,
                                            DoubleProvider maximumLinearRate, double dt, YoFixedFrameSpatialVector rawSpatialVector)
   {
      super(new RateLimitedYoMutableFrameVector3D(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawSpatialVector.getAngularPart()),
            new RateLimitedYoMutableFrameVector3D(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawSpatialVector.getLinearPart()));
      this.rateLimitedAngularPart = (RateLimitedYoMutableFrameVector3D) getAngularPart();
      this.rateLimitedLinearPart = (RateLimitedYoMutableFrameVector3D) getLinearPart();
   }

   public RateLimitedYoMutableSpatialVector(RateLimitedYoMutableFrameVector3D yoAngularPart, RateLimitedYoMutableFrameVector3D yoLinearPart)
   {
      super(yoAngularPart, yoLinearPart);
      this.rateLimitedAngularPart = yoAngularPart;
      this.rateLimitedLinearPart = yoLinearPart;
   }

   public void reset()
   {
      rateLimitedAngularPart.reset();
      rateLimitedLinearPart.reset();
   }

   public void update()
   {
      rateLimitedAngularPart.update();
      rateLimitedLinearPart.update();
   }

   public void update(FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      rateLimitedAngularPart.update(rawAngularPart);
      rateLimitedLinearPart.update(rawLinearPart);
   }

   public void update(Vector3DReadOnly rawAngularPart, Vector3DReadOnly rawLinearPart)
   {
      rateLimitedAngularPart.update(rawAngularPart);
      rateLimitedLinearPart.update(rawLinearPart);
   }

   public void update(double rawAngularX, double rawAngularY, double rawAngularZ, double rawLinearX, double rawLinearY, double rawLinearZ)
   {
      rateLimitedAngularPart.update(rawAngularX, rawAngularY, rawAngularZ);
      rateLimitedLinearPart.update(rawLinearX, rawLinearY, rawLinearZ);
   }
}
