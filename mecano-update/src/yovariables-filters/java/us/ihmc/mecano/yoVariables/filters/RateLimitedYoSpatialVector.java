package us.ihmc.mecano.yoVariables.filters;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RateLimitedYoSpatialVector extends YoFixedFrameSpatialVector
{
   private final RateLimitedYoFrameVector3D rateLimitedAngularPart;
   private final RateLimitedYoFrameVector3D rateLimitedLinearPart;

   public RateLimitedYoSpatialVector(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maximumAngularRate,
                                     DoubleProvider maximumLinearRate, double dt, FrameVector3DReadOnly rawAngularPart, FrameVector3DReadOnly rawLinearPart)
   {
      super(new RateLimitedYoFrameVector3D(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawAngularPart),
            new RateLimitedYoFrameVector3D(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawLinearPart));
      this.rateLimitedAngularPart = (RateLimitedYoFrameVector3D) getAngularPart();
      this.rateLimitedLinearPart = (RateLimitedYoFrameVector3D) getLinearPart();
   }

   public RateLimitedYoSpatialVector(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider maximumAngularRate,
                                     DoubleProvider maximumLinearRate, double dt, YoFixedFrameSpatialVector rawSpatialVector)
   {
      super(new RateLimitedYoFrameVector3D(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawSpatialVector.getAngularPart()),
            new RateLimitedYoFrameVector3D(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawSpatialVector.getLinearPart()));
      this.rateLimitedAngularPart = (RateLimitedYoFrameVector3D) getAngularPart();
      this.rateLimitedLinearPart = (RateLimitedYoFrameVector3D) getLinearPart();
   }

   public RateLimitedYoSpatialVector(RateLimitedYoFrameVector3D yoAngularPart, RateLimitedYoFrameVector3D yoLinearPart)
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
