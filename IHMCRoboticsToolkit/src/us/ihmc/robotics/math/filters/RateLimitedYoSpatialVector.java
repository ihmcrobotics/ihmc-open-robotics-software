package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoSpatialVector;

public class RateLimitedYoSpatialVector extends YoSpatialVector
{
   private final RateLimitedYoFrameVector rateLimitedLinearPart;
   private final RateLimitedYoFrameVector rateLimitedAngularPart;

   public RateLimitedYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maximumLinearRate,
                                     YoDouble maximumAngularRate, double dt, YoFrameVector rawLinearPart, YoFrameVector rawAngularPart)
   {
      super(new RateLimitedYoFrameVector(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawLinearPart),
            new RateLimitedYoFrameVector(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawAngularPart));
      this.rateLimitedLinearPart = (RateLimitedYoFrameVector) linearPart;
      this.rateLimitedAngularPart = (RateLimitedYoFrameVector) angularPart;
   }

   public RateLimitedYoSpatialVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoDouble maximumLinearRate,
                                     YoDouble maximumAngularRate, double dt, YoSpatialVector rawSpatialVector)
   {
      super(new RateLimitedYoFrameVector(namePrefix, nameSuffix, registry, maximumLinearRate, dt, rawSpatialVector.getYoLinearPart()),
            new RateLimitedYoFrameVector(namePrefix, nameSuffix, registry, maximumAngularRate, dt, rawSpatialVector.getYoAngularPart()));
      this.rateLimitedLinearPart = (RateLimitedYoFrameVector) linearPart;
      this.rateLimitedAngularPart = (RateLimitedYoFrameVector) angularPart;
   }

   public RateLimitedYoSpatialVector(RateLimitedYoFrameVector yoLinearPart, RateLimitedYoFrameVector yoAngularPart)
   {
      super(yoLinearPart, yoAngularPart);
      this.rateLimitedLinearPart = yoLinearPart;
      this.rateLimitedAngularPart = yoAngularPart;
   }

   public void setMaxRate(double maximumLinearRate, double maximumAngularRate)
   {
      rateLimitedLinearPart.setMaxRate(maximumLinearRate);
      rateLimitedAngularPart.setMaxRate(maximumAngularRate);
   }

   public void reset()
   {
      rateLimitedLinearPart.reset();
      rateLimitedAngularPart.reset();
   }

   public void update()
   {
      rateLimitedLinearPart.update();
      rateLimitedAngularPart.update();
   }

   public void update(YoFrameVector rawLinearPart, YoFrameVector rawAngularPart)
   {
      rateLimitedLinearPart.update(rawLinearPart);
      rateLimitedAngularPart.update(rawAngularPart);
   }

   public void update(FrameVector3D rawLinearPart, FrameVector3D rawAngularPart)
   {
      rateLimitedLinearPart.update(rawLinearPart);
      rateLimitedAngularPart.update(rawAngularPart);
   }

   public void update(Vector3DReadOnly rawLinearPart, Vector3DReadOnly rawAngularPart)
   {
      rateLimitedLinearPart.update(rawLinearPart);
      rateLimitedAngularPart.update(rawAngularPart);
   }

   public void update(double rawLinearX, double rawLinearY, double rawLinearZ, double rawAngularX, double rawAngularY, double rawAngularZ)
   {
      rateLimitedLinearPart.update(rawLinearX, rawLinearY, rawLinearZ);
      rateLimitedAngularPart.update(rawAngularX, rawAngularY, rawAngularZ);
   }
}
