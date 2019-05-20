package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class AccelerationLimitedYoFrameVector3D extends YoFrameVector3D
{
   private final AccelerationLimitedYoVariable x, y, z;

   private AccelerationLimitedYoFrameVector3D(AccelerationLimitedYoVariable x, AccelerationLimitedYoVariable y, AccelerationLimitedYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static AccelerationLimitedYoFrameVector3D createAccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                             DoubleProvider maxRate, DoubleProvider maxAcceleration, double dt, ReferenceFrame referenceFrame)
   {
      AccelerationLimitedYoVariable x = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, dt);
      AccelerationLimitedYoVariable y = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, dt);
      AccelerationLimitedYoVariable z = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, dt);

      AccelerationLimitedYoFrameVector3D ret = new AccelerationLimitedYoFrameVector3D(x, y, z, referenceFrame);

      return ret;
   }


   public static AccelerationLimitedYoFrameVector3D createAccelerationLimitedYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                             DoubleProvider maxRate, DoubleProvider maxAcceleration, double dt, YoFrameVector3D unfilteredVector)
   {
      AccelerationLimitedYoVariable x = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, unfilteredVector.getYoX(), dt);
      AccelerationLimitedYoVariable y = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, unfilteredVector.getYoY(), dt);
      AccelerationLimitedYoVariable z = new AccelerationLimitedYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, maxRate, maxAcceleration, unfilteredVector.getYoZ(), dt);

      AccelerationLimitedYoFrameVector3D ret = new AccelerationLimitedYoFrameVector3D(x, y, z, unfilteredVector.getReferenceFrame());

      return ret;
   }

   public void setGainsByPolePlacement(double w0, double zeta)
   {
      x.setGainsByPolePlacement(w0, zeta);
      y.setGainsByPolePlacement(w0, zeta);
      z.setGainsByPolePlacement(w0, zeta);
   }

   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
      z.update(zUnfiltered);
   }

   public void update(Vector3DReadOnly vector3DUnfiltered)
   {
      x.update(vector3DUnfiltered.getX());
      y.update(vector3DUnfiltered.getY());
      z.update(vector3DUnfiltered.getZ());
   }

   public void update(FrameVector3D vector3DUnfiltered)
   {
      x.update(vector3DUnfiltered.getX());
      y.update(vector3DUnfiltered.getY());
      z.update(vector3DUnfiltered.getZ());
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
