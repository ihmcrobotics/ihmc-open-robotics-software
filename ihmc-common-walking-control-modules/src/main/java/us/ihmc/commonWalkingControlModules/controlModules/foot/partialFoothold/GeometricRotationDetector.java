package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class GeometricRotationDetector implements FootRotationDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector3D groundPlaneNormal;
   private final FrameVector3D footNormal = new FrameVector3D();

   private final YoDouble angleFootGround;
   private final YoDouble angleThreshold;
   private final YoBoolean footRotating;

   public GeometricRotationDetector(String namePrefix, ExplorationParameters explorationParameters, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      groundPlaneNormal = new YoFrameVector3D(namePrefix + "PlaneNormal", worldFrame, registry);

      angleFootGround = new YoDouble(namePrefix + "AngleToGround", registry);
      angleThreshold = explorationParameters.getGeometricDetectionAngleThreshold();
      footRotating = new YoBoolean(namePrefix + "RotatingGeometry", registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
   }

   public boolean compute()
   {
      groundPlaneNormal.set(worldFrame, 0.0, 0.0, 1.0);

      double cosAlpha = Math.abs(groundPlaneNormal.dot(footNormal));
      double alpha = Math.acos(cosAlpha);
      angleFootGround.set(alpha);
      footRotating.set(alpha > angleThreshold.getDoubleValue());

      return footRotating.getBooleanValue();
   }

   public boolean isRotating()
   {
      return footRotating.getBooleanValue();
   }
}
