package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.List;

public class EnvironmentConstraintProvider
{
   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private final DoubleProvider maxAngleForSteppable;
   private final DoubleProvider minimumAreaForSteppable;

   private final RecyclingArrayList<PlanarRegion> planarRegionsList = new RecyclingArrayList<>(PlanarRegion.class);
   private final YoInteger numberOfPlanarListsToConsider;

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;

   public EnvironmentConstraintProvider(ICPOptimizationParameters optimizationParameters,
                                        String yoNamePrefix,
                                        YoVariableRegistry registry)
   {
      maxAngleForSteppable = new DoubleParameter(yoNamePrefix + "MaxAngleForSteppable", registry, maxNormalAngleFromVertical);
      minimumAreaForSteppable = new DoubleParameter(yoNamePrefix + "MinimumAreaForSteppable", registry, minimumAreaToConsider);

      numberOfPlanarListsToConsider = new YoInteger(yoNamePrefix + "NumberOfPlanarListsToConsider", registry);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean(yoNamePrefix + "SwitchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(optimizationParameters.switchPlanarRegionConstraintsAutomatically());
   }

   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      planarRegionsList.clear();
      numberOfPlanarListsToConsider.set(0);

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);

         if (isRegionValidForStepping(planarRegion))
         {
            planarRegionsList.add().set(planarRegions.get(i));
            numberOfPlanarListsToConsider.increment();
         }
      }
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      planarRegion.getNormal(planeNormal);

      double angle = planeNormal.angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      // TODO switch to the concave hull
      return planarRegion.getConvexHull().getArea() > minimumAreaForSteppable.getValue();
   }
}
