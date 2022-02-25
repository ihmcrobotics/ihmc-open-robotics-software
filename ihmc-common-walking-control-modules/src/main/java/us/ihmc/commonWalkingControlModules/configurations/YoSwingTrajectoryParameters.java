package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoSwingTrajectoryParameters
{
   private final BooleanParameter doHeelTouchdownIfPossible;
   private final DoubleParameter heelTouchdownAngle;
   private final DoubleParameter maximumHeightForHeelTouchdown;
   private final DoubleParameter heelTouchdownLengthRatio;

   private final BooleanParameter doToeTouchdownIfPossible;
   private final DoubleParameter toeTouchdownAngle;
   private final DoubleParameter stepDownHeightForToeTouchdown;
   private final DoubleParameter toeTouchdownDepthRatio;

   private final BooleanParameter addOrientationMidpointForClearance;
   private final DoubleParameter midpointOrientationInterpolationForClearance;

   private final DoubleParameter minHeightDifferenceForObstacleClearance;
   private final DoubleParameter finalSwingHeightOffset;

   private final List<DoubleProvider> defaultWaypointProportions = new ArrayList<>();
   private final List<DoubleProvider> defaultObstacleClearanceWaypointProportions = new ArrayList<>();

   private final ParameterVector3D touchdownVelocityWeight;
   private final FrameParameterVector3D touchdownVelocity;
   private final FrameParameterVector3D touchdownAcceleration;
   private final DoubleProvider finalCoMVelocityInjectionRatio;
   private final DoubleProvider finalCoMAccelerationInjectionRatio;

   private final BooleanProvider ignoreInitialAngularVelocityZ;
   private final DoubleProvider maxInitialLinearVelocityMagnitude;
   private final DoubleProvider maxInitialAngularVelocityMagnitude;

   private final DoubleProvider velocityAdjustmentDamping;

   public YoSwingTrajectoryParameters(String namePrefix, WalkingControllerParameters walkingControllerParameters, YoRegistry registry)
   {
      this(namePrefix, walkingControllerParameters, walkingControllerParameters.getSwingTrajectoryParameters(), registry);
   }

   public YoSwingTrajectoryParameters(String namePrefix, WalkingControllerParameters walkingControllerParameters, SwingTrajectoryParameters parameters,
                                      YoRegistry registry)
   {
      doHeelTouchdownIfPossible = new BooleanParameter(namePrefix + "DoHeelTouchdownIfPossible", registry, parameters.doHeelTouchdownIfPossible());
      heelTouchdownAngle = new DoubleParameter(namePrefix + "HeelTouchdownAngle", registry, parameters.getHeelTouchdownAngle());
      maximumHeightForHeelTouchdown = new DoubleParameter(namePrefix + "MaximumHeightForHeelTouchdown",
                                                          registry,
                                                          parameters.getMaximumHeightForHeelTouchdown());
      heelTouchdownLengthRatio = new DoubleParameter(namePrefix + "HeelTouchdownLengthRatio", registry, parameters.getHeelTouchdownLengthRatio());

      doToeTouchdownIfPossible = new BooleanParameter(namePrefix + "DoToeTouchdownIfPossible", registry, parameters.doToeTouchdownIfPossible());
      toeTouchdownAngle = new DoubleParameter(namePrefix + "ToeTouchdownAngle", registry, parameters.getToeTouchdownAngle());
      stepDownHeightForToeTouchdown = new DoubleParameter(namePrefix + "StepDownHeightForToeTouchdown",
                                                          registry,
                                                          parameters.getStepDownHeightForToeTouchdown());
      toeTouchdownDepthRatio = new DoubleParameter(namePrefix + "ToeTouchdownDepthRatio", registry, parameters.getToeTouchdownDepthRatio());

      addOrientationMidpointForClearance = new BooleanParameter(namePrefix + "AddOrientationMidpointForClearance",
                                                                registry,
                                                                parameters.addOrientationMidpointForObstacleClearance());
      midpointOrientationInterpolationForClearance = new DoubleParameter(namePrefix + "MidpointOrientationInterpolationForClearance",
                                                                         registry,
                                                                         parameters.midpointOrientationInterpolationForObstacleClearance());

      minHeightDifferenceForObstacleClearance = new DoubleParameter(namePrefix + "MinHeightDifferenceForObstacleClearance",
                                                                    registry,
                                                                    parameters.getMinHeightDifferenceForStepUpOrDown());

      int numberWaypoints = 2;
      double[] defaultWaypointProportions = parameters.getSwingWaypointProportions();
      double[] defaultObstacleClearanceWaypointProportions = parameters.getObstacleClearanceProportions();

      for (int i = 0; i < numberWaypoints; i++)
      {
         DoubleParameter waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion" + i, registry, defaultWaypointProportions[i]);
         DoubleParameter obstacleClearanceWaypointProportion = new DoubleParameter(namePrefix + "ObstacleClearanceWaypointProportion" + i,
                                                                                   registry,
                                                                                   defaultObstacleClearanceWaypointProportions[i]);
         this.defaultWaypointProportions.add(waypointProportion);
         this.defaultObstacleClearanceWaypointProportions.add(obstacleClearanceWaypointProportion);
      }

      this.touchdownVelocityWeight = new ParameterVector3D(namePrefix + "TouchdownVelocityWeight", parameters.getTouchdownVelocityWeight(), registry);

      finalSwingHeightOffset = new DoubleParameter(namePrefix + "FinalHeightOffset", registry, parameters.getDesiredTouchdownHeightOffset());

      Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, parameters.getDesiredTouchdownVelocity());
      touchdownVelocity = new FrameParameterVector3D(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), defaultTouchdownVelocity, registry);
      finalCoMVelocityInjectionRatio = new DoubleParameter(namePrefix + "FinalCoMVelocityInjectionRatio",
                                                           registry,
                                                           parameters.getFinalCoMVelocityInjectionRatio());
      finalCoMAccelerationInjectionRatio = new DoubleParameter(namePrefix + "finalCoMAccelerationInjectionRatio",
                                                               registry,
                                                               parameters.getFinalCoMAccelerationInjectionRatio());

      Vector3D defaultTouchdownAcceleration = new Vector3D(0.0, 0.0, parameters.getDesiredTouchdownAcceleration());
      touchdownAcceleration = new FrameParameterVector3D(namePrefix + "TouchdownAcceleration",
                                                         ReferenceFrame.getWorldFrame(),
                                                         defaultTouchdownAcceleration,
                                                         registry);

      ignoreInitialAngularVelocityZ = new BooleanParameter(namePrefix + "IgnoreInitialAngularVelocityZ",
                                                           registry,
                                                           walkingControllerParameters.ignoreSwingInitialAngularVelocityZ());
      maxInitialLinearVelocityMagnitude = new DoubleParameter(namePrefix + "MaxInitialLinearVelocityMagnitude",
                                                              registry,
                                                              walkingControllerParameters.getMaxSwingInitialLinearVelocityMagnitude());
      maxInitialAngularVelocityMagnitude = new DoubleParameter(namePrefix + "MaxInitialAngularVelocityMagnitude",
                                                               registry,
                                                               walkingControllerParameters.getMaxSwingInitialAngularVelocityMagnitude());

      velocityAdjustmentDamping = new DoubleParameter(namePrefix + "VelocityAdjustmentDamping", registry, 0.0);//parameters.getSwingFootVelocityAdjustmentDamping());
   }

   public boolean doToeTouchdownIfPossible()
   {
      return doToeTouchdownIfPossible.getValue();
   }

   public double getToeTouchdownAngle()
   {
      return toeTouchdownAngle.getValue();
   }

   public double getToeTouchdownDepthRatio()
   {
      return toeTouchdownDepthRatio.getValue();
   }

   public double getStepDownHeightForToeTouchdown()
   {
      return stepDownHeightForToeTouchdown.getValue();
   }

   public boolean doHeelTouchdownIfPossible()
   {
      return doHeelTouchdownIfPossible.getValue();
   }

   public double getHeelTouchdownAngle()
   {
      return heelTouchdownAngle.getValue();
   }

   public double getHeelTouchdownLengthRatio()
   {
      return heelTouchdownLengthRatio.getValue();
   }

   public double getMaximumHeightForHeelTouchdown()
   {
      return maximumHeightForHeelTouchdown.getValue();
   }

   public boolean addOrientationMidpointForObstacleClearance()
   {
      return addOrientationMidpointForClearance.getValue();
   }

   public double getMidpointOrientationInterpolationForObstacleClearance()
   {
      return midpointOrientationInterpolationForClearance.getValue();
   }

   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return minHeightDifferenceForObstacleClearance.getValue();
   }

   public List<DoubleProvider> getSwingWaypointProportions()
   {
      return defaultWaypointProportions;
   }

   public List<DoubleProvider> getObstacleClearanceProportions()
   {
      return defaultObstacleClearanceWaypointProportions;
   }

   public Tuple3DReadOnly getTouchdownVelocityWeight()
   {
      return touchdownVelocityWeight;
   }

   public double getDesiredTouchdownHeightOffset()
   {
      return finalSwingHeightOffset.getValue();
   }

   public FrameVector3DReadOnly getDesiredTouchdownVelocity()
   {
      return touchdownVelocity;
   }

   public FrameVector3DReadOnly getDesiredTouchdownAcceleration()
   {
      return touchdownAcceleration;
   }

   public double getFinalCoMVelocityInjectionRatio()
   {
      return finalCoMVelocityInjectionRatio.getValue();
   }

   public double getFinalCoMAccelerationInjectionRatio()
   {
      return finalCoMAccelerationInjectionRatio.getValue();
   }

   public boolean ignoreSwingInitialAngularVelocityZ()
   {
      return ignoreInitialAngularVelocityZ.getValue();
   }

   public double getMaxSwingInitialLinearVelocityMagnitude()
   {
      return maxInitialLinearVelocityMagnitude.getValue();
   }

   public double getMaxSwingInitialAngularVelocityMagnitude()
   {
      return maxInitialAngularVelocityMagnitude.getValue();
   }

   public double getSwingFootVelocityAdjustmentDamping()
   {
      return velocityAdjustmentDamping.getValue();
   }
}
