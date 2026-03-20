package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.commonWalkingControlModules.controlModules.foot.SupportStateParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class LoadBearingParameters
{
   private static final double DEFAULT_COEFFICIENT_OF_FRICTION = 0.3;

   /**
    * Threshold [Nm] below which the hand has a non-slip feedback objective. Analogous to {@link SupportStateParameters#getFootLoadThreshold}
    */
   private final DoubleParameter normalForceThresholdForLoaded;

   /**
    * Tracking error threshold [m], if the hand has a position tracking error greater it will exit load bearing and hold position
    */
   private final DoubleParameter linearSlippingThreshold;

   /**
    * Stiffness of the hold position objective (when barely loaded)
    */
   private final DoubleParameter holdPositionStiffness;

   /**
    * Damping ratio of the hold position objective (when barely loaded)
    */
   private final DoubleParameter holdPositionDampingRatio;

   /**
    * Duration (in seconds) of time taken to load the hand
    */
   private final DoubleParameter handLoadDuration;

   /**
    * Duration (in seconds) of time taken to load the hand
    */
   private final DoubleParameter terminalHandSpeed;

   private final DoubleParameter rhoWeightInitial;

   private final DefaultYoPIDSE3Gains preContactFeedbackGains;
   private final DefaultYoPIDSE3Gains postContactFeedbackGains;
   private final YoFrameVector3D preContactPositionWeights;
   private final YoFrameVector3D preContactOrientationWeights;

   private final YoFrameVector3D postContactPositionWeights;
   private final YoFrameVector3D postContactOrientationWeights;
   private final DefaultPID3DGains collisionGains = new DefaultPID3DGains();

   private final YoDouble coefficientOfFriction;

   private final YoBoolean doSmoothLoading;
   private final YoBoolean enableCollisionAvoidance;

   public LoadBearingParameters(YoRegistry registry)
   {
      normalForceThresholdForLoaded = new DoubleParameter("handLoadedForceThreshold", registry, 12.0);
      linearSlippingThreshold = new DoubleParameter("loadBearingLinearTrackingSlipThreshold", registry, 0.09);

      holdPositionStiffness = new DoubleParameter("kpXYHandLoadBearingPosition", registry, 100.0);
      holdPositionDampingRatio = new DoubleParameter("zetaXYHandLoadBearingPosition", registry, 0.65);

      handLoadDuration = new DoubleParameter("handLoadDuration", registry, 0.8);
      terminalHandSpeed = new DoubleParameter("terminalHandSpeed", registry, 0.9);

      rhoWeightInitial = new DoubleParameter("rhoWeightInitial", registry, 1.5);
      preContactFeedbackGains = new DefaultYoPIDSE3Gains("PreContact", GainCoupling.XYZ, false, registry);
      postContactFeedbackGains = new DefaultYoPIDSE3Gains("PostContact", GainCoupling.XY, false, registry);
      configureGains();

      preContactPositionWeights = new YoFrameVector3D("preContactPositionWeights", ReferenceFrame.getWorldFrame(), registry);
      preContactOrientationWeights = new YoFrameVector3D("preContactOrientationWeights", ReferenceFrame.getWorldFrame(), registry);
      preContactPositionWeights.set(2.5, 2.5, 2.5);
      preContactOrientationWeights.set(0.0, 0.0, 0.0);

      postContactPositionWeights = new YoFrameVector3D("postContactPositionWeights", ReferenceFrame.getWorldFrame(), registry);
      postContactOrientationWeights = new YoFrameVector3D("postContactOrientationWeights", ReferenceFrame.getWorldFrame(), registry);
      postContactPositionWeights.set(50.0, 50.0, 50.0);
      postContactOrientationWeights.set(0.3, 0.3, 0.3);

      doSmoothLoading = new YoBoolean("doSmoothLoading", registry);
      doSmoothLoading.set(false);

      coefficientOfFriction = new YoDouble("coefficientOfFriction", registry);
      coefficientOfFriction.set(DEFAULT_COEFFICIENT_OF_FRICTION);

      collisionGains.setProportionalGains(0.0, 0.0, 100.0);
      collisionGains.setDampingRatios(0.0, 0.0, 0.25);

      enableCollisionAvoidance = new YoBoolean("enableCollisionAvoidance", registry);
      enableCollisionAvoidance.set(false);
   }

   public double getNormalForceThresholdForLoaded()
   {
      return normalForceThresholdForLoaded.getValue();
   }

   public double getLinearTrackingSlipThreshold()
   {
      return linearSlippingThreshold.getValue();
   }

   public double getHoldPositionStiffness()
   {
      return holdPositionStiffness.getValue();
   }

   public double getHoldPositionDampingRatio()
   {
      return holdPositionDampingRatio.getValue();
   }

   public double getHandLoadDuration()
   {
      return handLoadDuration.getValue();
   }

   public double getTerminalHandSpeed()
   {
      return terminalHandSpeed.getValue();
   }

   public double getRhoWeightInitial()
   {
      return rhoWeightInitial.getValue();
   }

   private void configureGains()
   {
      ///////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////// PRE CONTACT ///////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////

      double kpPosition = 100.0;
      double zetaPosition = 0.4;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      preContactFeedbackGains.setPositionProportionalGains(kpPosition);
      preContactFeedbackGains.setPositionDerivativeGains(GainCalculator.computeDerivativeGain(kpPosition, zetaPosition));
      preContactFeedbackGains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);

      double kpOrientation = 0.0;
      double kdOrientation = 0.0;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;
      preContactFeedbackGains.setOrientationProportionalGains(kpOrientation);
      preContactFeedbackGains.setOrientationDerivativeGains(kdOrientation);
      preContactFeedbackGains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      ///////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////// POST CONTACT //////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////

      double kpXYPosition = 100.0;
      double kpZPosition = 0.0;
      double zetaXYPosition = 1.0;
      double kdXYPosition = GainCalculator.computeDerivativeGain(kpXYPosition, zetaXYPosition);
      double kdZ = 0.0;
      maxLinearAcceleration = Double.POSITIVE_INFINITY;
      maxLinearJerk = Double.POSITIVE_INFINITY;
      postContactFeedbackGains.setPositionProportionalGains(kpXYPosition, kpXYPosition, kpZPosition);
      postContactFeedbackGains.setPositionDerivativeGains(kdXYPosition, kdXYPosition, kdZ);
      postContactFeedbackGains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);

      double kpXYOrientation = 100.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 0.35;
      double kdOrientationXY = GainCalculator.computeDerivativeGain(kpXYOrientation, zetaOrientation);
      double kdOrientationZ = GainCalculator.computeDerivativeGain(kpZOrientation, zetaOrientation);
      maxAngularAcceleration = Double.POSITIVE_INFINITY;
      maxAngularJerk = Double.POSITIVE_INFINITY;
      postContactFeedbackGains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      postContactFeedbackGains.setOrientationDerivativeGains(kdOrientationXY, kdOrientationXY, kdOrientationZ);
      postContactFeedbackGains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      // bias towards nominal, but heavily limit feedback
      postContactFeedbackGains.getOrientationGains().setMaxProportionalError(Math.toRadians(12.0));
      postContactFeedbackGains.getOrientationGains().setMaxDerivativeError(Math.toRadians(25.0));
   }

   public DefaultYoPIDSE3Gains getPreContactFeedbackGains()
   {
      return preContactFeedbackGains;
   }

   public DefaultYoPIDSE3Gains getPostContactFeedbackGains()
   {
      return postContactFeedbackGains;
   }

   public Vector3DReadOnly getPreContactPositionWeights()
   {
      return preContactPositionWeights;
   }

   public Vector3DReadOnly getPreContactOrientationWeights()
   {
      return preContactOrientationWeights;
   }

   public Vector3DReadOnly getPostContactPositionWeights()
   {
      return postContactPositionWeights;
   }

   public Vector3DReadOnly getPostContactOrientationWeights()
   {
      return postContactOrientationWeights;
   }

   public boolean doSmoothLoading()
   {
      return doSmoothLoading.getValue();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getValue();
   }

   public DefaultPID3DGains getCollisionGains()
   {
      return collisionGains;
   }

   public boolean enableCollisionAvoidance()
   {
      return enableCollisionAvoidance.getValue();
   }
}
