package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.euclid.filters.BacklashCompensatingVelocityYoFrameVector3D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * PelvisKinematicsBasedPositionCalculator estimates the pelvis position and linear velocity using
 * the leg kinematics.
 *
 * @author Sylvain
 */
public class PelvisKinematicsBasedLinearStateCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyBasics, SingleFootEstimator> footEstimatorMap;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint3D rootJointPosition = new YoFramePoint3D("estimatedRootJointPositionKinematics", worldFrame, registry);
   private final YoFrameVector3D rootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocityKinematics", worldFrame, registry);
   private final YoFrameVector3D averagedTrustedFootVelocity = new YoFrameVector3D("averagedTrustedFootVelocity", worldFrame, registry);
   private final DoubleProvider alphaRootJointLinearVelocity;

   /** Debug variable */
   private final BacklashCompensatingVelocityYoFrameVector3D rootJointLinearVelocityFDDebug;

   private final BooleanProvider correctTrustedFeetPositions;

   private final YoBoolean kinematicsIsUpToDate = new YoBoolean("kinematicsIsUpToDate", registry);
   private final BooleanProvider trustCoPAsNonSlippingContactPoint;

   private final BooleanParameter assumeTrustedFootAtZeroHeight = new BooleanParameter("assumeTrustedFootAtZeroHeight", registry, false);
   private final BooleanProvider useControllerDesiredCoP;

   public PelvisKinematicsBasedLinearStateCalculator(Map<RigidBodyBasics, SingleFootEstimator> footEstimatorMap,
                                                     double estimatorDT,
                                                     StateEstimatorParameters stateEstimatorParameters,
                                                     YoRegistry parentRegistry)
   {
      this.footEstimatorMap = footEstimatorMap;
      alphaRootJointLinearVelocity = new DoubleParameter("alphaRootJointLinearVelocityKinematics",
                                                         registry,
                                                         stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      trustCoPAsNonSlippingContactPoint = new BooleanParameter("trustCoPAsNonSlippingContactPoint",
                                                               registry,
                                                               stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());

      correctTrustedFeetPositions = new BooleanParameter("correctTrustedFeetPositions", registry, stateEstimatorParameters.correctTrustedFeetPositions());
      useControllerDesiredCoP = new BooleanParameter("useControllerDesiredCoP", registry, stateEstimatorParameters.useControllerDesiredCenterOfPressure());

      /*
       * These are for debug purposes, not need to clutter the state estimator parameters class with them.
       */
      YoDouble alphaLinearVelocityDebug = new YoDouble("alphaRootJointLinearVelocityBacklashKinematics", registry);
      YoDouble slopTimeLinearVelocityDebug = new YoDouble("slopTimeRootJointLinearVelocityBacklashKinematics", registry);
      alphaLinearVelocityDebug.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(16.0, estimatorDT));
      slopTimeLinearVelocityDebug.set(0.03);
      rootJointLinearVelocityFDDebug = new BacklashCompensatingVelocityYoFrameVector3D("estimatedRootJointLinearVelocityBacklashKin",
                                                                                                                                   "",
                                                                                                                                   alphaLinearVelocityDebug,
                                                                                                                                   estimatorDT,
                                                                                                                                   slopTimeLinearVelocityDebug,
                                                                                                                                   registry,
                                                                                                                                   rootJointPosition);
      /*
       * -------------------------------------------------------------------------------------------------
       */


      parentRegistry.addChild(registry);
   }

   /**
    * Estimates the foot positions corresponding to the given rootJointPosition
    *
    * @param rootJointPosition
    */
   public void initialize(FramePoint3DReadOnly rootJointPosition)
   {
      rootJointLinearVelocityFDDebug.reset();
      rootJointLinearVelocityFDDebug.setToZero();
      rootJointLinearVelocity.setToZero();

      this.rootJointPosition.set(rootJointPosition);

      kinematicsIsUpToDate.set(false);
   }


   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state
    */
   public void updateKinematics()
   {
      rootJointPosition.setToZero();
      kinematicsIsUpToDate.set(true);
   }

   /**
    * Updates this module when no foot can be trusted.
    * 
    * @param rootJointPosition       the current estimated pelvis position. This module does not compute
    *                             an estimate.
    * @param rootJointLinearVelocity the current estimated pelvis linear velocity. This module does not
    *                             compute an estimate. If {@code null}, a zero velocity is assumed.
    */
   public void setRootJointState(FramePoint3DReadOnly rootJointPosition, FrameVector3DReadOnly rootJointLinearVelocity)
   {
      this.rootJointPosition.set(rootJointPosition);

      if (rootJointLinearVelocity != null)
         this.rootJointLinearVelocity.set(rootJointLinearVelocity);
      else
         this.rootJointLinearVelocity.setToZero();
   }

   public boolean getKinematicsIsUpToDate()
   {
      return kinematicsIsUpToDate.getBooleanValue();
   }

   public boolean getAssumeTrustedFootAtZeroHeight()
   {
      return assumeTrustedFootAtZeroHeight.getValue();
   }

   public boolean getTrustCoPAsNonSlippingContactPoint()
   {
      return trustCoPAsNonSlippingContactPoint.getValue();
   }

   public boolean getUseControllerDesiredCoP()
   {
      return useControllerDesiredCoP.getValue();
   }

   public boolean getCorrectTrustedFeetPositions()
   {
      return correctTrustedFeetPositions.getValue();
   }

   public void estimatePelvisLinearState(List<RigidBodyBasics> trustedFeet, FramePoint3DReadOnly pelvisPosition, FrameVector3DReadOnly pelvisLinearVelocity)
   {
      if (!kinematicsIsUpToDate.getBooleanValue())
         throw new RuntimeException("Leg kinematics needs to be updated before trying to estimate the pelvis position/linear velocity.");

      if (trustedFeet.isEmpty())
      {
         rootJointPosition.set(pelvisPosition);
         if (pelvisLinearVelocity != null)
            rootJointLinearVelocity.set(pelvisLinearVelocity);
         else
            rootJointLinearVelocity.setToZero();
      }
      else
      {
         averagedTrustedFootVelocity.setToZero();
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));

            double scaleFactor = 1.0 / trustedFeet.size();

            // The root joint position is zeroed when we call #updateKinematics
            rootJointPosition.scaleAdd(scaleFactor, footEstimator.getRootJointPositionFromKinematics(), rootJointPosition);
            averagedTrustedFootVelocity.scaleAdd(scaleFactor, footEstimator.getCopVelocityInWorld(), averagedTrustedFootVelocity);

         }

         // We want the average velocity of the feet to trend to zero, since the trusted feet are assumed to not be moving.
         rootJointLinearVelocity.scaleAdd(-alphaRootJointLinearVelocity.getValue(),
                                          averagedTrustedFootVelocity,
                                          rootJointLinearVelocity);
      }

      rootJointLinearVelocityFDDebug.update();

      kinematicsIsUpToDate.set(false);
   }

   public FramePoint3DReadOnly getPelvisPosition()
   {
      return rootJointPosition;
   }

   public FrameVector3DReadOnly getPelvisVelocity()
   {
      return rootJointLinearVelocity;
   }
}
