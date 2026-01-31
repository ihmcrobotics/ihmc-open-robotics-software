package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.euclid.filters.BacklashCompensatingVelocityYoFrameVector3D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
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
public class PelvisKinematicsBasedLinearStateCalculator implements SCS2YoGraphicHolder
{
   private static final boolean VISUALIZE = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FloatingJointBasics rootJoint;
   private final SingleFootEstimator[] footEstimators;
   private final Map<RigidBodyBasics, SingleFootEstimator> footEstimatorMap = new HashMap<>();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint3D rootJointPosition = new YoFramePoint3D("estimatedRootJointPositionKinematics", worldFrame, registry);
   private final YoFrameVector3D rootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocityKinematics", worldFrame, registry);
   private final DoubleProvider alphaRootJointLinearVelocity;

   /** Debug variable */
   private final BacklashCompensatingVelocityYoFrameVector3D rootJointLinearVelocityFDDebug;

   private final BooleanProvider correctTrustedFeetPositions;

   private final YoBoolean kinematicsIsUpToDate = new YoBoolean("kinematicsIsUpToDate", registry);
   private final BooleanProvider useControllerDesiredCoP;

   public PelvisKinematicsBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
                                                     Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies,
                                                     Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                                     CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                                     double estimatorDT,
                                                     StateEstimatorParameters stateEstimatorParameters,
                                                     YoRegistry parentRegistry)
   {
      rootJoint = inverseDynamicsStructure.getRootJoint();
      RigidBodyBasics[] feetRigidBodies = feetContactablePlaneBodies.keySet().toArray(new RigidBodyBasics[0]);

      DoubleProvider footToRootJointPositionBreakFrequency = new DoubleParameter("FootToRootJointPositionBreakFrequency",
                                                                  registry,
                                                                  stateEstimatorParameters.getKinematicsPelvisPositionFilterFreqInHertz());
      alphaRootJointLinearVelocity = new DoubleParameter("alphaRootJointLinearVelocityKinematics",
                                                         registry,
                                                         stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      BooleanProvider trustCoPAsNonSlippingContactPoint = new BooleanParameter("trustCoPAsNonSlippingContactPoint",
                                                               registry,
                                                               stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());
      BooleanParameter assumeTrustedFootAtZeroHeight = new BooleanParameter("assumeTrustedFootAtZeroHeight", registry, false);

      useControllerDesiredCoP = new BooleanParameter("useControllerDesiredCoP", registry, stateEstimatorParameters.useControllerDesiredCenterOfPressure());
      DoubleProvider copFilterBreakFrequency = new DoubleParameter("CopFilterBreakFrequency", registry, stateEstimatorParameters.getCoPFilterFreqInHertz());
      correctTrustedFeetPositions = new BooleanParameter("correctTrustedFeetPositions", registry, stateEstimatorParameters.correctTrustedFeetPositions());

      footEstimators = new SingleFootEstimator[feetRigidBodies.length];
      for (int i = 0; i < feetRigidBodies.length; i++)
      {
         RigidBodyBasics footRigidBody = feetRigidBodies[i];
         ContactablePlaneBody contactableFoot = feetContactablePlaneBodies.get(footRigidBody);
         FootSwitchInterface footSwitch = footSwitches.get(footRigidBody);
         footEstimators[i] = new SingleFootEstimator(rootJoint,
                                                     contactableFoot,
                                                     footSwitch,
                                                     footToRootJointPositionBreakFrequency,
                                                     copFilterBreakFrequency,
                                                     trustCoPAsNonSlippingContactPoint,
                                                     assumeTrustedFootAtZeroHeight,
                                                     centerOfPressureDataHolderFromController,
                                                     estimatorDT,
                                                     registry);
         footEstimatorMap.put(footRigidBody, footEstimators[i]);
      }

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
    * Estimates the foot positions corresponding to the given pelvisPosition
    *
    * @param pelvisPosition
    */
   public void initialize(FramePoint3DReadOnly pelvisPosition)
   {
      for (SingleFootEstimator footEstimator : footEstimators)
      {
         footEstimator.initialize();
      }
      setPelvisLinearVelocityToZero();

      updateKinematics();
      setPelvisPosition(pelvisPosition);

      for (SingleFootEstimator footEstimator : footEstimators)
      {
         footEstimator.updateUntrustedFootPosition(pelvisPosition, useControllerDesiredCoP.getValue());
      }
      kinematicsIsUpToDate.set(false);
   }

   private final Twist tempRootBodyTwist = new Twist();

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state
    */
   public void updateKinematics()
   {
      rootJointPosition.setToZero();
      tempRootBodyTwist.setIncludingFrame(rootJoint.getJointTwist());
      tempRootBodyTwist.getLinearPart().setMatchingFrame(rootJointLinearVelocity);

      for (SingleFootEstimator footEstimator : footEstimators)
         footEstimator.updateFootLinearVelocityInWorld(tempRootBodyTwist);

      for (SingleFootEstimator footEstimator : footEstimators)
         footEstimator.updateKinematics();

      kinematicsIsUpToDate.set(true);
   }



   /**
    * Updates this module when no foot can be trusted.
    * 
    * @param pelvisPosition       the current estimated pelvis position. This module does not compute
    *                             an estimate.
    * @param pelvisLinearVelocity the current estimated pelvis linear velocity. This module does not
    *                             compute an estimate. If {@code null}, a zero velocity is assumed.
    */
   public void updateNoTrustedFeet(FramePoint3DReadOnly pelvisPosition, FrameVector3DReadOnly pelvisLinearVelocity)
   {
      for (SingleFootEstimator footEstimator : footEstimators)
      {
         footEstimator.updateUntrustedFootPosition(pelvisPosition, useControllerDesiredCoP.getValue());
      }

      rootJointPosition.set(pelvisPosition);

      if (pelvisLinearVelocity != null)
         rootJointLinearVelocity.set(pelvisLinearVelocity);
      else
         rootJointLinearVelocity.setToZero();
   }

   public void estimatePelvisLinearState(List<RigidBodyBasics> trustedFeet, List<RigidBodyBasics> unTrustedFeet, FramePoint3DReadOnly pelvisPosition)
   {
      if (!kinematicsIsUpToDate.getBooleanValue())
         throw new RuntimeException("Leg kinematics needs to be updated before trying to estimate the pelvis position/linear velocity.");

      for (int i = 0; i < trustedFeet.size(); i++)
      {
         SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));
         footEstimator.computeFootPositionInWorld(useControllerDesiredCoP.getValue());
         footEstimator.updatePelvisWithKinematics(trustedFeet.size(), alphaRootJointLinearVelocity.getValue(), rootJointPosition, rootJointLinearVelocity);
      }

      rootJointLinearVelocityFDDebug.update();

      if (correctTrustedFeetPositions.getValue())
      {
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));
            footEstimator.updateTrustedFootPosition(rootJointPosition);
         }
      }

      for (int i = 0; i < unTrustedFeet.size(); i++)
      {
         SingleFootEstimator footEstimator = footEstimatorMap.get(unTrustedFeet.get(i));
         footEstimator.updateUntrustedFootPosition(pelvisPosition, useControllerDesiredCoP.getValue());
      }

      kinematicsIsUpToDate.set(false);
   }


   public void setPelvisPosition(FramePoint3DReadOnly pelvisPosition)
   {
      rootJointPosition.set(pelvisPosition);
   }

   public void setPelvisLinearVelocity(FrameVector3DReadOnly pelvisLinearVelocity)
   {
      rootJointLinearVelocityFDDebug.reset();
      rootJointLinearVelocityFDDebug.set(pelvisLinearVelocity);
      rootJointLinearVelocity.set(pelvisLinearVelocity);
   }

   public void setPelvisLinearVelocityToZero()
   {
      rootJointLinearVelocityFDDebug.reset();
      rootJointLinearVelocityFDDebug.setToZero();
      rootJointLinearVelocity.setToZero();
   }

   public FramePoint3DReadOnly getPelvisPosition()
   {
      return rootJointPosition;
   }

   public FrameVector3DReadOnly getPelvisVelocity()
   {
      return rootJointLinearVelocity;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (!VISUALIZE)
         return null;

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (SingleFootEstimator footEstimator : footEstimators)
      {
         group.addChild(footEstimator.getSCS2YoGraphics());
      }
      return group;
   }
}
