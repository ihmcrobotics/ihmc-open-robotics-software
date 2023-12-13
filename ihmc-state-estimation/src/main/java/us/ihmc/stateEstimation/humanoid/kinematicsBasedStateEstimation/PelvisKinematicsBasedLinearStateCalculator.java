package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashCompensatingVelocityYoFrameVector;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
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
   private final RigidBodyBasics[] feetRigidBodies;
   private final SingleFootEstimator[] footEstimators;
   private final Map<RigidBodyBasics, SingleFootEstimator> footEstimatorMap = new HashMap<>();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint3D rootJointPosition = new YoFramePoint3D("estimatedRootJointPositionKinematics", worldFrame, registry);
   private final YoFrameVector3D rootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocityKinematics", worldFrame, registry);
   private final DoubleProvider alphaRootJointLinearVelocity;

   /** Debug variable */
   private final YoDouble alphaLinearVelocityDebug = new YoDouble("alphaRootJointLinearVelocityBacklashKinematics", registry);
   /** Debug variable */
   private final YoDouble slopTimeLinearVelocityDebug = new YoDouble("slopTimeRootJointLinearVelocityBacklashKinematics", registry);
   /** Debug variable */
   private final BacklashCompensatingVelocityYoFrameVector rootJointLinearVelocityFDDebug;

   private final DoubleProvider footToRootJointPositionBreakFrequency;
   private final BooleanProvider correctTrustedFeetPositions;

   private final DoubleProvider copFilterBreakFrequency;

   private final YoBoolean kinematicsIsUpToDate = new YoBoolean("kinematicsIsUpToDate", registry);
   private final BooleanProvider useControllerDesiredCoP;
   private final BooleanProvider trustCoPAsNonSlippingContactPoint;

   private final BooleanParameter assumeTrustedFootAtZeroHeight = new BooleanParameter("assumeTrustedFootAtZeroHeight", registry, false);

   public PelvisKinematicsBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
                                                     Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies,
                                                     Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                                     CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                                     double estimatorDT,
                                                     StateEstimatorParameters stateEstimatorParameters,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                                     YoRegistry parentRegistry)
   {
      rootJoint = inverseDynamicsStructure.getRootJoint();
      feetRigidBodies = feetContactablePlaneBodies.keySet().toArray(new RigidBodyBasics[0]);

      footToRootJointPositionBreakFrequency = new DoubleParameter("FootToRootJointPositionBreakFrequency",
                                                                  registry,
                                                                  stateEstimatorParameters.getKinematicsPelvisPositionFilterFreqInHertz());
      alphaRootJointLinearVelocity = new DoubleParameter("alphaRootJointLinearVelocityKinematics",
                                                         registry,
                                                         stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      trustCoPAsNonSlippingContactPoint = new BooleanParameter("trustCoPAsNonSlippingContactPoint",
                                                               registry,
                                                               stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());
      useControllerDesiredCoP = new BooleanParameter("useControllerDesiredCoP", registry, stateEstimatorParameters.useControllerDesiredCenterOfPressure());
      copFilterBreakFrequency = new DoubleParameter("CopFilterBreakFrequency", registry, stateEstimatorParameters.getCoPFilterFreqInHertz());
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
                                                     centerOfPressureDataHolderFromController,
                                                     estimatorDT,
                                                     registry);
         footEstimatorMap.put(footRigidBody, footEstimators[i]);
      }

      /*
       * These are for debug purposes, not need to clutter the state estimator parameters class with them.
       */
      alphaLinearVelocityDebug.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, estimatorDT));
      slopTimeLinearVelocityDebug.set(0.03);
      rootJointLinearVelocityFDDebug = BacklashCompensatingVelocityYoFrameVector.createBacklashCompensatingVelocityYoFrameVector("estimatedRootJointLinearVelocityBacklashKin",
                                                                                                                                 "",
                                                                                                                                 alphaLinearVelocityDebug,
                                                                                                                                 estimatorDT,
                                                                                                                                 slopTimeLinearVelocityDebug,
                                                                                                                                 registry,
                                                                                                                                 rootJointPosition);
      /*
       * -------------------------------------------------------------------------------------------------
       */

      if (VISUALIZE)
      {
         for (SingleFootEstimator footEstimator : footEstimators)
         {
            footEstimator.createVisualization(yoGraphicsListRegistry);
         }
      }

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
         footEstimator.updateUntrustedFootPosition(pelvisPosition);
      }
      kinematicsIsUpToDate.set(false);
   }

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state
    */
   public void updateKinematics()
   {
      rootJointPosition.setToZero();
      updateKinematicsNewTwist();

      for (SingleFootEstimator footEstimator : footEstimators)
         footEstimator.updateKinematics();

      kinematicsIsUpToDate.set(true);
   }

   private final Twist tempRootBodyTwist = new Twist();

   private void updateKinematicsNewTwist()
   {
      tempRootBodyTwist.setIncludingFrame(rootJoint.getJointTwist());
      tempRootBodyTwist.getLinearPart().setMatchingFrame(rootJointLinearVelocity);

      for (SingleFootEstimator footEstimator : footEstimators)
         footEstimator.updateFootLinearVelocityInWorld(tempRootBodyTwist);
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
         footEstimator.updateUntrustedFootPosition(pelvisPosition);
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

      if (assumeTrustedFootAtZeroHeight.getValue())
      {
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));
            if (trustCoPAsNonSlippingContactPoint.getValue())
            {
               footEstimator.updateCoPPosition(trustCoPAsNonSlippingContactPoint.getValue(), useControllerDesiredCoP.getValue());
               footEstimator.copPositionInWorld.setZ(0.0);
               footEstimator.correctFootPositionsUsingCoP(trustCoPAsNonSlippingContactPoint.getValue());
            }
            else
            {
               footEstimator.footPositionInWorld.setZ(0.0);
            }
            footEstimator.updatePelvisWithKinematics(trustedFeet.size(), alphaRootJointLinearVelocity.getValue(), rootJointPosition, rootJointLinearVelocity);
         }
      }
      else
      {
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));
            footEstimator.updateCoPPosition(trustCoPAsNonSlippingContactPoint.getValue(), useControllerDesiredCoP.getValue());
            footEstimator.correctFootPositionsUsingCoP(trustCoPAsNonSlippingContactPoint.getValue());
            footEstimator.updatePelvisWithKinematics(trustedFeet.size(), alphaRootJointLinearVelocity.getValue(), rootJointPosition, rootJointLinearVelocity);
         }
      }

      rootJointLinearVelocityFDDebug.update();

      if (correctTrustedFeetPositions.getValue())
      {
         for (int i = 0; i < trustedFeet.size(); i++)
         {
            SingleFootEstimator footEstimator = footEstimatorMap.get(trustedFeet.get(i));
            footEstimator.updateTrustedFootPosition(trustCoPAsNonSlippingContactPoint.getValue(), rootJointPosition);
         }
      }

      for (int i = 0; i < unTrustedFeet.size(); i++)
      {
         SingleFootEstimator footEstimator = footEstimatorMap.get(unTrustedFeet.get(i));
         footEstimator.updateUntrustedFootPosition(pelvisPosition);
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

   public void getFootToRootJointPosition(FramePoint3D positionToPack, RigidBodyBasics foot)
   {
      positionToPack.setIncludingFrame(footEstimatorMap.get(foot).footToRootJointPosition);
   }

   public FrameVector3DReadOnly getFootVelocityInWorld(RigidBodyBasics foot)
   {
      return footEstimatorMap.get(foot).footVelocityInWorld;
   }

   private static class SingleFootEstimator implements SCS2YoGraphicHolder
   {
      private final RigidBodyBasics foot;

      private final ReferenceFrame rootJointFrame;
      private final ReferenceFrame soleFrame;
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final YoFrameVector3D footVelocityInWorld;
      private final AlphaFilteredYoFrameVector footToRootJointPosition;
      private final YoFramePoint3D footPositionInWorld;
      /** Debug variable */
      private final YoFramePoint3D rootJointPositionPerFoot;
      private final YoFramePoint3D copPositionInWorld;
      private final AlphaFilteredYoFramePoint2d copFilteredInFootFrame;
      private final YoFramePoint2D copRawInFootFrame;
      private final FrameConvexPolygon2D footPolygon;
      private final FrameLineSegment2D footCenterCoPLineSegment;
      private final FootSwitchInterface footSwitch;
      private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;

      private final FramePoint2DBasics[] intersectionPoints = new FramePoint2DBasics[] {new FramePoint2D(), new FramePoint2D()};

      public SingleFootEstimator(FloatingJointBasics rootJoint,
                                 ContactablePlaneBody contactableFoot,
                                 FootSwitchInterface footSwitch,
                                 DoubleProvider footToRootJointPositionBreakFrequency,
                                 DoubleProvider copFilterBreakFrequency,
                                 CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                 double estimatorDT,
                                 YoRegistry registry)
      {
         this.rootJointFrame = rootJoint.getFrameAfterJoint();
         this.footSwitch = footSwitch;
         this.centerOfPressureDataHolderFromController = centerOfPressureDataHolderFromController;
         foot = contactableFoot.getRigidBody();
         soleFrame = contactableFoot.getSoleFrame();

         String namePrefix = foot.getName();

         DoubleProvider alphaFoot = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(footToRootJointPositionBreakFrequency.getValue(),
                                                                                                          estimatorDT);
         footToRootJointPosition = new AlphaFilteredYoFrameVector(namePrefix + "FootToRootJointPosition", "", registry, alphaFoot, worldFrame);
         rootJointPositionPerFoot = new YoFramePoint3D(namePrefix + "BasedRootJointPosition", worldFrame, registry);
         footPositionInWorld = new YoFramePoint3D(namePrefix + "FootPositionInWorld", worldFrame, registry);
         footPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
         footCenterCoPLineSegment = new FrameLineSegment2D(soleFrame);
         copRawInFootFrame = new YoFramePoint2D(namePrefix + "CoPRawInFootFrame", soleFrame, registry);

         DoubleProvider alphaCop = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(copFilterBreakFrequency.getValue(), estimatorDT);
         copFilteredInFootFrame = new AlphaFilteredYoFramePoint2d(namePrefix + "CoPFilteredInFootFrame", "", registry, alphaCop, copRawInFootFrame);
         copFilteredInFootFrame.update(0.0, 0.0);
         copPositionInWorld = new YoFramePoint3D(namePrefix + "CoPPositionsInWorld", worldFrame, registry);
         footVelocityInWorld = new YoFrameVector3D(namePrefix + "VelocityInWorld", worldFrame, registry);
      }

      public void createVisualization(YoGraphicsListRegistry yoGraphicsListRegistry)
      {
         if (yoGraphicsListRegistry == null)
            return;

         String sidePrefix = foot.getName();
         YoGraphicPosition copInWorld = new YoGraphicPosition(sidePrefix + "StateEstimatorCoP", copPositionInWorld, 0.005, YoAppearance.DeepPink());
         YoArtifactPosition artifact = copInWorld.createArtifact();
         artifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact("StateEstimator", artifact);
      }

      @Override
      public YoGraphicDefinition getSCS2YoGraphics()
      {
         YoGraphicPoint2DDefinition copVisual = YoGraphicDefinitionFactory.newYoGraphicPoint2D(foot.getName() + "StateEstimatorCoP",
                                                                                               copPositionInWorld,
                                                                                               0.01,
                                                                                               ColorDefinitions.DeepPink(),
                                                                                               DefaultPoint2DGraphic.CIRCLE);
         return copVisual;
      }

      public void initialize()
      {
         footToRootJointPosition.reset();
         copFilteredInFootFrame.reset();
         copFilteredInFootFrame.update(0.0, 0.0);
         footVelocityInWorld.setToZero();
      }

      private final FramePoint3D tempFramePoint = new FramePoint3D();

      /**
       * Estimates the pelvis position and linear velocity using the leg kinematics
       *
       * @param trustedFoot          is the foot used to estimates the pelvis state
       * @param numberOfTrustedSides is only one or both legs used to estimate the pelvis state
       */
      private void updatePelvisWithKinematics(int numberOfTrustedFeet,
                                              double alphaVelocityUpdate,
                                              FixedFramePoint3DBasics rootJointPosition,
                                              FixedFrameVector3DBasics rootJointLinearVelocity)
      {
         double scaleFactor = 1.0 / numberOfTrustedFeet;

         rootJointPositionPerFoot.add(footPositionInWorld, footToRootJointPosition);
         rootJointPosition.scaleAdd(scaleFactor, rootJointPositionPerFoot, rootJointPosition);

         rootJointLinearVelocity.scaleAdd(-scaleFactor * alphaVelocityUpdate, footVelocityInWorld, rootJointLinearVelocity);
      }

      /**
       * updates the position of a swinging foot
       *
       * @param swingingFoot   a foot in swing
       * @param pelvisPosition the current pelvis position
       */
      private void updateUntrustedFootPosition(FramePoint3DReadOnly pelvisPosition)
      {
         footPositionInWorld.sub(pelvisPosition, footToRootJointPosition);

         copPositionInWorld.set(footPositionInWorld);

         copRawInFootFrame.setToZero();
         copFilteredInFootFrame.setToZero();
      }

      private final FrameVector3D tempFrameVector = new FrameVector3D();

      private void updateTrustedFootPosition(boolean trustCoPAsNonSlippingContactPoint, FramePoint3DReadOnly rootJointPosition)
      {
         if (trustCoPAsNonSlippingContactPoint)
         {
            tempFrameVector.setIncludingFrame(rootJointPosition);
            tempFrameVector.sub(footToRootJointPosition); // New foot position
            tempFrameVector.sub(footPositionInWorld); // Delta from previous to new foot position
            copPositionInWorld.add(tempFrameVector); // New CoP position
         }

         footPositionInWorld.set(rootJointPosition);
         footPositionInWorld.sub(footToRootJointPosition);
      }

      private final FramePoint2D tempCoP2d = new FramePoint2D();
      private final FrameVector3D tempCoPOffset = new FrameVector3D();

      /**
       * Compute the foot CoP. The CoP is the point on the support foot trusted to be not slipping.
       *
       * @param trustedSide
       * @param footSwitch
       */
      private void updateCoPPosition(boolean trustCoPAsNonSlippingContactPoint, boolean useControllerDesiredCoP)
      {
         if (trustCoPAsNonSlippingContactPoint)
         {
            if (useControllerDesiredCoP)
               centerOfPressureDataHolderFromController.getCenterOfPressure(tempCoP2d, foot);
            else
               footSwitch.getCenterOfPressure(tempCoP2d);

            if (tempCoP2d.containsNaN())
            {
               tempCoP2d.setToZero(soleFrame);
            }
            else
            {
               boolean isCoPInsideFoot = footPolygon.isPointInside(tempCoP2d);
               if (!isCoPInsideFoot)
               {
                  if (footSwitch.getFootLoadPercentage() > 0.2)
                  {
                     footCenterCoPLineSegment.set(soleFrame, 0.0, 0.0, tempCoP2d.getX(), tempCoP2d.getY());
                     int intersections = footPolygon.intersectionWith(footCenterCoPLineSegment, intersectionPoints[0], intersectionPoints[1]);

                     if (intersections == 0)
                     {
                        LogTools.info("Found no solution for the CoP projection.");
                        tempCoP2d.setToZero(soleFrame);
                     }
                     else
                     {
                        tempCoP2d.set(intersectionPoints[0]);

                        if (intersections == 2)
                           LogTools.info("Found two solutions for the CoP projection.");
                     }
                  }
                  else // If foot barely loaded and actual CoP outside, then don't update the raw CoP right below
                  {
                     tempCoP2d.setIncludingFrame(copRawInFootFrame);
                  }
               }
            }

            copRawInFootFrame.set(tempCoP2d);

            tempCoPOffset.setIncludingFrame(soleFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
            copFilteredInFootFrame.update();
            tempCoPOffset.setIncludingFrame(soleFrame,
                                            copFilteredInFootFrame.getX() - tempCoPOffset.getX(),
                                            copFilteredInFootFrame.getY() - tempCoPOffset.getY(),
                                            0.0);

            tempCoPOffset.changeFrame(worldFrame);
            copPositionInWorld.add(tempCoPOffset);
         }
         else
         {
            tempCoP2d.setToZero(soleFrame);
         }
      }

      /**
       * Assuming the CoP is not moving, the foot position can be updated. That way we can see if the foot
       * is on the edge.
       *
       * @param plantedFoot
       */
      private void correctFootPositionsUsingCoP(boolean trustCoPAsNonSlippingContactPoint)
      {
         if (!trustCoPAsNonSlippingContactPoint)
            return;

         tempCoPOffset.setIncludingFrame(copFilteredInFootFrame, 0.0);
         tempCoPOffset.changeFrame(worldFrame);
         footPositionInWorld.sub(copPositionInWorld, tempCoPOffset);
      }

      /**
       * Updates the different kinematics related stuff that is used to estimate the pelvis state
       */
      public void updateKinematics()
      {
         tempFramePoint.setToZero(rootJointFrame);
         tempFramePoint.changeFrame(soleFrame);

         tempFrameVector.setIncludingFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToRootJointPosition.update(tempFrameVector);
      }

      private final Twist tempRootBodyTwist = new Twist();
      private final Twist footTwistInWorld = new Twist();

      private void updateFootLinearVelocityInWorld(TwistReadOnly rootBodyTwist)
      {
         computeFootLinearVelocityInWorld(rootBodyTwist, footVelocityInWorld);
      }

      private void computeFootLinearVelocityInWorld(TwistReadOnly rootBodyTwist, FixedFrameVector3DBasics footLinearVelocityToPack)
      {
         tempRootBodyTwist.setIncludingFrame(rootBodyTwist);
         tempRootBodyTwist.setBaseFrame(worldFrame);
         tempRootBodyTwist.changeFrame(foot.getBodyFixedFrame());

         foot.getBodyFixedFrame().getTwistRelativeToOther(rootJointFrame, footTwistInWorld);
         footTwistInWorld.add(tempRootBodyTwist);
         footTwistInWorld.setBodyFrame(soleFrame);
         footTwistInWorld.changeFrame(worldFrame);

         footTwistInWorld.getLinearVelocityAt(copPositionInWorld, footLinearVelocityToPack);
      }
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
