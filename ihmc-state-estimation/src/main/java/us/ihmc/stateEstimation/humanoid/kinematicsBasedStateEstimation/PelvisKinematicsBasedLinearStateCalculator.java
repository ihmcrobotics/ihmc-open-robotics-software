package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashCompensatingVelocityYoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * PelvisKinematicsBasedPositionCalculator estimates the pelvis position and linear velocity using the leg kinematics.
 * @author Sylvain
 *
 */
public class PelvisKinematicsBasedLinearStateCalculator
{
   private static final boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FloatingInverseDynamicsJoint rootJoint;
   private final List<RigidBody> feetRigidBodies;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame rootJointFrame;
   private final Map<RigidBody, ReferenceFrame> soleFrames = new LinkedHashMap<RigidBody, ReferenceFrame>();
   private final Map<RigidBody, ReferenceFrame> copFrames = new LinkedHashMap<RigidBody, ReferenceFrame>();

   private final YoFramePoint3D rootJointPosition = new YoFramePoint3D("estimatedRootJointPositionWithKinematics", worldFrame, registry);

   private final Map<RigidBody, YoFrameVector3D> footVelocitiesInWorld = new LinkedHashMap<RigidBody, YoFrameVector3D>();
   private final Map<RigidBody, Twist> footTwistsInWorld = new LinkedHashMap<RigidBody, Twist>();
   private final YoFrameVector3D rootJointLinearVelocityNewTwist = new YoFrameVector3D("estimatedRootJointVelocityNewTwist", worldFrame, registry);
   private final DoubleProvider alphaRootJointLinearVelocityNewTwist;

   /** Debug variable */
   private final YoDouble alphaRootJointLinearVelocityBacklashKinematics = new YoDouble("alphaRootJointLinearVelocityBacklashKinematics", registry);
   /** Debug variable */
   private final YoDouble slopTimeRootJointLinearVelocityBacklashKinematics = new YoDouble("slopTimeRootJointLinearVelocityBacklashKinematics", registry);
   /** Debug variable */
   private final BacklashCompensatingVelocityYoFrameVector rootJointLinearVelocityBacklashKinematics;

   private final DoubleProvider footToRootJointPositionBreakFrequency;
   private final Map<RigidBody, AlphaFilteredYoFrameVector> footToRootJointPositions = new LinkedHashMap<RigidBody, AlphaFilteredYoFrameVector>();
   private final Map<RigidBody, YoFramePoint3D> footPositionsInWorld = new LinkedHashMap<RigidBody, YoFramePoint3D>();
   /** Debug variable */
   private final Map<RigidBody, YoFramePoint3D> rootJointPositionsPerFoot = new LinkedHashMap<>();
   private final BooleanProvider correctTrustedFeetPositions;

   private final Map<RigidBody, YoFramePoint3D> copPositionsInWorld = new LinkedHashMap<RigidBody, YoFramePoint3D>();

   private final DoubleProvider copFilterBreakFrequency;
   private final Map<RigidBody, AlphaFilteredYoFramePoint2d> copsFilteredInFootFrame = new LinkedHashMap<RigidBody, AlphaFilteredYoFramePoint2d>();
   private final Map<RigidBody, YoFramePoint2D> copsRawInFootFrame = new LinkedHashMap<RigidBody, YoFramePoint2D>();

   private final Map<RigidBody, FrameConvexPolygon2D> footPolygons = new LinkedHashMap<RigidBody, FrameConvexPolygon2D>();
   private final Map<RigidBody, FrameLineSegment2D> footCenterCoPLineSegments = new LinkedHashMap<RigidBody, FrameLineSegment2D>();

   private final YoBoolean kinematicsIsUpToDate = new YoBoolean("kinematicsIsUpToDate", registry);
   private final BooleanProvider useControllerDesiredCoP;
   private final BooleanProvider trustCoPAsNonSlippingContactPoint;

   // temporary variables
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FramePoint2D tempCoP2d = new FramePoint2D();
   private final FramePoint3D tempCoP = new FramePoint3D();
   private final FrameVector3D tempCoPOffset = new FrameVector3D();

   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;

   private final FramePoint2DBasics[] intersectionPoints = new FramePoint2DBasics[] {new FramePoint2D(), new FramePoint2D()};

   public PelvisKinematicsBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, Map<RigidBody, ? extends ContactablePlaneBody> feetContactablePlaneBodies,
         Map<RigidBody, FootSwitchInterface> footSwitches, CenterOfPressureDataHolder centerOfPressureDataHolderFromController, double estimatorDT,
         StateEstimatorParameters stateEstimatorParameters, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.footSwitches = footSwitches;
      this.centerOfPressureDataHolderFromController = centerOfPressureDataHolderFromController;
      this.feetRigidBodies = new ArrayList<>(feetContactablePlaneBodies.keySet());

      footToRootJointPositionBreakFrequency = new DoubleParameter("FootToRootJointPositionBreakFrequency", registry, stateEstimatorParameters.getKinematicsPelvisPositionFilterFreqInHertz());
      alphaRootJointLinearVelocityNewTwist = new DoubleParameter("alphaRootJointLinearVelocityNewTwist", registry, stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      trustCoPAsNonSlippingContactPoint = new BooleanParameter("trustCoPAsNonSlippingContactPoint", registry, stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());
      useControllerDesiredCoP = new BooleanParameter("useControllerDesiredCoP", registry, stateEstimatorParameters.useControllerDesiredCenterOfPressure());
      copFilterBreakFrequency = new DoubleParameter("CopFilterBreakFrequency", registry, stateEstimatorParameters.getCoPFilterFreqInHertz());
      correctTrustedFeetPositions = new BooleanParameter("correctTrustedFeetPositions", registry, stateEstimatorParameters.correctTrustedFeetPositions());

      /* These are for debug purposes, not need to clutter the state estimator parameters class with them. */
      alphaRootJointLinearVelocityBacklashKinematics.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, estimatorDT));
      slopTimeRootJointLinearVelocityBacklashKinematics.set(0.03);
      rootJointLinearVelocityBacklashKinematics = BacklashCompensatingVelocityYoFrameVector.createBacklashCompensatingVelocityYoFrameVector("estimatedRootJointLinearVelocityBacklashKin", "",
            alphaRootJointLinearVelocityBacklashKinematics, estimatorDT, slopTimeRootJointLinearVelocityBacklashKinematics, registry, rootJointPosition);
      /* ------------------------------------------------------------------------------------------------- */

      for (int i = 0; i < feetRigidBodies.size(); i++)
      {
         RigidBody foot = feetRigidBodies.get(i);
         ReferenceFrame soleFrame = feetContactablePlaneBodies.get(foot).getSoleFrame();
         soleFrames.put(foot, soleFrame);

         String namePrefix = foot.getName();

         DoubleProvider alphaFoot = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(footToRootJointPositionBreakFrequency.getValue(), estimatorDT);
         AlphaFilteredYoFrameVector footToRootJointPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FootToRootJointPosition", "", registry, alphaFoot, worldFrame);
         footToRootJointPositions.put(foot, footToRootJointPosition);

         YoFramePoint3D rootJointPosition = new YoFramePoint3D(namePrefix + "BasedRootJointPosition", worldFrame, registry);
         rootJointPositionsPerFoot.put(foot, rootJointPosition);

         YoFramePoint3D footPositionInWorld = new YoFramePoint3D(namePrefix + "FootPositionInWorld", worldFrame, registry);
         footPositionsInWorld.put(foot, footPositionInWorld);

         FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(feetContactablePlaneBodies.get(foot).getContactPoints2d()));
         footPolygons.put(foot, footPolygon);

         FrameLineSegment2D tempLineSegment = new FrameLineSegment2D(new FramePoint2D(soleFrame), new FramePoint2D(soleFrame, 1.0, 1.0)); // TODO need to give distinct points that's not convenient
         footCenterCoPLineSegments.put(foot, tempLineSegment);

         YoFramePoint2D copRawInFootFrame = new YoFramePoint2D(namePrefix + "CoPRawInFootFrame", soleFrames.get(foot), registry);
         copsRawInFootFrame.put(foot, copRawInFootFrame);

         DoubleProvider alphaCop = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(copFilterBreakFrequency.getValue(), estimatorDT);
         AlphaFilteredYoFramePoint2d copFilteredInFootFrame = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(namePrefix + "CoPFilteredInFootFrame", "", registry, alphaCop, copRawInFootFrame);
         copFilteredInFootFrame.update(0.0, 0.0);
         copsFilteredInFootFrame.put(foot, copFilteredInFootFrame);

         YoFramePoint3D copPositionInWorld = new YoFramePoint3D(namePrefix + "CoPPositionsInWorld", worldFrame, registry);
         copPositionsInWorld.put(foot, copPositionInWorld);

         YoFrameVector3D footVelocityInWorld = new YoFrameVector3D(namePrefix + "VelocityInWorld", worldFrame, registry);
         footVelocitiesInWorld.put(foot, footVelocityInWorld);

         footTwistsInWorld.put(foot, new Twist());

         ReferenceFrame copFrame = new ReferenceFrame("copFrame", soleFrame)
         {
            private final Vector3D copOffset = new Vector3D();
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.setIdentity();
               copOffset.set(copFilteredInFootFrame);
               transformToParent.setTranslation(copOffset);
            }
         };
         copFrames.put(foot, copFrame);
      }

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            for (int i = 0; i < feetRigidBodies.size(); i++)
            {
               RigidBody foot = feetRigidBodies.get(i);
               String sidePrefix = foot.getName();
               YoGraphicPosition copInWorld = new YoGraphicPosition(sidePrefix + "StateEstimatorCoP", copPositionsInWorld.get(foot), 0.005, YoAppearance.DeepPink());
               YoArtifactPosition artifact = copInWorld.createArtifact();
               artifact.setVisible(false);
               yoGraphicsListRegistry.registerArtifact("StateEstimator", artifact);
            }
         }
      }

      parentRegistry.addChild(registry);
   }

   private void reset()
   {
      rootJointPosition.setToZero();
   }

   /**
    * Estimates the foot positions corresponding to the given pelvisPosition
    * @param pelvisPosition
    */
   public void initialize(FramePoint3D pelvisPosition)
   {
      updateKinematics();
      setPelvisPosition(pelvisPosition);

      for (int i = 0; i < feetRigidBodies.size(); i++)
      {
         RigidBody foot = feetRigidBodies.get(i);
         updateUntrustedFootPosition(foot, pelvisPosition);
      }
      kinematicsIsUpToDate.set(false);
   }

   /**
    * Estimates the pelvis position and linear velocity using the leg kinematics
    * @param trustedFoot is the foot used to estimates the pelvis state
    * @param numberOfTrustedSides is only one or both legs used to estimate the pelvis state
    */
   private void updatePelvisWithKinematics(RigidBody trustedFoot, int numberOfTrustedFeet)
   {
      double scaleFactor = 1.0 / numberOfTrustedFeet;

      tempPosition.set(footToRootJointPositions.get(trustedFoot));
      tempPosition.scale(scaleFactor);
      rootJointPosition.add(tempPosition);
      tempPosition.set(footPositionsInWorld.get(trustedFoot));
      tempPosition.scale(scaleFactor);
      rootJointPosition.add(tempPosition);

      YoFramePoint3D rootJointPositionPerFoot = rootJointPositionsPerFoot.get(trustedFoot);
      rootJointPositionPerFoot.set(footPositionsInWorld.get(trustedFoot));
      rootJointPositionPerFoot.add(footToRootJointPositions.get(trustedFoot));

      tempFrameVector.setIncludingFrame(footVelocitiesInWorld.get(trustedFoot));
      tempFrameVector.scale(scaleFactor * alphaRootJointLinearVelocityNewTwist.getValue());
      rootJointLinearVelocityNewTwist.sub(tempFrameVector);
   }

   /**
    * updates the position of a swinging foot
    * @param swingingFoot a foot in swing
    * @param pelvisPosition the current pelvis position
    */
   private void updateUntrustedFootPosition(RigidBody swingingFoot, FramePoint3D pelvisPosition)
   {
      YoFramePoint3D footPositionInWorld = footPositionsInWorld.get(swingingFoot);
      footPositionInWorld.set(pelvisPosition);
      footPositionInWorld.sub(footToRootJointPositions.get(swingingFoot));

      copPositionsInWorld.get(swingingFoot).set(footPositionInWorld);

      copsRawInFootFrame.get(swingingFoot).setToZero();
      copsFilteredInFootFrame.get(swingingFoot).setToZero();
   }

   private void updateTrustedFootPosition(RigidBody trustedFoot)
   {
      YoFramePoint3D footPositionInWorld = footPositionsInWorld.get(trustedFoot);
      AlphaFilteredYoFrameVector footToRootJointPosition = footToRootJointPositions.get(trustedFoot);

      if (trustCoPAsNonSlippingContactPoint.getValue())
      {
         tempPosition.setIncludingFrame(footToRootJointPosition);
         tempFramePoint.setIncludingFrame(rootJointPosition);
         tempFramePoint.sub(tempPosition); // New foot position
         tempPosition.set(footPositionInWorld); // Previous foot position
         tempFrameVector.sub(tempFramePoint, tempPosition); // Delta from previous to new foot position
         copPositionsInWorld.get(trustedFoot).add(tempFrameVector); // New CoP position
      }

      footPositionInWorld.set(rootJointPosition);
      footPositionInWorld.sub(footToRootJointPosition);
   }

   /**
    * Compute the foot CoP. The CoP is the point on the support foot trusted to be not slipping.
    * @param trustedSide
    * @param footSwitch
    */
   private void updateCoPPosition(RigidBody trustedFoot)
   {
      AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedFoot);
      ReferenceFrame footFrame = soleFrames.get(trustedFoot);

      if (trustCoPAsNonSlippingContactPoint.getValue())
      {

         if (useControllerDesiredCoP.getValue())
            centerOfPressureDataHolderFromController.getCenterOfPressure(tempCoP2d, trustedFoot);
         else
            footSwitches.get(trustedFoot).computeAndPackCoP(tempCoP2d);

         if (tempCoP2d.containsNaN())
         {
            tempCoP2d.setToZero();
         }
         else
         {
            FrameConvexPolygon2D footPolygon = footPolygons.get(trustedFoot);
            boolean isCoPInsideFoot = footPolygon.isPointInside(tempCoP2d);
            if (!isCoPInsideFoot)
            {
               if (footSwitches.get(trustedFoot).computeFootLoadPercentage() > 0.2)
               {
                  FrameLineSegment2D footCenterCoPLineSegment = footCenterCoPLineSegments.get(trustedFoot);
                  footCenterCoPLineSegment.set(footFrame, 0.0, 0.0, tempCoP2d.getX(), tempCoP2d.getY());
                  int intersections = footPolygon.intersectionWith(footCenterCoPLineSegment, intersectionPoints[0], intersectionPoints[1]);

                  if (intersections == 0)
                  {
                     System.out.println("In " + getClass().getSimpleName() + ": Found no solution for the CoP projection.");
                     tempCoP2d.setToZero(footFrame);
                  }
                  else
                  {
                     tempCoP2d.set(intersectionPoints[0]);

                     if (intersections == 2)
                        System.out.println("In " + getClass().getSimpleName() + ": Found two solutions for the CoP projection.");
                  }


               }
               else // If foot barely loaded and actual CoP outside, then don't update the raw CoP right below
               {
                  tempCoP2d.setIncludingFrame(copsRawInFootFrame.get(trustedFoot));
               }
            }
         }

         copsRawInFootFrame.get(trustedFoot).set(tempCoP2d);

         tempCoPOffset.setIncludingFrame(footFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
         copFilteredInFootFrame.update();
         tempCoPOffset.setIncludingFrame(footFrame, copFilteredInFootFrame.getX() - tempCoPOffset.getX(), copFilteredInFootFrame.getY() - tempCoPOffset.getY(), 0.0);

         tempCoPOffset.changeFrame(worldFrame);
         copPositionsInWorld.get(trustedFoot).add(tempCoPOffset);
      }
      else
      {
         tempCoP2d.setToZero();
      }
   }

   /**
    * Assuming the CoP is not moving, the foot position can be updated. That way we can see if the foot is on the edge.
    * @param plantedFoot
    */
   private void correctFootPositionsUsingCoP(RigidBody plantedFoot)
   {
      if (!useControllerDesiredCoP.getValue())
         return;

      AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(plantedFoot);
      tempCoPOffset.setIncludingFrame(copFilteredInFootFrame.getReferenceFrame(), copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);

      tempCoPOffset.changeFrame(worldFrame);

      YoFramePoint3D footPositionInWorld = footPositionsInWorld.get(plantedFoot);
      footPositionInWorld.set(copPositionsInWorld.get(plantedFoot));
      footPositionInWorld.sub(tempCoPOffset);
   }

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state
    */
   public void updateKinematics()
   {
      reset();
      updateKinematicsNewTwist();

      for (int i = 0; i < feetRigidBodies.size(); i++)
      {
         RigidBody foot = feetRigidBodies.get(i);
         tempFramePoint.setToZero(rootJointFrame);
         tempFramePoint.changeFrame(soleFrames.get(foot));

         tempFrameVector.setIncludingFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToRootJointPositions.get(foot).update(tempFrameVector);
      }

      kinematicsIsUpToDate.set(true);
   }

   private final Twist tempRootBodyTwist = new Twist();

   private void updateKinematicsNewTwist()
   {
      rootJoint.getJointTwist(tempRootBodyTwist);

      tempFrameVector.setIncludingFrame(rootJointLinearVelocityNewTwist);
      tempFrameVector.changeFrame(tempRootBodyTwist.getExpressedInFrame());

      tempRootBodyTwist.setLinearPart(tempFrameVector);
      rootJoint.setJointTwist(tempRootBodyTwist);
      rootJoint.updateFramesRecursively();

      for (int i = 0; i < feetRigidBodies.size(); i++)
      {
         RigidBody foot = feetRigidBodies.get(i);
         Twist footTwistInWorld = footTwistsInWorld.get(foot);
         YoFrameVector3D footVelocityInWorld = footVelocitiesInWorld.get(foot);

         foot.getBodyFixedFrame().getTwistOfFrame(footTwistInWorld);
         footTwistInWorld.changeBodyFrameNoRelativeTwist(soleFrames.get(foot));
         footTwistInWorld.changeFrame(soleFrames.get(foot));

         tempCoP2d.setIncludingFrame(this.copsFilteredInFootFrame.get(foot));
         tempCoP.setIncludingFrame(tempCoP2d, 0.0);
         footTwistInWorld.changeFrame(footTwistInWorld.getBaseFrame());
         tempCoP.changeFrame(footTwistInWorld.getExpressedInFrame());
         footTwistInWorld.getLinearVelocityOfPointFixedInBodyFrame(tempFrameVector, tempCoP);

         tempFrameVector.changeFrame(worldFrame);
         footVelocityInWorld.set(tempFrameVector);
      }
   }

   public void updateFeetPositionsWhenTrustingIMUOnly(FramePoint3D pelvisPosition)
   {
      for (int i = 0; i < feetRigidBodies.size(); i++)
      {
         RigidBody foot = feetRigidBodies.get(i);
         updateUntrustedFootPosition(foot, pelvisPosition);
      }
   }

   public void estimatePelvisLinearState(List<RigidBody> trustedFeet, List<RigidBody> unTrustedFeet, FramePoint3D pelvisPosition)
   {
      if (!kinematicsIsUpToDate.getBooleanValue())
         throw new RuntimeException("Leg kinematics needs to be updated before trying to estimate the pelvis position/linear velocity.");

      for(int i = 0; i < trustedFeet.size(); i++)
      {
         RigidBody trustedFoot = trustedFeet.get(i);
         updateCoPPosition(trustedFoot);
         correctFootPositionsUsingCoP(trustedFoot);
         updatePelvisWithKinematics(trustedFoot, trustedFeet.size());
      }

      rootJointLinearVelocityBacklashKinematics.update();

      if (correctTrustedFeetPositions.getValue())
      {
         for(int i = 0; i < trustedFeet.size(); i++)
         {
            RigidBody trustedFoot = trustedFeet.get(i);
            updateTrustedFootPosition(trustedFoot);
         }
      }

      for(int i = 0; i < unTrustedFeet.size(); i++)
      {
         RigidBody unTrustedFoot = unTrustedFeet.get(i);
         updateUntrustedFootPosition(unTrustedFoot, pelvisPosition);
      }

      kinematicsIsUpToDate.set(false);
   }

   public void setPelvisPosition(FramePoint3D pelvisPosition)
   {
      rootJointPosition.set(pelvisPosition);
   }

   public void getRootJointPositionAndVelocity(FramePoint3D positionToPack, FrameVector3D linearVelocityToPack)
   {
      getPelvisPosition(positionToPack);
      getPelvisVelocity(linearVelocityToPack);
   }

   public void getPelvisPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(rootJointPosition);
   }

   public void getPelvisVelocity(FrameVector3D linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(rootJointLinearVelocityNewTwist);
   }

   public void getFootToPelvisPosition(FramePoint3D positionToPack, RigidBody foot)
   {
      positionToPack.setIncludingFrame(footToRootJointPositions.get(foot));
   }
}
