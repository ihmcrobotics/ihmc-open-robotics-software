package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

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
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFramePoint2D;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

class SingleFootEstimator implements SCS2YoGraphicHolder
{
   private static boolean USE_IMU_DATA = true;

   private final RigidBodyBasics foot;

   private final ReferenceFrame rootJointFrame;
   private final ReferenceFrame soleFrame;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector3D footIKLinearVelocityInWorld;
   private final YoFrameVector3D footIKAngularVelocityInWorld;
   private final YoFrameVector3D footIMULinearVelocityInWorld;
   private final YoFrameVector3D footIMUAngularVelocityInWorld;
   private final YoFrameVector3D footFusedLinearVelocityInWorld;
   private final YoFrameVector3D footFusedAngularVelocityInWorld;

   private final YoFrameVector3D copVelocityInWorld;
   private final AlphaFilteredYoFrameVector3D footToRootJointPosition;
   private final YoFramePoint3D footPositionInWorld;
   /** Debug variable */
   private final YoFramePoint3D rootJointPositionPerFoot;

   private final YoFramePoint3D copPositionInWorld;
   private final YoFramePoint2D previousCoPInFootFrame;

   private final AlphaFilteredYoFramePoint2D copFilteredInFootFrame;
   private final YoFramePoint2D copRawInFootFrame;
   private final FrameConvexPolygon2D footPolygon;
   private final FrameLineSegment2D footCenterCoPLineSegment;
   private final FootSwitchInterface footSwitch;
   private final IMUSensorReadOnly footIMU;
   private final IMUBiasProvider imuBiasProvider;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;
   private final DoubleProvider footAlphaLeakIMUOnly;
   private final DoubleProvider imuAgainstKinematicsForVelocityBreakFrequency;
   private final DoubleProvider imuAgainstKinematicsForPositionBreakFrequency;
   private final BooleanProvider cancelGravityFromAccelerationMeasurement;
   private final FrameVector3DReadOnly gravityVector;
   private final double estimatorDT;

   private final FramePoint2DBasics[] intersectionPoints = new FramePoint2DBasics[] {new FramePoint2D(), new FramePoint2D()};

   public SingleFootEstimator(FloatingJointBasics rootJoint,
                              ContactablePlaneBody contactableFoot,
                              FootSwitchInterface footSwitch,
                              IMUSensorReadOnly footIMU,
                              IMUBiasProvider imuBiasProvider,
                              DoubleProvider footToRootJointPositionBreakFrequency,
                              DoubleProvider copFilterBreakFrequency,
                              CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                              BooleanProvider cancelGravityFromAccelerationMeasurement,
                              FrameVector3DReadOnly gravityVector,
                              DoubleProvider imuAgainstKinematicsForPositionBreakFrequency,
                              DoubleProvider imuAgainstKinematicsForVelocityBreakFrequency,
                              DoubleProvider footAlphaLeakIMUOnly,
                              double estimatorDT,
                              YoRegistry registry)
   {
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.footSwitch = footSwitch;
      this.footIMU = footIMU;
      this.imuBiasProvider = imuBiasProvider;
      this.centerOfPressureDataHolderFromController = centerOfPressureDataHolderFromController;
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;
      this.gravityVector = gravityVector;
      this.imuAgainstKinematicsForPositionBreakFrequency = imuAgainstKinematicsForPositionBreakFrequency;
      this.imuAgainstKinematicsForVelocityBreakFrequency = imuAgainstKinematicsForVelocityBreakFrequency;
      this.footAlphaLeakIMUOnly = footAlphaLeakIMUOnly;
      this.estimatorDT = estimatorDT;
      foot = contactableFoot.getRigidBody();
      soleFrame = contactableFoot.getContactFrame();

      String namePrefix = foot.getName();

      DoubleProvider alphaFoot = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(footToRootJointPositionBreakFrequency.getValue(), estimatorDT);
      footToRootJointPosition = new AlphaFilteredYoFrameVector3D(namePrefix + "FootToRootJointPosition", "", registry, alphaFoot, worldFrame);
      rootJointPositionPerFoot = new YoFramePoint3D(namePrefix + "BasedRootJointPosition", worldFrame, registry);
      footPositionInWorld = new YoFramePoint3D(namePrefix + "FootPositionInWorld", worldFrame, registry);
      footPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2D()));
      footCenterCoPLineSegment = new FrameLineSegment2D(soleFrame);
      copRawInFootFrame = new YoFramePoint2D(namePrefix + "CoPRawInFootFrame", soleFrame, registry);
      previousCoPInFootFrame = new YoFramePoint2D(namePrefix + "PreviousCoPInFootFrame", soleFrame, registry);

      footIKLinearVelocityInWorld = new YoFrameVector3D(namePrefix + "IKLinearVelocityInWorld", worldFrame, registry);
      footIKAngularVelocityInWorld = new YoFrameVector3D(namePrefix + "IKAngularVelocityInWorld", worldFrame, registry);
      footIMULinearVelocityInWorld = new YoFrameVector3D(namePrefix + "IMULinearVelocityInWorld", worldFrame, registry);
      footIMUAngularVelocityInWorld = new YoFrameVector3D(namePrefix + "IMUAngularVelocityInWorld", worldFrame, registry);
      footFusedLinearVelocityInWorld = new YoFrameVector3D(namePrefix + "FusedLinearVelocityInWorld", worldFrame, registry);
      footFusedAngularVelocityInWorld = new YoFrameVector3D(namePrefix + "FusedAngularVelocityInWorld", worldFrame, registry);

      DoubleProvider alphaCop = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(copFilterBreakFrequency.getValue(), estimatorDT);
      copFilteredInFootFrame = new AlphaFilteredYoFramePoint2D(namePrefix + "CoPFilteredInFootFrame", "", registry, alphaCop, copRawInFootFrame);
      copFilteredInFootFrame.update(0.0, 0.0);
      copPositionInWorld = new YoFramePoint3D(namePrefix + "CoPPositionInWorld", worldFrame, registry);
      copVelocityInWorld = new YoFrameVector3D(namePrefix + "CoPVelocityInWorld", worldFrame, registry);
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
      return YoGraphicDefinitionFactory.newYoGraphicPoint2D(foot.getName() + "StateEstimatorCoP", copPositionInWorld,
                                                            0.01,
                                                            ColorDefinitions.DeepPink(),
                                                            DefaultPoint2DGraphic.CIRCLE);
   }

   public void initialize()
   {
      footToRootJointPosition.reset();
      copFilteredInFootFrame.reset();
      copFilteredInFootFrame.update(0.0, 0.0);
      copVelocityInWorld.setToZero();
   }

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final Twist tempRootBodyTwist = new Twist();
   private final Twist footTwistInWorld = new Twist();
   private final Twist imuTwist = new Twist();


   private final FrameVector3D tempFrameVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state. This should be called before calling the other objectives.
    */
   public void updateKinematics(TwistReadOnly rootBodyTwist)
   {
      computeFootTwistInWorld(rootBodyTwist);
      computeCoPLinearVelocityInWorld(copVelocityInWorld);

      tempFramePoint.setToZero(rootJointFrame);
      tempFramePoint.changeFrame(soleFrame);

      tempFrameVector.setIncludingFrame(tempFramePoint);
      tempFrameVector.changeFrame(worldFrame);

      footToRootJointPosition.update(tempFrameVector);
   }

   /**
    * One of the options for computing the
    * @param useControllerDesiredCoP
    * @param zPosition
    */
   public void updateCoPAndFootSettingZ(boolean trustCoPAsNonSlippingContactPoint, boolean useControllerDesiredCoP, double zPosition)
   {
      updateCoPPosition(trustCoPAsNonSlippingContactPoint, useControllerDesiredCoP);
      copPositionInWorld.setZ(zPosition);
      correctFootPositionsUsingCoP(trustCoPAsNonSlippingContactPoint);
   }

   public void updateCoPAndFootPosition(boolean trustCoPAsNonSlippingContactPoint, boolean useControllerDesiredCoP)
   {
      updateCoPPosition(trustCoPAsNonSlippingContactPoint, useControllerDesiredCoP);
      correctFootPositionsUsingCoP(trustCoPAsNonSlippingContactPoint);
   }

   public void setFootZPositionInWorld(double zPosition)
   {
      footPositionInWorld.setZ(zPosition);
   }


   /**
    * Estimates the pelvis position and linear velocity using the leg kinematics
    *
    * @param trustedFoot          is the foot used to estimates the pelvis state
    * @param numberOfTrustedSides is only one or both legs used to estimate the pelvis state
    */
   public void correctPelvisFromKinematics(int numberOfTrustedFeet,
                                           double alphaVelocityUpdate,
                                           FixedFramePoint3DBasics rootJointPosition,
                                           FixedFrameVector3DBasics rootJointLinearVelocity)
   {
      double scaleFactor = 1.0 / numberOfTrustedFeet;

      rootJointPositionPerFoot.add(footPositionInWorld, footToRootJointPosition);
      rootJointPosition.scaleAdd(scaleFactor, rootJointPositionPerFoot, rootJointPosition);

      rootJointLinearVelocity.scaleAdd(-scaleFactor * alphaVelocityUpdate, copVelocityInWorld, rootJointLinearVelocity);
   }

   /**
    * Corrects the position of the foot and center of pressure in the world using the root joint position and the forward kinematics from the root joint to the
    * foot. If the foot is not trusted, it sets the center of pressure from the foot. If it is trusted, but is non slipping, we shift the center of pressure
    * based on this correction.
    *
    * <p>
    *    This should be called once the fusion of the different feet to compute the root joint state has occurred.
    * </p>
    *
    * @param isTrusted whether the foot is trusted, and therefore the CoP is trusted.
    * @param trustCoPAsNonSlippingContactPoint whether to shift the CoP of a trusted foot based on the correction
    * @param rootJointPosition accurate root joint position to apply.
    */
   public void correctFootPositionFromRootJoint(boolean isTrusted, boolean trustCoPAsNonSlippingContactPoint, FramePoint3DReadOnly rootJointPosition)
   {
      tempPoint.set(footPositionInWorld);
      footPositionInWorld.sub(rootJointPosition, footToRootJointPosition);

      if (!isTrusted)
      {
         copPositionInWorld.set(footPositionInWorld);
         copRawInFootFrame.setToZero();
         copFilteredInFootFrame.setToZero();
      }
      else if (trustCoPAsNonSlippingContactPoint)
      {
         tempFrameVector.sub(footPositionInWorld, tempPoint); // Delta from previous to new foot position
         copPositionInWorld.add(tempFrameVector); // New CoP position
      }
   }

   public FrameVector3DReadOnly getFootToRootJointPosition()
   {
      return footToRootJointPosition;
   }

   public FrameVector3DReadOnly getCopVelocityInWorld()
   {
      return copVelocityInWorld;
   }

   public FrameVector3DReadOnly getFootLinearVelocityInWorld()
   {
      return footFusedLinearVelocityInWorld;
   }

   public FrameVector3DReadOnly getFootAngularVelocityInWorld()
   {
      return footFusedAngularVelocityInWorld;
   }

   private final FramePoint2D tempCoP2d = new FramePoint2D();
   private final FrameVector3D tempCoPOffset = new FrameVector3D();

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


         // Update the CoP values.
         previousCoPInFootFrame.set(copFilteredInFootFrame);
         copRawInFootFrame.set(tempCoP2d);
         copFilteredInFootFrame.update();

         // Update the change in the CoP from the previous tick.
         tempCoPOffset.setIncludingFrame(soleFrame,
                                         copFilteredInFootFrame.getX() - previousCoPInFootFrame.getX(),
                                         copFilteredInFootFrame.getY() - previousCoPInFootFrame.getY(),
                                         0.0);
         tempCoPOffset.changeFrame(worldFrame);
         // Apply this same change to the CoP in world.
         copPositionInWorld.add(tempCoPOffset);
      }
      else
      {
         tempCoP2d.setToZero(soleFrame);
         copRawInFootFrame.setToZero();
         copFilteredInFootFrame.setToZero();
         copPositionInWorld.setFromReferenceFrame(soleFrame);
      }

   }

   private void correctFootPositionsUsingCoP(boolean trustCoPAsNonSlippingContactPoint)
   {
      if (!trustCoPAsNonSlippingContactPoint)
         return;

      // Compute where the CoP position is in the world, according to the kinematics
      tempCoPOffset.setIncludingFrame(copFilteredInFootFrame, 0.0);
      tempCoPOffset.changeFrame(worldFrame);
      // Update where the foot must be, according to this CoP location
      footPositionInWorld.sub(copPositionInWorld, tempCoPOffset);
   }

   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();

   private void computeFootTwistInWorld(TwistReadOnly rootBodyTwist)
   {
      tempRootBodyTwist.setIncludingFrame(rootBodyTwist);
      tempRootBodyTwist.setBaseFrame(worldFrame);
      tempRootBodyTwist.changeFrame(foot.getBodyFixedFrame());

      foot.getBodyFixedFrame().getTwistRelativeToOther(rootJointFrame, footTwistInWorld);
      footTwistInWorld.add(tempRootBodyTwist);
      footTwistInWorld.setBodyFrame(soleFrame);
      footTwistInWorld.changeFrame(worldFrame);

      if (USE_IMU_DATA)
      {
         // Record the kinematic data
         footIKLinearVelocityInWorld.set(footTwistInWorld.getLinearPart());
         footIKAngularVelocityInWorld.set(footTwistInWorld.getAngularPart());

         // Get the previous IMU twist estimate from the fused data set
         imuTwist.setToZero(worldFrame);
         imuTwist.getLinearPart().set(footFusedLinearVelocityInWorld);
         imuTwist.getAngularPart().set(footFusedAngularVelocityInWorld);
         imuTwist.changeFrame(footIMU.getMeasurementFrame());

         // Compute the acceleration reading from the IMU
         linearAcceleration.setIncludingFrame(footIMU.getMeasurementFrame(), footIMU.getLinearAccelerationMeasurement());
         FrameVector3DReadOnly accelerationBias = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(footIMU);
         if (accelerationBias != null)
            linearAcceleration.sub(accelerationBias);

         // Update acceleration (minus gravity)
         if (cancelGravityFromAccelerationMeasurement.getValue())
         {
            linearAcceleration.changeFrame(worldFrame);
            linearAcceleration.add(gravityVector);
            linearAcceleration.changeFrame(footIMU.getMeasurementFrame());
         }

         // Compute the gyro reading from the IMU
         angularVelocity.setIncludingFrame(footIMU.getMeasurementFrame(), footIMU.getAngularVelocityMeasurement());
         FrameVector3DReadOnly gyroBias = imuBiasProvider.getAngularVelocityBiasInIMUFrame(footIMU);
         if (gyroBias != null)
            angularVelocity.sub(gyroBias);

         // Integrate the linear acceleration into the velocity measurement
         imuTwist.getLinearPart().scaleAdd(estimatorDT * footAlphaLeakIMUOnly.getValue(), linearAcceleration, imuTwist.getLinearPart());
         // Set the angular part from the gyro
         imuTwist.getAngularPart().set(angularVelocity);

         imuTwist.changeFrame(worldFrame);

         footIMUAngularVelocityInWorld.set(imuTwist.getAngularPart());
         footIMULinearVelocityInWorld.set(imuTwist.getLinearPart());

         // Fuse the IMU and kinematics data for velocity
         double alpha = AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForVelocityBreakFrequency.getValue(), estimatorDT);
         footFusedAngularVelocityInWorld.interpolate(footIKAngularVelocityInWorld, footIMUAngularVelocityInWorld, alpha);
         footFusedLinearVelocityInWorld.interpolate(footIKLinearVelocityInWorld, footIMULinearVelocityInWorld, alpha);
      }
      else
      {
         footIKLinearVelocityInWorld.setToNaN();
         footIKAngularVelocityInWorld.setToNaN();
         footIMULinearVelocityInWorld.setToNaN();
         footIMUAngularVelocityInWorld.setToNaN();

         footFusedLinearVelocityInWorld.set(footTwistInWorld.getLinearPart());
         footFusedAngularVelocityInWorld.set(footTwistInWorld.getAngularPart());
      }
   }

   private void computeCoPLinearVelocityInWorld(FixedFrameVector3DBasics footLinearVelocityToPack)
   {
      footTwistInWorld.getLinearPart().set(footFusedLinearVelocityInWorld);
      footTwistInWorld.getAngularPart().set(footFusedAngularVelocityInWorld);

      footTwistInWorld.getLinearVelocityAt(copPositionInWorld, footLinearVelocityToPack);
   }
}
