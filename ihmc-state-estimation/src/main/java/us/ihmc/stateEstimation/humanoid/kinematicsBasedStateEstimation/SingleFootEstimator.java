package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
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
   private final RigidBodyBasics foot;

   private final ReferenceFrame rootJointFrame;
   private final ReferenceFrame soleFrame;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanProvider trustCoPAsNonSlippingContactPoint;
   private final BooleanProvider assumeTrustedFootAtZeroHeight;

   private final YoFrameVector3D footVelocityInWorld;
   private final AlphaFilteredYoFrameVector3D footToRootJointPosition;
   private final YoFrameVector3D footToRootJointLinearVelocity;
   private final YoFrameVector3D footToRootJointAngularVelocity;
   private final YoFramePoint3D footPositionInWorld;


   private final YoFramePoint3D copPositionInWorld;
   private final AlphaFilteredYoFramePoint2D copFilteredInFootFrame;
   private final YoFramePoint2D copRawInFootFrame;
   private final FrameConvexPolygon2D footPolygon;
   private final FrameLineSegment2D footCenterCoPLineSegment;
   private final FootSwitchInterface footSwitch;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;

   /** Debug variable */
   private final YoFramePoint3D rootJointPositionPerFoot;

   // Temp variables
   private final FramePoint2DBasics[] intersectionPoints = new FramePoint2DBasics[] {new FramePoint2D(), new FramePoint2D()};
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   private final Twist rootBodyTwistInFootFrame = new Twist();
   private final Twist footTwistInWorld = new Twist();


   public SingleFootEstimator(FloatingJointBasics rootJoint,
                              ContactablePlaneBody contactableFoot,
                              FootSwitchInterface footSwitch,
                              DoubleProvider footToRootJointPositionBreakFrequency,
                              DoubleProvider copFilterBreakFrequency,
                              BooleanProvider trustCoPAsNonSlippingContactPoint,
                              BooleanProvider assumeTrustedFootAtZeroHeight,
                              CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                              double estimatorDT,
                              YoRegistry registry)
   {
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.footSwitch = footSwitch;
      this.centerOfPressureDataHolderFromController = centerOfPressureDataHolderFromController;
      this.trustCoPAsNonSlippingContactPoint = trustCoPAsNonSlippingContactPoint;
      this.assumeTrustedFootAtZeroHeight = assumeTrustedFootAtZeroHeight;
      foot = contactableFoot.getRigidBody();
      soleFrame = contactableFoot.getContactFrame();

      String namePrefix = foot.getName();

      DoubleProvider alphaFoot = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(footToRootJointPositionBreakFrequency.getValue(), estimatorDT);
      footToRootJointPosition = new AlphaFilteredYoFrameVector3D(namePrefix + "FootToRootJointPosition", "", registry, alphaFoot, worldFrame);
      footToRootJointLinearVelocity = new YoFrameVector3D(namePrefix + "FootToRootJointLinearVelocity", worldFrame, registry);
      footToRootJointAngularVelocity = new YoFrameVector3D(namePrefix + "FootToRootJointAngularVelocity", worldFrame, registry);
      rootJointPositionPerFoot = new YoFramePoint3D(namePrefix + "BasedRootJointPosition", worldFrame, registry);
      footPositionInWorld = new YoFramePoint3D(namePrefix + "FootPositionInWorld", worldFrame, registry);
      footPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2D()));
      footCenterCoPLineSegment = new FrameLineSegment2D(soleFrame);
      copRawInFootFrame = new YoFramePoint2D(namePrefix + "CoPRawInFootFrame", soleFrame, registry);

      DoubleProvider alphaCop = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(copFilterBreakFrequency.getValue(), estimatorDT);
      copFilteredInFootFrame = new AlphaFilteredYoFramePoint2D(namePrefix + "CoPFilteredInFootFrame", "", registry, alphaCop, copRawInFootFrame);
      copFilteredInFootFrame.update(0.0, 0.0);
      copPositionInWorld = new YoFramePoint3D(namePrefix + "CoPPositionsInWorld", worldFrame, registry);
      footVelocityInWorld = new YoFrameVector3D(namePrefix + "VelocityInWorld", worldFrame, registry);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return YoGraphicDefinitionFactory.newYoGraphicPoint2D(foot.getName() + "StateEstimatorCoP",
                                                            copPositionInWorld,
                                                            0.01,
                                                            ColorDefinitions.DeepPink(),
                                                            DefaultPoint2DGraphic.CIRCLE);
   }

   public void initialize()
   {
      footToRootJointPosition.reset();
      copFilteredInFootFrame.reset();
      copFilteredInFootFrame.update(0.0, 0.0);
      footVelocityInWorld.setToZero();
   }

   /**
    * Estimates the pelvis position and linear velocity using the leg kinematics
    *
    * @param trustedFoot          is the foot used to estimates the pelvis state
    * @param numberOfTrustedSides is only one or both legs used to estimate the pelvis state
    */
   public void updatePelvisWithKinematics(int numberOfTrustedFeet,
                                           double alphaVelocityUpdate,
                                           FixedFramePoint3DBasics rootJointPosition,
                                           FixedFrameVector3DBasics rootJointLinearVelocity)
   {
      double scaleFactor = 1.0 / numberOfTrustedFeet;

      rootJointPositionPerFoot.add(footPositionInWorld, footToRootJointPosition);
      rootJointPosition.scaleAdd(scaleFactor, rootJointPositionPerFoot, rootJointPosition);

      // Based on the previous estimate of the root joint velocity, we computed the foot velocity, which is located at the center of pressure. However, if
      // we assume that the center of pressure isn't moving, we should subtract this velocity from the kinematics to make the velocity zero at that point.
      rootJointLinearVelocity.scaleAdd(-scaleFactor * alphaVelocityUpdate, footVelocityInWorld, rootJointLinearVelocity);
   }

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state. This includes the vector from the foot to the root, as well as
    * the velocity of the foot at the contact point.
    */
   public void updateKinematics(TwistReadOnly rootBodyTwist)
   {
      // Compute the position of the root joint relative to the foot.
      tempFramePoint.setToZero(rootJointFrame);
      tempFramePoint.changeFrame(soleFrame);

      // Store the vector from the foot to the root, and make sure it's expressed in the world frame.
      tempFrameVector.setIncludingFrame(tempFramePoint);
      tempFrameVector.changeFrame(worldFrame);

      // Update the filtered offset vector.
      footToRootJointPosition.update(tempFrameVector);

      // Compute the twist of the root body, expressed in the foot frame.
      rootBodyTwistInFootFrame.setIncludingFrame(rootBodyTwist);
      rootBodyTwistInFootFrame.setBaseFrame(worldFrame);
      rootBodyTwistInFootFrame.changeFrame(foot.getBodyFixedFrame());

      // Get the twist relative to the root. This is purely based on kinematics.
      foot.getBodyFixedFrame().getTwistRelativeToOther(rootJointFrame, footTwistInWorld);
      // Update some debug variables for tracking the twist of the foot relative the root
      footToRootJointLinearVelocity.setMatchingFrame(footTwistInWorld.getLinearPart());
      footToRootJointAngularVelocity.setMatchingFrame(footTwistInWorld.getAngularPart());
      // Update the twist of the foot in the world, by adding the root twist to it.
      footTwistInWorld.add(rootBodyTwistInFootFrame);
      footTwistInWorld.setBodyFrame(soleFrame);
      footTwistInWorld.changeFrame(worldFrame);

      // Compute the velocity at the CoP position, which we want to be zero.
      footTwistInWorld.getLinearVelocityAt(copPositionInWorld, footVelocityInWorld);
   }

   /**
    * updates the position of a swinging foot
    *
    * @param swingingFoot   a foot in swing
    * @param pelvisPosition the current pelvis position
    */
   public void updateUntrustedFootPosition(FramePoint3DReadOnly pelvisPosition, boolean useControllerDesiredCoP)
   {
      footPositionInWorld.sub(pelvisPosition, footToRootJointPosition);
      copPositionInWorld.set(footPositionInWorld);

      if (footSwitch.hasFootHitGroundFiltered())
      {
         // The foot is still on the ground. This means that we still have a valid CoP, and may trust it again very soon. We shouldn't zero out the cop
         // position, as that causes a bunch of discrete jumps as to where the filtered value is.
         if (useControllerDesiredCoP)
            centerOfPressureDataHolderFromController.getCenterOfPressure(tempCoP2d, foot);
         else
            footSwitch.getCenterOfPressure(tempCoP2d);
         tempCoP2d.checkReferenceFrameMatch(copRawInFootFrame.getReferenceFrame());
         copRawInFootFrame.set(tempCoP2d);
         copFilteredInFootFrame.update();

         // Cache the frame
         ReferenceFrame oldFrame = tempCoPOffset.getReferenceFrame();
         // Compute the cop position offset in world.
         tempCoPOffset.setIncludingFrame(soleFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
         tempCoPOffset.changeFrame(worldFrame);

         if (copRawInFootFrame.containsNaN() || copFilteredInFootFrame.containsNaN() || tempCoPOffset.containsNaN())
         {
            // Something was corrupted, fallback to center of the foot.
            copRawInFootFrame.setToZero();
            copFilteredInFootFrame.setToZero();
         }
         else
         {
            // Offset the cop position in the world based on the foot.
            copPositionInWorld.add(tempCoPOffset);
         }

         // Set the frame back so it's not busted elsewhere.
         tempCoPOffset.setReferenceFrame(oldFrame);
      }
      else
      {
         copRawInFootFrame.setToZero();
         copFilteredInFootFrame.setToZero();
      }
   }

   public void updateTrustedFootPosition(FramePoint3DReadOnly rootJointPosition)
   {
      if (trustCoPAsNonSlippingContactPoint.getValue())
      {
         tempFrameVector.setIncludingFrame(rootJointPosition);
         tempFrameVector.sub(footToRootJointPosition); // New foot position
         tempFrameVector.sub(footPositionInWorld); // Delta from previous to new foot position
         copPositionInWorld.add(tempFrameVector); // New CoP position
      }

      footPositionInWorld.sub(rootJointPosition, footToRootJointPosition);
   }
   private final FramePoint2D tempCoP2d = new FramePoint2D();

   private final FrameVector3D tempCoPOffset = new FrameVector3D();

   /**
    * Compute the foot position in the world
    */
   public void computeFootPositionInWorld(boolean useControllerDesiredCoP)
   {
      if (trustCoPAsNonSlippingContactPoint.getValue())
      {
         computeCoPPositionInFootFrame(useControllerDesiredCoP, copRawInFootFrame, copRawInFootFrame);

         // First, store the CoP position from the previous tick.
         tempCoPOffset.setIncludingFrame(soleFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
         // Update the filtered value, which is computed from the raw value
         copFilteredInFootFrame.update();
         // Get the distance that the CoP moved within the foot in the last update, which is the newly filtered value minus the previously stored value.
         tempCoPOffset.setIncludingFrame(soleFrame,
                                         copFilteredInFootFrame.getX() - tempCoPOffset.getX(),
                                         copFilteredInFootFrame.getY() - tempCoPOffset.getY(),
                                         0.0);

         // If the cop position "isn't moving", it should only move by the amount that it moved within the foot. So get the amount in the foot in the world
         // frame, and add it to the CoP position to get the updated value.
         tempCoPOffset.changeFrame(worldFrame);
         copPositionInWorld.add(tempCoPOffset);

         if (assumeTrustedFootAtZeroHeight.getValue())
         {  // If the CoP position height is held at zero, override hte updated value.
            copPositionInWorld.setZ(0.0);
         }

         correctFootPositionsUsingCoP();
      }
      else if (assumeTrustedFootAtZeroHeight.getValue())
      {
         // We're assuming the foot isn't moving in XY, so leave those alone. Override the position in Z to zero.
         footPositionInWorld.setZ(0.0);
      }
   }

   private void computeCoPPositionInFootFrame(boolean useControllerDesiredCoP,
                                              FramePoint2DReadOnly previousCoPPosition,
                                              FixedFramePoint2DBasics copPositionToPack)
   {
      if (useControllerDesiredCoP)
         centerOfPressureDataHolderFromController.getCenterOfPressure(tempCoP2d, foot);
      else
         footSwitch.getCenterOfPressure(tempCoP2d);

      // We need to update the CoP, making sure it's in a good state.
      if (tempCoP2d.containsNaN())
      {  // If the value contains NaN, it's likely because the foot isn't in contact enough to return a valid CoP. In this event, we can assume the
         // CoP is located in the middle of the foot.
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
                  LogTools.warn("Found no solution for the CoP projection.");
                  tempCoP2d.setToZero(soleFrame);
               }
               else
               {
                  tempCoP2d.set(intersectionPoints[0]);

                  if (intersections == 2)
                     LogTools.warn("Found two solutions for the CoP projection.");
               }
            }
            else // If foot barely loaded and actual CoP outside, then don't update the raw CoP right below
            {
               tempCoP2d.setIncludingFrame(previousCoPPosition);
            }
         }
      }

      copPositionToPack.set(tempCoP2d);
   }

   /**
    * Assuming the CoP is not moving, the foot position can be updated. That way we can see if the foot is on the edge.
    */
   private void correctFootPositionsUsingCoP()
   {
      tempCoPOffset.setIncludingFrame(copFilteredInFootFrame, 0.0);
      tempCoPOffset.changeFrame(worldFrame);
      footPositionInWorld.sub(copPositionInWorld, tempCoPOffset);
   }

}
