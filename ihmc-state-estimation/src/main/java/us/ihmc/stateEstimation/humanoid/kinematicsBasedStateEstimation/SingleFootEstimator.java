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
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFramePoint2D;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

class SingleFootEstimator implements SCS2YoGraphicHolder
{
   private final RigidBodyBasics foot;

   private final ReferenceFrame rootJointFrame;
   private final ReferenceFrame soleFrame;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector3D footVelocityInWorld;
   private final AlphaFilteredYoFrameVector3D footToRootJointPosition;
   private final YoFramePoint3D footPositionInWorld;
   /** Debug variable */
   private final YoFramePoint3D rootJointPositionPerFoot;
   private final YoFramePoint3D copPositionInWorld;
   private final AlphaFilteredYoFramePoint2D copFilteredInFootFrame;
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
      soleFrame = contactableFoot.getContactFrame();

      String namePrefix = foot.getName();

      DoubleProvider alphaFoot = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(footToRootJointPositionBreakFrequency.getValue(),
                                                                                                       estimatorDT);
      footToRootJointPosition = new AlphaFilteredYoFrameVector3D(namePrefix + "FootToRootJointPosition", "", registry, alphaFoot, worldFrame);
      rootJointPositionPerFoot = new YoFramePoint3D(namePrefix + "BasedRootJointPosition", worldFrame, registry);
      footPositionInWorld = new YoFramePoint3D(namePrefix + "FootPositionInWorld", worldFrame, registry);
      footPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2D()));
      footCenterCoPLineSegment = new FrameLineSegment2D(soleFrame);
      copRawInFootFrame = new YoFramePoint2D(namePrefix + "CoPRawInFootFrame", soleFrame, registry);

      DoubleProvider alphaCop = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(copFilterBreakFrequency.getValue(), estimatorDT);
      copFilteredInFootFrame = new AlphaFilteredYoFramePoint2D(namePrefix + "CoPFilteredInFootFrame", "", registry, alphaCop, copRawInFootFrame);
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
   public void updatePelvisWithKinematics(int numberOfTrustedFeet,
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
   public void updateUntrustedFootPosition(FramePoint3DReadOnly pelvisPosition)
   {
      footPositionInWorld.sub(pelvisPosition, footToRootJointPosition);

      copPositionInWorld.set(footPositionInWorld);

      copRawInFootFrame.setToZero();
      copFilteredInFootFrame.setToZero();
   }

   private final FrameVector3D tempFrameVector = new FrameVector3D();

   public void updateTrustedFootPosition(boolean trustCoPAsNonSlippingContactPoint, FramePoint3DReadOnly rootJointPosition)
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

   public void setCoPZPositionInWorld(boolean useControllerDesiredCoP, double zPosition)
   {
      updateCoPPosition(true, useControllerDesiredCoP);
      copPositionInWorld.setZ(zPosition);
      correctFootPositionsUsingCoP(true);
   }

   public void setFootZPositionInWorld(double zPosition)
   {
      footPositionInWorld.setZ(zPosition);
   }

   private final FramePoint2D tempCoP2d = new FramePoint2D();
   private final FrameVector3D tempCoPOffset = new FrameVector3D();

   /**
    * Compute the foot CoP. The CoP is the point on the support foot trusted to be not slipping.
    *
    * @param trustedSide
    * @param footSwitch
    */
   public void updateCoPPosition(boolean trustCoPAsNonSlippingContactPoint, boolean useControllerDesiredCoP)
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
   public void correctFootPositionsUsingCoP(boolean trustCoPAsNonSlippingContactPoint)
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

   public void updateFootLinearVelocityInWorld(TwistReadOnly rootBodyTwist)
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

   public FrameVector3DReadOnly getFootToRootJointPosition()
   {
      return footToRootJointPosition;
   }

   public FrameVector3DReadOnly getFootVelocityInWorld()
   {
      return footVelocityInWorld;
   }
}
