package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.filters.BacklashCompensatingVelocityYoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


/**
 * PelvisKinematicsBasedPositionCalculator estimates the pelvis position and linear velocity using the leg kinematics.
 * @author Sylvain
 *
 */
public class PelvisKinematicsBasedLinearStateCalculator
{
   private static final boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final TwistCalculator twistCalculator;

   private final SixDoFJoint rootJoint;
   private final SideDependentList<ContactablePlaneBody> bipedFeet;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame rootJointFrame;
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> copFrames = new SideDependentList<ReferenceFrame>();

   private final YoFramePoint rootJointPosition = new YoFramePoint("estimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocity = new YoFrameVector("estimatedRootJointVelocityWithKinematics", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocityTwist = new YoFrameVector("estimatedRootJointVelocityWithTwist", worldFrame, registry);
   private final BooleanYoVariable useTwistToComputeRootJointLinearVelocity = new BooleanYoVariable("useTwistToComputeRootJointLinearVelocity", registry);
   
   private final YoFrameVector leftFootVelocityInWorld = new YoFrameVector("leftFootVelocityInWorld", worldFrame, registry);
   private final YoFrameVector rightFootVelocityInWorld = new YoFrameVector("rightFootVelocityInWorld", worldFrame, registry);
   private final SideDependentList<YoFrameVector> footVelocitiesInWorld = new SideDependentList<YoFrameVector>(leftFootVelocityInWorld, rightFootVelocityInWorld);
   private final SideDependentList<Twist> footTwistsInWorld = new SideDependentList<Twist>(new Twist(), new Twist());
   private final YoFrameVector rootJointLinearVelocityNewTwist = new YoFrameVector("estimatedRootJointVelocityNewTwist", worldFrame, registry);
   private final DoubleYoVariable alphaRootJointLinearVelocityNewTwist = new DoubleYoVariable("alphaRootJointLinearVelocityNewTwist", registry);

   private final DoubleYoVariable alphaRootJointLinearVelocityBacklashKinematics = new DoubleYoVariable("alphaRootJointLinearVelocityBacklashKinematics", registry);
   private final DoubleYoVariable slopTimeRootJointLinearVelocityBacklashKinematics = new DoubleYoVariable("slopTimeRootJointLinearVelocityBacklashKinematics", registry);
   private final BacklashCompensatingVelocityYoFrameVector rootJointLinearVelocityBacklashKinematics;

   private final DoubleYoVariable alphaFootToRootJointPosition = new DoubleYoVariable("alphaFootToRootJointPosition", registry);
   private final SideDependentList<AlphaFilteredYoFrameVector> footToRootJointPositions = new SideDependentList<AlphaFilteredYoFrameVector>();

   private final SideDependentList<YoFramePoint> footPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final SideDependentList<YoFramePoint> copPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final DoubleYoVariable alphaCoPFilter = new DoubleYoVariable("alphaCoPFilter", registry);
   private final SideDependentList<AlphaFilteredYoFramePoint2d> copsFilteredInFootFrame = new SideDependentList<AlphaFilteredYoFramePoint2d>();
   private final SideDependentList<YoFramePoint2d> copsRawInFootFrame = new SideDependentList<YoFramePoint2d>();

   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameLineSegment2d> footCenterCoPLineSegments = new SideDependentList<FrameLineSegment2d>();

   private final BooleanYoVariable kinematicsIsUpToDate = new BooleanYoVariable("kinematicsIsUpToDate", registry);
   private final BooleanYoVariable useControllerDesiredCoP = new BooleanYoVariable("useControllerDesiredCoP", registry);
   private final BooleanYoVariable trustCoPAsNonSlippingContactPoint = new BooleanYoVariable("trustCoPAsNonSlippingContactPoint", registry);

   // temporary variables
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempPosition = new FramePoint();
   private final FramePoint2d tempCoP2d = new FramePoint2d();
   private final FramePoint tempCoP = new FramePoint();
   private final FrameVector tempCoPOffset = new FrameVector();
   private final RobotSide[] singleElementRobotSideArray = new RobotSide[1];

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;

   public PelvisKinematicsBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, SideDependentList<ContactablePlaneBody> bipedFeet,
         SideDependentList<FootSwitchInterface> footSwitches, CenterOfPressureDataHolder centerOfPressureDataHolderFromController, double estimatorDT,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.bipedFeet = bipedFeet;
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      this.footSwitches = footSwitches;
      this.centerOfPressureDataHolderFromController = centerOfPressureDataHolderFromController;

      rootJointLinearVelocityBacklashKinematics = BacklashCompensatingVelocityYoFrameVector.createBacklashCompensatingVelocityYoFrameVector("estimatedRootJointLinearVelocityBacklashKin", "", 
            alphaRootJointLinearVelocityBacklashKinematics, estimatorDT, slopTimeRootJointLinearVelocityBacklashKinematics, registry, rootJointPosition);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = bipedFeet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         AlphaFilteredYoFrameVector footToRootJointPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(sidePrefix + "FootToRootJointPosition", "", registry, alphaFootToRootJointPosition, worldFrame);
         footToRootJointPositions.put(robotSide, footToRootJointPosition);

         YoFramePoint footPositionInWorld = new YoFramePoint(sidePrefix + "FootPositionInWorld", worldFrame, registry);
         footPositionsInWorld.put(robotSide, footPositionInWorld);

         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(bipedFeet.get(robotSide).getContactPoints2d());
         footPolygons.put(robotSide, footPolygon);

         FrameLineSegment2d tempLineSegment = new FrameLineSegment2d(new FramePoint2d(soleFrame), new FramePoint2d(soleFrame, 1.0, 1.0)); // TODO need to give distinct points that's not convenient
         footCenterCoPLineSegments.put(robotSide, tempLineSegment);

         YoFramePoint2d copRawInFootFrame = new YoFramePoint2d(sidePrefix + "CoPRawInFootFrame", soleFrames.get(robotSide), registry);
         copsRawInFootFrame.put(robotSide, copRawInFootFrame);

         final AlphaFilteredYoFramePoint2d copFilteredInFootFrame = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(sidePrefix + "CoPFilteredInFootFrame", "", registry, alphaCoPFilter, copRawInFootFrame);
         copFilteredInFootFrame.update(0.0, 0.0); // So the next point will be filtered
         copsFilteredInFootFrame.put(robotSide, copFilteredInFootFrame);

         YoFramePoint copPositionInWorld = new YoFramePoint(sidePrefix + "CoPPositionsInWorld", worldFrame, registry);
         copPositionsInWorld.put(robotSide, copPositionInWorld);

         ReferenceFrame copFrame = new ReferenceFrame("copFrame", soleFrame)
         {
            private static final long serialVersionUID = -1926704435608610401L;
            private final Vector3d copOffset = new Vector3d();
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.setIdentity();
               copFilteredInFootFrame.get(copOffset);
               transformToParent.setTranslation(copOffset);
            }
         };
         copFrames.put(robotSide, copFrame);
      }

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
               YoGraphicPosition copInWorld = new YoGraphicPosition(sidePrefix + "StateEstimatorCoP", copPositionsInWorld.get(robotSide), 0.005, YoAppearance.DeepPink());
               yoGraphicsListRegistry.registerArtifact("StateEstimator", copInWorld.createArtifact());
            }
         }
      }

      parentRegistry.addChild(registry);
   }

   public void setTrustCoPAsNonSlippingContactPoint(boolean trustCoP)
   {
      trustCoPAsNonSlippingContactPoint.set(trustCoP);
   }

   public void useControllerDesiredCoP(boolean useControllerDesiredCoP)
   {
      this.useControllerDesiredCoP.set(useControllerDesiredCoP);
   }

   public void setAlphaPelvisPosition(double alphaFilter)
   {
      alphaFootToRootJointPosition.set(alphaFilter); 
   }

   public void setPelvisLinearVelocityAlphaNewTwist(double alpha)
   {
      alphaRootJointLinearVelocityNewTwist.set(alpha);
   }
   
   public void setPelvisLinearVelocityBacklashParameters(double alphaFilter, double slopTime)
   {
      alphaRootJointLinearVelocityBacklashKinematics.set(alphaFilter);
      slopTimeRootJointLinearVelocityBacklashKinematics.set(slopTime);
   }

   public void setAlphaCenterOfPressure(double alphaFilter)
   {
      alphaCoPFilter.set(alphaFilter);
   }
   
   public void enableTwistEstimation(boolean enable)
   {
      useTwistToComputeRootJointLinearVelocity.set(enable);
   }

   private void reset()
   {
      rootJointPosition.setToZero();
      rootJointLinearVelocity.setToZero();
      rootJointLinearVelocityTwist.setToZero();
   }

   /**
    * Estimates the foot positions corresponding to the given pelvisPosition
    * @param pelvisPosition
    */
   public void initialize(FramePoint pelvisPosition)
   {
      updateKinematics();
      setPelvisPosition(pelvisPosition);

      for(RobotSide robotSide : RobotSide.values)
         updateFootPosition(robotSide, pelvisPosition);

      kinematicsIsUpToDate.set(false);
   }

   /**
    * Estimates the pelvis position and linear velocity using the leg kinematics
    * @param trustedSide which leg is used to estimates the pelvis state
    * @param numberOfTrustedSides is only one or both legs used to estimate the pelvis state
    */
   private void updatePelvisWithKinematics(RobotSide trustedSide, int numberOfTrustedSides)
   {
      double scaleFactor = 1.0 / numberOfTrustedSides;

      footToRootJointPositions.get(trustedSide).getFrameTuple(tempPosition);
      tempPosition.scale(scaleFactor);
      rootJointPosition.add(tempPosition);
      footPositionsInWorld.get(trustedSide).getFrameTuple(tempPosition);
      tempPosition.scale(scaleFactor);
      rootJointPosition.add(tempPosition);

      footVelocitiesInWorld.get(trustedSide).getFrameTupleIncludingFrame(tempFrameVector);
      tempFrameVector.scale(scaleFactor * alphaRootJointLinearVelocityNewTwist.getDoubleValue());
      rootJointLinearVelocityNewTwist.sub(tempFrameVector);
   }

   /**
    * updates the position of the swinging foot
    * @param ignoredSide side of the swinging foot
    * @param pelvisPosition the current pelvis position
    */
   private void updateFootPosition(RobotSide ignoredSide, FramePoint pelvisPosition)
   {
      YoFramePoint footPositionInWorld = footPositionsInWorld.get(ignoredSide);
      footPositionInWorld.set(footToRootJointPositions.get(ignoredSide));
      footPositionInWorld.scale(-1.0);
      footPositionInWorld.add(pelvisPosition);

      copPositionsInWorld.get(ignoredSide).set(footPositionInWorld);

      copsRawInFootFrame.get(ignoredSide).setToZero();
      copsFilteredInFootFrame.get(ignoredSide).setToZero();
   }

   /**
    * Compute the foot CoP. The CoP is the point on the support foot trusted to be not slipping.
    * @param trustedSide
    * @param footSwitch
    */
   private void updateCoPPosition(RobotSide trustedSide)
   {
      AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedSide);
      ReferenceFrame footFrame = soleFrames.get(trustedSide);

      if (useControllerDesiredCoP.getBooleanValue())
         centerOfPressureDataHolderFromController.getCenterOfPressure(tempCoP2d, trustedSide);
      else
         footSwitches.get(trustedSide).computeAndPackCoP(tempCoP2d);
      
      if (trustCoPAsNonSlippingContactPoint.getBooleanValue())
      {
         if (tempCoP2d.containsNaN())
         {
            tempCoP2d.setToZero();
         }
         else
         {
            FrameConvexPolygon2d footPolygon = footPolygons.get(trustedSide);
            boolean isCoPInsideFoot = footPolygon.isPointInside(tempCoP2d);
            if (!isCoPInsideFoot)
            {
               if (footSwitches.get(trustedSide).computeFootLoadPercentage() > 0.2)
               {
                  FrameLineSegment2d footCenterCoPLineSegment = footCenterCoPLineSegments.get(trustedSide);
                  footCenterCoPLineSegment.set(footFrame, 0.0, 0.0, tempCoP2d.getX(), tempCoP2d.getY());
                  // TODO Garbage
                  FramePoint2d[] intersectionPoints = footPolygon.intersectionWith(footCenterCoPLineSegment);

                  if (intersectionPoints.length == 2)
                     System.out.println("In " + getClass().getSimpleName() + ": Found two solutions for the CoP projection.");

                  if (intersectionPoints.length == 0)
                  {
                     System.out.println("In " + getClass().getSimpleName() + ": Found no solution for the CoP projection.");
                     tempCoP2d.setToZero(footFrame);
                  }
                  else
                     tempCoP2d.set(intersectionPoints[0]);
               }
               else // If foot barely loaded and actual CoP outside, then don't update the raw CoP right below
               {
                  copsRawInFootFrame.get(trustedSide).getFrameTuple2dIncludingFrame(tempCoP2d);
               }
            }
         }
      }
      else
      {
         tempCoP2d.setToZero();
      }

      copsRawInFootFrame.get(trustedSide).set(tempCoP2d);

      tempCoPOffset.setIncludingFrame(footFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
      copFilteredInFootFrame.update();
      tempCoPOffset.setIncludingFrame(footFrame, copFilteredInFootFrame.getX() - tempCoPOffset.getX(), copFilteredInFootFrame.getY() - tempCoPOffset.getY(), 0.0);

      tempCoPOffset.changeFrame(worldFrame);
      copPositionsInWorld.get(trustedSide).add(tempCoPOffset);
   }

   /**
    * Assuming the CoP is not moving, the foot position can be updated. That way we can see if the foot is on the edge.
    * @param trustedSide
    */
   private void correctFootPositionsUsingCoP(RobotSide trustedSide)
   {
      AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedSide);
      tempCoPOffset.setIncludingFrame(copFilteredInFootFrame.getReferenceFrame(), copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);

      tempCoPOffset.changeFrame(worldFrame);

      YoFramePoint footPositionIWorld = footPositionsInWorld.get(trustedSide);
      footPositionIWorld.set(copPositionsInWorld.get(trustedSide));
      footPositionIWorld.sub(tempCoPOffset);
   }

   /**
    * Updates the different kinematics related stuff that is used to estimate the pelvis state
    */
   public void updateKinematics()
   {
      reset();
      updateKinematicsNewTwist();
      
      twistCalculator.compute();

      for(RobotSide robotSide : RobotSide.values)
      {
         tempFramePoint.setToZero(rootJointFrame);
         tempFramePoint.changeFrame(soleFrames.get(robotSide));

         tempFrameVector.setIncludingFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToRootJointPositions.get(robotSide).update(tempFrameVector);
      }

      kinematicsIsUpToDate.set(true);
   }
   
   private final Twist tempRootBodyTwist = new Twist();
   
   private void updateKinematicsNewTwist()
   {      
      rootJoint.packJointTwist(tempRootBodyTwist);
      
      rootJointLinearVelocityNewTwist.getFrameTupleIncludingFrame(tempFrameVector);
      tempFrameVector.changeFrame(tempRootBodyTwist.getExpressedInFrame());
      
      tempRootBodyTwist.setLinearPart(tempFrameVector);
      rootJoint.setJointTwist(tempRootBodyTwist);
      
      twistCalculator.compute();

      for(RobotSide robotSide : RobotSide.values)
      {
         Twist footTwistInWorld = footTwistsInWorld.get(robotSide);
         YoFrameVector footVelocityInWorld = footVelocitiesInWorld.get(robotSide);

         twistCalculator.packTwistOfBody(footTwistInWorld, bipedFeet.get(robotSide).getRigidBody());
         footTwistInWorld.changeBodyFrameNoRelativeTwist(soleFrames.get(robotSide));
         footTwistInWorld.changeFrame(soleFrames.get(robotSide));

         this.copsFilteredInFootFrame.get(robotSide).getFrameTuple2dIncludingFrame(tempCoP2d);
         tempCoP.setXYIncludingFrame(tempCoP2d);
         footTwistInWorld.changeFrame(footTwistInWorld.getBaseFrame());
         tempCoP.changeFrame(footTwistInWorld.getExpressedInFrame());
         footTwistInWorld.packLinearVelocityOfPointFixedInBodyFrame(tempFrameVector, tempCoP);

         tempFrameVector.changeFrame(worldFrame);
         footVelocityInWorld.set(tempFrameVector);
      }
   }

   public void estimatePelvisLinearStateForDoubleSupport()
   {
      estimatePelvisLinearState(RobotSide.values);
   }

   public void estimatePelvisLinearStateForSingleSupport(FramePoint pelvisPosition, RobotSide trustedSide)
   {
      estimatePelvisLinearState(trustedSide);
      updateFootPosition(trustedSide.getOppositeSide(), pelvisPosition);
   }

   public void updateFeetPositionsWhenTrustingIMUOnly(FramePoint pelvisPosition)
   {
      for (RobotSide robotSide : RobotSide.values)
         updateFootPosition(robotSide, pelvisPosition);
   }

   private void estimatePelvisLinearState(RobotSide trustedSide)
   {
      singleElementRobotSideArray[0] = trustedSide;
      estimatePelvisLinearState(singleElementRobotSideArray);
   }
   
   private void estimatePelvisLinearState(RobotSide[] listOfTrustedSides)
   {
      if (!kinematicsIsUpToDate.getBooleanValue())
         throw new RuntimeException("Leg kinematics needs to be updated before trying to estimate the pelvis position/linear velocity.");

      for(RobotSide trustedSide : listOfTrustedSides)
      {
         updateCoPPosition(trustedSide);
         correctFootPositionsUsingCoP(trustedSide);
         updatePelvisWithKinematics(trustedSide, listOfTrustedSides.length);
      }
      rootJointLinearVelocityBacklashKinematics.update();

      kinematicsIsUpToDate.set(false);
   }

   public void setPelvisPosition(FramePoint pelvisPosition)
   {
      rootJointPosition.set(pelvisPosition);
   }

   public void getRootJointPositionAndVelocity(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPelvisPosition(positionToPack);
      getPelvisVelocity(linearVelocityToPack);
   }

   public void getPelvisPosition(FramePoint positionToPack)
   {
      rootJointPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getPelvisVelocity(FrameVector linearVelocityToPack)
   {
      if (useTwistToComputeRootJointLinearVelocity.getBooleanValue())
         rootJointLinearVelocityNewTwist.getFrameTupleIncludingFrame(linearVelocityToPack);
      else
         rootJointLinearVelocityBacklashKinematics.getFrameTupleIncludingFrame(linearVelocityToPack);
   }

   public void getFootToPelvisPosition(FramePoint positionToPack, RobotSide robotSide)
   {
      footToRootJointPositions.get(robotSide).getFrameTupleIncludingFrame(positionToPack);
   }
}
