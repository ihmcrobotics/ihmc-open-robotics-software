package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.BacklashCompensatingVelocityYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

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

   private final RigidBody pelvis;
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   
   private final ReferenceFrame pelvisFrame;
   private final SideDependentList<ReferenceFrame> footFrames = new SideDependentList<ReferenceFrame>();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable alphaFootToPelvisPosition = new DoubleYoVariable("alphaFootToPelvisPosition", registry);
   private final DoubleYoVariable alphaFootToPelvisVelocity = new DoubleYoVariable("alphaFootToPelvisVelocity", registry);
   private final DoubleYoVariable alphaFootToPelvisAccel = new DoubleYoVariable("alphaFootToPelvisAcceleration", registry);

   private final YoFramePoint pelvisPositionKinematics = new YoFramePoint("estimatedPelvisPositionWithKinematics", worldFrame, registry);   
   private final YoFrameVector pelvisVelocityKinematics = new YoFrameVector("estimatedPelvisVelocityWithKinematics", worldFrame, registry);
   private final YoFrameVector pelvisVelocityTwist = new YoFrameVector("estimatedPelvisVelocityWithTwist", worldFrame, registry);
   private final BooleanYoVariable useTwistToComputePelvisVelocity = new BooleanYoVariable("useTwistToComputePelvisVelocity", registry);

   private final DoubleYoVariable alphaPelvisVelocityBacklashKinematics = new DoubleYoVariable("alphaPelvisVelocityBacklashKinematics", registry);
   private final DoubleYoVariable slopTimePelvisVelocityBacklashKinematics = new DoubleYoVariable("slopTimePelvisVelocityBacklashKinematics", registry);
   private final BacklashCompensatingVelocityYoFrameVector pelvisVelocityBacklashKinematics;
   
   private final SideDependentList<AlphaFilteredYoFrameVector> footToPelvisPositions = new SideDependentList<AlphaFilteredYoFrameVector>();
   private final SideDependentList<FilteredVelocityYoFrameVector> footToPelvisVelocities = new SideDependentList<FilteredVelocityYoFrameVector>();
   private final SideDependentList<FilteredVelocityYoFrameVector> footToPelvisAccelerations = new SideDependentList<FilteredVelocityYoFrameVector>();
   private final SideDependentList<Twist> pelvisToFootTwists = new SideDependentList<Twist>(new Twist(), new Twist());

   private final SideDependentList<YoFramePoint> footPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final SideDependentList<YoFramePoint> copPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final DoubleYoVariable alphaCoPFilter = new DoubleYoVariable("alphaCoPFilter", registry);
   private final SideDependentList<AlphaFilteredYoFramePoint2d> copsFilteredInFootFrame = new SideDependentList<AlphaFilteredYoFramePoint2d>();
   private final SideDependentList<YoFramePoint2d> copsRawInFootFrame = new SideDependentList<YoFramePoint2d>();

   private final SideDependentList<FrameLine2d> ankleToCoPLines = new SideDependentList<FrameLine2d>();
   
   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>();
   
   private final BooleanYoVariable kinematicsIsUpToDate = new BooleanYoVariable("kinematicsIsUpToDate", registry);
   
   // temporary variables
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();
   private final FramePoint2d tempCoP = new FramePoint2d();
   private final FrameVector tempCoPOffset = new FrameVector();
   private final SideDependentList<FrameLineSegment2d> tempLineSegments = new SideDependentList<FrameLineSegment2d>();
   
   public PelvisKinematicsBasedLinearStateCalculator(TwistCalculator twistCalculator, RigidBody pelvis, ReferenceFrame pelvisFrame,
         SideDependentList<ContactablePlaneBody> bipedFeet, double estimatorDT, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.pelvis = pelvis;
      this.bipedFeet = bipedFeet;
      this.pelvisFrame = pelvisFrame;
      this.twistCalculator = twistCalculator;
      
      pelvisVelocityBacklashKinematics = BacklashCompensatingVelocityYoFrameVector.createBacklashCompensatingVelocityYoFrameVector("estimatedPelvisVelocityBacklashKin", "", 
            alphaPelvisVelocityBacklashKinematics, estimatorDT, slopTimePelvisVelocityBacklashKinematics, registry, pelvisPositionKinematics);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = bipedFeet.get(robotSide).getPlaneFrame();
         footFrames.put(robotSide, footFrame);
         
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         AlphaFilteredYoFrameVector footToPelvisPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(sidePrefix + "FootToPelvisPosition", "", registry, alphaFootToPelvisPosition, worldFrame);
         footToPelvisPositions.put(robotSide, footToPelvisPosition);
         
         FilteredVelocityYoFrameVector footToPelvisVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(sidePrefix + "FootToPelvisVelocity", "", alphaFootToPelvisVelocity, estimatorDT, registry, footToPelvisPosition);
         footToPelvisVelocities.put(robotSide, footToPelvisVelocity);
         
         FilteredVelocityYoFrameVector footToPelvisAccel = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(sidePrefix + "FootToPelvisAcceleration", "", alphaFootToPelvisAccel, estimatorDT, registry, footToPelvisVelocity);
         footToPelvisAccelerations.put(robotSide, footToPelvisAccel);

         YoFramePoint footPositionInWorld = new YoFramePoint(sidePrefix + "FootPositionInWorld", worldFrame, registry);
         footPositionsInWorld.put(robotSide, footPositionInWorld);

         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(bipedFeet.get(robotSide).getContactPoints2d());
         footPolygons.put(robotSide, footPolygon);

         FrameLine2d ankleToCoPLine = new FrameLine2d(footFrame, new Point2d(), new Point2d(1.0, 0.0));
         ankleToCoPLines.put(robotSide, ankleToCoPLine);
         
         FrameLineSegment2d tempLineSegment = new FrameLineSegment2d(new FramePoint2d(footFrame), new FramePoint2d(footFrame, 1.0, 1.0)); // TODO need to give distinct points that's not convenient
         tempLineSegments.put(robotSide, tempLineSegment);
         
         YoFramePoint2d copRawInFootFrame = new YoFramePoint2d(sidePrefix + "CoPRawInFootFrame", footFrames.get(robotSide), registry);
         copsRawInFootFrame.put(robotSide, copRawInFootFrame);
         
         AlphaFilteredYoFramePoint2d copFilteredInFootFrame = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(sidePrefix + "CoPFilteredInFootFrame", "", registry, alphaCoPFilter, copRawInFootFrame);
         copFilteredInFootFrame.update(0.0, 0.0); // So the next point will be filtered
         copsFilteredInFootFrame.put(robotSide, copFilteredInFootFrame);

         YoFramePoint copPositionInWorld = new YoFramePoint(sidePrefix + "CoPPositionsInWorld", worldFrame, registry);
         copPositionsInWorld.put(robotSide, copPositionInWorld);
      }

      if (VISUALIZE && dynamicGraphicObjectsListRegistry != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            DynamicGraphicPosition copInWorld = new DynamicGraphicPosition(sidePrefix + "StateEstimatorCoP", copPositionsInWorld.get(robotSide), 0.005, YoAppearance.DeepPink());
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("StateEstimatorCoP", copInWorld);
            dynamicGraphicObjectsListRegistry.registerArtifact("StateEstimatorCoP", copInWorld.createArtifact());
//            DynamicGraphicVector footToPelvis = new DynamicGraphicVector(sidePrefix + "FootToPelvis", footPositionsInWorld.get(robotSide), footToPelvisPositions.get(robotSide), 1.0, YoAppearance.Blue());
//            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("footToPelvis", footToPelvis);
         }
      }
      
      parentRegistry.addChild(registry);
   }

   public void setAlphaPelvisPosition(double alphaFilter)
   {
      alphaFootToPelvisPosition.set(alphaFilter); 
   }
   
   public void setAlphaPelvisLinearVelocity(double alphaFilter)
   {
      alphaFootToPelvisVelocity.set(alphaFilter);
   }
   
   public void setPelvisLinearVelocityBacklashParameters(double alphaFilter, double sloptime)
   {
      alphaPelvisVelocityBacklashKinematics.set(alphaFilter);
      slopTimePelvisVelocityBacklashKinematics.set(sloptime);
   }
   
   public void setAlphaCenterOfPressure(double alphaFilter)
   {
      alphaCoPFilter.set(alphaFilter);
   }
   
   private void reset()
   {
      pelvisPositionKinematics.setToZero();
      pelvisVelocityKinematics.setToZero();
      pelvisVelocityTwist.setToZero();
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
      
      footToPelvisPositions.get(trustedSide).getFramePoint(tempPosition);
      tempPosition.scale(scaleFactor);
      pelvisPositionKinematics.add(tempPosition);
      footPositionsInWorld.get(trustedSide).getFramePoint(tempPosition);
      tempPosition.scale(scaleFactor);
      pelvisPositionKinematics.add(tempPosition);
      
      footToPelvisVelocities.get(trustedSide).getFrameVector(tempVelocity);
      tempVelocity.scale(scaleFactor);
      pelvisVelocityKinematics.add(tempVelocity);
     
      YoFramePoint2d copPosition2d = copsFilteredInFootFrame.get(trustedSide);
      tempFramePoint.set(copPosition2d.getReferenceFrame(), copPosition2d.getX(), copPosition2d.getY(), 0.0);
      tempFramePoint.changeFrame(pelvisToFootTwists.get(trustedSide).getBaseFrame());
      pelvisToFootTwists.get(trustedSide).changeFrame(pelvisToFootTwists.get(trustedSide).getBaseFrame());
      pelvisToFootTwists.get(trustedSide).packVelocityOfPointFixedInBodyFrame(tempVelocity, tempFramePoint);
      tempVelocity.changeFrame(worldFrame);
      tempVelocity.scale(-scaleFactor); // We actually want footToPelvis velocity
      pelvisVelocityTwist.add(tempVelocity);
   }

   /**
    * updates the position of the swinging foot
    * @param ignoredSide side of the swinging foot
    * @param pelvisPosition the current pelvis position
    */
   private void updateFootPosition(RobotSide ignoredSide, FramePoint pelvisPosition)
   {
      YoFramePoint footPositionInWorld = footPositionsInWorld.get(ignoredSide);
      footPositionInWorld.set(footToPelvisPositions.get(ignoredSide));
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
   private void updateCoPPosition(RobotSide trustedSide, WrenchBasedFootSwitch footSwitch)
   {
         AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedSide);
         ReferenceFrame footFrame = footFrames.get(trustedSide);
         
         footSwitch.computeAndPackCoP(tempCoP);
         
         if (tempCoP.containsNaN())
         {
            tempCoP.setToZero();
         }
         else
         {
            FrameConvexPolygon2d footPolygon = footPolygons.get(trustedSide);
            boolean isCoPInsideFoot = footPolygon.isPointInside(tempCoP);
            if (!isCoPInsideFoot)
            {
               FrameLineSegment2d tempLineSegment = tempLineSegments.get(trustedSide);
               footPolygon.getClosestEdge(tempLineSegment, tempCoP);

               FrameLine2d ankleToCoPLine = ankleToCoPLines.get(trustedSide);
               ankleToCoPLine.set(footFrame, 0.0, 0.0, tempCoP.getX(), tempCoP.getY());
               tempLineSegment.intersectionWith(tempCoP, ankleToCoPLine);
            }
         }
         
         copsRawInFootFrame.get(trustedSide).set(tempCoP);
         
         tempCoPOffset.set(footFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
         copFilteredInFootFrame.update();
         tempCoPOffset.set(footFrame, copFilteredInFootFrame.getX() - tempCoPOffset.getX(), copFilteredInFootFrame.getY() - tempCoPOffset.getY(), 0.0);

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
      tempCoPOffset.set(copFilteredInFootFrame.getReferenceFrame(), copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);

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
      twistCalculator.compute();
      
      for(RobotSide robotSide : RobotSide.values)
      {
         tempFramePoint.setToZero(pelvisFrame);
         tempFramePoint.changeFrame(footFrames.get(robotSide));

         tempFrameVector.setAndChangeFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToPelvisPositions.get(robotSide).update(tempFrameVector);
         footToPelvisVelocities.get(robotSide).update();
         footToPelvisAccelerations.get(robotSide).update();
         
         Twist pelvisToFootTwist = pelvisToFootTwists.get(robotSide);
         twistCalculator.packRelativeTwist(pelvisToFootTwist, pelvis, bipedFeet.get(robotSide).getRigidBody());
      }
      
      kinematicsIsUpToDate.set(true);
   }

   public void estimatePelvisLinearStateForDoubleSupport(SideDependentList<WrenchBasedFootSwitch> footSwitches)
   {
      estimatePelvisLinearState(footSwitches, RobotSide.values());
   }
   
   public void estimatePelvisLinearStateForSingleSupport(FramePoint pelvisPosition, SideDependentList<WrenchBasedFootSwitch> footSwitches, RobotSide trustedSide)
   {
      estimatePelvisLinearState(footSwitches, trustedSide);
      updateFootPosition(trustedSide.getOppositeSide(), pelvisPosition);
   }
   
   private void estimatePelvisLinearState(SideDependentList<WrenchBasedFootSwitch> footSwitches, RobotSide...listOfTrustedSides)
   {
      if (!kinematicsIsUpToDate.getBooleanValue())
         throw new RuntimeException("Leg kinematics needs to be updated before trying to estimate the pelvis position/linear velocity.");
      
      for(RobotSide trustedSide : listOfTrustedSides)
      {
         updateCoPPosition(trustedSide, footSwitches.get(trustedSide));
         correctFootPositionsUsingCoP(trustedSide);
         updatePelvisWithKinematics(trustedSide, listOfTrustedSides.length);
      }
      pelvisVelocityBacklashKinematics.update();

      kinematicsIsUpToDate.set(false);
   }

   public void setPelvisPosition(FramePoint pelvisPosition)
   {
      pelvisPositionKinematics.set(pelvisPosition);
   }
   
   public void getPelvisPositionAndVelocity(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPelvisPosition(positionToPack);
      getPelvisVelocity(linearVelocityToPack);
   }
   
   public void getPelvisPosition(FramePoint positionToPack)
   {
      pelvisPositionKinematics.getFramePointAndChangeFrameOfPackedPoint(positionToPack);
   }
   
   public void getPelvisVelocity(FrameVector linearVelocityToPack)
   {
      if (useTwistToComputePelvisVelocity.getBooleanValue())
         pelvisVelocityTwist.getFrameVectorAndChangeFrameOfPackedVector(linearVelocityToPack);
      else
         pelvisVelocityBacklashKinematics.getFrameVectorAndChangeFrameOfPackedVector(linearVelocityToPack);
   }
   
   public void getFootToPelvisPosition(FramePoint positionToPack, RobotSide robotSide)
   {
      footToPelvisPositions.get(robotSide).getFramePointAndChangeFrameOfPackedPoint(positionToPack);
   }

   public void getFootToPelvisVelocity(FrameVector linearVelocityToPack, RobotSide robotSide)
   {
      footToPelvisVelocities.get(robotSide).getFrameVectorAndChangeFrameOfPackedVector(linearVelocityToPack);
   }

   public void getFootToPelvisAcceleration(FrameVector linearAccelerationToPack, RobotSide robotSide)
   {
      footToPelvisAccelerations.get(robotSide).getFrameVectorAndChangeFrameOfPackedVector(linearAccelerationToPack);
   }
}
