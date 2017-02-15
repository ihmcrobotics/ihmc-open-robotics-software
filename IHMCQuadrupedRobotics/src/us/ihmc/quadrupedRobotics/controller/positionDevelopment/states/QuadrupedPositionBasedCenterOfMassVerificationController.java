package us.ihmc.quadrupedRobotics.controller.positionDevelopment.states;

import java.awt.Color;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class QuadrupedPositionBasedCenterOfMassVerificationController implements QuadrupedController
{
   private static final double DEFAULT_COM_HEIGHT_Z_FILTER_BREAK_FREQUENCY = 0.5;

   private final double dt;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable robotTimestamp;
   private DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);
   private DoubleYoVariable footZHeightOnPickUp = new DoubleYoVariable("footZHeightOnPickUp", registry);
   private DoubleYoVariable footZHeightOnTouchdown = new DoubleYoVariable("footZHeightOnTouchdown", registry);
   private DoubleYoVariable timeToSTayInMoveFeetAfterTouchDown = new DoubleYoVariable("timeToSTayInMoveFeetAfterTouchDown", registry);
   private final SideDependentList<ReferenceFrame> sideDependentMidTrotLineZUpFrames = new SideDependentList<ReferenceFrame>();


   public enum COM_ESTIMATE_STATES
   {
      MOVE_COM_AROUND, ALPHA_FILTERING_DESIREDS, PICK_UP_FEET, PUT_DOWN_FEET
   }

   private final StateMachine<COM_ESTIMATE_STATES> stateMachine;
   private final FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerOnTransitionIn;
   private final QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators;
   private final FullRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;
   private final OneDoFJoint[] oneDoFJoints;

   private final EnumYoVariable<TrotPair> trotPairToRaise = new EnumYoVariable<>("trotPairToRaise", getYoVariableRegistry(), TrotPair.class);
   private final EnumYoVariable<TrotPair> trotPairInAir = new EnumYoVariable<>("trotPairInAir", getYoVariableRegistry(), TrotPair.class);

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final DoubleYoVariable filteredDesiredCoMHeightAlphaBreakFrequency = new DoubleYoVariable("filteredDesiredCoMHeightAlphaBreakFrequency", registry);
   private final DoubleYoVariable filteredDesiredCoMHeightAlpha = new DoubleYoVariable("filteredDesiredCoMHeightAlpha", registry);
   private final AlphaFilteredYoVariable filteredDesiredCoMHeight = new AlphaFilteredYoVariable("filteredDesiredCoMHeight", registry, filteredDesiredCoMHeightAlpha, desiredCoMHeight);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint desiredCenterOfMassPosition = new YoFramePoint("desiredCenterOfMassPosition", worldFrame, registry);
   private final YoFrameOrientation desiredCenterOfMassOrientation = new YoFrameOrientation("desiredCenterOfMassOrientation", worldFrame, registry);
   private final YoFramePose desiredCenterOfMassPose = new YoFramePose(desiredCenterOfMassPosition, desiredCenterOfMassOrientation);
   private final FramePose desiredCenterOfMassPoseForPacking = new FramePose(worldFrame);
   private final PoseReferenceFrame desiredCoMPoseReferenceFrame = new PoseReferenceFrame("desiredCoMPoseReferenceFrame", desiredCenterOfMassPoseForPacking);

   private final ConvexPolygon2d supportPolygonHolder = new ConvexPolygon2d();
   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();

   private final QuadrantDependentList<YoMinimumJerkTrajectory> footTrajectories = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> currentFeetLocations = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFramePoint> desiredFeetLocations = new QuadrantDependentList<YoFramePoint>();

   private final QuadrantDependentList<YoFrameVector> desiredFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector>();
   private final FramePoint desiredFootPosition = new FramePoint(worldFrame);
   private final FramePoint desiredFootPositionInLegAttachmentFrame = new FramePoint();

   public QuadrupedPositionBasedCenterOfMassVerificationController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedLegInverseKinematicsCalculator quadrupedInverseKinematicsCalulcator, YoVariableRegistry parentRegistry)
   {
      //Set Initial Values
      this.swingTime.set(2.0);
      this.footZHeightOnPickUp.set(0.04);
      this.footZHeightOnTouchdown.set(0.0);
      this.timeToSTayInMoveFeetAfterTouchDown.set(1.0);
      this.desiredCoMHeight.set(0.65);

      this.trotPairToRaise.set(TrotPair.NONE);
      this.trotPairInAir.set(TrotPair.NONE);

      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.dt = runtimeEnvironment.getControlDT();
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);
      this.inverseKinematicsCalculators = quadrupedInverseKinematicsCalulcator;
      this.oneDoFJoints = fullRobotModel.getOneDoFJoints();

      filteredDesiredCoMHeightAlphaBreakFrequency.set(DEFAULT_COM_HEIGHT_Z_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMHeightAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMHeightAlphaBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            filteredDesiredCoMHeightAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));
         }
      });

      referenceFrames.updateFrames();
      desiredCenterOfMassPoseForPacking.setToZero(referenceFrames.getCenterOfMassFrame());
      desiredCenterOfMassPoseForPacking.changeFrame(worldFrame);
      desiredCenterOfMassPose.set(desiredCenterOfMassPoseForPacking);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         footTrajectories.set(robotQuadrant, new YoMinimumJerkTrajectory(prefix + "FootTrajectory", registry));

         ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);

         YoFramePoint currentFootPosition = new YoFramePoint(prefix + "currentFootPosition", ReferenceFrame.getWorldFrame(), registry);
         currentFeetLocations.set(robotQuadrant, currentFootPosition);

         YoFramePoint desiredFootLocation = new YoFramePoint(prefix + "desiredFootPosition", ReferenceFrame.getWorldFrame(), registry);

         FramePoint footPosition = new FramePoint(legAttachmentFrame);
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         footPosition.setZ(0.0);

         desiredFootLocation.set(footPosition);
         desiredFeetLocations.set(robotQuadrant, desiredFootLocation);

         YoFrameVector footPositionInLegAttachementFrame = new YoFrameVector(prefix + "FootPositionInLegFrame", legAttachmentFrame, registry);
         desiredFeetPositionsInLegAttachmentFrame.set(robotQuadrant, footPositionInLegAttachementFrame);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RobotQuadrant hindSoleQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide);
         RobotQuadrant frontSoleQuadrantOppositeSide = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide.getOppositeSide());

         MidFrameZUpFrame midTrotLineZUpFrame = new MidFrameZUpFrame("hind" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Front" + robotSide.getOppositeSide().getCamelCaseNameForMiddleOfExpression() + "MidTrotLineZUpFrame", worldFrame, referenceFrames.getFootFrame(hindSoleQuadrant), referenceFrames.getFootFrame(frontSoleQuadrantOppositeSide));
         sideDependentMidTrotLineZUpFrames.put(robotSide, midTrotLineZUpFrame);
      }

      //Create State Machine and States
      this.stateMachine = new StateMachine<COM_ESTIMATE_STATES>("centerOfMassVerificationStateMachine", "walkingStateTranistionTime", COM_ESTIMATE_STATES.class, runtimeEnvironment.getRobotTimestamp(), registry);
      this.filterDesiredsToMatchCrawlControllerOnTransitionIn = new FilterDesiredsToMatchCrawlControllerState(modelFactory);
      MoveCenterOfMassAround moveCenterOfMassAroundState = new MoveCenterOfMassAround();
      MoveFeet pickFeetUp = new MoveFeet("pickFeetUp", footZHeightOnPickUp, trotPairToRaise, COM_ESTIMATE_STATES.PICK_UP_FEET);
      MoveFeet putFeetDown = new MoveFeet("putFeetDown", footZHeightOnTouchdown, trotPairInAir, COM_ESTIMATE_STATES.PUT_DOWN_FEET);

      stateMachine.addState(pickFeetUp);
      stateMachine.addState(putFeetDown);
      stateMachine.addState(moveCenterOfMassAroundState);
      stateMachine.addState(filterDesiredsToMatchCrawlControllerOnTransitionIn);

      //setup transitions
      //start with ALPHA_FILTERING_DESIREDS, once finished goto ESTIMATE_COM,
      //once in ESTIMATE_COM, trotPairToRaise can trigger pick up and put down,
      //both pick up and put down transition back to ESTIMATE_COM when done
      FilterToCoMShiftTransition filterDesiredsToCoMShiftTransitionCondition = new FilterToCoMShiftTransition(filterDesiredsToMatchCrawlControllerOnTransitionIn);
      StateTransition<COM_ESTIMATE_STATES> filterDesiredsToCoMShiftTransition = new StateTransition<COM_ESTIMATE_STATES>(COM_ESTIMATE_STATES.MOVE_COM_AROUND, filterDesiredsToCoMShiftTransitionCondition);
      filterDesiredsToMatchCrawlControllerOnTransitionIn.addStateTransition(filterDesiredsToCoMShiftTransition);

      PickUpTrotLineFeetTransitionCondition pickUpTransitionCondition = new PickUpTrotLineFeetTransitionCondition();
      PutDownTrotLineFeetTransitionCondition putDownTransitionCondition = new PutDownTrotLineFeetTransitionCondition();

      StateTransition<COM_ESTIMATE_STATES> pickUpFeetTransition = new StateTransition<QuadrupedPositionBasedCenterOfMassVerificationController.COM_ESTIMATE_STATES>(COM_ESTIMATE_STATES.PICK_UP_FEET, pickUpTransitionCondition);
      StateTransition<COM_ESTIMATE_STATES> putDownFeetTransition = new StateTransition<QuadrupedPositionBasedCenterOfMassVerificationController.COM_ESTIMATE_STATES>(COM_ESTIMATE_STATES.PUT_DOWN_FEET, putDownTransitionCondition);

      moveCenterOfMassAroundState.addStateTransition(pickUpFeetTransition);
      moveCenterOfMassAroundState.addStateTransition(putDownFeetTransition);

      createGraphicsAndArtifacts(runtimeEnvironment.getGraphicsListRegistry());
      parentRegistry.addChild(registry);
   }

   private void createGraphicsAndArtifacts(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("comEstimateQuadSupportPolygonArtifact", supportPolygon, Color.blue, false);

      yoGraphicsListRegistry.registerArtifact("supportPolygon", supportPolygonArtifact);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = "comEstimate" + robotQuadrant.getCamelCaseNameForMiddleOfExpression();

         YoFramePoint footPosition = currentFeetLocations.get(robotQuadrant);
         YoGraphicPosition currentFootPositionViz = new YoGraphicPosition(prefix + "currentFootPositionViz", footPosition, 0.02, getYoAppearance(robotQuadrant), GraphicType.BALL_WITH_CROSS);

         yoGraphicsListRegistry.registerYoGraphic("currentFootPosition", currentFootPositionViz);
         yoGraphicsListRegistry.registerArtifact("currentFootPosition", currentFootPositionViz.createArtifact());

         YoFramePoint desiredFootPosition = desiredFeetLocations.get(robotQuadrant);
         YoGraphicPosition desiredFootPositionViz = new YoGraphicPosition(prefix + "desiredFootPositionViz", desiredFootPosition, 0.01, YoAppearance.Red());

         yoGraphicsListRegistry.registerYoGraphic("Desired Feet", desiredFootPositionViz);
         yoGraphicsListRegistry.registerArtifact("Desired Feet", desiredFootPositionViz.createArtifact());
      }
   }

   private AppearanceDefinition getYoAppearance(RobotQuadrant robotQuadrant)
   {
      switch (robotQuadrant)
      {
      case FRONT_LEFT:
         return YoAppearance.White();
      case FRONT_RIGHT:
         return YoAppearance.Yellow();
      case HIND_LEFT:
         return YoAppearance.Blue();
      case HIND_RIGHT:
         return YoAppearance.Black();
      default:
         throw new RuntimeException("bad quad");
      }
   }

   @Override
   public ControllerEvent process()
   {
      referenceFrames.updateFrames();
      updateFeetLocations();
      updateGraphics();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      updateDesiredHeight();
      updateDesiredCoMPose();
      updateDesiredFootPositionsBasedOnDesiredCenterOfMass();
      useInverseKinematicsToGetJointPositionsAndStoreInFullRobotModel(fullRobotModel);

      if (stateMachine.isCurrentState(COM_ESTIMATE_STATES.ALPHA_FILTERING_DESIREDS))
      {
         filterDesiredsToMatchCrawlControllerOnTransitionIn.filterDesireds(oneDoFJoints);
      }
      return null;
   }

   private final FramePoint currentFootLocation = new FramePoint();

   /**
    * update actual feet locations and the four foot polygon, using the desired locations
    */
   private void updateFeetLocations()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         currentFootLocation.setToZero(footFrame);
         currentFootLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint yoFootLocation = currentFeetLocations.get(robotQuadrant);
         yoFootLocation.set(currentFootLocation);

         // Use the desired foot locations instead of the actual locations
         YoFramePoint desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         fourFootSupportPolygon.setFootstep(robotQuadrant, desiredFootLocation.getFrameTuple());
      }
   }

   private void updateGraphics()
   {
      drawSupportPolygon(fourFootSupportPolygon, supportPolygon);
   }

   /**
    * desired CoM height in world
    */
   private void updateDesiredHeight()
   {
      filteredDesiredCoMHeight.update();
      double zHeight = filteredDesiredCoMHeight.getDoubleValue();
      desiredCenterOfMassPosition.setZ(zHeight);
   }

   private void updateDesiredCoMPose()
   {
      desiredCenterOfMassPose.getFramePose(desiredCenterOfMassPoseForPacking);
      desiredCoMPoseReferenceFrame.setPoseAndUpdate(desiredCenterOfMassPoseForPacking);
   }

   private void updateDesiredFootPositionsBasedOnDesiredCenterOfMass()
   {
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);

         //get the pose from desired feet to the desired center of mass
         desiredFootPosition.setIncludingFrame(desiredFeetLocations.get(robotQuadrant).getFrameTuple());
         desiredFootPosition.changeFrame(desiredCoMPoseReferenceFrame);

         desiredFootPositionInLegAttachmentFrame.setIncludingFrame(centerOfMassFrame, desiredFootPosition.getPoint());
         desiredFootPositionInLegAttachmentFrame.changeFrame(legAttachmentFrame);

         desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).set(desiredFootPositionInLegAttachmentFrame.getPoint());
      }
   }

   private final Vector3d desiredFootPositionForInverseKinematics = new Vector3d();

   private void useInverseKinematicsToGetJointPositionsAndStoreInFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).get(desiredFootPositionForInverseKinematics);
         inverseKinematicsCalculators.solveForEndEffectorLocationInBodyAndUpdateDesireds(robotQuadrant, desiredFootPositionForInverseKinematics, fullRobotModel);
      }
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
   {
      supportPolygonHolder.clear();
      for (RobotQuadrant quadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         supportPolygonHolder.addVertex(supportPolygon.getFootstep(quadrant).getX(), supportPolygon.getFootstep(quadrant).getY());
      }
      supportPolygonHolder.update();
      yoFramePolygon.setConvexPolygon2d(supportPolygonHolder);
   }

   private class FilterToCoMShiftTransition implements StateTransitionCondition
   {
      private final FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerState;

      public FilterToCoMShiftTransition(FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerState)
      {
         this.filterDesiredsToMatchCrawlControllerState = filterDesiredsToMatchCrawlControllerState;
      }

      @Override
      public boolean checkCondition()
      {
         return filterDesiredsToMatchCrawlControllerState.isInterpolationFinished();
      }
   }

   private class PickUpTrotLineFeetTransitionCondition implements StateTransitionCondition
   {
      public PickUpTrotLineFeetTransitionCondition()
      {
      }

      @Override
      public boolean checkCondition()
      {
         return (trotPairInAir.getEnumValue() == TrotPair.NONE && trotPairToRaise.getEnumValue() != TrotPair.NONE);
      }
   }

   private class PutDownTrotLineFeetTransitionCondition implements StateTransitionCondition
   {
      public PutDownTrotLineFeetTransitionCondition()
      {
      }

      @Override
      public boolean checkCondition()
      {
         return (trotPairInAir.getEnumValue() != TrotPair.NONE && trotPairToRaise.getEnumValue() != trotPairInAir.getEnumValue());
      }
   }

   private class CenterOfMassIncrementVariableHolder
   {
      private final DoubleYoVariable alpha;

      private final DoubleYoVariable desiredCenterOfMassLargeIncrementX;
      private final DoubleYoVariable desiredCenterOfMassSmallIncrementX;
      private final DoubleYoVariable desiredCenterOfMassLargeIncrementY;
      private final DoubleYoVariable desiredCenterOfMassSmallIncrementY;

      private final AlphaFilteredYoVariable filteredDesiredCenterOfMassLargeIncrementX;
      private final AlphaFilteredYoVariable filteredDesiredCenterOfMassSmallIncrementX;
      private final AlphaFilteredYoVariable filteredDesiredCenterOfMassLargeIncrementY;
      private final AlphaFilteredYoVariable filteredDesiredCenterOfMassSmallIncrementY;
      private final TrotPair trotPair;
      private final ReferenceFrame trotLineFrame;
      private final ReferenceFrame initialCenterOfMassReferenceFrame;

      private final FramePoint initialCenterOfMass;
      private final FramePoint workingFramePoint = new FramePoint();

      public CenterOfMassIncrementVariableHolder(TrotPair trotPair, ReferenceFrame trotLineFrame, ReferenceFrame initialCenterOfMassReferenceFrame)
      {
         this.trotPair = trotPair;
         String name = trotPair.getFrameName();
         this.trotLineFrame = trotLineFrame;
         this.initialCenterOfMassReferenceFrame = initialCenterOfMassReferenceFrame;
         initialCenterOfMass = new FramePoint(initialCenterOfMassReferenceFrame);

         alpha = new DoubleYoVariable(name + "desiredCenterOfMassIncrementAlpha", registry);
         alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.5, dt));

         desiredCenterOfMassLargeIncrementX = new DoubleYoVariable(name + "DesiredCenterOfMassLargeIncrementX", registry);
         desiredCenterOfMassSmallIncrementX = new DoubleYoVariable(name + "DesiredCenterOfMassSmallIncrementX", registry);
         desiredCenterOfMassLargeIncrementY = new DoubleYoVariable(name + "DesiredCenterOfMassLargeIncrementY", registry);
         desiredCenterOfMassSmallIncrementY = new DoubleYoVariable(name + "DesiredCenterOfMassSmallIncrementY", registry);

         filteredDesiredCenterOfMassLargeIncrementX = new AlphaFilteredYoVariable(name + "FilteredDesiredCenterOfMassLargeIncrementX", registry, alpha, desiredCenterOfMassLargeIncrementX);
         filteredDesiredCenterOfMassSmallIncrementX = new AlphaFilteredYoVariable(name + "FilteredDesiredCenterOfMassSmallIncrementX", registry, alpha, desiredCenterOfMassSmallIncrementX);
         filteredDesiredCenterOfMassLargeIncrementY = new AlphaFilteredYoVariable(name + "FilteredDesiredCenterOfMassLargeIncrementY", registry, alpha, desiredCenterOfMassLargeIncrementY);
         filteredDesiredCenterOfMassSmallIncrementY = new AlphaFilteredYoVariable(name + "FilteredDesiredCenterOfMassSmallIncrementY", registry, alpha, desiredCenterOfMassSmallIncrementY);
      }

      public void update()
      {
         filteredDesiredCenterOfMassLargeIncrementX.update();
         filteredDesiredCenterOfMassSmallIncrementX.update();
         filteredDesiredCenterOfMassLargeIncrementY.update();
         filteredDesiredCenterOfMassSmallIncrementY.update();
      }

      public void getPosition(FramePoint framePointToPack)
      {
//         double x = desiredCenterOfMassLargeIncrementX.getDoubleValue() + desiredCenterOfMassSmallIncrementX.getDoubleValue();
//         double y = desiredCenterOfMassLargeIncrementY.getDoubleValue() + desiredCenterOfMassSmallIncrementY.getDoubleValue();
         double x = filteredDesiredCenterOfMassLargeIncrementX.getDoubleValue() + filteredDesiredCenterOfMassSmallIncrementX.getDoubleValue();
         double y = filteredDesiredCenterOfMassLargeIncrementY.getDoubleValue() + filteredDesiredCenterOfMassSmallIncrementY.getDoubleValue();

         framePointToPack.setToZero(trotLineFrame);
//         framePointToPack.changeFrame(trotLineFrame);
         framePointToPack.add(x, y, 0.0);
      }

      public void setPosition(FramePoint newPosition, boolean reset)
      {
         newPosition.changeFrame(trotLineFrame);

         double x = newPosition.getX();
         double y = newPosition.getY();

         desiredCenterOfMassLargeIncrementX.set(x);
         desiredCenterOfMassSmallIncrementX.set(0.0);

         desiredCenterOfMassLargeIncrementY.set(y);
         desiredCenterOfMassSmallIncrementY.set(0.0);

         if (reset)
         {
            filteredDesiredCenterOfMassLargeIncrementX.reset();
            filteredDesiredCenterOfMassSmallIncrementX.reset();

            filteredDesiredCenterOfMassLargeIncrementX.update();
            filteredDesiredCenterOfMassSmallIncrementX.update();

            filteredDesiredCenterOfMassLargeIncrementY.reset();
            filteredDesiredCenterOfMassSmallIncrementY.reset();

            filteredDesiredCenterOfMassLargeIncrementY.update();
            filteredDesiredCenterOfMassSmallIncrementY.update();
            update();
         }
      }
   }

   private class MoveCenterOfMassAround extends State<COM_ESTIMATE_STATES>
   {
      private final EnumYoVariable<TrotPair> frameToIncrementOver = new EnumYoVariable<>("trotPairToMoveCenterOfMassOver", registry, TrotPair.class);
      private final ReferenceFrame rootFrame = referenceFrames.getRootJointFrame();
      private final YoFramePoint rootJointToCenterOfTrotLines = new YoFramePoint("rootJointToCenterOfTrotLines", rootFrame, registry);
      private final YoFramePoint estimatedBodyCenterOfMassPosition = new YoFramePoint("estimatedBodyCenterOfMassPosition", rootFrame, registry);
      private final FramePoint centerOfTrotLinesFramePoint = new FramePoint();
      private final FramePoint shiftedCenterOfMass = new FramePoint();
      private final FramePoint intialCenterOfMass = new FramePoint();
      private final TranslationReferenceFrame intialCenterOfMassReferenceFrame = new TranslationReferenceFrame("intialCenterOfMassReferenceFrame", worldFrame);
      private final HashMap<TrotPair, CenterOfMassIncrementVariableHolder> incrementHolders = new HashMap<>();
      private final double totalMass;
      private final double bodyMass;
      private final BooleanYoVariable shiftCoMOverCenterOfFeet = new BooleanYoVariable("shiftCoMOverCenterOfFeet", registry);

      public MoveCenterOfMassAround()
      {
         super(COM_ESTIMATE_STATES.MOVE_COM_AROUND);
         frameToIncrementOver.set(TrotPair.NONE);
         intialCenterOfMass.changeFrame(worldFrame);
         desiredCenterOfMassPosition.getFrameTuple(intialCenterOfMass);
         intialCenterOfMassReferenceFrame.updateTranslation(intialCenterOfMass);

         totalMass = fullRobotModel.getTotalMass();
         bodyMass = fullRobotModel.getPelvis().getInertia().getMass();

         for (TrotPair trotPair : TrotPair.values())
         {
            ReferenceFrame trotLineReferenceFrame = getReferenceFrameForTrotPair(trotPair);
            CenterOfMassIncrementVariableHolder centerOfMassIncrementHolder = new CenterOfMassIncrementVariableHolder(trotPair, trotLineReferenceFrame, intialCenterOfMassReferenceFrame);
            centerOfMassIncrementHolder.setPosition(intialCenterOfMass, true);
            incrementHolders.put(trotPair, centerOfMassIncrementHolder);
         }
      }

      private ReferenceFrame getReferenceFrameForTrotPair(TrotPair trotPair)
      {
         switch (trotPair)
         {
         case HINDLEFT_FRONTRIGHT:
            return getMidTrotLineZUpFrame(RobotQuadrant.HIND_LEFT);

         case HINDRIGHT_FRONTLEFT:
            return getMidTrotLineZUpFrame(RobotQuadrant.HIND_RIGHT);

         default:
            return worldFrame;
         }
      }

      public ReferenceFrame getMidTrotLineZUpFrame(RobotQuadrant quadrantAssocaitedWithTrotLine)
      {
         if(quadrantAssocaitedWithTrotLine.isQuadrantInHind())
         {
            return sideDependentMidTrotLineZUpFrames.get(quadrantAssocaitedWithTrotLine.getSide());
         }
         return sideDependentMidTrotLineZUpFrames.get(quadrantAssocaitedWithTrotLine.getOppositeSide());
      }

      @Override
      public void doAction()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            sideDependentMidTrotLineZUpFrames.get(robotSide).update();
         }

         TrotPair currentTrotPair = frameToIncrementOver.getEnumValue();

         CenterOfMassIncrementVariableHolder incrementHolder = incrementHolders.get(currentTrotPair);
         centerOfTrotLinesFramePoint.setToZero(referenceFrames.getCenterOfFourFeetFrame());

         if(shiftCoMOverCenterOfFeet.getBooleanValue())
         {
            centerOfTrotLinesFramePoint.changeFrame(worldFrame);
            shiftCoMOverCenterOfFeet.set(false);
            incrementHolder.setPosition(centerOfTrotLinesFramePoint, false);
         }
         incrementHolder.update();

         incrementHolder.getPosition(shiftedCenterOfMass);
         shiftedCenterOfMass.changeFrame(worldFrame);
         desiredCenterOfMassPosition.set(shiftedCenterOfMass);

         centerOfTrotLinesFramePoint.changeFrame(rootFrame);
         rootJointToCenterOfTrotLines.set(centerOfTrotLinesFramePoint);


         estimatedBodyCenterOfMassPosition.set(centerOfTrotLinesFramePoint);
         estimatedBodyCenterOfMassPosition.scale(totalMass / bodyMass);

         for(TrotPair trotPairToUpdate : TrotPair.values)
         {
            if(trotPairToUpdate != currentTrotPair)
            {
               CenterOfMassIncrementVariableHolder centerOfMassIncrementVariableHolder = incrementHolders.get(trotPairToUpdate);
               centerOfMassIncrementVariableHolder.setPosition(shiftedCenterOfMass, true);
            }
         }
      }

      @Override
      public void doTransitionIntoAction()
      {

      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   private class MoveFeet extends State<COM_ESTIMATE_STATES>
   {
      private final DoubleYoVariable zHeight;
      private final COM_ESTIMATE_STATES state;
      private EnumYoVariable<TrotPair> currentTrotPairToMove;
      private EnumYoVariable<TrotPair> trotPairSource;

      public MoveFeet(String name, DoubleYoVariable zHeight, EnumYoVariable<TrotPair> trotPairToManage, COM_ESTIMATE_STATES state)
      {
         super(state);
         this.state = state;
         this.zHeight = zHeight;
         this.trotPairSource = trotPairToManage;
         this.currentTrotPairToMove = new EnumYoVariable<>(name + "CurrentTrotPairToMove", registry, TrotPair.class);
         this.setDefaultNextState(COM_ESTIMATE_STATES.MOVE_COM_AROUND);
      }

      @Override
      public void doAction()
      {
         double timeInState = this.getTimeInCurrentState();

         if (timeInState > swingTime.getDoubleValue() + timeToSTayInMoveFeetAfterTouchDown.getDoubleValue())
         {
            this.transitionToDefaultNextState();
         }

         TrotPair trotPair = currentTrotPairToMove.getEnumValue();
         RobotQuadrant[] trotPairQuadrants = trotPair.getQuadrants();

         if (state == COM_ESTIMATE_STATES.PUT_DOWN_FEET && trotPair == TrotPair.NONE)
         {
            TrotPair feetInAir = trotPairInAir.getEnumValue();
            trotPairQuadrants = feetInAir.getQuadrants();
         }

         for (int i = 0; i < trotPairQuadrants.length; i++)
         {
            RobotQuadrant robotQuadrant = trotPairQuadrants[i];
            YoFramePoint desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
            YoMinimumJerkTrajectory yoMinimumJerkTrajectory = footTrajectories.get(robotQuadrant);
            yoMinimumJerkTrajectory.computeTrajectory(timeInState);
            double newZ = yoMinimumJerkTrajectory.getPosition();
            desiredFootLocation.setZ(newZ);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {

         TrotPair trotPair = trotPairSource.getEnumValue();
         currentTrotPairToMove.set(trotPair);
         RobotQuadrant[] trotPairQuadrants = trotPair.getQuadrants();

         for (int i = 0; i < trotPairQuadrants.length; i++)
         {
            RobotQuadrant robotQuadrant = trotPairQuadrants[i];
            YoFramePoint desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
            YoMinimumJerkTrajectory yoMinimumJerkTrajectory = footTrajectories.get(robotQuadrant);
            double footZHeight = desiredFootLocation.getZ();
            yoMinimumJerkTrajectory.setParams(footZHeight, 0.0, 0.0, zHeight.getDoubleValue(), 0.0, 0.0, 0.0, swingTime.getDoubleValue());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (state == COM_ESTIMATE_STATES.PUT_DOWN_FEET)
         {
            trotPairInAir.set(TrotPair.NONE);
         }

         if (state == COM_ESTIMATE_STATES.PICK_UP_FEET)
         {
            trotPairInAir.set(currentTrotPairToMove.getEnumValue());
         }
      }
   }

   private class FilterDesiredsToMatchCrawlControllerState extends State<COM_ESTIMATE_STATES>
   {
      private final AlphaFilteredYoVariable filterStandPrepDesiredsToWalkingDesireds;
      private final FullRobotModel initialDesiredsUponEnteringFullRobotModel;

      public FilterDesiredsToMatchCrawlControllerState(QuadrupedModelFactory modelFactory)
      {
         super(COM_ESTIMATE_STATES.ALPHA_FILTERING_DESIREDS);

         initialDesiredsUponEnteringFullRobotModel = modelFactory.createFullRobotModel();

         double filterStandPrepDesiredsToWalkingDesiredsAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.5, dt);
         filterStandPrepDesiredsToWalkingDesireds = new AlphaFilteredYoVariable("filterStandPrepDesiredsToWalkingDesireds", registry, filterStandPrepDesiredsToWalkingDesiredsAlpha);
      }

      public void filterDesireds(OneDoFJoint[] oneDoFJointsActual)
      {
         for (int i = 0; i < oneDoFJointsActual.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJointsActual[i];
            String jointName = actualOneDoFJoint.getName();

            OneDoFJoint intialOneDoFJoint = initialDesiredsUponEnteringFullRobotModel.getOneDoFJointByName(jointName);
            //            OneDoFJoint desiredOneDofJoint = finalDesiredsFullRobotModel.getOneDoFJointByName(jointName);

            double alpha = filterStandPrepDesiredsToWalkingDesireds.getDoubleValue();

            double alphaFilteredQ = (1.0 - alpha) * intialOneDoFJoint.getqDesired() + alpha * actualOneDoFJoint.getqDesired();
            actualOneDoFJoint.setqDesired(alphaFilteredQ);
         }
      }

      public boolean isInterpolationFinished()
      {
         return filterStandPrepDesiredsToWalkingDesireds.getDoubleValue() >= 0.9999;
      }

      @Override
      public void doAction()
      {
         filterStandPrepDesiredsToWalkingDesireds.update(1.0);
      }

      @Override
      public void doTransitionIntoAction()
      {
         filterStandPrepDesiredsToWalkingDesireds.reset();
         filterStandPrepDesiredsToWalkingDesireds.update(0.0);

         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJoints[i];
            OneDoFJoint initialOneDofJoint = initialDesiredsUponEnteringFullRobotModel.getOneDoFJointByName(actualOneDoFJoint.getName());
            initialOneDofJoint.setqDesired(actualOneDoFJoint.getqDesired());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void onEntry()
   {
      for (OneDoFJoint oneDofJoint : oneDoFJoints)
      {
         oneDofJoint.setUnderPositionControl(true);
      }

      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      stateMachine.setCurrentState(COM_ESTIMATE_STATES.ALPHA_FILTERING_DESIREDS);
   }

   @Override
   public void onExit()
   {

   }

   private enum TrotPair
   {
      NONE(), HINDLEFT_FRONTRIGHT(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_RIGHT), HINDRIGHT_FRONTLEFT(RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_LEFT);

      public static final TrotPair[] values = values();
      private final RobotQuadrant[] quadrants;

      TrotPair(RobotQuadrant hind, RobotQuadrant front)
      {
         quadrants = new RobotQuadrant[] {hind, front};
      }

      public String getFrameName()
      {
         switch (this)
         {
         case HINDLEFT_FRONTRIGHT:
            return "HINDLEFT_FRONTRIGHT";
         case HINDRIGHT_FRONTLEFT:
            return "HINDRIGHT_FRONTLEFT";
         default:
            return "world";
         }
      }

      TrotPair()
      {
         quadrants = null;
      }

      public RobotQuadrant[] getQuadrants()
      {
         return quadrants;

      }
   }
}
