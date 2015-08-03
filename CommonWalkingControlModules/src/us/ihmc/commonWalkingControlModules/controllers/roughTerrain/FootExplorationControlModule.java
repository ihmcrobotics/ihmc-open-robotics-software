package us.ihmc.commonWalkingControlModules.controllers.roughTerrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePlane3d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

/**
 * This module can be used to allow a terrain exploration using the foot. 
 * The module explores the next foothold moving the foot CoP while gradually shifting
 * the weight of the robot from one foot to the other. 
 * Eventually a new foothold is computed based on the performance of the previous test and 
 * the contact points of the foot are updated to take into account the limits of the foothold.
 * If the area of the new foothold is too small the module can request a re-planning of the footstep.
 * 
 * Note. this module to work needs the LookAheadCoMHeightTrajectoryGenerator.
 *  
 * @author Robot Lab
 *
 */
public class FootExplorationControlModule
{
   public static final boolean IS_ACTIVE = false;

   public enum FeetExplorationState
   {
      SWING, EXPLORATION, ICP_SHIFT, RECOVER
   }

   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // control of CoM height
   private static boolean adjustCoMHeightBasedOnICP = false;
   private static boolean lowerCoMDuringSwing = false && !adjustCoMHeightBasedOnICP;
   private static double maxCoMHeightOffset = 0.00;
   private static double minCoMHeightOffset = -0.10; // for simulation use -0.1, real robot -0.03. Note: This in general is useful if the steps are too far each other, so use short steps

   // exploration state parameters
   private static double icpShiftTime = 3;
   private static double copShiftTime = 2;
   private static double copTransitionRestingTime = 1; // must be less than copShiftTime/2
   private static double ICPShiftRestingTime = 2;
   private static double copToContactPointScalingFactor = 0.8; // how far we explore, max 1.0
   private static double copPercentageToSwitchToNextContactPoint = 2; // if this value is >1 the exploration will continue to keep the desired cop at the maximum value of copToContactPointScalingFactor
   private static double maxAbsCoPError = 0.04;
   private static boolean adjustContactPointInXNOnly = true;
   
   // swing state parameters
   private static double icpBiasToInnerSole = 0.01;
   private static double swingTimeForExploration = 4;
   private static double safetyScalingOfContactPoints = 1.0; //0.85
   private static boolean forceFootCoPOverCMP = true;
   private static double footCoPOverCMPGainForQPSolver = 2.0;

   // unstable situations
   private static double initialMaxICPAbsError = 0.04;
   private DoubleYoVariable maxICPAbsError;
   private static double maxICPAbsErrorForSingleSupport = 0.01;
   private static double minCoPDistanceFromBorder = 0.01;
   private static double recoverStateContactPointsScaleFactor = 0.9;
   private static double recoverTime = 5;
   private static int minimumNumberOfContactPointsToHaveAPolygon = 3;

   // parameter of tilt foot
   private static double zeroVelocity = 0.2;
   private static double alphaForJointVelocity = 0.95;
   private static double alphaForFootAngularRate = 0.95;
   private static int numberOfJointCheckedForZeroVelocity = 2;

   //  ICP positions parameters
   private static double firstInternalICPPercentage = 0.95;
   private static double firstExternalICPPercentage = 0.3;
   private static double secondExternalICPPercentage = 0.5;
   private static double lastExternalICPPercentage = 1.0;
   private static double icpPercentageToConsiderTransferFinished = 0.99;
   private static double icpPercentageToDetermineIfIsLastShift = 0.95;
   private static double maxStallTimeToActivateDoToeOff = 2;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable performCoPExploration;
   private final BooleanYoVariable footHoldIsSafeEnough;
   private final DoubleYoVariable initialExplorationTime;
   private final MomentumBasedController momentumBasedController;
   private final FootExplorationCoPPlanner footExplorationCoPPlanner;
   private final BooleanYoVariable isMasteringFullExploration;
   private final BooleanYoVariable swingIsFinished;
   private final DoubleYoVariable ICPPositionPercentage;
   private final DoubleYoVariable finalICPPositionPercentage;
   private final FootExplorationICPPlanner footExplorationICPPlanner;
   private final BooleanYoVariable icpIsCloseEnoughToDesired;
   private final DoubleYoVariable explorationICPErrorAbs;
   private final IntegerYoVariable icpIndex;
   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable transferToNextFootStepIsDone;
   private final BooleanYoVariable icpIsCloseEnoughForSingleSupport;
   private final DoubleYoVariable footstepCentroidsDistance;
   private final StateMachine<FeetExplorationState> stateMachine;
   private final List<? extends PlaneContactState> planeContactStates;
   private final IntegerYoVariable numberOfPlaneContactStates;
   private final IntegerYoVariable currentContactPointNumber;
   private final LookAheadCoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final DoubleYoVariable comIncrement;
   private final List<ContactablePlaneBody> contactablePlaneBodyList;
   private final FeetManager feetManager;
   private final BooleanYoVariable defaultDoToeOff;
   private final DoubleYoVariable defaultSwingTime;
   private final HashMap<PlaneContactState, Boolean> needToResetContactPoints;
   private final HashMap<PlaneContactState, ArrayList<Point2d>> defaultContactPoints;
   private final BooleanYoVariable isRecovering;
   private final BooleanYoVariable doToeOffIfPossibleHasBeenChanged;
   private final BooleanYoVariable swingTimeHasBeenChanged;
   private final BooleanYoVariable comHeightOffsetHasBeenChanged;
   private final DoubleYoVariable defaultCOMHeightOffset;
   private final DoubleYoVariable feetExplorationCoPError;
   private final SideDependentList<DoubleYoVariable> footAngularRate = new SideDependentList<DoubleYoVariable>();
   private final AlphaFilteredYoVariable ankleJointVelocity;
   private final DoubleYoVariable initialStallTime;
   private final DoubleYoVariable timeInStallBecauseICPIsNotCloseEnough;

   private InverseDynamicsJoint jointX, jointY;
   private DenseMatrix64F jointVelocityMatrix;
   private Footstep nextFootStep; // be aware that is the nextfootstep from WalkingHighLevel (is not a copy), use only to re-plan
   private RobotSide footUderCoPControl; // be aware that is the swingSide from WalkingHighLevel (is not a copy)
   private FrameConvexPolygon2d supportFootPolygon;
   private FramePoint supportFootCentroid;
   private FramePoint nextFootStepCentroid;
   private FrameLine2d ICPTrajectory;
   private FramePoint2d desiredICPLocal;
   private FramePoint2d startCoP;
   private FrameVector2d desiredICPVelocityLocal;
   private ArrayList<Double> constantICPPointPercentages;
   private PlaneContactState currentPlaneContactStateToExplore;
   private PlaneContactState currentPlaneContactInSupport;
   private PlaneContactState planeContactStateToRescale;
   private FramePoint initialCoP;
   private FramePoint finalCoP;
   private ReferenceFrame CoPFrame;
   private RobotSide loadBearingSide;
   private final FramePoint2d tempFramePoint2d = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();
   // to delete
   static
   {
      if (false)
      {
         icpShiftTime = 2;
         copShiftTime = 1;
         copTransitionRestingTime = .5; // must be less than copShiftTime/2
         ICPShiftRestingTime = 1;
      }
   }

   public FootExplorationControlModule(YoVariableRegistry parentRegistry, MomentumBasedController momentumBasedController, DoubleYoVariable yoTime,
         CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, SwingTimeCalculationProvider swingTimeCalculationProvider, 
         FeetManager feetManager)
   {
      performCoPExploration = new BooleanYoVariable("performCoPExploration", registry);
      footHoldIsSafeEnough = new BooleanYoVariable("footHoldIsSafe", registry);
      initialExplorationTime = new DoubleYoVariable("initialExplorationTime", registry);
      isMasteringFullExploration = new BooleanYoVariable("isMasteringFullExploration", registry);
      swingIsFinished = new BooleanYoVariable("swingIsFinished", registry);
      ICPPositionPercentage = new DoubleYoVariable("ICPPositionPercentage", registry);
      finalICPPositionPercentage = new DoubleYoVariable("finalICPPositionPercentage", registry);
      icpIsCloseEnoughToDesired = new BooleanYoVariable("icpIsCloseEnoughToDesired", registry);
      explorationICPErrorAbs = new DoubleYoVariable("explorationICPErrorAbs", registry);
      maxICPAbsError = new DoubleYoVariable("maxICPAbsError", registry);
      maxICPAbsError.set(initialMaxICPAbsError);
      footExplorationICPPlanner = new FootExplorationICPPlanner("footExplorationICPPlanner", worldFrame, totalPercentageProvider, initialICPPositionProvider,
            finalICPPositionProvider, registry);
      planeContactStates = momentumBasedController.getPlaneContactStates();
      contactablePlaneBodyList = momentumBasedController.getContactablePlaneBodyList();
      icpIsCloseEnoughForSingleSupport = new BooleanYoVariable("icpIsCloseEnoughForSingleSupport", registry);
      footstepCentroidsDistance = new DoubleYoVariable("footstepCentroidsDistance", registry);
      transferToNextFootStepIsDone = new BooleanYoVariable("transferToNextFootStepIsDone", registry);
      numberOfPlaneContactStates = new IntegerYoVariable("numberOfPlaneContactStates", registry);
      defaultDoToeOff = new BooleanYoVariable("defaultDoToeOff", registry);
      defaultSwingTime = new DoubleYoVariable("defaultSwingTime", registry);
      icpIndex = new IntegerYoVariable("icpIndex", registry);
      isRecovering = new BooleanYoVariable("isRecovering", registry);
      String namePrefix = "feetExploration";
      stateMachine = new StateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", FeetExplorationState.class, yoTime, registry);
      footExplorationCoPPlanner = new FootExplorationCoPPlanner("footExplorationCoPPlanner", worldFrame, totalPercentageProvider, initialCoPPositionProvider,
            finalCoPPositionProvider, registry);
      ankleJointVelocity = new AlphaFilteredYoVariable("ankleJointVelocity", registry, alphaForJointVelocity);
      for (RobotSide side : RobotSide.values)
      {
         footAngularRate.put(side, new AlphaFilteredYoVariable(side.name() + "FootAngularRate", registry, alphaForFootAngularRate));
      }
      currentContactPointNumber = new IntegerYoVariable("currentContactPointNumber", registry);
      feetExplorationCoPError = new DoubleYoVariable("feetExplorationCoPError", registry);

      comIncrement = new DoubleYoVariable("comIncrement", registry);
      doToeOffIfPossibleHasBeenChanged = new BooleanYoVariable("doToeOffIfPossibleHasBeenChanged", registry);
      swingTimeHasBeenChanged = new BooleanYoVariable("swingTimeHasBeenChanged", registry);
      comHeightOffsetHasBeenChanged = new BooleanYoVariable("comHeightOffsetHasBeenChanged", registry);
      defaultCOMHeightOffset = new DoubleYoVariable("defaultCOMHeightOffset", registry);
      needToResetContactPoints = new HashMap<PlaneContactState, Boolean>();
      defaultContactPoints = new HashMap<PlaneContactState, ArrayList<Point2d>>();
      initialStallTime = new DoubleYoVariable("initialStallTime", registry);
      timeInStallBecauseICPIsNotCloseEnough = new DoubleYoVariable("timeInStallBecauseICPIsNotCloseEnough", registry);
      
      this.yoTime = yoTime;
      this.feetManager = feetManager;
      this.momentumBasedController = momentumBasedController;
      this.centerOfMassHeightTrajectoryGenerator = (LookAheadCoMHeightTrajectoryGenerator) centerOfMassHeightTrajectoryGenerator;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      performCoPExploration.set(IS_ACTIVE);
      ICPTrajectory = new FrameLine2d(worldFrame, new Point2d(0.0, 0.0), new Point2d(1.0, 1.0));
      desiredICPLocal = new FramePoint2d(worldFrame);
      desiredICPVelocityLocal = new FrameVector2d(worldFrame);
      nextFootStepCentroid = new FramePoint(worldFrame);
      supportFootCentroid = new FramePoint(worldFrame);
      constantICPPointPercentages = new ArrayList<Double>();

      numberOfPlaneContactStates.set(planeContactStates.size());
      initialExplorationTime.set(Double.NEGATIVE_INFINITY);
      doToeOffIfPossibleHasBeenChanged.set(false);
      swingTimeHasBeenChanged.set(false);
      comHeightOffsetHasBeenChanged.set(false);

      setupStateMachine();
      reset();

      parentRegistry.addChild(registry);
   }

   /**
    * This is the main method of the module. When the module is active this method is called from the
    * high level every control tick. 
    *  
    * @param desiredICPToPack - desired ICP to pack
    * @param desiredICPVelocityToPack - desired ICP velocity to pack
    * @param currentCapturePoint - current ICP
    */
   public void masterFullExploration(YoFramePoint2d desiredICPToPack, YoFrameVector2d desiredICPVelocityToPack, FramePoint2d currentCapturePoint, FramePoint2d desiredCMP)
   {
      updateFootAngularRate();
      updateAnkleVelocity();

      this.desiredCMP.setIncludingFrame(desiredCMP);
      desiredICPLocal.changeFrame(desiredICPToPack.getReferenceFrame());
      desiredICPVelocityLocal.changeFrame(desiredICPVelocityToPack.getReferenceFrame());

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      desiredICPToPack.set(desiredICPLocal.getX(), desiredICPLocal.getY());
      desiredICPVelocityToPack.set(desiredICPVelocityLocal.getX(), desiredICPVelocityLocal.getY());
      explorationICPErrorAbs.set(Math.abs(currentCapturePoint.distance(desiredICPLocal)));
      icpIsCloseEnoughToDesired.set(explorationICPErrorAbs.getDoubleValue() < maxICPAbsError.getDoubleValue());
      icpIsCloseEnoughForSingleSupport.set(explorationICPErrorAbs.getDoubleValue() < maxICPAbsErrorForSingleSupport);

      boolean explorationIsFinishedAndICPIsCloseToFinalPosition = checkIfExplorationIsFinished();
      if (explorationIsFinishedAndICPIsCloseToFinalPosition)
      {
         scaleContactPointLocation(currentPlaneContactStateToExplore, safetyScalingOfContactPoints);
         planeContactStateToRescale = currentPlaneContactStateToExplore;
         currentPlaneContactInSupport = currentPlaneContactStateToExplore;
         transferToNextFootStepIsDone.set(true);
      }
   }

   /**
    * This method initialize the variables for the exploration only if an exploration has 
    * been requested. Furthermore when some high level variables are changed, we store 
    * their original value and set 'true' the corresponding 'hasBeenChanged' variable, in 
    * this way when the exploration is finished we can restore the original value.
    *    
    * @param nextFootStep - next footstep that, if module is active, will be explored
    * @param supportFootPolygon - current support foot polygon
    * @param footUderCoPControl - side, left or right, of the foot that, if the module is 
    *                             active, will be used for the exploration
    */
   public void initialize(Footstep nextFootStep, FrameConvexPolygon2d supportFootPolygon, RobotSide footUderCoPControl)
   {
      if (nextFootStep != null)
      {
         for (int i = 0; i < numberOfPlaneContactStates.getIntegerValue(); i++)
         {
            PlaneContactState planeContactState = planeContactStates.get(i);
            RigidBody planeBody = planeContactState.getRigidBody();
            RigidBody nextFootStepBody = nextFootStep.getBody();

            if (planeBody.equals(nextFootStepBody))
               currentPlaneContactStateToExplore = planeContactState;
         }

         if (needToResetContactPoints.get(currentPlaneContactStateToExplore) != null && needToResetContactPoints.get(currentPlaneContactStateToExplore))
         {
            resetContactPoints(defaultContactPoints.get(currentPlaneContactStateToExplore), currentPlaneContactStateToExplore.getContactPoints());
            needToResetContactPoints.put(currentPlaneContactStateToExplore, false);
         }
      }

      if (performCoPExploration.getBooleanValue() && nextFootStep != null)
      {
         this.nextFootStep = nextFootStep;
         this.footUderCoPControl = footUderCoPControl;
         this.loadBearingSide = footUderCoPControl.getOppositeSide();
         this.supportFootPolygon = new FrameConvexPolygon2d(supportFootPolygon);

         FramePoint2d tempCentroid = supportFootPolygon.getCentroid();
         supportFootCentroid = new FramePoint(tempCentroid.getReferenceFrame(), tempCentroid.getX(), tempCentroid.getY(), 0.0);
         supportFootCentroid.changeFrame(worldFrame);

         ReferenceFrame nextFootStepSoleFrame = nextFootStep.getSoleReferenceFrame();
         nextFootStepCentroid.setToZero(nextFootStepSoleFrame);
         nextFootStepCentroid.changeFrame(worldFrame);

         ICPTrajectory.set(worldFrame, supportFootCentroid.getX(), supportFootCentroid.getY(), nextFootStepCentroid.getX(), nextFootStepCentroid.getY());
         computeConstantICPPoints();

         ContactablePlaneBody contactablePlaneBodyOfFootUnderCoPControl = momentumBasedController.getContactableFeet().get(footUderCoPControl);
         CoPFrame = contactablePlaneBodyOfFootUnderCoPControl.getSoleFrame();
         startCoP = new FramePoint2d(CoPFrame, 0.0, 0.0);

         jointX = currentPlaneContactStateToExplore.getRigidBody().getParentJoint();
         jointY = currentPlaneContactStateToExplore.getRigidBody().getParentJoint().getPredecessor().getParentJoint();
         jointVelocityMatrix = new DenseMatrix64F(numberOfJointCheckedForZeroVelocity, 1);

         defaultDoToeOff.set(feetManager.getWalkOnTheEdgesManager().doToeOffIfPossible());
         feetManager.getWalkOnTheEdgesManager().setDoToeOffIfPossible(false);
         doToeOffIfPossibleHasBeenChanged.set(true);

         defaultCOMHeightOffset.set(centerOfMassHeightTrajectoryGenerator.getOffsetHeightAboveGround());
         comHeightOffsetHasBeenChanged.set(true);

         ArrayList<Point2d> currentPlaneContactPoints = getCopyOfContactPointLocation(currentPlaneContactStateToExplore.getContactPoints());
         defaultContactPoints.put(currentPlaneContactStateToExplore, currentPlaneContactPoints);
         needToResetContactPoints.put(currentPlaneContactStateToExplore, true);

         defaultSwingTime.set(swingTimeCalculationProvider.getValue());
         swingTimeCalculationProvider.setSwingTime(swingTimeForExploration);
         swingTimeHasBeenChanged.set(true);

         isMasteringFullExploration.set(true);
         footExplorationICPPlanner.initialize();

         stateMachine.setCurrentState(FeetExplorationState.SWING);
      }
   }

   public void reset()
   {
      isMasteringFullExploration.set(false);
      //      footHoldIsSafeEnough.set(false); // TODO define better in the exploration phase and if is not safe enough (have a look to the area) re-plan
      footHoldIsSafeEnough.set(true);
      swingIsFinished.set(false);
      icpIsCloseEnoughToDesired.set(false);
      constantICPPointPercentages.clear();
      icpIndex.set(0);
      transferToNextFootStepIsDone.set(false);
      ICPPositionPercentage.set(0.0);
      initialStallTime.set(Double.NaN);
      timeInStallBecauseICPIsNotCloseEnough.set(0.0);
      resetWalkingVariableIfNecessary();
   }

   private void setupStateMachine()
   {
      State<FeetExplorationState> performingSwingState = new PerformingSwingState();
      State<FeetExplorationState> explorationState = new ExplorationState();
      State<FeetExplorationState> icpShiftingState = new ICPShiftingState();
      State<FeetExplorationState> recover = new RecoverState();

      DoneWithSwingCondition doneWithSwingCondition = new DoneWithSwingCondition();
      DoneWithSubExplorationCondition doneWithExplorationCondition = new DoneWithSubExplorationCondition();
      DoneWithICPShiftCondition doneWithICPShiftCondition = new DoneWithICPShiftCondition();
      UnsafeICPCondition unsafeICPCondition = new UnsafeICPCondition();
      DoneWithRecoverCondition doneWithRecoverCondition = new DoneWithRecoverCondition();

      StateTransition<FeetExplorationState> toICPShiftFromSwing = new StateTransition<FeetExplorationState>(FeetExplorationState.ICP_SHIFT,
            doneWithSwingCondition);
      StateTransition<FeetExplorationState> toICPShiftFromSubExploration = new StateTransition<FeetExplorationState>(FeetExplorationState.ICP_SHIFT,
            doneWithExplorationCondition);
      StateTransition<FeetExplorationState> toExploration = new StateTransition<FeetExplorationState>(FeetExplorationState.EXPLORATION,
            doneWithICPShiftCondition);
      StateTransition<FeetExplorationState> toRecover = new StateTransition<FeetExplorationState>(FeetExplorationState.RECOVER, unsafeICPCondition);
      StateTransition<FeetExplorationState> toExplorationFromRecover = new StateTransition<FeetExplorationState>(FeetExplorationState.EXPLORATION,
            doneWithRecoverCondition);

      performingSwingState.addStateTransition(toICPShiftFromSwing);
      explorationState.addStateTransition(toRecover);
      explorationState.addStateTransition(toICPShiftFromSubExploration);
      icpShiftingState.addStateTransition(toRecover);
      icpShiftingState.addStateTransition(toExploration);
      recover.addStateTransition(toExplorationFromRecover);

      stateMachine.addState(performingSwingState);
      stateMachine.addState(explorationState);
      stateMachine.addState(icpShiftingState);
      stateMachine.addState(recover);
   }

   /**
    * This is the first state. It controls the swing of the foot keeping the ICP inside of the 
    * polygon of the support foot.
    *
    */
   private class PerformingSwingState extends State<FeetExplorationState>
   {
      private FramePoint2d constantICP;
      private double timeInSwing;
      private double initialSwingTime;
      private FrameVector cmpNormal; 
      private FramePoint cmpPointCopy;
      private FrameLine cmpLine;
      private FrameVector soleNormal; 
      private FramePoint solePoint;
      private FramePlane3d solePlane;
      private FramePoint projectedCMP;

      public PerformingSwingState()
      {
         super(FeetExplorationState.SWING);
         
         cmpNormal = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
         cmpPointCopy = new FramePoint(worldFrame);
         cmpLine = new FrameLine(cmpPointCopy, cmpNormal);
         soleNormal = new FrameVector(worldFrame);
         solePoint = new FramePoint(worldFrame);
         solePlane = new FramePlane3d(worldFrame);
         projectedCMP = new FramePoint(worldFrame);
      }

      @Override
      public void doAction()
      {
         if (forceFootCoPOverCMP)
         {
            FramePoint projectedCmp = getProjectedCMPOnFootSole();
            momentumBasedController.setFootCoPOffsetX(projectedCmp.getX());
            momentumBasedController.setFootCoPOffsetY(projectedCmp.getY());
            
            timeInSwing = yoTime.getDoubleValue() - initialSwingTime;
            FootSwitchInterface footSwitchInterface = momentumBasedController.getFootSwitches().get(footUderCoPControl);
            WrenchBasedFootSwitch wrenchBasedFootSwitch = (WrenchBasedFootSwitch) footSwitchInterface;
            if (wrenchBasedFootSwitch.getForceMagnitudePastThreshhold() && timeInSwing > swingTimeForExploration / 2)
               momentumBasedController.setFeetCoPControlIsActive(false);
         }

         if (adjustCoMHeightBasedOnICP)
         {
            lowerCoMBasedOnBoolean(icpIsCloseEnoughForSingleSupport.getBooleanValue());
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         double bias = loadBearingSide.negateIfLeftSide(icpBiasToInnerSole);
         FrameVector biasVector = new FrameVector(CoPFrame, 0.0, bias, 0.0, "icpBias");
         biasVector.changeFrame(worldFrame);
         double desiredICPY = supportFootCentroid.getY() + biasVector.getY();
         double desiredICPX = supportFootCentroid.getX() + biasVector.getX();
         constantICP = new FramePoint2d(worldFrame, desiredICPX, desiredICPY);

         initialSwingTime = yoTime.getDoubleValue();
         if (forceFootCoPOverCMP)
         {
            momentumBasedController.setFeetCoPControlIsActive(true);
            momentumBasedController.setSideOfFootUnderCoPControl(loadBearingSide);
         }

         setICPLocalToDesired(constantICP);
         desiredICPVelocityLocal.setToZero();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         finalICPPositionPercentage.set(constantICPPointPercentages.get(0));
         icpIndex.increment();
         swingTimeCalculationProvider.setSwingTime(defaultSwingTime.getDoubleValue());
         if (planeContactStateToRescale != null)
         {
            scaleContactPointLocation(planeContactStateToRescale, 1 / safetyScalingOfContactPoints);
            planeContactStateToRescale = null;
         }

         updateNextFootstepCentroid();
         footExplorationICPPlanner.initialize();
         
         if (forceFootCoPOverCMP)
            momentumBasedController.setFeetCoPControlIsActive(false);
      }
      
      /**
       * This method computes the projection of the CMP (which is assumed to be drawn on a flat ground)
       * on the foot sole of the current load-bearing leg. The projection is done in order to have a point 
       * on the sole where we can try to keep the local foot CoP to increase the stability of the robot 
       * during slow swings and with reduced foot polygons.
       * 
       * Note. For the moment the projection is computed as the intersection of the line passing through the CMP
       * and normal to the ground, and the foot sole plane. In this way we are not using the proper definition of CMP
       * (the line should connect the CMP and the robot CoM), but we always obtain a foot CoP that is 
       * 'pushing a bit more'.
       * 
       * @return FramePoint - CMP in foot sole ReferenceFrame and already projected on the foot sole
       */
      private FramePoint getProjectedCMPOnFootSole()
      {
        ReferenceFrame cmpFrame = desiredCMP.getReferenceFrame();
        cmpNormal.setIncludingFrame(cmpFrame, 0.0, 0.0, 1.0); 
        cmpPointCopy.setIncludingFrame(cmpFrame, desiredCMP.getX(), desiredCMP.getY(), 0.0);
        cmpLine.changeFrame(cmpFrame);
        cmpLine.setDirection(cmpNormal);
        cmpLine.setOrigin(cmpPointCopy);
        
        ReferenceFrame soleFrame = momentumBasedController.getContactableFeet().get(loadBearingSide).getSoleFrame();
        soleNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0); 
        solePoint.setIncludingFrame(soleFrame, 0.0, 0.0, 0.0);
        solePlane.changeFrame(soleFrame);
        solePlane.setNormal(soleNormal.getVector());
        solePlane.setPoint(solePoint.getPoint());
        
        solePlane.changeFrame(cmpFrame);
        
        projectedCMP.changeFrame(cmpFrame); 
        solePlane.getIntersectionWithLine(projectedCMP, cmpLine);

        projectedCMP.changeFrame(soleFrame);
        
        return projectedCMP;
      }
   }

   /**
    * This is the real exploration phase. In this state the module moves the local 
    * CoP of the foot that is used for the exploration and check if the foot is tilting.
    * The CoP is moved along the segment that connects the initial desired CoP to the final
    * desired CoP. The position is computed from the CoP planner using a percentage between 
    * 0 and 1 of the position along the segment.
    *
    */
   private class ExplorationState extends State<FeetExplorationState>
   {
      FramePoint2d tempICP;
      double copVelocity;
      double initialCoPTime;
      int lastContactPointNumber;
      double copTime;
      ContactPointInterface contactPoint;
      private final DoubleYoVariable CoPPositionPercentage = new DoubleYoVariable("CoPPositionPercentage", registry);
      private final DoubleYoVariable explorationTime = new DoubleYoVariable("explorationTime", registry);;
      private final BooleanYoVariable ankleJointIsMoving = new BooleanYoVariable("ankleJointIsMoving", registry);

      public ExplorationState()
      {
         super(FeetExplorationState.EXPLORATION);
         tempICP = new FramePoint2d(worldFrame);
         CoPPositionPercentage.set(Double.NaN);
         explorationTime.set(Double.NaN);
      }

      @Override
      public void doAction()
      {
         //TODO: first attempt to find a suitable startCoP (maybe explore both areas and select the bigger)
         // maybe find the axis of rotation.

         explorationTime.set(yoTime.getDoubleValue() - initialExplorationTime.getDoubleValue());

         if (explorationTime.getDoubleValue() < ICPShiftRestingTime)
         {
            return;
         }

         if (currentContactPointNumber.getIntegerValue() < currentPlaneContactStateToExplore.getTotalNumberOfContactPoints())
         {
            if (currentContactPointNumber.getIntegerValue() != lastContactPointNumber)
            {
               initialCoPTime = yoTime.getDoubleValue();
               lastContactPointNumber = currentContactPointNumber.getIntegerValue();
               contactPoint = currentPlaneContactStateToExplore.getContactPoints().get(currentContactPointNumber.getIntegerValue());
               contactPoint.getPosition2d(tempFramePoint2d);
               copVelocity = 1.0 / copShiftTime;
               initialCoP = new FramePoint(CoPFrame, startCoP.getX(), startCoP.getY(), 0.0);
               finalCoP = new FramePoint(CoPFrame, tempFramePoint2d.getX(), tempFramePoint2d.getY(), 0.0);
               footExplorationCoPPlanner.initialize();
               CoPPositionPercentage.set(0.0);
               ankleJointVelocity.set(0.0);
               setAndGetDesiredFootCop();
            }

            copTime = yoTime.getDoubleValue() - initialCoPTime;

            if (copTime > copTransitionRestingTime)
            {
               double currentCoPPercentage = (copTime - copTransitionRestingTime) * copVelocity;
               adjustContactPointLocationIfNecessary(currentCoPPercentage);

               if (currentCoPPercentage > copPercentageToSwitchToNextContactPoint)
                  currentContactPointNumber.increment();
            }
         }

         if (adjustCoMHeightBasedOnICP)
         {
            lowerCoMBasedOnBoolean(icpIsCloseEnoughToDesired.getBooleanValue());
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         currentContactPointNumber.set(0);
         lastContactPointNumber = -1;
         initialExplorationTime.set(yoTime.getDoubleValue());
         momentumBasedController.setFeetCoPControlIsActive(true);
         momentumBasedController.setSideOfFootUnderCoPControl(footUderCoPControl);
         footExplorationICPPlanner.getICPPosition(tempICP);
         setICPLocalToDesired(tempICP);
         desiredICPVelocityLocal.setToZero();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         momentumBasedController.setFeetCoPControlIsActive(false);
         finalICPPositionPercentage.set(constantICPPointPercentages.get(icpIndex.getIntegerValue()));
         icpIndex.increment();
         CoPPositionPercentage.set(Double.NaN);
         explorationTime.set(Double.NaN);
         updateNextFootstepCentroid();
         footExplorationICPPlanner.initialize();
      }

      /**
       * This method moves the CoP computing a CoPScaledPercentage that is a value used inside of the CoP
       * planner to compute the the desired CoP position as a percentage (between 0 and copToContactPointScalingFactor)
       * of the line that connects the initial and final CoP points. 
       * Furthermore this method adjusts the contact point location if the foot tilts during the exploration.
       * The location of the new contact point is defined as the measured foot CoP. To avoid cases 
       * where for some reason (e.g. joint limits) the measured CoP is in another location respect
       * to the desire CoP, we check the error between desired and actual. If the error is greater 
       * than a threshold or if the actual CoP is NaN we skip the current contact point.
       * We also check the position of the mirrored CoP, and if is too close to the border of the
       * foot polygon we skip the current contact point.
       * 
       * @param currentCoPPercentage - CoP Percentage to move the CoP using the CoPPlanner
       */
      private void adjustContactPointLocationIfNecessary(double currentCoPPercentage)
      {
         FrameVector2d copError = new FrameVector2d(CoPFrame);
         double copScaledPercentage = currentCoPPercentage * copToContactPointScalingFactor;
         CoPPositionPercentage.set(Math.max(0.0, Math.min(copScaledPercentage, copToContactPointScalingFactor)));
         FramePoint2d desiredCoP = setAndGetDesiredFootCop();
         ankleJointIsMoving.set(checkIfAnkleJointIsMoving());

         if (ankleJointIsMoving.getBooleanValue())
         {
            FramePoint2d actualCop = getMeasuredFootCoP();
            if (actualCop.containsNaN())
            {
               currentContactPointNumber.increment();
               return;
            }

            copError.setToZero();
            copError.sub(desiredCoP, actualCop);
            feetExplorationCoPError.set(copError.length());

            if (feetExplorationCoPError.getDoubleValue() > maxAbsCoPError)
            {
               currentContactPointNumber.increment();
               return;
            }

            double x = actualCop.getX();
            double y = actualCop.getY();
            Vector2d vector = new Vector2d(x, y);
            if (adjustContactPointInXNOnly)
            {
               contactPoint.getPosition2d(tempFramePoint2d);
               tempFramePoint2d.setX(vector.getX());
               contactPoint.setPosition2d(tempFramePoint2d);
            }
            else
            {
               contactPoint.getPosition2d(tempFramePoint2d);
               tempFramePoint2d.set(vector);
               contactPoint.setPosition2d(tempFramePoint2d);
            }

            updateNextFootstepCentroid();
            footExplorationICPPlanner.initialize();
            currentContactPointNumber.increment();
         }
         else
         {
            skipContactPointIfMirroredCoPIsCloseToBorder();
         }
      }

      /**
       * This method checks all the contact planes that are not the contact plane that
       * is currently under CoP control (i.e. the one that we are exploring). 
       * For all those contact planes, if are in contact and with a minimum number
       * of contact points, the method checks if their local CoP is too close to
       * the border of the polygon constructed with their contact points.
       * If the CoP distance from the border is less than a threshold the contact point 
       * counter is incremented. 
       */
      private void skipContactPointIfMirroredCoPIsCloseToBorder()
      {
         outerloop: for (int i = 0; i < numberOfPlaneContactStates.getIntegerValue(); i++)
         {
            ContactablePlaneBody contactablePlaneBody = contactablePlaneBodyList.get(i);
            if (contactablePlaneBody.getRigidBody() != currentPlaneContactStateToExplore.getRigidBody())
            {
               for (int j = 0; j < numberOfPlaneContactStates.getIntegerValue(); j++)
               {
                  PlaneContactState planeContactState = planeContactStates.get(j);
                  RigidBody planeBody = planeContactState.getRigidBody();
                  RigidBody contactablePlaneRigidBody = contactablePlaneBody.getRigidBody();

                  if (planeBody.equals(contactablePlaneRigidBody) && planeContactState.inContact()
                        && planeContactState.getTotalNumberOfContactPoints() >= minimumNumberOfContactPointsToHaveAPolygon)
                  {
                     List<? extends ContactPointInterface> points = planeContactState.getContactPoints();

                     FramePoint2d localCoP = momentumBasedController.getDesiredCoP(contactablePlaneBody);
                     double minDistance = getClosestEdgeDistance(points, localCoP);

                     if (Math.abs(minDistance) < minCoPDistanceFromBorder)
                     {
                        System.out.println("Skipping contact point n. " + currentContactPointNumber.getIntegerValue() + " because the mirrored CoP is too close to the polygon border.");
                        currentContactPointNumber.increment();
                        break outerloop;
                     }
                  }
               }
            }
         }
      }

      /**
       * This method returns the distance from a pointToCheck and the closest border of the polygon constructed 
       * using a List of ContactPoints.
       * In particular a FrameConvexPolygon2d is constructed from the List of contact point, and subsequently 
       * for each couple of vertex we construct a 2d line and measure the distance of the point that we want 
       * to check. Eventually only the smallest distance is returned.
       *  
       * @param points - a List of ContactPoint to construct the convex polygon
       * @param pointToCheck - the point that needs to be checked for the distance from each edge of the polygon
       * @return double - the distance of the point from the closest edge of the polygon
       */
      private double getClosestEdgeDistance(List<? extends ContactPointInterface> points, FramePoint2d pointToCheck)
      {
         double ret = Double.POSITIVE_INFINITY;
         FrameConvexPolygon2d polygon = createAPolygonFromContactPoints(points);

         for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         {
            FrameLine2d line = new FrameLine2d(polygon.getFrameVertex(i), polygon.getNextFrameVertex(i));
            double distance = line.distance(pointToCheck);
            if (distance < ret)
               ret = distance;
         }

         return ret;
      }

      /**
       * This method updates the foot angular rate and than checks if the rate 
       * of the foot that is under CoP control is greater than the zeroVelocity.
       * 
       * @return boolean - true if the foot angular rate is greater than the zeroVelocity.
       */
      private boolean checkIfAnkleJointIsMoving()
      {
         return footAngularRate.get(footUderCoPControl).getDoubleValue() > zeroVelocity;
      }

      /**
       * This method gets the X and Y components of the CoP from the CoPPlanner, and sets the 
       * offsets inside of the momentum based controller in order to move the CoP inside of the foot. 
       *  
       * @return FramePoint2d - The desired CoP position represented in the reference frame CoPFrame.
       */
      private FramePoint2d setAndGetDesiredFootCop()
      {
         FramePoint2d positionToPack = new FramePoint2d(CoPFrame);
         footExplorationCoPPlanner.getCoPPosition(positionToPack, CoPPositionPercentage.getDoubleValue());
         momentumBasedController.setFootCoPOffsetX(positionToPack.getX());
         momentumBasedController.setFootCoPOffsetY(positionToPack.getY());

         return positionToPack;
      }
   }

   /**
    * In this state the module simply shifts the ICP from the initial position
    * to a percentage (between 0 and 1) of the line connecting the initial 
    * and final ICP points. For this purpose is used the local ICP planner.
    * 
    * Note. Temporary there is a control of the CoM height based on the distance 
    * between current and desired ICP. This is necessary because sometimes if the 
    * step is too long the robot can not move the ICP close enough to the next 
    * footstep center point. This hack should be removed maybe allowing the 'toe off'.
    *
    */
   private class ICPShiftingState extends State<FeetExplorationState>
   {
      double initialICPPositionPercentage;
      double tempICPPositionPercentage;
      double initialTime;
      double time;
      double icpVelocity;
      FramePoint2d tempICP;

      public ICPShiftingState()
      {
         super(FeetExplorationState.ICP_SHIFT);
         tempICP = new FramePoint2d(worldFrame);
      }

      @Override
      public void doAction()
      {
         time = yoTime.getDoubleValue() - initialTime;
         tempICPPositionPercentage = initialICPPositionPercentage + time * icpVelocity;
         tempICPPositionPercentage = Math.min(tempICPPositionPercentage, finalICPPositionPercentage.getDoubleValue());

         ICPPositionPercentage.set(tempICPPositionPercentage);
         footExplorationICPPlanner.getICPPosition(tempICP);
         setICPLocalToDesired(tempICP);
         desiredICPVelocityLocal.setToZero();

         if (lowerCoMDuringSwing)
         {
            if (finalICPPositionPercentage.getDoubleValue() == 1.0)
            {
               centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(minCoMHeightOffset);
            }
            else
            {
               centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(0.0);
            }
         }

         if (adjustCoMHeightBasedOnICP)
         {
            boolean controlBoolean;

            if (finalICPPositionPercentage.getDoubleValue() == constantICPPointPercentages.get(constantICPPointPercentages.size() - 1))
            {
               controlBoolean = icpIsCloseEnoughForSingleSupport.getBooleanValue();
            }
            else
            {
               controlBoolean = icpIsCloseEnoughToDesired.getBooleanValue();
            }

            lowerCoMBasedOnBoolean(controlBoolean);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         updateNextFootstepCentroid();
         footExplorationICPPlanner.initialize();
         initialICPPositionPercentage = ICPPositionPercentage.getDoubleValue();
         initialTime = yoTime.getDoubleValue();
         icpVelocity = (finalICPPositionPercentage.getDoubleValue() - initialICPPositionPercentage) / icpShiftTime;
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   /**
    * This state is meant to be used when an unstable situation is detected.
    * The exploration is stopped and the desired ICP moved to a safe position.
    *
    */
   private class RecoverState extends State<FeetExplorationState>
   {
      private double initialTime;
      private double time;

      public RecoverState()
      {
         super(FeetExplorationState.RECOVER);
      }

      @Override
      public void doAction()
      {
         time = yoTime.getDoubleValue() - initialTime;

         if (time > recoverTime)
         {
            scaleContactPointLocation(currentPlaneContactInSupport, recoverStateContactPointsScaleFactor);
            isRecovering.set(false);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         initialTime = yoTime.getDoubleValue();
         isRecovering.set(true);
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   private class DoneWithSwingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return swingIsFinished.getBooleanValue();
      }
   }

   private class DoneWithSubExplorationCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return currentContactPointNumber.getIntegerValue() + 1 > currentPlaneContactStateToExplore.getTotalNumberOfContactPoints();
      }
   }

   private class DoneWithRecoverCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return !isRecovering.getBooleanValue();
      }
   }

   private class UnsafeICPCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return !icpIsCloseEnoughToDesired.getBooleanValue() && swingIsFinished.getBooleanValue()
               && stateMachine.getCurrentState().getStateEnum() != FeetExplorationState.ICP_SHIFT;
      }
   }

   private class DoneWithICPShiftCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         double absDifference = Math.abs(ICPPositionPercentage.getDoubleValue() - finalICPPositionPercentage.getDoubleValue());

         return absDifference < 0.001 && ICPPositionPercentage.getDoubleValue() < icpPercentageToDetermineIfIsLastShift 
                && icpIsCloseEnoughForSingleSupport.getBooleanValue();
      }
   }

   //-------------------------------------------------- ICP Planner----------------------------------------------------------------------
   /**
    * This is a simple ICP planner that computes the desired ICP using a strait line position trajectory.
    *
    */
   private class FootExplorationICPPlanner extends StraightLinePositionTrajectoryGenerator
   {
      private FramePoint tempPositionToPack;

      public FootExplorationICPPlanner(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
            PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
      {
         super(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
         tempPositionToPack = new FramePoint(worldFrame);
      }

      public void getICPPosition(FramePoint2d desiredICPPositionToPack)
      {
         compute(ICPPositionPercentage.getDoubleValue());
         get(tempPositionToPack);
         tempPositionToPack.changeFrame(desiredICPPositionToPack.getReferenceFrame());
         desiredICPPositionToPack.set(tempPositionToPack.getX(), tempPositionToPack.getY());
      }

      public void initialize()
      {
         super.initialize();
      }
   }

   private PositionProvider initialICPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         supportFootCentroid.checkReferenceFrameMatch(worldFrame);
         positionToPack.setIncludingFrame(worldFrame, supportFootCentroid.getX(), supportFootCentroid.getY(), 0.0);
      }
   };

   private PositionProvider finalICPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         nextFootStepCentroid.checkReferenceFrameMatch(worldFrame);
         positionToPack.setIncludingFrame(worldFrame, nextFootStepCentroid.getX(), nextFootStepCentroid.getY(), 0.0);
      }
   };

   private DoubleProvider totalPercentageProvider = new DoubleProvider()
   {
      @Override
      public double getValue()
      {
         return 1.0;
      }
   };

   //--------------------------- CoP Planner --------------------------------------------------
   /**
    * This is a simple CoP planner that computes the desired CoP using a strait line position trajectory.
    * Since the trajectory generator can not update the ReferenceFrame, we do everything in worldFrame
    * and later we simple swap the ReferenceFrame without using the method to update the coordinates.
    *
    */
   private class FootExplorationCoPPlanner extends StraightLinePositionTrajectoryGenerator
   {
      private FramePoint tempPositionToPack;

      public FootExplorationCoPPlanner(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
            PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
      {
         super(namePrefix, referenceFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider, parentRegistry);
         tempPositionToPack = new FramePoint(worldFrame);
      }

      public void getCoPPosition(FramePoint2d desiredCoPPositionToPack, double percentage)
      {
         compute(percentage);
         get(tempPositionToPack);
         desiredCoPPositionToPack.set(tempPositionToPack.getX(), tempPositionToPack.getY());
      }

      public void initialize()
      {
         super.initialize();
      }
   }

   private PositionProvider initialCoPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         initialCoP.checkReferenceFrameMatch(CoPFrame);
         positionToPack.setIncludingFrame(worldFrame, initialCoP.getX(), initialCoP.getY(), 0.0);
      }
   };

   private PositionProvider finalCoPPositionProvider = new PositionProvider()
   {
      @Override
      public void get(FramePoint positionToPack)
      {
         finalCoP.checkReferenceFrameMatch(CoPFrame);
         positionToPack.setIncludingFrame(worldFrame, finalCoP.getX(), finalCoP.getY(), 0.0);
      }
   };

   // CLASS METHODS---------------------------------------------------------------------------------------------------- 
   private void updateNextFootstepCentroid()
   {
      ArrayList<FramePoint2d> points = new ArrayList<FramePoint2d>();
      for (int i = 0; i < currentPlaneContactStateToExplore.getTotalNumberOfContactPoints(); i++)
      {
         FramePoint2d framePoint2d = new FramePoint2d();
         currentPlaneContactStateToExplore.getContactPoints().get(i).getPosition2d(framePoint2d);
         points.add(framePoint2d);
      }

      FrameConvexPolygon2d newPolygon = new FrameConvexPolygon2d(points);
      FramePoint2d tempCentroid = newPolygon.getCentroid();
      nextFootStepCentroid = new FramePoint(tempCentroid.getReferenceFrame(), tempCentroid.getX(), tempCentroid.getY(), 0.0);
      nextFootStepCentroid.changeFrame(worldFrame);
   }

   private void setICPLocalToDesired(FramePoint2d desiredICP)
   {
      ReferenceFrame icpReferenceFrame = desiredICPLocal.getReferenceFrame();
      desiredICP.changeFrame(icpReferenceFrame);
      desiredICPLocal.set(desiredICP);
   }

   /**
    * This method computes the percentages that are used to derive the position of the desired ICP along the 
    * ICP trajectory (defined as the segment connecting the initial desired ICP and the final desired ICP).
    * The last percentage should be 1.0, meaning that the last desired ICP corresponds to the final desired ICP.
    * 
    */
   private void computeConstantICPPoints()
   {
      // compute the first internal one as a percentage of the whole ICP trajectory.
      supportFootPolygon.changeFrame(ICPTrajectory.getReferenceFrame());
      FramePoint2d[] intersectionPoints = supportFootPolygon.intersectionWith(ICPTrajectory);
      FramePoint2d initialPoint = new FramePoint2d(supportFootCentroid.getReferenceFrame(), supportFootCentroid.getX(), supportFootCentroid.getY());
      FramePoint2d endPoint = new FramePoint2d(nextFootStepCentroid.getReferenceFrame(), nextFootStepCentroid.getX(), nextFootStepCentroid.getY());
      footstepCentroidsDistance.set(Math.abs(endPoint.distance(initialPoint)));
      FramePoint2d pointOnEdge = getClosestPointToAReferencePoint(intersectionPoints, endPoint);

      double lengthOfSegmentInsideSupportFoot = Math.abs(initialPoint.distance(pointOnEdge));
      //      constantICPPoints.add((lengthOfSegmentInsideSupportFoot * firstInternalICPPercentage)/footstepCentroidsDistance.getDoubleValue());
      constantICPPointPercentages.add(firstExternalICPPercentage);
      //      constantICPPoints.add(secondExternalICPPercentage);
      constantICPPointPercentages.add(lastExternalICPPercentage);
   }

   private FramePoint2d getClosestPointToAReferencePoint(FramePoint2d[] listOfPoints, FramePoint2d referencePoint)
   {
      int numberOfPoint = listOfPoints.length;
      double distance = Double.POSITIVE_INFINITY;
      FramePoint2d ret = null;

      for (int i = 0; i < numberOfPoint; i++)
      {
         FramePoint2d point = listOfPoints[i];
         double dist = point.distanceSquared(referencePoint);

         if (dist < distance)
         {
            distance = dist;
            ret = point;
         }
      }

      return ret;
   }

   /**
    * Consider switch to toe-off 
    */
   @Deprecated
   private void lowerCoMBasedOnBoolean(boolean adjust)
   {
      if (adjust)
      {
         // now this is done in a wrong way because the velocity of CoM depends on the control loop. Each tick we increment or decrement.
         comIncrement.add(0.002);
         comIncrement.set(Math.min(maxCoMHeightOffset, comIncrement.getDoubleValue()));
         centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(comIncrement.getDoubleValue());
      }
      else
      {
         comIncrement.add(-0.002);
         comIncrement.set(Math.max(minCoMHeightOffset, comIncrement.getDoubleValue()));
         centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(comIncrement.getDoubleValue());
      }
   }

   private ArrayList<Point2d> getCopyOfContactPointLocation(List<? extends ContactPointInterface> contactPoints)
   {
      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         Point2d point2d = new Point2d();
         contactPoints.get(i).getPosition2d(point2d);
         ret.add(point2d);
      }

      return ret;
   }

   private void resetContactPoints(ArrayList<Point2d> defaultPoint, List<? extends ContactPointInterface> oldPoints)
   {
      for (int i = 0; i < defaultPoint.size(); i++)
      {
         double x = defaultPoint.get(i).getX();
         double y = defaultPoint.get(i).getY();
         oldPoints.get(i).getPosition2d(tempFramePoint2d);
         tempFramePoint2d.set(x, y);
         oldPoints.get(i).setPosition2d(tempFramePoint2d);
      }
   }

   private void scaleContactPointLocation(PlaneContactState currentPlaneContactState, double factor)
   {
      FrameConvexPolygon2d polygon = createAPolygonFromContactPoints(currentPlaneContactState.getContactPoints());
      FramePoint2d centroid = polygon.getCentroid();
      polygon.scale(centroid, factor);

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         ContactPointInterface contactPoint = currentPlaneContactState.getContactPoints().get(i);
         FramePoint2d scaledPoint = polygon.getFrameVertex(i);
         contactPoint.setPosition2d(scaledPoint);
      }
   }

   private FrameConvexPolygon2d createAPolygonFromContactPoints(List<? extends ContactPointInterface> points)
   {
      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(points.get(0).getReferenceFrame());
      for (int i = 0; i < points.size(); i++)
      {
         points.get(i).getPosition2d(tempFramePoint2d);
         polygon.addVertex(tempFramePoint2d);
      }
      polygon.update();

      return polygon;
   }

   private void resetWalkingVariableIfNecessary()
   {
      if (doToeOffIfPossibleHasBeenChanged.getBooleanValue())
      {
         feetManager.getWalkOnTheEdgesManager().setDoToeOffIfPossible(defaultDoToeOff.getBooleanValue());
         doToeOffIfPossibleHasBeenChanged.set(false);
      }

      if (swingTimeHasBeenChanged.getBooleanValue())
      {
         swingTimeCalculationProvider.setSwingTime(defaultSwingTime.getDoubleValue());
         swingTimeHasBeenChanged.set(false);
      }

      if (comHeightOffsetHasBeenChanged.getBooleanValue())
      {
         centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(defaultCOMHeightOffset.getDoubleValue());
         comHeightOffsetHasBeenChanged.set(false);
      }
   }
   
   private boolean checkIfExplorationIsFinished()
   {
      if(ICPPositionPercentage.getDoubleValue() > icpPercentageToConsiderTransferFinished)
      {
         if(icpIsCloseEnoughForSingleSupport.getBooleanValue())
            return true;
         
         if(initialStallTime.isNaN())
            initialStallTime.set(yoTime.getDoubleValue());
         
         timeInStallBecauseICPIsNotCloseEnough.set(yoTime.getDoubleValue() - initialStallTime.getDoubleValue());
         
         if(timeInStallBecauseICPIsNotCloseEnough.getDoubleValue() > maxStallTimeToActivateDoToeOff)
            feetManager.getWalkOnTheEdgesManager().setDoToeOffIfPossible(true);
         
         return false;
      }
      else
      {
         if(!initialStallTime.isNaN())
         {
            initialStallTime.set(Double.NaN);
            timeInStallBecauseICPIsNotCloseEnough.set(0.0);
         }
         
         return false;
      }
   }

   /**
    * This method returns the current measured foot CoP.
    * 
    * @return FramePoint2d - current measured foot CoP.
    */
   private FramePoint2d getMeasuredFootCoP()
   {
      SideDependentList<FootSwitchInterface> footSwitches = momentumBasedController.getFootSwitches();
      FramePoint2d copActual = new FramePoint2d();
      footSwitches.get(footUderCoPControl).computeAndPackCoP(copActual);

      return copActual;
   }

   /**
    * This method updates the angular component of the velocity of both feet.
    * In particular this quantity considers also the relative motion between the leg and the foot
    * such that, if for example the foot is not moving, but the robot main body is moving, the measured 
    * velocity is almost null. Instead considering the ankle velocities, we would have 
    * obtained a greater velocity.
    * 
    */
   Twist tmpTwist = new Twist();
   Vector3d tmpVector = new Vector3d();

   private void updateFootAngularRate()
   {
      for (RobotSide side : RobotSide.values)
      {
         momentumBasedController.getTwistCalculator().packTwistOfBody(tmpTwist, momentumBasedController.getFullRobotModel().getFoot(side));
         tmpTwist.changeFrame(ReferenceFrame.getWorldFrame());
         tmpTwist.packAngularPart(tmpVector);
         footAngularRate.get(side).set(tmpVector.length());
      }
   }

   /**
    * This method checks is the roll and pitch absolute value of the ankle angular velocities 
    * are greater than the zeroVelocity. 
    * Note. Currently this method is not used. Instead the foot angular rate is used.
    * 
    * @return boolean - true if the ankle velocity abs is greater than the zeroVelocity.
    */
   private void updateAnkleVelocity()
   {
      double absVelocity = 0.0;
      jointX.packVelocityMatrix(jointVelocityMatrix, 0);
      jointY.packVelocityMatrix(jointVelocityMatrix, 1);
      int numRows = jointVelocityMatrix.getNumRows();
      for (int i = 0; i < numRows; i++)
      {
         if (Math.abs(jointVelocityMatrix.get(i, 0)) > absVelocity)
            absVelocity = Math.abs(jointVelocityMatrix.get(i, 0));
      }
      ankleJointVelocity.update(absVelocity);
   }

   // API

   public boolean isDone()
   {
      return footHoldIsSafeEnough.getBooleanValue() && transferToNextFootStepIsDone.getBooleanValue();
   }

   public boolean isControllingSwingFoot()
   {
      return isMasteringFullExploration.getBooleanValue();
   }

   public boolean isActive()
   {
      return performCoPExploration.getBooleanValue();
   }

   public void activateFootCoPExploration()
   {
      performCoPExploration.set(true);
   }

   public void deactivateFootCoPExploration()
   {
      performCoPExploration.set(false);
   }

   public boolean footCoPExplorationIsActive()
   {
      return performCoPExploration.getBooleanValue();
   }

   public boolean footHoldIsSafeEnough()
   {
      return footHoldIsSafeEnough.getBooleanValue();
   }

   public void setSwingIsFinished(boolean swingIsFinished)
   {
      this.swingIsFinished.set(swingIsFinished);
   }
}