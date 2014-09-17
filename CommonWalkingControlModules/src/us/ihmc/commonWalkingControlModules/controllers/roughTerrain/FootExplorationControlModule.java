package us.ihmc.commonWalkingControlModules.controllers.roughTerrain;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
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
 * @author Robot Lab
 *
 */
public class FootExplorationControlModule
{
   public enum FeetExplorationState
   {
      SWING, EXPLORATION, ICP_SHIFT, RECOVER
   }
   
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   // control of CoM height
   private static boolean lowerCoMDuringSwing = false;
   private static boolean adjustCoMHeightBasedOnICP = false;
   private static double maxCoMHeightOffset = 0.02;
   private static double minCoMHeightOffset = -0.1;
   
   // exploration timing
   private static double icpShiftTime = 1.5;
   private static double copShiftTime = 2;
   private static double copTransitionRestingTime = 1; // must be less than copShiftTime/2
   
   // detection of unstable situations
   private static double maxICPAbsError = 0.04;
   private static double maxICPAbsErrorForSingleSupport = 0.015;
   private static double minCoPDistanceFromBorder = 0.005;
   
   // detection of foot tilt
   private static double zeroVelocity = 0.2;
   private static double alphaForJointVelocity = 0.96;
   private static int numberOfJointCheckedForZeroVelocity = 2;
   
   // definition of constant ICP positions
   private static double firstInternalICPPercentage = 0.8;
   private static double secondInternalICPPercentage = 0.95;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BooleanYoVariable performCoPExploration;
   private final DoubleYoVariable explorationTime;
   private final DoubleYoVariable initialTime;
   private final BooleanYoVariable isExploringFootHold;
   private final BooleanYoVariable footHoldIsSafeEnough;
   private final DoubleYoVariable initialExplorationTime;
   private final MomentumBasedController momentumBasedController;
   private final BooleanYoVariable isControllingSwingFoot;
   private final BooleanYoVariable swingIsFinished;
   private final DoubleYoVariable ICPPositionPercentage;
   private final DoubleYoVariable finalICPPositionPercentage;
   private final FootExplorationICPPlanner footExplorationICPPlanner;
   private final BooleanYoVariable icpIsCloseEnoughToDesired;
   private final DoubleYoVariable explorationICPErrorAbs;
   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable transferToNextFootStepIsDone;
   private final BooleanYoVariable icpIsCloseEnoughForSingleSupport;
   private final DoubleYoVariable footstepCentroidsDistance;
   private final StateMachine<FeetExplorationState> stateMachine;
   private final IntegerYoVariable icpIndex;
   private final BooleanYoVariable explorationIsInProgress;
   private final List<? extends PlaneContactState> planeContactStates;
   private final IntegerYoVariable numberOfPlaneContactStates;
   private final DoubleYoVariable CoPPositionPercentage;
   private final AlphaFilteredYoVariable jointVelocity;
   private final BooleanYoVariable jointIsMoving;
   private final IntegerYoVariable currentContactPointNumber;
   private final LookAheadCoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final DoubleYoVariable comIncrement;
   private final List<ContactablePlaneBody> contactablePlaneBodyList;
   private final FeetManager feetManager;
   private final BooleanYoVariable defaultDoToeOff;
   
   private Footstep nextFootStep;
   private RobotSide footUderCoPControl;
   private FrameConvexPolygon2d supportFootPolygon;
   private FramePoint supportFootCentroid;
   private FramePoint nextFootStepCentroid; 
   private FrameLine2d ICPTrajectory;
   private FramePoint2d desiredICPLocal;
   private FramePoint2d startCoP;
   private FrameVector2d desiredICPVelocityLocal;
   private ArrayList<Double> constantICPPoints;
   private PlaneContactState currentPlaneContactState;
   private FramePoint initialCoP;
   private FramePoint finalCoP;
   private ReferenceFrame CoPFrame;
   private FootExplorationCoPPlanner footExplorationCoPPlanner;
   private InverseDynamicsJoint jointX, jointY;
   private DenseMatrix64F jointVelocityMatrix;
   private List<? extends ContactPoint> defaultContactPoints;
   
   public FootExplorationControlModule(YoVariableRegistry parentRegistry, MomentumBasedController momentumBasedController, DoubleYoVariable yoTime, 
                                       LookAheadCoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, SwingTimeCalculationProvider swingTimeCalculationProvider, FeetManager feetManager)
   {
      performCoPExploration = new BooleanYoVariable("performCoPExploration", registry);
      explorationTime = new DoubleYoVariable("explorationTime", registry);
      initialTime = new DoubleYoVariable("initialTime", registry);
      isExploringFootHold = new BooleanYoVariable("isExploring", registry);
      footHoldIsSafeEnough = new BooleanYoVariable("footHoldIsSafe", registry);
      initialExplorationTime = new DoubleYoVariable("initialExplorationTime", registry); 
      isControllingSwingFoot = new BooleanYoVariable("isControllingSwingFoot", registry);
      swingIsFinished = new BooleanYoVariable("swingIsFinished", registry);
      ICPPositionPercentage = new DoubleYoVariable("ICPPositionPercentage", registry);
      finalICPPositionPercentage = new DoubleYoVariable("finalICPPositionPercentage", registry);
      icpIsCloseEnoughToDesired = new BooleanYoVariable("icpIsInsideNextFootStep", registry);
      explorationICPErrorAbs = new DoubleYoVariable("explorationICPErrorAbs", registry);
      footExplorationICPPlanner = new FootExplorationICPPlanner("footExplorationICPPlanner", worldFrame, totalPercentageProvider, initialICPPositionProvider,
                                                                finalICPPositionProvider, registry);
      planeContactStates = momentumBasedController.getPlaneContactStates();
      contactablePlaneBodyList = momentumBasedController.getContactablePlaneBodyList();
      icpIsCloseEnoughForSingleSupport = new BooleanYoVariable("icpIsCloseEnoughForSingleSupport", registry);
      footstepCentroidsDistance = new DoubleYoVariable("footstepCentroidsDistance", registry);
      transferToNextFootStepIsDone = new BooleanYoVariable("transferToNextFootStepIsDone", registry);
      explorationIsInProgress =  new BooleanYoVariable("explorationIsInProgress", registry);
      numberOfPlaneContactStates = new IntegerYoVariable("numberOfPlaneContactStates", registry);
      defaultDoToeOff = new BooleanYoVariable("defaultDoToeOff", registry);
      icpIndex = new IntegerYoVariable("icpIndex", registry);
      CoPPositionPercentage = new DoubleYoVariable("CoPPositionPercentage", registry);
      String namePrefix = "feetExploration";
      stateMachine = new StateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", FeetExplorationState.class, yoTime, registry);
      footExplorationCoPPlanner = new FootExplorationCoPPlanner("footExplorationCoPPlanner", worldFrame, totalPercentageProvider, initialCoPPositionProvider, finalCoPPositionProvider, registry);
      jointVelocity = new AlphaFilteredYoVariable("jointVelocity", registry, alphaForJointVelocity);
      jointIsMoving = new BooleanYoVariable("jointIsMoving", registry);
      currentContactPointNumber = new IntegerYoVariable("currentContactPointNumber", registry);
      comIncrement = new DoubleYoVariable("comIncrement", registry);
      this.yoTime = yoTime;
      this.feetManager = feetManager;
      this.momentumBasedController = momentumBasedController;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      performCoPExploration.set(false);
      ICPTrajectory = new FrameLine2d(worldFrame, new Point2d(0.0, 0.0), new Point2d(1.0, 1.0));
      desiredICPLocal = new FramePoint2d(worldFrame);
      desiredICPVelocityLocal = new FrameVector2d(worldFrame);
      nextFootStepCentroid = new FramePoint(worldFrame);
      constantICPPoints = new ArrayList<Double>();
      
      numberOfPlaneContactStates.set(planeContactStates.size());
      initialTime.set(Double.NaN);
      initialExplorationTime.set(Double.NEGATIVE_INFINITY);
      parentRegistry.addChild(registry);
      
      setupStateMachine();
      reset();
   }
   
   public void controlSwingFoot(YoFramePoint2d desiredICPToPack, YoFrameVector2d desiredICPVelocityToPack, FramePoint2d currentCapturePoint)
   {
      desiredICPLocal.changeFrame(desiredICPToPack.getReferenceFrame());
      desiredICPVelocityLocal.changeFrame(desiredICPVelocityToPack.getReferenceFrame());
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      desiredICPToPack.set(desiredICPLocal.getX(), desiredICPLocal.getY());
      desiredICPVelocityToPack.set(desiredICPVelocityLocal.getX(), desiredICPVelocityLocal.getY());
      explorationICPErrorAbs.set(Math.abs(currentCapturePoint.distance(desiredICPLocal)));
      icpIsCloseEnoughToDesired.set(explorationICPErrorAbs.getDoubleValue() < maxICPAbsError);
      icpIsCloseEnoughForSingleSupport.set(explorationICPErrorAbs.getDoubleValue() < maxICPAbsErrorForSingleSupport);
      
      if (ICPPositionPercentage.getDoubleValue()>0.99 && icpIsCloseEnoughForSingleSupport.getBooleanValue())
         transferToNextFootStepIsDone.set(true);
      
      if (adjustCoMHeightBasedOnICP)
      {
         if (icpIsCloseEnoughToDesired.getBooleanValue())
         {
            comIncrement.add(0.0005);
            comIncrement.set(Math.min(maxCoMHeightOffset, comIncrement.getDoubleValue()));
            centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(comIncrement.getDoubleValue());
         }
         else
         {
            comIncrement.add(-0.0005);
            comIncrement.set(Math.max(minCoMHeightOffset, comIncrement.getDoubleValue()));
            centerOfMassHeightTrajectoryGenerator.setOffsetHeightAboveGround(comIncrement.getDoubleValue());
         }
      }
   }
   
   public void initialize(Footstep nextFootStep, FrameConvexPolygon2d supportFootPolygon, RobotSide footUderCoPControl)
   {
      if (performCoPExploration.getBooleanValue())
      {
         this.nextFootStep = nextFootStep;
         this.footUderCoPControl = footUderCoPControl;
         this.supportFootPolygon = new FrameConvexPolygon2d(supportFootPolygon);

         FramePoint2d tempCentroid = supportFootPolygon.getCentroid();
         supportFootCentroid = new FramePoint(tempCentroid.getReferenceFrame(), tempCentroid.getX(), tempCentroid.getY(), 0.0);
         supportFootCentroid.changeFrame(worldFrame);

         nextFootStep.getPositionIncludingFrame(nextFootStepCentroid);
         nextFootStepCentroid.changeFrame(worldFrame);

         ICPTrajectory.set(worldFrame, supportFootCentroid.getX(), supportFootCentroid.getY(), nextFootStepCentroid.getX(), nextFootStepCentroid.getY());
         computeConstantICPPoints();
         
         for(int i = 0; i < numberOfPlaneContactStates.getIntegerValue(); i++)
         {
            PlaneContactState planeContactState = planeContactStates.get(i);
            RigidBody planeBody = planeContactState.getRigidBody();
            RigidBody nextFootStepBody = nextFootStep.getBody().getRigidBody();
            
            if(planeBody.equals(nextFootStepBody))
               currentPlaneContactState = planeContactState;
         }
         
         CoPFrame = momentumBasedController.getContactableFeet().get(footUderCoPControl).getSoleFrame();
         startCoP = new FramePoint2d(CoPFrame, 0.0, 0.0);

         jointX = currentPlaneContactState.getRigidBody().getParentJoint();
         jointY = currentPlaneContactState.getRigidBody().getParentJoint().getPredecessor().getParentJoint();
         jointVelocityMatrix = new DenseMatrix64F(numberOfJointCheckedForZeroVelocity, 1);
         
         defaultDoToeOff.set(feetManager.getWalkOnTheEdgesManager().doToeOffIfPossible());
         feetManager.getWalkOnTheEdgesManager().setDoToeOffIfPossible(false);
         
         defaultContactPoints = currentPlaneContactState.getContactPoints();
         
         isControllingSwingFoot.set(true);
         footExplorationICPPlanner.initialize();
         swingTimeCalculationProvider.setSwingTime(1.5);
         stateMachine.setCurrentState(FeetExplorationState.SWING);  
      }
   }
   
   public void reset()
   {
      isControllingSwingFoot.set(false);
//      footHoldIsSafeEnough.set(false);
      // TODO define better in the exploration phase
      footHoldIsSafeEnough.set(true);
      swingIsFinished.set(false);
      icpIsCloseEnoughToDesired.set(false);
      constantICPPoints.clear();
      icpIndex.set(0);
      explorationIsInProgress.set(false);
      transferToNextFootStepIsDone.set(false);
      explorationTime.set(Double.POSITIVE_INFINITY);
      ICPPositionPercentage.set(0.0);
      feetManager.getWalkOnTheEdgesManager().setDoToeOffIfPossible(defaultDoToeOff.getBooleanValue());
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
      
      StateTransition<FeetExplorationState> toICPShiftFromSwing = new StateTransition<FeetExplorationState>(FeetExplorationState.ICP_SHIFT, doneWithSwingCondition);
      StateTransition<FeetExplorationState> toICPShiftFromSubExploration = new StateTransition<FeetExplorationState>(FeetExplorationState.ICP_SHIFT, doneWithExplorationCondition);
      StateTransition<FeetExplorationState> toExploration = new StateTransition<FeetExplorationState>(FeetExplorationState.EXPLORATION, doneWithICPShiftCondition);
      StateTransition<FeetExplorationState> toRecover = new StateTransition<FeetExplorationState>(FeetExplorationState.RECOVER, unsafeICPCondition);
      
      performingSwingState.addStateTransition(toICPShiftFromSwing);
      explorationState.addStateTransition(toRecover);
      explorationState.addStateTransition(toICPShiftFromSubExploration);
      icpShiftingState.addStateTransition(toRecover);
      icpShiftingState.addStateTransition(toExploration);
      
      stateMachine.addState(performingSwingState);
      stateMachine.addState(explorationState);
      stateMachine.addState(icpShiftingState);
      stateMachine.addState(recover);
   }
   
   private class PerformingSwingState extends State<FeetExplorationState>
   {
      FramePoint2d constantICP;

      public PerformingSwingState()
      {
         super(FeetExplorationState.SWING);
      }

      @Override
      public void doAction()
      {
               
      }

      @Override
      public void doTransitionIntoAction()
      {
         constantICP = new FramePoint2d(worldFrame, supportFootCentroid.getX(), supportFootCentroid.getY());
         setICPLocalToDesired(constantICP);
         desiredICPVelocityLocal.setToZero();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         finalICPPositionPercentage.set(constantICPPoints.get(0));
         icpIndex.increment();
      }
   }

   private class ExplorationState extends State<FeetExplorationState>
   {
      FramePoint2d tempICP;
      double slopeCoP;
      double initialCoPTime;
      int lastContactPointNumber;
      double copTime;
      ContactPoint contactPoint;

      public ExplorationState()
      {
         super(FeetExplorationState.EXPLORATION);
         tempICP = new FramePoint2d(worldFrame);
      }

      @Override
      public void doAction()
      {
         //TODO: first attempt to find a suitable sartCoP (maybe explore both areas and select the bigger)
         
         if (currentContactPointNumber.getIntegerValue() < currentPlaneContactState.getTotalNumberOfContactPoints())
         {
            if(currentContactPointNumber.getIntegerValue() != lastContactPointNumber)
            {
               initialCoPTime = yoTime.getDoubleValue();
               lastContactPointNumber =  currentContactPointNumber.getIntegerValue();
               contactPoint = currentPlaneContactState.getContactPoints().get(currentContactPointNumber.getIntegerValue());
               FramePoint2d position = contactPoint.getPosition2d();
               slopeCoP = 1/copShiftTime;
               initialCoP = new FramePoint(CoPFrame, startCoP.getX(), startCoP.getY(), 0.0);
               finalCoP =  new FramePoint(CoPFrame, position.getX(), position.getY(), 0.0);
               footExplorationCoPPlanner.initialize();
               CoPPositionPercentage.set(0.0);
               jointVelocity.set(0.0);
            }
            
            copTime = yoTime.getDoubleValue() - initialCoPTime;
            adjustContactPointLocationIfNecessary(copTime);
            
            if((copTime - copTransitionRestingTime)*slopeCoP > 1.5)
               currentContactPointNumber.increment();
            
         }
         
         explorationTime.set(yoTime.getDoubleValue() - initialExplorationTime.getDoubleValue());         
      }

      @Override
      public void doTransitionIntoAction()
      {
         currentContactPointNumber.set(0);
         lastContactPointNumber = -1;
         initialExplorationTime.set(yoTime.getDoubleValue());
         momentumBasedController.setFeetCoPControlIsActive(true);
         momentumBasedController.setSideOfFootUnderCoPControl(footUderCoPControl);
         isExploringFootHold.set(true);
         explorationIsInProgress.set(true);
         footExplorationICPPlanner.getICPPosition(tempICP);
         setICPLocalToDesired(tempICP);
         desiredICPVelocityLocal.setToZero();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         momentumBasedController.setFeetCoPControlIsActive(false);
         finalICPPositionPercentage.set(constantICPPoints.get(icpIndex.getIntegerValue()));
         icpIndex.increment();
         isExploringFootHold.set(false);
      }
      
      private void adjustContactPointLocationIfNecessary(double time)
      { 
         CoPPositionPercentage.set(Math.max(0.0, Math.min((time-copTransitionRestingTime)*slopeCoP, 1.0)));
         FramePoint2d positionToPack = new FramePoint2d(CoPFrame);
         footExplorationCoPPlanner.getCoPPosition(positionToPack);
         momentumBasedController.setFootCoPOffsetX(positionToPack.getX());
         momentumBasedController.setFootCoPOffsetY(positionToPack.getY());    
         jointIsMoving.set(checkJointIsNotMoving());

         if(jointIsMoving.getBooleanValue() && time > copTransitionRestingTime)
         {
            contactPoint.getPosition().set(positionToPack.getX(), positionToPack.getY(), 0.0);
            contactPoint.getPosition2d().set(positionToPack.getX(), positionToPack.getY());
            updateNextFootstepCentroid();
            footExplorationICPPlanner.initialize();
            currentContactPointNumber.increment();
         }
         
         if(!jointIsMoving.getBooleanValue())
         {
            skipContactPointIfMirroredCoPIsCloseToBorder();
         }
      }
      
      private void skipContactPointIfMirroredCoPIsCloseToBorder()
      {
         outerloop: for (int i = 0; i < numberOfPlaneContactStates.getIntegerValue(); i++)
         {
            ContactablePlaneBody contactablePlaneBody = contactablePlaneBodyList.get(i);
            if (contactablePlaneBody.getRigidBody() != currentPlaneContactState.getRigidBody())
            {
               for (int j = 0; j < numberOfPlaneContactStates.getIntegerValue(); j++)
               {
                  PlaneContactState planeContactState = planeContactStates.get(j);
                  RigidBody planeBody = planeContactState.getRigidBody();
                  RigidBody contactablePlaneRigidBody = contactablePlaneBody.getRigidBody();

                  if (planeBody.equals(contactablePlaneRigidBody))
                  {
                     List<? extends ContactPoint> points = planeContactState.getContactPoints();
                     FramePoint2d localCoP = momentumBasedController.getCoP(contactablePlaneBody);
                     double minDistance = getClosesEdgeDistance(points, localCoP);

                     if (Math.abs(minDistance) < minCoPDistanceFromBorder)
                     {
                        currentContactPointNumber.increment();
                        break outerloop;
                     }
                  }
               }
            }
         }
      }
      
      private double getClosesEdgeDistance(List<? extends ContactPoint> points, FramePoint2d pointToCheck)
      {
         double ret = Double.POSITIVE_INFINITY;
         ArrayList<FramePoint2d> frameVertices = new ArrayList<FramePoint2d>();
         for(int i = 0; i < points.size(); i++)
         {
            frameVertices.add(points.get(i).getPosition2d());
         }
         FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(frameVertices);
         polygon.update();

         for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         {
            FrameLine2d line = new FrameLine2d(polygon.getFrameVertex(i), polygon.getNextFrameVertex(i));
            double distance = line.distance(pointToCheck);
            if (distance < ret)
               ret = distance;
         }
         
         return ret;
      }
      
      private void resetContactPoints(List<? extends ContactPoint> defaultPoint, List<? extends ContactPoint> oldPoints)
      {
         
      }
      
      private boolean checkJointIsNotMoving()
      {
         jointX.packVelocityMatrix(jointVelocityMatrix, 0);
         jointY.packVelocityMatrix(jointVelocityMatrix, 1);
         int numRows = jointVelocityMatrix.getNumRows();
         for(int i = 0; i<numRows; i++)
         {
            jointVelocity.update(jointVelocityMatrix.get(i, 0) * jointVelocityMatrix.get(i, 0));
         }
         
         return jointVelocity.getDoubleValue() > zeroVelocity;
      }
      
      private void updateNextFootstepCentroid()
      {
         ArrayList<FramePoint2d> points = new ArrayList<FramePoint2d>(); 
         for(int i = 0; i < currentPlaneContactState.getTotalNumberOfContactPoints(); i++)
         {
            points.add(currentPlaneContactState.getContactPoints().get(i).getPosition2d());
         }
         
         FrameConvexPolygon2d newPolygon = new FrameConvexPolygon2d(points);
         FramePoint2d tempCentroid = newPolygon.getCentroid();
         nextFootStepCentroid = new FramePoint(tempCentroid.getReferenceFrame(), tempCentroid.getX(), tempCentroid.getY(), 0.0);
         nextFootStepCentroid.changeFrame(worldFrame);
      }
   }
   
   private class ICPShiftingState extends State<FeetExplorationState>
   {
      double initialICPPositionPercentage;
      double tempICPPositionPercentage;
      double initialTime;
      double time;
      double slope;
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
         tempICPPositionPercentage = initialICPPositionPercentage + time*slope;
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
      }

      @Override
      public void doTransitionIntoAction()
      {
         initialICPPositionPercentage = ICPPositionPercentage.getDoubleValue();
         initialTime = yoTime.getDoubleValue();
         slope = (finalICPPositionPercentage.getDoubleValue() - initialICPPositionPercentage)/icpShiftTime;
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }
   
   private class RecoverState extends State<FeetExplorationState>
   {

      public RecoverState()
      {
         super(FeetExplorationState.RECOVER);
      }

      @Override
      public void doAction()
      {
         
      }

      @Override
      public void doTransitionIntoAction()
      {
         ICPPositionPercentage.set(0.0);
         icpIndex.increment();
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }
   
   private class DoneWithSwingCondition implements StateTransitionCondition
   {
      public DoneWithSwingCondition()
      { 
      }
      
      public boolean checkCondition()
      {
         return swingIsFinished.getBooleanValue();
      }     
   }
   
   private class DoneWithSubExplorationCondition implements StateTransitionCondition
   {
      public DoneWithSubExplorationCondition()
      { 
      }
      
      public boolean checkCondition()
      {
         return currentContactPointNumber.getIntegerValue()+1 > currentPlaneContactState.getTotalNumberOfContactPoints();
      }
   }
   
   private class UnsafeICPCondition implements StateTransitionCondition
   {
      public UnsafeICPCondition()
      { 
      }
      
      public boolean checkCondition()
      {
         return !icpIsCloseEnoughToDesired.getBooleanValue() && 
                swingIsFinished.getBooleanValue() && 
                stateMachine.getCurrentState().getStateEnum() != FeetExplorationState.ICP_SHIFT;
      }
   }
   
   private class DoneWithICPShiftCondition implements StateTransitionCondition
   {
      public DoneWithICPShiftCondition()
      { 
      }
      
      public boolean checkCondition()
      {
         double absDifference = Math.abs(ICPPositionPercentage.getDoubleValue() - finalICPPositionPercentage.getDoubleValue());
         
         return absDifference < 0.001 && ICPPositionPercentage.getDoubleValue() < 0.95 && icpIsCloseEnoughForSingleSupport.getBooleanValue();
      }      
   }
   
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

      public void getCoPPosition(FramePoint2d desiredCoPPositionToPack)
      {
         compute(CoPPositionPercentage.getDoubleValue());
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
   
   private void setICPLocalToDesired(FramePoint2d desiredICP)
   {
      ReferenceFrame icpReferenceFrame = desiredICPLocal.getReferenceFrame();
      desiredICP.changeFrame(icpReferenceFrame);
      desiredICPLocal.set(desiredICP);
   }
   
   private void computeConstantICPPoints()
   {
      supportFootPolygon.changeFrame(ICPTrajectory.getReferenceFrame());
      FramePoint2d[] intersectioPoints = supportFootPolygon.intersectionWith(ICPTrajectory);
      FramePoint2d initialPoint = new FramePoint2d(supportFootCentroid.getReferenceFrame(), supportFootCentroid.getX(), supportFootCentroid.getY());
      FramePoint2d endPoint = new FramePoint2d(nextFootStepCentroid.getReferenceFrame(), nextFootStepCentroid.getX(), nextFootStepCentroid.getY());
      footstepCentroidsDistance.set(Math.abs(endPoint.distance(initialPoint)));
      FramePoint2d pointOnEdge = getClosestPointToAReferencePoint(intersectioPoints, endPoint);
          
      double lengthOfSegmentInsideSupportFoot = Math.abs(initialPoint.distance(pointOnEdge));
//      constantICPPoints.add((lengthOfSegmentInsideSupportFoot * firstInternalICPPercentage)/footstepCentroidsDistance.getDoubleValue());
      constantICPPoints.add((lengthOfSegmentInsideSupportFoot * secondInternalICPPercentage)/footstepCentroidsDistance.getDoubleValue());
      constantICPPoints.add(0.5);
      constantICPPoints.add(1.0);
   }
   
   private FramePoint2d getClosestPointToAReferencePoint(FramePoint2d[] listOfPoints, FramePoint2d referencePoint)
   {
      int numberOfPoint = listOfPoints.length;
      double distance = Double.POSITIVE_INFINITY;
      FramePoint2d ret = new FramePoint2d();
      
      for (int i = 0; i < numberOfPoint; i++)
      {
         FramePoint2d point = listOfPoints[i];
         double dist = point.distanceSquared(referencePoint);
         
         if(dist < distance)
         {
            distance = dist;
            ret = point;
         }
      }
      
      return ret;
   }
   
   public boolean isDone()
   {
      return footHoldIsSafeEnough.getBooleanValue() && transferToNextFootStepIsDone.getBooleanValue();
   }
   
   public boolean isActive()
   {
      return performCoPExploration.getBooleanValue();
   }
   
   public boolean isControllingSwingFoot()
   {
      return isControllingSwingFoot.getBooleanValue();
   }
   
   public boolean isExploringFootHold()
   {
      return isExploringFootHold.getBooleanValue();
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
