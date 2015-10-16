package us.ihmc.exampleSimulations.hw1PointMassWalker;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PrismaticJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class PointMassController implements RobotController
{
   private final double controlDT;
   private final PointMassIDRobot robot;
   private final StateMachine<WalkingStates> stateMachine;
   private final YoVariableRegistry registry = new YoVariableRegistry("registry");
   private final String name = getClass().getSimpleName();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
   private final ArtifactList artifactList = new ArtifactList(name);

   private final YoFramePoint2d yoDesiredCoP = new YoFramePoint2d("desiredCenterOfPressure", worldFrame, registry);
   private final YoFramePoint2d yoDesiredICP = new YoFramePoint2d("yoDesiredCapturePoint", worldFrame, registry);
   private final YoFramePoint yodesiredPositionSwingFoot = new YoFramePoint("yoDesiredPositionSwingFoot", worldFrame, registry);

   private final SideDependentList<YoFramePoint> yoFootPositions;

   private ParabolicCartesianTrajectoryGenerator swingTrajectory;
   private final DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);
   private static final double groundClearance = 0.15;
   private BagOfBalls swingTrajectoryViz;

   private final DoubleYoVariable kpSupportKnee = new DoubleYoVariable("kpSupportKnee", registry);
   private final DoubleYoVariable kdSupportKnee = new DoubleYoVariable("kdSupportKnee", registry);
   private final DoubleYoVariable kpSwingFoot = new DoubleYoVariable("kpSwingFoot", registry);
   private final DoubleYoVariable kdSwingFoot = new DoubleYoVariable("kdSwingFoot", registry);
   private final DoubleYoVariable kpCapturePoint = new DoubleYoVariable("kpCapturePoint", registry);
   private final DoubleYoVariable qDesSupportKnee = new DoubleYoVariable("qDesSupportKnee", registry);

   private final DoubleYoVariable desiredWalkingVelocity = new DoubleYoVariable("desiredWalkingVelocity", registry);
   private final DoubleYoVariable alphaDesiredVelocity = new DoubleYoVariable("alphaDesiredVelocity", registry);
   private final AlphaFilteredYoVariable desiredWalkingVelocityFiltered = new AlphaFilteredYoVariable("desiredWalkingVelocitySmoothed", registry,
         alphaDesiredVelocity, desiredWalkingVelocity); // filter the velocity so that the transition from one to another is smoother
   private final DoubleYoVariable desiredVelocityToICPFactorDistanceThreshold = new DoubleYoVariable("desiredVelocityToICPFactorDistanceThreshold", registry);

   private final DoubleYoVariable stepLengthFactor = new DoubleYoVariable("stepLengthFactor", registry);
   private final DoubleYoVariable stepLengthCorrectionFactor = new DoubleYoVariable("stepLengthCorrectionFactor", registry);
   private final DoubleYoVariable defaultStepLength = new DoubleYoVariable("defaultStepLength", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final SideDependentList<DoubleYoVariable> distanceFootToCoPbasedWeights = new SideDependentList<>();
   private final int numOfBalls = 100;
 
   
   /**
    * Constructor
    */
   public PointMassController(PointMassIDRobot robot, DoubleYoVariable yoTime, double controlDT)
   {
      this.controlDT = controlDT;
      this.robot = robot;
      stateMachine = new StateMachine<>("stateMachine", "switchTime", WalkingStates.class, yoTime, registry);
      yoFootPositions = robot.getYoFootPositions();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         distanceFootToCoPbasedWeights.put(robotSide, new DoubleYoVariable(sidePrefix + "distanceFootToCoPbasedWeights", registry));
      }
      
      setupStateMachine();
      initializeControlParameters();
      initializeVizualizers();
      
   }

   /**
    * Initialize and setup
    */
   public void initializeVizualizers()
   {
      // 2D CoP and ICP visualizer
      YoArtifactPosition desiredCoPArtifact = new YoArtifactPosition("DesiredCoP", yoDesiredCoP.getYoX(), yoDesiredCoP.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Red().getColor().get(), 0.01);
      YoArtifactPosition desiredICPArtifact = new YoArtifactPosition("DesiredICP", yoDesiredICP.getYoX(), yoDesiredICP.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Blue().getColor().get(), 0.01);
      artifactList.add(desiredCoPArtifact);
      artifactList.add(desiredICPArtifact);
      
      // 3D trajectory visualizer
      YoGraphicPosition desiredPositionSwingFootViz = new YoGraphicPosition("desiredPositionSwingFoot", yodesiredPositionSwingFoot, 0.03, YoAppearance.Green());
      yoGraphicsList.add(desiredPositionSwingFootViz);
      
      DoubleProvider swingTimeProvider = new YoVariableDoubleProvider(swingTime);
      swingTrajectory = new ParabolicCartesianTrajectoryGenerator("swingTrajectory", worldFrame, swingTimeProvider, groundClearance, registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      swingTrajectoryViz = new BagOfBalls(numOfBalls, 0.005, registry, yoGraphicsListRegistry);
      yoGraphicsList.addAll(yoGraphicsListRegistry.getYoGraphicsLists().get(0).getYoGraphics());
   }

   public void initializeControlParameters()
   {
      alphaDesiredVelocity.set(0.9999);
      defaultStepLength.set(0.30);
      maxStepLength.set(0.70);

      desiredVelocityToICPFactorDistanceThreshold.set(0.7);
      stepLengthFactor.set(0.2);
      stepLengthCorrectionFactor.set(0.5);

      kpSupportKnee.set(200.0);
      kdSupportKnee.set(100.0);
      qDesSupportKnee.set(0.0);

      kpSwingFoot.set(200.0);
      kdSwingFoot.set(20.0);

      kpCapturePoint.set(2.0);

      swingTime.set(0.35);
   }

   public void setupStateMachine()
   {
      DoubleSupportState standingState = new DoubleSupportState();
      stateMachine.addState(standingState);

      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingStates singleSupportEnum = WalkingStates.getSingleSuppportStateFromSwingSide(robotSide);
         SingleSupport singleSupportState = new SingleSupport(singleSupportEnum);

         DoubleToSingleSupportCondition fromStandingToSingleSupportCondition = new DoubleToSingleSupportCondition(robotSide);
         StateTransition<WalkingStates> standingStateTransition = new StateTransition<>(singleSupportEnum, fromStandingToSingleSupportCondition);
         standingState.addStateTransition(standingStateTransition);

         SingleToDoubleSupportCondition fromSingleSupportToTransferCondition = new SingleToDoubleSupportCondition(robotSide);
         StateTransition<WalkingStates> singleSupportStateTransition = new StateTransition<>(WalkingStates.DOUBLE_SUPPORT, fromSingleSupportToTransferCondition);
         singleSupportState.addStateTransition(singleSupportStateTransition);

         stateMachine.addState(singleSupportState);
      }

      stateMachine.setCurrentState(WalkingStates.DOUBLE_SUPPORT);
   }

   /**
    * Double Support State
    */
   private class DoubleSupportState extends State<WalkingStates>
   {

      private final FramePoint2d desiredICP = new FramePoint2d();
      private final FramePoint2d realICP = new FramePoint2d();
      private final FramePoint2d desiredCoP = new FramePoint2d();
      private final FrameVector2d tempControl = new FrameVector2d();

      public DoubleSupportState()
      {
         super(WalkingStates.DOUBLE_SUPPORT);
      }

      @Override
      public void doAction()
      {
         // Hips
         for (RobotSide robotSide : RobotSide.values)
            robot.getHip(robotSide).setTau(0.0);


         // ICP
         if (Math.abs(desiredWalkingVelocityFiltered.getDoubleValue()) < 1.0e-3)
         {
            desiredICP.setToZero(robot.getMidSoleZUpFrame());
            desiredICP.changeFrame(worldFrame);
         }
         else
         {
            robot.getCoM(desiredICP);
            double desICPXsecondTerm = desiredVelocityToICPFactorDistanceThreshold.getDoubleValue() / robot.getOmega0() * desiredWalkingVelocityFiltered.getDoubleValue();
            desiredICP.add(desICPXsecondTerm, 0.0);
         }

         yoDesiredICP.set(desiredICP);
         robot.getCapturePoint(realICP);
         
         //CoP
         computeDesiredCoP();     
         
         // Desired Fz (external) -- from PD controller
         double desiredFzToMaintainHeight = 0.0;
         
         for (RobotSide robotSide : RobotSide.values)
         {
            double legPitch = robot.getBodyPitch() + robot.getHip(robotSide).getQ();
            PrismaticJoint knee = robot.getKnee(robotSide);
            double pControl = kpSupportKnee.getDoubleValue() * (qDesSupportKnee.getDoubleValue() - knee.getQ());
            double dControl = - kdSupportKnee.getDoubleValue() * knee.getQd();
            double kneeTau = pControl + dControl;
            desiredFzToMaintainHeight += kneeTau / Math.cos(legPitch);
         }
         
         // Total Fz (external) 
         double totalExternalFz = desiredFzToMaintainHeight - robot.getTotalRobotMass() * robot.getGravity();
         
         
         // KNEE TORQUE CALCULATIONS
         
            // A) Variables
         FrameVector2d u1 = new FrameVector2d();
         FrameVector2d u2 = new FrameVector2d();
         FrameVector2d d1 = new FrameVector2d();
         FrameVector2d d2 = new FrameVector2d();
         
         double theta1 = robot.getBodyPitch() + robot.getHipPitch(RobotSide.LEFT);
         double theta2 = robot.getBodyPitch() + robot.getHipPitch(RobotSide.RIGHT);
         
         u1.set(Math.cos(theta1), Math.sin(theta1));
         u2.set(Math.cos(theta2), Math.sin(theta2));
         d1.set(0.0, desiredCoP.getX() - yoFootPositions.get(RobotSide.LEFT).getX());
         d2.set(0.0, desiredCoP.getX() - yoFootPositions.get(RobotSide.RIGHT).getX());

         boolean isD1GreatEnough = d1.lengthSquared() > 1.0e-5;
         boolean isD2GreatEnough = d2.lengthSquared() > 1.0e-5;
         double rightKneeTau;
         double leftKneeTau;
        
            // Weights as a function of distance FROM foot TO CoP
         double distanceFeetX = Math.abs(yoFootPositions.get(RobotSide.LEFT).getX() - yoFootPositions.get(RobotSide.RIGHT).getX());
         distanceFootToCoPbasedWeights.get(RobotSide.LEFT).set(Math.abs(yoFootPositions.get(RobotSide.RIGHT).getX() - desiredCoP.getX()) / distanceFeetX); //  d1/(d1 + d2)
         distanceFootToCoPbasedWeights.get(RobotSide.RIGHT).set(1.0 - distanceFootToCoPbasedWeights.get(RobotSide.LEFT).getDoubleValue()); //d2/(d1+d2) = 1 - d1/(d1 + d2)

            // B) Calculations
         if (!isD1GreatEnough || !isD2GreatEnough)    // to deal with exceptions
         {
            leftKneeTau = distanceFootToCoPbasedWeights.get(RobotSide.LEFT).getDoubleValue() * totalExternalFz;
            rightKneeTau = distanceFootToCoPbasedWeights.get(RobotSide.RIGHT).getDoubleValue() * totalExternalFz;
         }
        
         else  // general case
         {
            rightKneeTau = u1.cross(d1) * totalExternalFz;
            rightKneeTau /= u2.getX() * u1.cross(d1) - u1.getX() * u2.cross(d2);
            leftKneeTau = - rightKneeTau * u2.cross(d2) / u1.cross(d1);
         }
         
         robot.getKnee(RobotSide.LEFT).setTau(leftKneeTau);
         robot.getKnee(RobotSide.RIGHT).setTau(rightKneeTau);        
      }

      // *** Compute CoP ***
      private void computeDesiredCoP()
      {
         double minFootX = Math.min(yoFootPositions.get(RobotSide.LEFT).getX(), yoFootPositions.get(RobotSide.RIGHT).getX());
         double maxFootX = Math.max(yoFootPositions.get(RobotSide.LEFT).getX(), yoFootPositions.get(RobotSide.RIGHT).getX());

         desiredCoP.setIncludingFrame(realICP);

         tempControl.setIncludingFrame(realICP);
         tempControl.sub(desiredICP);
         tempControl.scale(kpCapturePoint.getDoubleValue() / robot.getOmega0());

         desiredCoP.add(tempControl);

         tempControl.setIncludingFrame(worldFrame, desiredWalkingVelocityFiltered.getDoubleValue(), 0.0);
         tempControl.scale(-1.0 / robot.getOmega0());

         desiredCoP.add(tempControl);

         desiredCoP.setX(MathTools.clipToMinMax(desiredCoP.getX(), minFootX, maxFootX));
         yoDesiredCoP.set(desiredCoP);
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

   /**
    * Single Support State
    */
   private class SingleSupport extends State<WalkingStates>
   {
      private final RobotSide swingSide, supportSide;
      private final ReferenceFrame swingSoleFrame;
      private final GeometricJacobian swingLegJacobian;

      // swing trajectory variables 
      private final FramePoint initialPosition = new FramePoint();
      private final FrameVector initialVelocity = new FrameVector();
      private final FrameVector initialAcceleration = new FrameVector();
      private final FramePoint finalDesiredPosition = new FramePoint();
      private final FrameVector finalDesiredVelocity = new FrameVector();
      private int nTicksSinceTrajectoryIsDone = 0;
      
      private final FramePoint desiredPosition = new FramePoint();
      private final FramePoint desiredPositionForViz = new FramePoint();
      private final FrameVector pControl = new FrameVector();

      private final FrameVector currentVelocity = new FrameVector();
      private final FrameVector desiredVelocity = new FrameVector();
      private final FrameVector dControl = new FrameVector();
      
      // tau computations
      private final Wrench desiredSwingFootWrench = new Wrench();
      
      public SingleSupport(WalkingStates stateEnum)
      {
         super(stateEnum);
         swingSide = stateEnum.getSwingSide();
         supportSide = swingSide.getOppositeSide();
         swingSoleFrame = robot.getSoleFrame(swingSide);
         swingLegJacobian = robot.getLegJacobian(swingSide);
         System.out.println(swingLegJacobian);
      }

      @Override
      public void doAction()
      {
         // Support knee
         PrismaticJoint knee = robot.getKnee(supportSide);
         double kneeTau = kpSupportKnee.getDoubleValue() * (qDesSupportKnee.getDoubleValue() - knee.getQ()) - kdSupportKnee.getDoubleValue() * knee.getQd();
         
         double totalWeight = robot.getTotalRobotMass() * robot.getGravity();
         FrameVector footForce = new FrameVector(worldFrame, 0.0, 0.0, -totalWeight);
         footForce.changeFrame(knee.getFrameAfterJoint()); //TODO why not just the weight in world?
         
         knee.setTau(kneeTau + footForce.getZ()); //total tau = controller + FF(weight)
         
         // Swing
         updateSwingTrajectory();
         swingTrajectory.compute(getTimeInCurrentState());

         if (swingTrajectory.isDone())
         {  
            nTicksSinceTrajectoryIsDone++;
            desiredVelocity.setIncludingFrame(finalDesiredVelocity);
         }
         
         else
         {
            swingTrajectory.packVelocity(desiredVelocity);      
         }
         
         // 1) proportional control
         swingTrajectory.packPosition(desiredPosition);
         desiredPosition.add(0.0, 0.0, nTicksSinceTrajectoryIsDone * finalDesiredVelocity.getZ() * controlDT);  
         yodesiredPositionSwingFoot.setAndMatchFrame(desiredPosition); 
         desiredPosition.changeFrame(swingSoleFrame);      
         pControl.setIncludingFrame(desiredPosition);
         pControl.scale(kpSwingFoot.getDoubleValue());
         
         // 2) derivative control   
         robot.getFootLinearVelocity(swingSide, currentVelocity);
         dControl.setIncludingFrame(desiredVelocity);
         dControl.sub(currentVelocity);
         dControl.changeFrame(swingSoleFrame);
         dControl.scale(kdSwingFoot.getDoubleValue());
         
         // 3) computing torques using wrench and jacobian
         desiredSwingFootWrench.setToZero(robot.getLowerRigidBody(swingSide).getBodyFixedFrame(), swingSoleFrame);
         desiredSwingFootWrench.setLinearPart(pControl);
         desiredSwingFootWrench.addLinearPart(dControl);
         System.out.println(desiredSwingFootWrench);

         DenseMatrix64F desiredSwingTaus = swingLegJacobian.computeJointTorques(desiredSwingFootWrench);
//         System.out.println(desiredSwingTaus);
         robot.getHip(supportSide).setTau(-desiredSwingTaus.get(0, 0));
         robot.getHip(swingSide).setTau(desiredSwingTaus.get(0, 0));
         robot.getKnee(swingSide).setTau(desiredSwingTaus.get(1, 0));
      }

      //***  Update swing trajectory  ***
      private void updateSwingTrajectory()
      {
         yoFootPositions.get(supportSide).getFrameTuple(finalDesiredPosition);
         finalDesiredPosition.changeFrame(worldFrame);
         
         double dx = Math.signum(robot.getXVelocity()) * defaultStepLength.getDoubleValue(); // Use the sign to make it walk in both directions depending on if v>0 or v<0
         dx += stepLengthFactor.getDoubleValue() * desiredWalkingVelocityFiltered.getDoubleValue();
         dx -= stepLengthCorrectionFactor.getDoubleValue() * (desiredWalkingVelocityFiltered.getDoubleValue() - robot.getXVelocity());
        
         if (Math.abs(dx) > maxStepLength.getDoubleValue())
            dx = Math.signum(dx) * maxStepLength.getDoubleValue(); //just take the biggest step possible
         
         finalDesiredPosition.add(dx, 0.0, 0.0);
         swingTrajectory.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredPosition, finalDesiredVelocity);

         double deltaT = swingTime.getDoubleValue() / numOfBalls;
         for (int i = 0; i < numOfBalls; i++)
         {
            swingTrajectory.computeNextTick(desiredPositionForViz, deltaT);
            swingTrajectoryViz.setBall(desiredPositionForViz, i);
         }
      }
      
      @Override
      public void doTransitionIntoAction()
      {
         nTicksSinceTrajectoryIsDone = 0;
         yoDesiredCoP.setByProjectionOntoXYPlane(yoFootPositions.get(supportSide));
         initialPosition.setToZero(swingSoleFrame);
         initialPosition.changeFrame(worldFrame);
         updateSwingTrajectory();

      }

      @Override
      public void doTransitionOutOfAction()
      {
     

      }

   }

   /**
    * Transition conditions
    */
   
   // (1) Double support to single support
   private class DoubleToSingleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide nextSwingSide, nextSupportSide;

      public DoubleToSingleSupportCondition(RobotSide nextSwingSide)
      {
         this.nextSwingSide = nextSwingSide;
         this.nextSupportSide = nextSwingSide.getOppositeSide();
      }

      @Override //TODO use Math.signum to avoid repetition
      public boolean checkCondition()  
      {
         double desICPXsecondTerm = desiredVelocityToICPFactorDistanceThreshold.getDoubleValue() / robot.getOmega0() * desiredWalkingVelocityFiltered.getDoubleValue();
         
         if (robot.getXVelocity() > 0.0) //robot moving towards the right
         {
            if (yoFootPositions.get(nextSwingSide).getX() > yoFootPositions.get(nextSupportSide).getX()) //if the foot that's going to swing is in front of the one that is going to be the support
               return false;
                        
            if (robot.getYoCapturePoint().getX() < yoFootPositions.get(nextSupportSide).getX() + desICPXsecondTerm) //if the ICP is behind the next support foot
               return false;
         }
         else //robot moving towards the left 
         {
            if (yoFootPositions.get(nextSwingSide).getX() < yoFootPositions.get(nextSupportSide).getX())
               return false;
            if (robot.getYoCapturePoint().getX() > yoFootPositions.get(nextSupportSide).getX()  + desICPXsecondTerm)
               return false;
         }
         
         return true;
      }

   }

   // (2) Single support to double support
   private class SingleToDoubleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide swingSide;

      public SingleToDoubleSupportCondition(RobotSide swingSide)
      {
         this.swingSide = swingSide;
      }

      @Override
      public boolean checkCondition()
      {
         boolean minimumTimeHasPassed = stateMachine.getCurrentState().getTimeInCurrentState() > 0.5 * swingTime.getDoubleValue();
         boolean touchdownDetected = robot.getFootForce(swingSide) > 5.0;
         
         return minimumTimeHasPassed && touchdownDetected;
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }

   @Override
   public void doControl()
   {
      robot.updateIDRobot();
      desiredWalkingVelocityFiltered.update();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      robot.updateSCSRobot();
   }

   /**
    *  Getters and setters
    */
   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }
}
