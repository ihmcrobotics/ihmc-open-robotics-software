package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class ExploreFootPolygonState extends AbstractFootControlState
{
   private boolean done = true;
   public enum ExplorationMethod
   {
      SPRIAL, LINES, FAST_LINE
   };

   private final EnumYoVariable<ExplorationMethod> explorationMethod;

   private final HoldPositionState internalHoldPositionState;

   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();

   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;

   private final DoubleYoVariable lastShrunkTime, spiralAngle;
   private final double dt;

   /**
    * This is the amount of time after touch down during which no foothold exploration is done
    */
   private final DoubleYoVariable recoverTime;

   /**
    * This is the amount of time the line exploration uses to go to a corner
    */
   private final DoubleYoVariable timeToGoToCorner;

   /**
    * This is the amount of time the line exploration will keep the cop in a corner
    */
   private final DoubleYoVariable timeToStayInCorner;

   /**
    * The weight the cop command gets for the qp solver
    */
   private final DoubleYoVariable copCommandWeight;
   private final YoFrameVector2d copCommandWeightVector;

   private final IntegerYoVariable yoCurrentCorner;

   private final DoubleYoVariable timeBeforeExploring;

   public ExploreFootPolygonState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.EXPLORE_POLYGON, footControlHelper);
      String footName = contactableFoot.getName();

      explorationMethod = new EnumYoVariable<ExplorationMethod>(footName + "ExplorationMethod", registry, ExplorationMethod.class);
      explorationMethod.set(ExplorationMethod.LINES);

      dt = momentumBasedController.getControlDT();
      ExplorationParameters explorationParameters =
            footControlHelper.getWalkingControllerParameters().getOrCreateExplorationParameters(registry);

      YoVariableRegistry childRegistry = new YoVariableRegistry("ExploreFootPolygon");
      registry.addChild(childRegistry);
      internalHoldPositionState = new HoldPositionState(footControlHelper, null, gains, childRegistry);
      internalHoldPositionState.setDoSmartHoldPosition(false);
      internalHoldPositionState.doFootholdAdjustments(false);

      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);

      centerOfPressureCommand.setContactingRigidBody(contactableFoot.getRigidBody());

      lastShrunkTime = new DoubleYoVariable(footName + "LastShrunkTime", registry);
      spiralAngle = new DoubleYoVariable(footName + "SpiralAngle", registry);

      recoverTime = explorationParameters.getRecoverTime();
      timeBeforeExploring = explorationParameters.getTimeBeforeExploring();

      timeToGoToCorner = explorationParameters.getTimeToGoToCorner();
      timeToGoToCorner.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            lastShrunkTime.set(getTimeInCurrentState());
         }
      });

      timeToStayInCorner = explorationParameters.getTimeToStayInCorner();
      timeToStayInCorner.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            lastShrunkTime.set(getTimeInCurrentState());
         }
      });

      copCommandWeight = explorationParameters.getCopCommandWeight();
      copCommandWeightVector = new YoFrameVector2d(footName + "CopCommandWeight", null, registry);
      copCommandWeightVector.set(copCommandWeight.getDoubleValue(), copCommandWeight.getDoubleValue());

      desiredOrientation.setToZero();
      desiredAngularVelocity.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      yoCurrentCorner = new IntegerYoVariable(footName + "CurrentCornerExplored", registry);
   }

   public void setWeight(double weight)
   {
      internalHoldPositionState.setWeight(weight);
   }

   public void setWeights(Vector3d angular, Vector3d linear)
   {
      internalHoldPositionState.setWeights(angular, linear);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      lastShrunkTime.set(0.0);
      spiralAngle.set(0.0);
      lastCornerCropped = 0;
      internalHoldPositionState.doTransitionIntoAction();
      done = false;
      partialFootholdControlModule.turnOnCropping();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      internalHoldPositionState.doTransitionOutOfAction();
      yoCurrentCorner.set(0);
   }

   private final Vector2d tempVector2d = new Vector2d();
   private final FramePoint2d shrunkPolygonCentroid = new FramePoint2d();
   private final FramePoint2d desiredCenterOfPressure = new FramePoint2d();
   private final FramePoint2d currentCorner = new FramePoint2d();
   private int currentCornerIdx = 0;
   private int lastCornerCropped = 0;

   @Override
   public void doSpecificAction()
   {
      double timeInState = getTimeInCurrentState();

      footSwitch.computeAndPackCoP(cop);
      momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);
      partialFootholdControlModule.compute(desiredCoP, cop);

      if (timeInState < timeBeforeExploring.getDoubleValue())
      {
         partialFootholdControlModule.clearCoPGrid();
      }

      boolean contactStateHasChanged = false;
      if (timeInState > recoverTime.getDoubleValue() && !done)
      {
         YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
         contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
         if (contactStateHasChanged)
         {
            contactState.notifyContactStateHasChanged();
            lastShrunkTime.set(timeInState);
            spiralAngle.add(Math.PI/2.0);
            done = false;
         }
      }

      if (timeInState > timeBeforeExploring.getDoubleValue() && timeInState > recoverTime.getDoubleValue() && !done)
      {
         // Foot exploration through CoP shifting...
         if (explorationMethod.getEnumValue() == ExplorationMethod.SPRIAL)
         {
            double freq = 0.6;
            double rampOutDuration = 0.3;
            double settleDuration = 0.1;

            double percentRampOut = (timeInState - lastShrunkTime.getDoubleValue() - settleDuration) / rampOutDuration;
            rampOutDuration = MathTools.clipToMinMax(rampOutDuration, 0.0, 1.0);

            boolean doSpiral = timeInState - lastShrunkTime.getDoubleValue() - settleDuration > rampOutDuration;

            if (doSpiral)
            {
               spiralAngle.add(2.0 * Math.PI * freq * dt);
            }

            if (timeInState - lastShrunkTime.getDoubleValue() - settleDuration - rampOutDuration > 1.0/freq)
            {
               done = true;
            }

            ReferenceFrame soleFrame = footControlHelper.getContactableFoot().getSoleFrame();
            partialFootholdControlModule.getShrunkPolygonCentroid(shrunkPolygonCentroid);
            shrunkPolygonCentroid.changeFrame(soleFrame);

            desiredCenterOfPressure.setIncludingFrame(soleFrame,
                  shrunkPolygonCentroid.getX() + 0.10 * percentRampOut * Math.cos(spiralAngle.getDoubleValue()),
                  shrunkPolygonCentroid.getY() + 0.05 * percentRampOut * Math.sin(spiralAngle.getDoubleValue()));

            partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
            desiredCenterOfPressure.scale(0.9);
         }
         else if (explorationMethod.getEnumValue() == ExplorationMethod.LINES)
         {
            if (contactStateHasChanged)
            {
               lastCornerCropped = currentCornerIdx+1;
            }

            double timeToGoToCorner = this.timeToGoToCorner.getDoubleValue();
            double timeToStayAtCorner = this.timeToStayInCorner.getDoubleValue();

            double timeToExploreCorner = timeToGoToCorner + timeToStayAtCorner;
            double timeExploring = timeInState - lastShrunkTime.getDoubleValue() + lastCornerCropped*timeToExploreCorner;

            partialFootholdControlModule.getSupportPolygon(supportPolygon);
            int corners = supportPolygon.getNumberOfVertices();
            currentCornerIdx = (int) (timeExploring / timeToExploreCorner);
            currentCornerIdx = currentCornerIdx % corners;
            yoCurrentCorner.set(currentCornerIdx);

            supportPolygon.getFrameVertex(currentCornerIdx, currentCorner);
            FramePoint2d centroid = supportPolygon.getCentroid();

            ReferenceFrame soleFrame = footControlHelper.getContactableFoot().getSoleFrame();
            currentCorner.changeFrame(soleFrame);
            centroid.changeFrame(soleFrame);
            desiredCenterOfPressure.changeFrame(soleFrame);

            double timeExploringCurrentCorner = timeExploring - (double)currentCornerIdx * timeToExploreCorner;
            if (timeExploringCurrentCorner <= timeToGoToCorner)
            {
               double percent = timeExploringCurrentCorner / timeToGoToCorner;
               percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
               desiredCenterOfPressure.interpolate(centroid, currentCorner, percent);
            }
            else
            {
               desiredCenterOfPressure.set(currentCorner);
            }

            if (timeInState - lastShrunkTime.getDoubleValue() > 2.0 * timeToExploreCorner * corners)
            {
               done = true;
            }
         }
         else if (explorationMethod.getEnumValue() == ExplorationMethod.FAST_LINE)
         {
            // quickly go forward and backward with the cop to get data for line fitting in the cop occupancy grid
            ReferenceFrame soleFrame = footControlHelper.getContactableFoot().getSoleFrame();

            double timeExploring = timeInState - lastShrunkTime.getDoubleValue();
            double timeToStay = this.timeToStayInCorner.getDoubleValue();
            double distanceToMoveFrontBack = 0.1;
            double distanceToMoveSideSide = 0.08;

            if (timeExploring < timeToStay)
            {
               desiredCenterOfPressure.setIncludingFrame(soleFrame, distanceToMoveFrontBack, 0.0);
               partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
            }
            else if (timeExploring < 2.0 * timeToStay)
            {
               desiredCenterOfPressure.setIncludingFrame(soleFrame, 0.0, distanceToMoveSideSide);
               partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
            }
            else if (timeExploring < 3.0 * timeToStay)
            {
               desiredCenterOfPressure.setIncludingFrame(soleFrame, -distanceToMoveFrontBack, 0.0);
               partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
            }
            else if (timeExploring < 4.0 * timeToStay)
            {
               desiredCenterOfPressure.setIncludingFrame(soleFrame, 0.0, -distanceToMoveSideSide);
               partialFootholdControlModule.projectOntoShrunkenPolygon(desiredCenterOfPressure);
            }
            else if (timeExploring < 4.0 * timeToStay + 0.5)
            {
               desiredCenterOfPressure.setIncludingFrame(soleFrame, 0.0, 0.0);
            }
            else
            {
               done = true;
            }
         }

         if (done)
            partialFootholdControlModule.informExplorationDone();

         centerOfPressureCommand.setDesiredCoP(desiredCenterOfPressure.getPoint());
         copCommandWeightVector.set(copCommandWeight.getDoubleValue(), copCommandWeight.getDoubleValue());
         copCommandWeightVector.get(tempVector2d);
         centerOfPressureCommand.setWeight(tempVector2d);
      }
      else
      {
         lastShrunkTime.set(timeInState);
         spiralAngle.set(0.0);
      }

      internalHoldPositionState.doSpecificAction();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      if (getTimeInCurrentState() > recoverTime.getDoubleValue() && !done)
      {
         inverseDynamicsCommandList.addCommand(centerOfPressureCommand);
      }

      if (attemptToStraightenLegs)
         inverseDynamicsCommandList.addCommand(straightLegsPrivilegedConfigurationCommand);
      else
         inverseDynamicsCommandList.addCommand(bentLegsPrivilegedConfigurationCommand);

      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return internalHoldPositionState.getFeedbackControlCommand();
   }

   public boolean isDoneExploring()
   {
      return done;
   }

}

