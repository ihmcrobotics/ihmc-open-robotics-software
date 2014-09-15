package us.ihmc.commonWalkingControlModules.controllers.roughTerrain;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;

/**
 * This module can be used to allow a terrain exploration using the foot. 
 * The module explores the next foothold moving the foot CoP while gradually shifting
 * the weight of the robot from one foot to the other. 
 * Eventually a new foothold is computed based on the performance of the previous test and 
 * the contact points of the foot are updated to take into account the limits of the foothold.
 * If the area of the new foothold is too small the module can do a re-planning of the footstep.
 *  
 * @author Robot Lab
 *
 */
public class FootExplorationControlModule
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double maxICPAbsError = 0.015;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("footExplorationModule");
   
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
   private final FootExplorationICPPlanner footExplorationICPPlanner;
   private final BooleanYoVariable icpIsCloseEnoughToDesired;
   private final DoubleYoVariable explorationICPError;
   
   private Footstep nextFootStep;
   private RobotSide footUderCoPControl;
   private FrameConvexPolygon2d supportFootPolygon;
   private FramePoint supportFootCentroid;
   private FramePoint nextFootStepCentroid; 
   private FrameLine2d ICPTrajectory;
   private FramePoint2d desiredICP;
   
   public FootExplorationControlModule(YoVariableRegistry parentRegistry, MomentumBasedController momentumBasedController)
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
      icpIsCloseEnoughToDesired = new BooleanYoVariable("icpIsInsideNextFootStep", registry);
      explorationICPError = new DoubleYoVariable("explorationICPError", registry);
      footExplorationICPPlanner = new FootExplorationICPPlanner("footExplorationICPPlanner", worldFrame, remainingSwingTimeProvider, initialICPPositionProvider,
                                                                finalICPPositionProvider, registry);
      
      this.momentumBasedController = momentumBasedController;
      performCoPExploration.set(false);
      ICPTrajectory = new FrameLine2d(worldFrame, new Point2d(0.0, 0.0), new Point2d(1.0, 1.0));
      desiredICP = new FramePoint2d(worldFrame);
      nextFootStepCentroid = new FramePoint(worldFrame);
      initialTime.set(Double.NaN);
      initialExplorationTime.set(Double.NEGATIVE_INFINITY);
      parentRegistry.addChild(registry);
   }
   
   public void controlSwingFoot(double time, YoFramePoint2d  desiredICPToPack, YoFrameVector2d desiredICPVelocityToPack, FramePoint2d currentCapturePoint)
   {
      if (!swingIsFinished.getBooleanValue())
      {
         ReferenceFrame icpReferenceFrame = desiredICPToPack.getReferenceFrame();
         FramePoint2d desiredICP = supportFootPolygon.getCentroid();
         desiredICP.changeFrame(icpReferenceFrame);
         desiredICPToPack.set(desiredICP);
         desiredICPVelocityToPack.setToZero();
      }
      else
      {
         performExploration(time, desiredICPToPack, desiredICPVelocityToPack);
      } 

      explorationICPError.set(currentCapturePoint.distance(desiredICP));
      icpIsCloseEnoughToDesired.set(Math.abs(explorationICPError.getDoubleValue()) < maxICPAbsError);
   }
   
   private void performExploration(double time, YoFramePoint2d  desiredICPToPack, YoFrameVector2d desiredICPVelocityToPack)
   {
      // starting from the center find a safe starting point and then move the cop 
      momentumBasedController.setFeetCoPControlIsActive(true);
//      momentumBasedController.setFeetCoPControlIsActive(false);
      momentumBasedController.setSideOfFootUnderCoPControl(footUderCoPControl);
      if(initialExplorationTime.getDoubleValue() < 0.0)
      {
         initialExplorationTime.set(time);
         isExploringFootHold.set(true);
      }
      explorationTime.set(time - initialExplorationTime.getDoubleValue());
      
      if(explorationTime.getDoubleValue() < Math.PI * 2)
      {
         momentumBasedController.setFootCoPOffsetX(0.1 * Math.sin(explorationTime.getDoubleValue()));
         momentumBasedController.setFootCoPOffsetY(0.03 * Math.cos(explorationTime.getDoubleValue()));
         
         ReferenceFrame icpReferenceFrame = desiredICPToPack.getReferenceFrame();
         footExplorationICPPlanner.getICPPosition(desiredICP);
         desiredICP.changeFrame(icpReferenceFrame);
         desiredICPToPack.set(desiredICP.getX(), desiredICP.getY());
         desiredICPVelocityToPack.setToZero();
      }
      else
      {
         if (ICPPositionPercentage.getDoubleValue() > 0.6)
         {
            ICPPositionPercentage.add(0.001);
            momentumBasedController.setFeetCoPControlIsActive(false);

            if (ICPPositionPercentage.getDoubleValue() > 0.95)
            {
               //            ICPPositionPercentage.set(0.9);

               footHoldIsSafeEnough.set(true);
               isExploringFootHold.set(false);
//               performCoPExploration.set(false);
//               initialExplorationTime.set(Double.NEGATIVE_INFINITY);
               momentumBasedController.setFeetCoPControlIsActive(false);
            }
         }
         else
         {
            ICPPositionPercentage.add(0.2);
            initialExplorationTime.set(Double.NEGATIVE_INFINITY);
         }

         ReferenceFrame icpReferenceFrame = desiredICPToPack.getReferenceFrame();
         footExplorationICPPlanner.getICPPosition(desiredICP);
         desiredICP.changeFrame(icpReferenceFrame);
         desiredICPToPack.set(desiredICP.getX(), desiredICP.getY());

      }
   }
   
   public void initialize(Footstep nextFootStep, double time, FrameConvexPolygon2d supportFootPolygon, RobotSide footUderCoPControl)
   {
      if (performCoPExploration.getBooleanValue())
      {
         initialTime.set(time);
         this.nextFootStep = nextFootStep;
         this.footUderCoPControl = footUderCoPControl;
         this.supportFootPolygon = new FrameConvexPolygon2d(supportFootPolygon);

         FramePoint2d tempCentroid = this.supportFootPolygon.getCentroid();
         supportFootCentroid = new FramePoint(tempCentroid.getReferenceFrame(), tempCentroid.getX(), tempCentroid.getY(), 0.0);
         supportFootCentroid.changeFrame(worldFrame);

         nextFootStep.getPositionIncludingFrame(nextFootStepCentroid);
         nextFootStepCentroid.changeFrame(worldFrame);

         ICPTrajectory.set(worldFrame, supportFootCentroid.getX(), supportFootCentroid.getY(), nextFootStepCentroid.getX(), nextFootStepCentroid.getY());

         isControllingSwingFoot.set(true);
         footExplorationICPPlanner.initialize();
      }
   }
   
   public void reset()
   {
      isControllingSwingFoot.set(false);
      footHoldIsSafeEnough.set(false);
      swingIsFinished.set(false);
      icpIsCloseEnoughToDesired.set(false);
      initialExplorationTime.set(Double.NEGATIVE_INFINITY);
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

      public void getICPVelocity(FrameVector2d desiredICPVelocityToPack)
      {
         desiredICPVelocityToPack.setToZero();
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

   private DoubleProvider remainingSwingTimeProvider = new DoubleProvider()
   {
      @Override
      public double getValue()
      {
         return 1.0;
      }
   };
   
   public boolean isDone()
   {
      return footHoldIsSafeEnough.getBooleanValue() && icpIsCloseEnoughToDesired.getBooleanValue();
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
