package us.ihmc.commonWalkingControlModules.controllers.roughTerrain;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

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
   
   private Footstep nextFootStep;
   
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
      
      this.momentumBasedController = momentumBasedController;
      
      initialTime.set(Double.NaN);
      parentRegistry.addChild(registry);
   }
   
   public void controlSwingFoot(double time, FramePoint2d desiredICPToPack, FrameConvexPolygon2d supportFootPolygon)
   {
      if (!swingIsFinished.getBooleanValue())
      {
         ReferenceFrame icpReferenceFrame = desiredICPToPack.getReferenceFrame();
         FramePoint2d desiredICP = supportFootPolygon.getCentroid();
         desiredICP.changeFrame(icpReferenceFrame);
         desiredICPToPack.set(desiredICP);
      }
      else
      {
         performExploration(time, desiredICPToPack, supportFootPolygon);
      }
      
   }
   
   private void performExploration(double time, FramePoint2d desiredICPToPack, FrameConvexPolygon2d supportFootPolygon)
   {
      // starting from the center find a safe starting point and then move the cop 
      momentumBasedController.setFeetCoPControlIsActive(true);
      if(initialExplorationTime.getDoubleValue() == Double.NaN)
      {
         initialExplorationTime.set(time);
         isExploringFootHold.set(true);
      }
      explorationTime.set(time - initialExplorationTime.getDoubleValue());
      
      if(explorationTime.getDoubleValue() < Math.PI)
      {
         momentumBasedController.setFootCoPOffsetX(Math.sin(explorationTime.getDoubleValue()*2));
         momentumBasedController.setFootCoPOffsetY(Math.cos(explorationTime.getDoubleValue()*2));
         
         // TODO: gradually move the ICP to nextfootstep
         ReferenceFrame icpReferenceFrame = desiredICPToPack.getReferenceFrame();
         FramePoint2d desiredICP = supportFootPolygon.getCentroid();
         desiredICP.changeFrame(icpReferenceFrame);
         desiredICPToPack.set(desiredICP);
      }
      else
      {
         isExploringFootHold.set(false);
         initialExplorationTime.set(Double.NaN);
         momentumBasedController.setFeetCoPControlIsActive(false);
      } 
   }
   
   public void initialize(Footstep nextFootStep, double time)
   {
      initialTime.set(time);
      this.nextFootStep = nextFootStep;
      
      if (performCoPExploration.getBooleanValue())
      {
         isControllingSwingFoot.set(true);
      }
   }
   
   public void reset()
   {
      isControllingSwingFoot.set(false);
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

}
