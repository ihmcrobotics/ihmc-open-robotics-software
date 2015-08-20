package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCenterOfPressureFilter;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;


public class SimpleDesiredCenterOfPressureFilter implements DesiredCenterOfPressureFilter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCenterOfPressureFilter");


   /*
    * Need to have three different filtered points because it matters in which frame you filter.
    */
   private final AlphaFilteredYoFramePoint2d filteredDesiredCoPDoubleSupport;
   private final SideDependentList<AlphaFilteredYoFramePoint2d> filteredDesiredCoPsSingleSupport = new SideDependentList<AlphaFilteredYoFramePoint2d>();

   private final DoubleYoVariable desiredCoPBreakFrequencyHertz = new DoubleYoVariable("desiredCoPBreakFrequencyHertz", registry);
   private final DoubleYoVariable alphaDesiredCoP = new DoubleYoVariable("alphaDesiredCoP", registry);
   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);
   private final OldBipedSupportPolygons bipedSupportPolygons;
   private final double controlDT;
   private final FramePoint2d returnedFilteredDesiredCoP = new FramePoint2d(ReferenceFrame.getWorldFrame());
   private final BooleanYoVariable resetCoPFiltersWhenGoingToDoubleSupport = new BooleanYoVariable("resetCoPFiltersWhenGoingToDoubleSupport", registry);

   public SimpleDesiredCenterOfPressureFilter(OldBipedSupportPolygons oldBipedSupportPolygons, CommonHumanoidReferenceFrames referenceFrames, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = oldBipedSupportPolygons;
      this.controlDT = controlDT;
      filteredDesiredCoPDoubleSupport = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d("filteredDesCoPDoubleSupport", "", registry,
              alphaDesiredCoP, referenceFrames.getMidFeetZUpFrame());

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = "filteredDesiredCoP" + robotSide.getCamelCaseNameForMiddleOfExpression();
         ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(robotSide);
         filteredDesiredCoPsSingleSupport.put(robotSide,
                 AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(namePrefix, "", registry, alphaDesiredCoP, ankleZUpFrame));
      }

      parentRegistry.addChild(registry);
   }


   public FramePoint2d filter(FramePoint2d desiredCenterOfPressure, RobotSide supportLeg)
   {
      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(desiredCoPBreakFrequencyHertz.getDoubleValue(), controlDT));
      FramePoint2d ret;

      if (supportLeg == null)
      {
         ret = filterDoubleSupport(desiredCenterOfPressure);
      }
      else
      {
         ret = filterSingleSupport(desiredCenterOfPressure, supportLeg);
      }

      supportLegPreviousTick.set(supportLeg);

      return ret;
   }


   private FramePoint2d filterSingleSupport(FramePoint2d desiredCenterOfPressure, RobotSide supportLeg)
   {
      RobotSide previousSupportLeg = supportLegPreviousTick.getEnumValue();
      if (previousSupportLeg != supportLeg)
      {
         // Only reset the filters when going to single support.
         resetCoPFilters();
      }

      AlphaFilteredYoFramePoint2d filteredDesiredCoP = filteredDesiredCoPsSingleSupport.get(supportLeg);
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);

      return filterAndProject(desiredCenterOfPressure, filteredDesiredCoP, supportPolygon);
   }
   

   private FramePoint2d filterDoubleSupport(FramePoint2d desiredCenterOfPressure)
   {
      RobotSide previousSupportLeg = supportLegPreviousTick.getEnumValue();
      if (previousSupportLeg != null)
      {
         if (resetCoPFiltersWhenGoingToDoubleSupport.getBooleanValue())
            resetCoPFilters();
         else
         {
            AlphaFilteredYoFramePoint2d singleSupportDesiredCoP = filteredDesiredCoPsSingleSupport.get(previousSupportLeg);
            FramePoint2d singleSupportDesiredCoPFramePoint2d = singleSupportDesiredCoP.getFramePoint2dCopy();
            singleSupportDesiredCoPFramePoint2d.changeFrame(filteredDesiredCoPDoubleSupport.getReferenceFrame());

            filteredDesiredCoPDoubleSupport.reset();
            filteredDesiredCoPDoubleSupport.update(singleSupportDesiredCoPFramePoint2d);
         }
      }

      AlphaFilteredYoFramePoint2d filteredDesiredCoP = filteredDesiredCoPDoubleSupport;
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      
      // test:
//      FramePoint2d midFeet = new FramePoint2d(filteredDesiredCoPDoubleSupport.getReferenceFrame());
//      supportPolygon.orthogonalProjection(midFeet);
//      return midFeet;

//    return filterAndProject(midFeet, filteredDesiredCoP, supportPolygon);

      
      
      return filterAndProject(desiredCenterOfPressure, filteredDesiredCoP, supportPolygon);
   }
   
   private FramePoint2d filterAndProject(FramePoint2d desiredCenterOfPressure, AlphaFilteredYoFramePoint2d filteredDesiredCoP, FrameConvexPolygon2d supportPolygon)
   {
      desiredCenterOfPressure.changeFrame(filteredDesiredCoP.getReferenceFrame());
      filteredDesiredCoP.update(desiredCenterOfPressure);
      filteredDesiredCoP.getFrameTuple2dIncludingFrame(returnedFilteredDesiredCoP);
      returnedFilteredDesiredCoP.changeFrame(supportPolygon.getReferenceFrame());
      supportPolygon.orthogonalProjection(returnedFilteredDesiredCoP);

      return returnedFilteredDesiredCoP;
   }

   private void resetCoPFilters()
   {
      filteredDesiredCoPDoubleSupport.reset();

      for (RobotSide robotSide : RobotSide.values)
      {
         filteredDesiredCoPsSingleSupport.get(robotSide).reset();
      }
   }

   public void setParametersForR2()
   {
      desiredCoPBreakFrequencyHertz.set(8.84);   
      resetCoPFiltersWhenGoingToDoubleSupport.set(false);
   }
   
   public void setParametersForM2V2()
   {
      desiredCoPBreakFrequencyHertz.set(8.84);
      resetCoPFiltersWhenGoingToDoubleSupport.set(false);
   }

   public void setParametersForR2InverseDynamics()
   {
      desiredCoPBreakFrequencyHertz.set(8.84);   
      resetCoPFiltersWhenGoingToDoubleSupport.set(true);
   }
}
