package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

public class SimpleDesiredCenterOfPressureFilter implements DesiredCenterOfPressureFilter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCenterOfPressureFilter");


   /*
    * Need to have three different filtered points because it matters in which frame you filter.
    */
   private final AlphaFilteredYoFramePoint2d filteredDesiredCoPDoubleSupport;
   private final SideDependentList<AlphaFilteredYoFramePoint2d> filteredDesiredCoPsSingleSupport = new SideDependentList<AlphaFilteredYoFramePoint2d>();

   private final DoubleYoVariable alphaDesiredCoP = new DoubleYoVariable("alphaDesiredCoP", registry);
   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", RobotSide.class, registry);
   private final CouplingRegistry couplingRegistry;
   private final double controlDT;
   private final FramePoint2d returnedFilteredDesiredCoP = new FramePoint2d(ReferenceFrame.getWorldFrame());

   public SimpleDesiredCenterOfPressureFilter(CouplingRegistry couplingRegistry, CommonWalkingReferenceFrames referenceFrames, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.controlDT = controlDT;
      filteredDesiredCoPDoubleSupport = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d("filteredDesCoPDoubleSupport", "", registry,
              alphaDesiredCoP, referenceFrames.getMidFeetZUpFrame());

      for (RobotSide robotSide : RobotSide.values())
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
      if (supportLegPreviousTick.getEnumValue() != supportLeg)
      {
         resetCoPFilters();
         supportLegPreviousTick.set(supportLeg);
      }

      AlphaFilteredYoFramePoint2d filteredDesiredCoP;
      FrameConvexPolygon2d supportPolygon;
      if (supportLeg == null)
      {
         filteredDesiredCoP = filteredDesiredCoPDoubleSupport;
         supportPolygon = couplingRegistry.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      }
      else
      {
         filteredDesiredCoP = filteredDesiredCoPsSingleSupport.get(supportLeg);
         supportPolygon = couplingRegistry.getBipedSupportPolygons().getFootPolygonInAnkleZUp(supportLeg);
      }

      desiredCenterOfPressure.changeFrame(filteredDesiredCoP.getReferenceFrame());
      filteredDesiredCoP.update(desiredCenterOfPressure);
      filteredDesiredCoP.getFramePoint2d(returnedFilteredDesiredCoP);
      supportPolygon.orthogonalProjection(returnedFilteredDesiredCoP);

      return returnedFilteredDesiredCoP;
   }

   private void resetCoPFilters()
   {
      filteredDesiredCoPDoubleSupport.reset();

      for (RobotSide robotSide : RobotSide.values())
      {
         filteredDesiredCoPsSingleSupport.get(robotSide).reset();
      }
   }

   public void setParametersForR2()
   {
      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(8.84, controlDT));      
   }
   
   public void setParametersForM2V2()
   {
      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(8.84, controlDT));      
   }
}
