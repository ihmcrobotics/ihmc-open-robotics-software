package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CenterOfMassControlMode;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class LimitedCenterOfMassHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;
   private final DoubleYoVariable desiredCenterOfMassHeightFinal;
   private final DoubleYoVariable maxCenterOfMassHeight;
   private final DoubleYoVariable minCenterOfMassHeight;
   private final EnumYoVariable<CenterOfMassControlMode> centerOfMassControlType;
   private final ProcessedSensorsInterface processedSensors;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final DoubleYoVariable maxCoMHeightDot;
   private final double controlDT;

   public LimitedCenterOfMassHeightTrajectoryGenerator(double controlDT, ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      registry = new YoVariableRegistry(getClass().getSimpleName());
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry); 
      desiredCenterOfMassHeightFinal = new DoubleYoVariable("desiredCenterOfMassHeightFinal", registry); 
      maxCenterOfMassHeight = new DoubleYoVariable("maxCenterOfMassHeight", registry); 
      minCenterOfMassHeight = new DoubleYoVariable("minCenterOfMassHeightFinal", registry); 
      
      centerOfMassControlType = EnumYoVariable.create("comControlType", CenterOfMassControlMode.class, registry);
      parentRegistry.addChild(registry);

      desiredCenterOfMassHeight.set(processedSensors.getCenterOfMassPositionInFrame(worldFrame, centerOfMassControlType.getEnumValue()).getZ());
      desiredCenterOfMassHeightFinal.set(1.15);
      
      maxCoMHeightDot = new DoubleYoVariable("maxCoMHeightDot", registry);
      maxCoMHeightDot.set(0.1);
      this.controlDT = controlDT;
      
      centerOfMassControlType.set(CenterOfMassControlMode.TOTAL_COM);
   }

   public void initialize(RobotSide supportLeg, RobotSide upcomingSupportLeg)
   {
      // empty
   }

   public void compute()
   {
      double finalHeight = MathTools.clipToMinMax(desiredCenterOfMassHeightFinal.getDoubleValue(), minCenterOfMassHeight.getDoubleValue(), maxCenterOfMassHeight.getDoubleValue());
      double error = finalHeight - desiredCenterOfMassHeight.getDoubleValue();
      double maximumChangePerTick = maxCoMHeightDot.getDoubleValue() * controlDT;

      double deltaHeading = MathTools.clipToMinMax(error, -maximumChangePerTick, maximumChangePerTick);

      desiredCenterOfMassHeight.set(desiredCenterOfMassHeight.getDoubleValue() + deltaHeading);
   }

   public double getDesiredCenterOfMassHeight()
   {
      return desiredCenterOfMassHeight.getDoubleValue();
   }

   public double getDesiredCenterOfMassHeightSlope()
   {
      return 0.0;
   }

   public double getDesiredCenterOfMassHeightSecondDerivative()
   {
      return 0.0;
   }

   public void setControlType(CenterOfMassControlMode centerOfMassControlType)
   {
      if (centerOfMassControlType != this.centerOfMassControlType.getEnumValue())
      {
         double currentHeight = processedSensors.getCenterOfMassPositionInFrame(worldFrame, centerOfMassControlType).getZ();
         this.desiredCenterOfMassHeightFinal.set(currentHeight);
         this.desiredCenterOfMassHeight.set(currentHeight);
         this.centerOfMassControlType.set(centerOfMassControlType);
      }
   }

   public void setFinalDesiredHeight(double height)
   {
      this.desiredCenterOfMassHeightFinal.set(height);
   }
   
   public void setMaximumDesiredHeight(double maxHeight)
   {
      maxCenterOfMassHeight.set(maxHeight);
   }
   
   public void setMinimumDesiredHeight(double minHeight)
   {
      minCenterOfMassHeight.set(minHeight);
   }
   
}
