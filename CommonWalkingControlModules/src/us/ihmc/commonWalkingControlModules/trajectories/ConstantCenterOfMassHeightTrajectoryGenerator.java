package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CenterOfMassControlMode;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ConstantCenterOfMassHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;
   private final EnumYoVariable<CenterOfMassControlMode> centerOfMassControlType;
   private final ProcessedSensorsInterface processedSensors;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public ConstantCenterOfMassHeightTrajectoryGenerator(ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      registry = new YoVariableRegistry(getClass().getSimpleName());
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry); 
      centerOfMassControlType = EnumYoVariable.create("comControlType", CenterOfMassControlMode.class, registry);
      parentRegistry.addChild(registry);

      desiredCenterOfMassHeight.set(1.15);
      centerOfMassControlType.set(CenterOfMassControlMode.TOTAL_COM);
   }

   public void initialize(RobotSide supportLeg, RobotSide upcomingSupportLeg)
   {
      // empty
   }

   public void compute()
   {
      // empty
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
         this.desiredCenterOfMassHeight.set(currentHeight);
         this.centerOfMassControlType.set(centerOfMassControlType);
      }
   }

}
