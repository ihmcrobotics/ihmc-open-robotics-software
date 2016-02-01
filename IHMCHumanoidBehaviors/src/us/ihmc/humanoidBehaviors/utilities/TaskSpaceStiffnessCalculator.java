package us.ihmc.humanoidBehaviors.utilities;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class TaskSpaceStiffnessCalculator
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final double lowPassCutoffFreq_Hz = 5.0;
   private final DoubleYoVariable alphaLowPass;

   private final YoFramePoint yoForcePointPosition;
   private final YoFrameVector yoForcePointForce;

   private final FilteredVelocityYoFrameVector yoForcePointVelocity;
   private final FilteredVelocityYoFrameVector yoForcePointForceRateOfChange;

   private final DoubleYoVariable yoForceAlongDirectionOfMotion;
   private final DoubleYoVariable yoForceRateOfChangeAlongDirectionOfMotion;

   private final DoubleYoVariable yoStiffnessAlongDirectionOfMotion;
   private final DoubleYoVariable yoMaxStiffness;

   private final YoFrameVector yoCrossProductOfCurrentVelWithForce;
   private final YoFrameVector yoDirectionOfFreeMotion;

   private final BooleanYoVariable addSimulatedSensorNoise;


   public TaskSpaceStiffnessCalculator(String namePrefix, double controlDT, YoVariableRegistry registry)
   {
      alphaLowPass = new DoubleYoVariable(namePrefix + "Alpha", registry);
      alphaLowPass.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(lowPassCutoffFreq_Hz, controlDT));

      yoForcePointPosition = new YoFramePoint(namePrefix + "Position", world, registry);
      yoForcePointForce = new YoFrameVector(namePrefix + "Force", world, registry);

      yoForcePointVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(namePrefix + "Velocity", "", alphaLowPass, controlDT, registry,
            yoForcePointPosition);
      yoForcePointForceRateOfChange = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(namePrefix + "ForceRateOfChange", "", alphaLowPass,
            controlDT, registry, yoForcePointForce);

      yoForceAlongDirectionOfMotion = new DoubleYoVariable(namePrefix + "ForceAlongDirOfMotion", registry);
      yoForceRateOfChangeAlongDirectionOfMotion = new DoubleYoVariable(namePrefix + "DeltaForceAlongDirOfMotion", registry);
            
      yoStiffnessAlongDirectionOfMotion = new DoubleYoVariable(namePrefix + "StiffnessAlongDirOfMotion", registry);
      yoMaxStiffness = new DoubleYoVariable(namePrefix + "MaxStiffness", registry);

      yoCrossProductOfCurrentVelWithForce = new YoFrameVector(namePrefix + "VelocityCrossForce", world, registry);
      yoDirectionOfFreeMotion = new YoFrameVector(namePrefix + "DirOfFreeMotion", world, registry);

      addSimulatedSensorNoise = new BooleanYoVariable(namePrefix + "AddSimulatedNoise", registry);
      addSimulatedSensorNoise.set(false);
   }

   public double getForceAlongDirectionOfMotion()
   {
      return yoForceAlongDirectionOfMotion.getDoubleValue();
   }
   
   public FrameVector getForceRateOfChange()
   {
      return yoForcePointForceRateOfChange.getFrameTuple();
   }
   
   public double getForceRateOfChangeAlongDirectionOfMotion()
   {
      return yoForceRateOfChangeAlongDirectionOfMotion.getDoubleValue();
   }
   
   public double getStiffnessAlongDirectionOfMotion()
   {
      return yoStiffnessAlongDirectionOfMotion.getDoubleValue();
   }

   public void update(ExternalForcePoint efp)
   {
      update(efp.getYoPosition().getFrameTuple(), efp.getYoForce().getFrameTuple());
   }

   private boolean updateHasBeenCalled = false;

   private final FrameVector directionOfMotion = new FrameVector();

   public void update(FramePoint forcePointPosition, FrameVector force)
   {
      yoForcePointPosition.set(forcePointPosition);

      if (addSimulatedSensorNoise.getBooleanValue())
      {
         double amp = 0.03;
         double bias = 0.05;
         yoForcePointPosition.add(amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias);
      }
      
      yoForcePointForce.set(force);

      if (updateHasBeenCalled)
      {
         yoForcePointVelocity.update();
         yoForcePointForceRateOfChange.update();

         directionOfMotion.set(yoForcePointVelocity.getFrameTuple());

         if (directionOfMotion.length() > 0.0)
         {
            directionOfMotion.normalize();
         }

         yoForceAlongDirectionOfMotion.set(yoForcePointForce.dot(directionOfMotion));
         yoForceRateOfChangeAlongDirectionOfMotion.set(yoForcePointForceRateOfChange.dot(directionOfMotion));

         double deltaForce = Math.abs(yoForceRateOfChangeAlongDirectionOfMotion.getDoubleValue());
         double velocity = yoForcePointVelocity.length();

         if (velocity > 1e-8)
         {
            yoStiffnessAlongDirectionOfMotion.set(deltaForce / velocity);

            if (yoStiffnessAlongDirectionOfMotion.getDoubleValue() > yoMaxStiffness.getDoubleValue())
            {
               yoMaxStiffness.set(yoStiffnessAlongDirectionOfMotion.getDoubleValue());
            }
         }

         doYoVectorCrossProduct(yoForcePointForce, yoForcePointVelocity, yoCrossProductOfCurrentVelWithForce);

         doYoVectorCrossProduct(yoCrossProductOfCurrentVelWithForce, yoForcePointForce, yoDirectionOfFreeMotion);
      }
      else
      {
         updateHasBeenCalled = true;
      }
   }

   private final FrameVector temp = new FrameVector();

   private void doYoVectorCrossProduct(YoFrameVector v1, YoFrameVector v2, YoFrameVector vecToPack)
   {
      temp.cross(v1.getFrameTuple(), v2.getFrameTuple());
      if (temp.length() > 0)
      {
         temp.normalize();
      }
      vecToPack.set(world, temp.getX(), temp.getY(), temp.getZ());
   }
}
