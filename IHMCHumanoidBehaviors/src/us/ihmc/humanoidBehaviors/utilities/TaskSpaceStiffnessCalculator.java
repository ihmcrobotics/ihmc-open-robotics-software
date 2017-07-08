package us.ihmc.humanoidBehaviors.utilities;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class TaskSpaceStiffnessCalculator
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final double lowPassCutoffFreq_Hz = 5.0;
   private final YoDouble alphaLowPass;

   private final YoFramePoint yoForcePointPosition;
   private final YoFrameVector yoForcePointForce;

   private final FilteredVelocityYoFrameVector yoForcePointVelocity;
   private final FilteredVelocityYoFrameVector yoForcePointForceRateOfChange;

   private final YoDouble yoForceAlongDirectionOfMotion;
   private final YoDouble yoForceRateOfChangeAlongDirectionOfMotion;

   private final YoDouble yoStiffnessAlongDirectionOfMotion;
   private final YoDouble yoMaxStiffness;

   private final YoFrameVector yoCrossProductOfCurrentVelWithForce;
   private final YoFrameVector yoDirectionOfFreeMotion;

   private final YoBoolean addSimulatedSensorNoise;


   public TaskSpaceStiffnessCalculator(String namePrefix, double controlDT, YoVariableRegistry registry)
   {
      alphaLowPass = new YoDouble(namePrefix + "Alpha", registry);
      alphaLowPass.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(lowPassCutoffFreq_Hz, controlDT));

      yoForcePointPosition = new YoFramePoint(namePrefix + "Position", world, registry);
      yoForcePointForce = new YoFrameVector(namePrefix + "Force", world, registry);

      yoForcePointVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(namePrefix + "Velocity", "", alphaLowPass, controlDT, registry,
            yoForcePointPosition);
      yoForcePointForceRateOfChange = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(namePrefix + "ForceRateOfChange", "", alphaLowPass,
            controlDT, registry, yoForcePointForce);

      yoForceAlongDirectionOfMotion = new YoDouble(namePrefix + "ForceAlongDirOfMotion", registry);
      yoForceRateOfChangeAlongDirectionOfMotion = new YoDouble(namePrefix + "DeltaForceAlongDirOfMotion", registry);
            
      yoStiffnessAlongDirectionOfMotion = new YoDouble(namePrefix + "StiffnessAlongDirOfMotion", registry);
      yoMaxStiffness = new YoDouble(namePrefix + "MaxStiffness", registry);

      yoCrossProductOfCurrentVelWithForce = new YoFrameVector(namePrefix + "VelocityCrossForce", world, registry);
      yoDirectionOfFreeMotion = new YoFrameVector(namePrefix + "DirOfFreeMotion", world, registry);

      addSimulatedSensorNoise = new YoBoolean(namePrefix + "AddSimulatedNoise", registry);
      addSimulatedSensorNoise.set(false);
   }

   public double getForceAlongDirectionOfMotion()
   {
      return yoForceAlongDirectionOfMotion.getDoubleValue();
   }
   
   public FrameVector3D getForceRateOfChange()
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

   private final FrameVector3D directionOfMotion = new FrameVector3D();

   public void update(FramePoint3D forcePointPosition, FrameVector3D force)
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

   private final FrameVector3D temp = new FrameVector3D();

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
