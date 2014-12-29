package us.ihmc.humanoidBehaviors.utilities;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import Jama.Matrix;

public class TaskSpaceStiffnessCalculator
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final double lowPassCutoffFreq_Hz = 5.0;
   private final DoubleYoVariable alphaLowPass;

   private final YoFramePoint yoForcePointPosition;
   private final YoFrameVector yoForcePointForce;

   private final FilteredVelocityYoFrameVector yoForcePointVelocity;
   private final FilteredVelocityYoFrameVector yoForcePointForceRateOfChange;

   private final DoubleYoVariable yoForceChangeAlongDirectionOfMotion;
   private final DoubleYoVariable yoStiffnessAlongDirectionOfMotion;
   private final DoubleYoVariable yoMaxStiffness;

   private final YoFrameVector yoCrossProductOfCurrentVelWithForce;
   private final YoFrameVector yoDirectionOfFreeMotion;

   //   private final DoubleYoVariable yoVelocityAlongDirectionOfForceChange;

   //   private final YoFrameVector yoForceMagnitudeGradient;
   //   private final YoFrameVector yoCrossProductOfCurrentVelWithForceMagGradient;
   //   private final YoFrameVector yoDirectionNormalToForceMagGradientCoplanarWithCurrentVelocity;

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

      yoForceChangeAlongDirectionOfMotion = new DoubleYoVariable(namePrefix + "DeltaForceAlongDirOfMotion", registry);
      yoStiffnessAlongDirectionOfMotion = new DoubleYoVariable(namePrefix + "StiffnessAlongDirOfMotion", registry);
      yoMaxStiffness = new DoubleYoVariable(namePrefix + "MaxStiffness", registry);

      yoCrossProductOfCurrentVelWithForce = new YoFrameVector(namePrefix + "VelocityCrossForce", world, registry);
      yoDirectionOfFreeMotion = new YoFrameVector(namePrefix + "DirOfFreeMotion", world, registry);

      //      yoVelocityAlongDirectionOfForceChange = new DoubleYoVariable(namePrefix + "VelocityAlongDirOfForceChange", registry);

      //      yoForceMagnitudeGradient = new YoFrameVector(namePrefix + "ForceMagGradient", world, registry);
      //      yoCrossProductOfCurrentVelWithForceMagGradient = new YoFrameVector(namePrefix + "VelocityCrossForceMagGradient", world, registry);
      //      yoDirectionNormalToForceMagGradientCoplanarWithCurrentVelocity = new YoFrameVector(namePrefix + "DirOfFreeMotion", world, registry);
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

         yoForceChangeAlongDirectionOfMotion.set(yoForcePointForceRateOfChange.dot(directionOfMotion));

         double deltaForce = Math.abs(yoForceChangeAlongDirectionOfMotion.getDoubleValue());
         double velocity = yoForcePointVelocity.length();

         if (velocity > 1e-8)
         {
            yoStiffnessAlongDirectionOfMotion.set(deltaForce / velocity);

            if (yoStiffnessAlongDirectionOfMotion.getDoubleValue() > yoMaxStiffness.getDoubleValue())
            {
               yoMaxStiffness.set(yoStiffnessAlongDirectionOfMotion.getDoubleValue());
            }
         }

         
         
         temp.cross(yoForcePointVelocity.getFrameTuple(), yoForcePointForce.getFrameTuple());
         if (temp.length() > 0)
         {
            temp.normalize();
         }
         yoCrossProductOfCurrentVelWithForce.set(world, temp.getX(), temp.getY(), temp.getZ());

         
         
         temp.cross(yoCrossProductOfCurrentVelWithForce.getFrameTuple(), yoForcePointForce.getFrameTuple());
         if (temp.length() > 0)
         {
            temp.normalize();
         }
         yoDirectionOfFreeMotion.set(world, temp.getX(), temp.getY(), temp.getZ());

         //         updateForceMagnitudeGradient();

         //         updateForceGradient();

         //         computeDirectionOfFreeMotion();

      }
      else
      {
         updateHasBeenCalled = true;
      }
   }

   private final FrameVector temp = new FrameVector();
   //
   //   private void computeDirectionOfFreeMotion()
   //   {
   //      temp.cross(yoForcePointVelocity.getFrameTuple(), yoForcePointForceRateOfChange.getFrameTuple());
   //      if (temp.length() > 0)
   //      {
   //         temp.normalize();
   //      }
   //      yoCrossProductOfCurrentVelWithForceMagGradient.set(world, temp.getX(), temp.getY(), temp.getZ());
   //
   //      temp.cross(yoCrossProductOfCurrentVelWithForceMagGradient.getFrameTuple(), yoForcePointForceRateOfChange.getFrameTuple());
   //      if (temp.length() > 0)
   //      {
   //         temp.normalize();
   //      }
   //      yoDirectionNormalToForceMagGradientCoplanarWithCurrentVelocity.set(world, temp.getX(), temp.getY(), temp.getZ());
   //   }

   //   private final FrameVector temp = new FrameVector();
   //
   //   private void computeDirectionOfFreeMotion()
   //   {
   //      temp.cross(yoForcePointVelocity.getFrameTuple(), yoForceMagnitudeGradient.getFrameTuple());
   //      if (temp.length() > 0)
   //      {
   //         temp.normalize();
   //      }
   //      yoCrossProductOfCurrentVelWithForceMagGradient.set(world, temp.getX(), temp.getY(), temp.getZ());
   //
   //      temp.cross(yoCrossProductOfCurrentVelWithForceMagGradient.getFrameTuple(), yoForceMagnitudeGradient.getFrameTuple());
   //      if (temp.length() > 0)
   //      {
   //         temp.normalize();
   //      }
   //      yoDirectionNormalToForceMagGradientCoplanarWithCurrentVelocity.set(world, temp.getX(), temp.getY(), temp.getZ());
   //   }

   //   private final FrameVector directionOfForceChange = new FrameVector();
   //
   //   private void updateForceMagnitudeGradient()
   //   {
   //      directionOfForceChange.set(yoForcePointForceRateOfChange.getFrameTuple());
   //      if (directionOfForceChange.length() > 0.0)
   //      {
   //         directionOfForceChange.normalize();
   //      }
   //
   //      yoVelocityAlongDirectionOfForceChange.set(yoForcePointVelocity.dot(directionOfForceChange));
   //      directionOfForceChange.scale(yoVelocityAlongDirectionOfForceChange.getDoubleValue());
   //
   //      double forceMagRateOfChange = yoForcePointForceRateOfChange.length();
   //      yoForceMagnitudeGradient.set(world, forceMagRateOfChange / directionOfForceChange.getX(), forceMagRateOfChange / directionOfForceChange.getY(),
   //            forceMagRateOfChange / directionOfForceChange.getZ());
   //   }

   //   private final Matrix forceGradient = new Matrix(3, 3);
   //
   //   private void updateForceGradient()
   //   {
   //      forceGradient.set(0, 0, yoForcePointForceRateOfChange.getX() / yoForcePointVelocity.getX());
   //      forceGradient.set(0, 1, yoForcePointForceRateOfChange.getX() / yoForcePointVelocity.getY());
   //      forceGradient.set(0, 2, yoForcePointForceRateOfChange.getX() / yoForcePointVelocity.getZ());
   //
   //      forceGradient.set(1, 0, yoForcePointForceRateOfChange.getY() / yoForcePointVelocity.getX());
   //      forceGradient.set(1, 1, yoForcePointForceRateOfChange.getY() / yoForcePointVelocity.getY());
   //      forceGradient.set(1, 2, yoForcePointForceRateOfChange.getY() / yoForcePointVelocity.getZ());
   //
   //      forceGradient.set(2, 0, yoForcePointForceRateOfChange.getZ() / yoForcePointVelocity.getX());
   //      forceGradient.set(2, 1, yoForcePointForceRateOfChange.getZ() / yoForcePointVelocity.getY());
   //      forceGradient.set(2, 2, yoForcePointForceRateOfChange.getZ() / yoForcePointVelocity.getZ());
   //   }
}
