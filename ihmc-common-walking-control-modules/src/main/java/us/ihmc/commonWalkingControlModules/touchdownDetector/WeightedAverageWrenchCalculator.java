package us.ihmc.commonWalkingControlModules.touchdownDetector;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.filters.WeightedAverageYoFrameVector3D;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class WeightedAverageWrenchCalculator implements WrenchCalculator
{
   private final WeightedAverageYoFrameVector3D averageLinearForce;
   private final WeightedAverageYoFrameVector3D averageAngularForce;

   private final HashMap<WrenchCalculator, YoFrameVector3D> angularForces = new HashMap<>();
   private final HashMap<WrenchCalculator, YoFrameVector3D> linearForces = new HashMap<>();

   private final List<WrenchCalculator> wrenchCalculators = new ArrayList<>();

   private final Wrench wrench = new Wrench();

   public WeightedAverageWrenchCalculator(String prefix, YoVariableRegistry registry, PairList<DoubleProvider, WrenchCalculator> wrenchCalculatorPairs)
   {
      wrench.setToZero(ReferenceFrame.getWorldFrame());

      averageAngularForce = new WeightedAverageYoFrameVector3D(prefix + "_WeightedAverageAngularForce", ReferenceFrame.getWorldFrame(), registry);
      averageLinearForce = new WeightedAverageYoFrameVector3D(prefix + "_WeightedAverageLinearForce", ReferenceFrame.getWorldFrame(), registry);

      for (ImmutablePair<DoubleProvider, WrenchCalculator> wrenchCalculatorPair : wrenchCalculatorPairs)
      {
         WrenchCalculator wrenchCalculator = wrenchCalculatorPair.getRight();
         YoFrameVector3D linearForce = new YoFrameVector3D(wrenchCalculator.getName() + "LinearForce", ReferenceFrame.getWorldFrame(), registry);
         YoFrameVector3D angularForce = new YoFrameVector3D(wrenchCalculator.getName() + "AngularForce", ReferenceFrame.getWorldFrame(), registry);

         linearForces.put(wrenchCalculator, linearForce);
         angularForces.put(wrenchCalculator, angularForce);

         averageLinearForce.addFrameVector3DToAverage(wrenchCalculatorPair.getLeft(), linearForce);
         averageAngularForce.addFrameVector3DToAverage(wrenchCalculatorPair.getLeft(), angularForce);

         wrenchCalculators.add(wrenchCalculator);
      }
   }

   @Override
   public void calculate()
   {
      for (int i = 0; i < wrenchCalculators.size(); i++)
      {
         WrenchCalculator wrenchCalculator = wrenchCalculators.get(i);
         wrenchCalculator.calculate();

         WrenchReadOnly wrench = wrenchCalculator.getWrench();
         linearForces.get(wrenchCalculator).set(wrench.getLinearPart());
         angularForces.get(wrenchCalculator).set(wrench.getAngularPart());
      }

      averageAngularForce.update();
      averageLinearForce.update();

      wrench.setToZero(ReferenceFrame.getWorldFrame());
      wrench.set(averageAngularForce, averageLinearForce);
   }

   @Override
   public WrenchReadOnly getWrench()
   {
      return wrench;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }
}
