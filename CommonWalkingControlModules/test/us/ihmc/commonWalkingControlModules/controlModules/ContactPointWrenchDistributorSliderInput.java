package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointGroundReactionWrenchDistributor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class ContactPointWrenchDistributorSliderInput implements VariableChangedListener
{
   private static final boolean DEBUG = false;
   private final ContactPointGroundReactionWrenchDistributor distributor;
   private final YoVariableRegistry registry = new YoVariableRegistry("ContactPointWrenchDistributorSliderInput");

   private final DoubleYoVariable forceXYWeight = new DoubleYoVariable("forceXYWeight", registry);
   private final DoubleYoVariable forceZWeight = new DoubleYoVariable("forceZWeight", registry);

   private final DoubleYoVariable torqueXYWeight = new DoubleYoVariable("torqueXYWeight", registry);
   private final DoubleYoVariable torqueZWeight = new DoubleYoVariable("torqueZWeight", registry);

   private final DoubleYoVariable epsilonRho = new DoubleYoVariable("epsilonRho", registry);

   private final DoubleYoVariable minimumNormalForce = new DoubleYoVariable("minimumNormalForce", registry);

   public ContactPointWrenchDistributorSliderInput(SimulationConstructionSet scs, YoVariableRegistry parentRegistry, ReferenceFrame centerOfMassFrame)
   {
      distributor = new ContactPointGroundReactionWrenchDistributor(centerOfMassFrame, parentRegistry);

      setTwansTunedWeights();

      updateWeights();

      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs);
      sliderBoard.setSlider(0, "forceXYWeight", registry, 0.0, 100.0);
      sliderBoard.setSlider(1, "forceZWeight", registry, 0.0, 100.0);
      sliderBoard.setSlider(2, "torqueXYWeight", registry, 0.0, 100.0);
      sliderBoard.setSlider(3, "torqueZWeight", registry, 0.0, 100.0);
      sliderBoard.setSlider(4, "epsilonRho", registry, 0.0, 10.0);
      sliderBoard.setSlider(5, "minimumNormalForce", registry, 0.0, 100.0);

      sliderBoard.attachVariableChangedListener(this);
      parentRegistry.addChild(registry);
   }

   public void setTwansTunedWeights()
   {
      forceXYWeight.set(1.0);
      forceZWeight.set(1.0);
      torqueXYWeight.set(1.0);
      torqueZWeight.set(1.0);

      epsilonRho.set(0.001);
      minimumNormalForce.set(0.0);
   }

   public void setWeightsForEncouragingLunging()
   {
      forceXYWeight.set(100.0);
      forceZWeight.set(100.0);
      torqueXYWeight.set(20.0);
      torqueZWeight.set(20.0);

      epsilonRho.set(0.2);
      minimumNormalForce.set(10.0);
   }

   public void setWeightsForDiscouragingLunging()
   {
      forceXYWeight.set(40.0);
      forceZWeight.set(100.0);
      torqueXYWeight.set(100.0);
      torqueZWeight.set(40.0);

      epsilonRho.set(0.2);
      minimumNormalForce.set(10.0);
   }

   public ContactPointGroundReactionWrenchDistributor getDistributor()
   {
      return distributor;
   }

   public void updateWeights()
   {
      printIfDebug("Updating Weights");
      printIfDebug("torqueXYWeight = " + torqueXYWeight);
      printIfDebug("torqueZWeight = " + torqueZWeight);
      printIfDebug("forceXYWeight = " + forceXYWeight);
      printIfDebug("forceZWeight = " + forceZWeight);

      double[] diagonalCWeights = new double[] { torqueXYWeight.getDoubleValue(), torqueXYWeight.getDoubleValue(), torqueZWeight.getDoubleValue(),
            forceXYWeight.getDoubleValue(), forceXYWeight.getDoubleValue(), forceZWeight.getDoubleValue() };
      distributor.setWeights(diagonalCWeights, 1.0, epsilonRho.getDoubleValue());
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   public void variableChanged(YoVariable<?> variable)
   {
      updateWeights();
   }
}