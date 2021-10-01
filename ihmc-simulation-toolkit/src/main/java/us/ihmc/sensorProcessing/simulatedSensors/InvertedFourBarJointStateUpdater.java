package us.ihmc.sensorProcessing.simulatedSensors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunction;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class InvertedFourBarJointStateUpdater implements Runnable
{
   private final InvertedFourBarJoint invertedFourBarJoint;
   private final OneDegreeOfFreedomJoint scsJointA, scsJointB, scsJointC, scsJointD;

   public InvertedFourBarJointStateUpdater(InvertedFourBarJoint invertedFourBarJoint, OneDegreeOfFreedomJoint scsJointA, OneDegreeOfFreedomJoint scsJointB,
                                           OneDegreeOfFreedomJoint scsJointC, OneDegreeOfFreedomJoint scsJointD)
   {
      this.invertedFourBarJoint = invertedFourBarJoint;
      this.scsJointA = scsJointA;
      this.scsJointB = scsJointB;
      this.scsJointC = scsJointC;
      this.scsJointD = scsJointD;
   }

   @Override
   public void run()
   {
      FourBarKinematicLoopFunction fourBarFunction = invertedFourBarJoint.getFourBarFunction();

      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      if (scsJointA != null && scsJointD != null)
      {
         invertedFourBarJoint.setQ(scsJointA.getQ() + scsJointD.getQ());
         invertedFourBarJoint.setQd(scsJointA.getQD() + scsJointD.getQD());
         invertedFourBarJoint.setQdd(scsJointA.getQDD() + scsJointD.getQDD());
         fourBarFunction.updateState(false, false);
         invertedFourBarJoint.setTau(loopJacobian.get(0) * scsJointA.getTau() + loopJacobian.get(3) * scsJointD.getTau());
      }
      else
      {
         invertedFourBarJoint.setQ(scsJointB.getQ() + scsJointC.getQ());
         invertedFourBarJoint.setQd(scsJointB.getQD() + scsJointC.getQD());
         invertedFourBarJoint.setQdd(scsJointB.getQDD() + scsJointC.getQDD());
         fourBarFunction.updateState(false, false);
         invertedFourBarJoint.setTau(loopJacobian.get(1) * scsJointB.getTau() + loopJacobian.get(2) * scsJointC.getTau());
      }
   }
}
