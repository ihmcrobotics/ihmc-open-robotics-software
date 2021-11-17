package us.ihmc.sensorProcessing.simulatedSensors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class CrossFourBarJointStateUpdater implements Runnable
{
   private final CrossFourBarJoint crossFourBarJoint;
   private final OneDegreeOfFreedomJoint scsJointA, scsJointB, scsJointC, scsJointD;

   public CrossFourBarJointStateUpdater(CrossFourBarJoint crossFourBarJoint,
                                        OneDegreeOfFreedomJoint scsJointA,
                                        OneDegreeOfFreedomJoint scsJointB,
                                        OneDegreeOfFreedomJoint scsJointC,
                                        OneDegreeOfFreedomJoint scsJointD)
   {
      this.crossFourBarJoint = crossFourBarJoint;
      this.scsJointA = scsJointA;
      this.scsJointB = scsJointB;
      this.scsJointC = scsJointC;
      this.scsJointD = scsJointD;
   }

   @Override
   public void run()
   {
      FourBarKinematicLoopFunction fourBarFunction = crossFourBarJoint.getFourBarFunction();

      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      if (scsJointA != null && scsJointD != null)
      {
         crossFourBarJoint.setQ(scsJointA.getQ() + scsJointD.getQ());
         crossFourBarJoint.setQd(scsJointA.getQD() + scsJointD.getQD());
         crossFourBarJoint.setQdd(scsJointA.getQDD() + scsJointD.getQDD());
         fourBarFunction.updateState(false, false);
         crossFourBarJoint.setTau(loopJacobian.get(0) * scsJointA.getTau() + loopJacobian.get(3) * scsJointD.getTau());
      }
      else
      {
         crossFourBarJoint.setQ(scsJointB.getQ() + scsJointC.getQ());
         crossFourBarJoint.setQd(scsJointB.getQD() + scsJointC.getQD());
         crossFourBarJoint.setQdd(scsJointB.getQDD() + scsJointC.getQDD());
         fourBarFunction.updateState(false, false);
         crossFourBarJoint.setTau(loopJacobian.get(1) * scsJointB.getTau() + loopJacobian.get(2) * scsJointC.getTau());
      }
   }
}
