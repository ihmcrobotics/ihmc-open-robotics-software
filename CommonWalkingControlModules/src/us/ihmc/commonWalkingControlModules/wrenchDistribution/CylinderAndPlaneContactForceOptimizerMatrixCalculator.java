package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.Collection;


import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;


public class CylinderAndPlaneContactForceOptimizerMatrixCalculator
{
   private final ReferenceFrame centerOfMassFrame;
   private final SpatialForceVector[] qRhoVectors;
   private final SpatialForceVector[] qPhiVectors;


   public CylinderAndPlaneContactForceOptimizerMatrixCalculator(ReferenceFrame centerOfMassFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      qRhoVectors = new SpatialForceVector[CylinderAndPlaneContactForceOptimizerNative.rhoSize];
      qPhiVectors = new SpatialForceVector[CylinderAndPlaneContactForceOptimizerNative.phiSize];

      for (int i = 0; i < CylinderAndPlaneContactForceOptimizerNative.rhoSize; i++)
      {
         qRhoVectors[i] = new SpatialForceVector(centerOfMassFrame);
      }

      for (int j = 0; j < CylinderAndPlaneContactForceOptimizerNative.phiSize; j++)
      {
         qPhiVectors[j] = new SpatialForceVector(centerOfMassFrame);
      }
   }

   public void computeAllMatriciesAndPopulateNativeInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput)
   {
      int rhoLocation = 0;
      int phiLocation = 0;
      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            OptimizerContactModel model = endEffector.getContactModel();
            for (int i = 0; i < model.getSizeInRho(); i++)
            {
               nativeInput.setRhoMin(rhoLocation, 0, model.getRhoMin(i));
               model.packQRhoBodyFrame(i, qRhoVectors[i], endEffector.getReferenceFrame());
               qRhoVectors[i].changeFrame(centerOfMassFrame);
               nativeInput.setQRho(rhoLocation, qRhoVectors[i]);
               rhoLocation++;
            }
   
            for (int i = 0; i < model.getSizeInPhi(); i++)
            {
               nativeInput.setPhiMin(phiLocation, 0, model.getPhiMin(i));
               nativeInput.setPhiMax(phiLocation, 0, model.getPhiMax(i));
               model.packQPhiBodyFrame(i, qPhiVectors[i], endEffector.getReferenceFrame());
               qPhiVectors[i].changeFrame(centerOfMassFrame);
               nativeInput.setQPhi(phiLocation, qPhiVectors[i]);
               phiLocation++;
            }
         }
      }
   }
}
