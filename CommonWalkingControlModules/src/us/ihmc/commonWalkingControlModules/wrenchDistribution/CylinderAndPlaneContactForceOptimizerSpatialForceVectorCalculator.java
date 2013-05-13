package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeOutput;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator
{
   private final ReferenceFrame centerOfMassFrame;
   private final Map<EndEffector, SpatialForceVector> spatialForceVectors = new LinkedHashMap<EndEffector, SpatialForceVector>();
   private final DenseMatrix64F tempVector = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F tempSum = new DenseMatrix64F(SpatialForceVector.SIZE, 1);


   public CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator(ReferenceFrame centerOfMassFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;
   }

   public void computeAllWrenchesBasedOnNativeOutputAndInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput, CylinderAndPlaneContactForceOptimizerNativeOutput nativeOutput)
   {
      int rhoLocation = 0;
      int phiLocation = 0;
      DenseMatrix64F rho = nativeOutput.getRho();
      DenseMatrix64F phi = nativeOutput.getPhi();

      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            tempSum.zero();
            OptimizerContactModel model = endEffector.getContactModel();
            for (int i = 0; i < model.getSizeInRho(); i++)
            {
               nativeInput.packQrho(rhoLocation, tempVector);
               double rhoOfI = rho.get(rhoLocation);

               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVector.times(j, rhoOfI);
                  tempSum.add(j, 0, tempVector.get(j));
               }

               rhoLocation++;
            }

            for (int i = 0; i < model.getSizeInPhi(); i++)
            {
               nativeInput.packQphi(phiLocation, tempVector);
               double phiOfI = phi.get(phiLocation);

               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVector.times(j, phiOfI);
                  tempSum.add(j, 0, tempVector.get(j));
               }

               phiLocation++;
            }

            SpatialForceVector spatialForceVector = getOrCreateSpatialForceVector(endEffector);
            spatialForceVector.set(centerOfMassFrame, tempSum);
         }
         else
         {
            tempSum.zero();
            SpatialForceVector spatialForceVector = getOrCreateSpatialForceVector(endEffector);
            spatialForceVector.set(centerOfMassFrame, tempSum);
         }
      }
   }

   public SpatialForceVector getSpatialForceVector(EndEffector endEffector)
   {
      return spatialForceVectors.get(endEffector);
   }

   private SpatialForceVector getOrCreateSpatialForceVector(EndEffector endEffector)
   {
      SpatialForceVector spatialForceVector = spatialForceVectors.get(endEffector);
      if (spatialForceVector == null)
      {
         spatialForceVector = new SpatialForceVector(centerOfMassFrame);
         spatialForceVectors.put(endEffector, spatialForceVector);
      }

      return spatialForceVector;
   }

   public Map<EndEffector, SpatialForceVector> getWrenches()
   {
      return spatialForceVectors;
   }
}
