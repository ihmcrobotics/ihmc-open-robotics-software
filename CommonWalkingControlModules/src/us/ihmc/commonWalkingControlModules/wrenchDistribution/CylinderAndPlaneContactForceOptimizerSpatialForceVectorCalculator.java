package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeOutput;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator
{
   private static final int PHISIZE = CylinderAndPlaneContactForceOptimizerNative.phiSize;
   private static final int RHOSIZE = CylinderAndPlaneContactForceOptimizerNative.rhoSize;
   private final ReferenceFrame centerOfMassFrame;
   private final Map<EndEffector, SpatialForceVector> spatialForceVectors = new LinkedHashMap<EndEffector, SpatialForceVector>();
   private final DenseMatrix64F tempVectorMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F tempSum = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private final DynamicGraphicVector[][][] graphicWrenches = new DynamicGraphicVector[2][][];
   private final DoubleYoVariable[][][] graphicYoDoubles = new DoubleYoVariable[2][][];
   private final FramePoint tempPoint;
   private final FrameVector tempVector;
   private final SpatialForceVector tempSpatialVector;
   private static final int X = 0;
   private static final int Y = 1;
   private static final int Z = 2;
   private static final int xx = 3;
   private static final int yy = 4;
   private static final int zz = 5;
   private static final int x = 6;
   private static final int y = 7;
   private static final int z = 8;
   private static final int LINEAR = 0;
   private static final int ANGULAR = 1;
   private final boolean visualize;



   public CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator(String name, ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      visualize = null != dynamicGraphicObjectsListRegistry;
      YoVariableRegistry registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);
      this.tempSpatialVector = new SpatialForceVector(centerOfMassFrame);

      if (visualize)
      {
         graphicWrenches[0] = new DynamicGraphicVector[RHOSIZE][2];
         graphicYoDoubles[0] = new DoubleYoVariable[RHOSIZE][9];

         graphicWrenches[1] = new DynamicGraphicVector[PHISIZE][2];
         graphicYoDoubles[1] = new DoubleYoVariable[PHISIZE][9];
         double scaleFactor = 0.01;

         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoLinear = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoAngular = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiLinear = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiAngular = new ArrayList<DynamicGraphicObject>();

         int q = 0;
         for (int i = 0; i < RHOSIZE; i++)
         {
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable(name + "rhoOptimizerOutputVectorsElement" + q + i + j, registry);
            }

            double greenLevel = 0.25 * i / (double) RHOSIZE;
            graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector( "RhoGraphicOutput" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsRhoLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector( "RhoGraphicOutput" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsRhoAngular.add(graphicWrenches[q][i][ANGULAR]);
         }

         q = 1;

         for (int i = 0; i < PHISIZE; i++)
         {
            double greenLevel = 0.25 * i / (double) PHISIZE;
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable( "rhoOptimizerOutputVectorsElement" + q + i + j, registry);
            }

            graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector( "PhiGraphicOutput" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsPhiLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector( "PhiGraphicOutput" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsPhiAngular.add(graphicWrenches[q][i][ANGULAR]);

         }

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("OptimizerOutputVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("OptimizerOutputVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("OptimizerOutputVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects( "OptimizerOutputVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);
      }

   }

   public void computeAllWrenchesBasedOnNativeOutputAndInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput, CylinderAndPlaneContactForceOptimizerNativeOutput nativeOutput)
   {
      int iRho = 0;
      int phiLocation = 0;
      DenseMatrix64F rho = nativeOutput.getRho();
      DenseMatrix64F phi = nativeOutput.getPhi();

      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            tempSum.zero();
            OptimizerContactModel model = endEffector.getContactModel();
            for (int iRhoModel = 0; iRhoModel < model.getSizeInRho(); iRhoModel++)
            {
               nativeInput.packQrho(iRho, tempVectorMatrix);
               double rhoOfI = rho.get(iRho);
               
               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVectorMatrix.times(j, rhoOfI);
                  tempSum.add(j, 0, tempVectorMatrix.get(j));
               }

               iRho++;
            }

            for (int iPhiModel = 0; iPhiModel < model.getSizeInPhi(); iPhiModel++)
            {
               nativeInput.packQphi(phiLocation, tempVectorMatrix);
               double phiOfI = phi.get(phiLocation);

               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVectorMatrix.times(j, phiOfI);
                  tempSum.add(j, 0, tempVectorMatrix.get(j));
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

      if (visualize)
      {
         int q = 0;
         iRho = 0;
         phiLocation = 0;
         for (EndEffector endEffector : endEffectors)
         {
            if (endEffector.isLoadBearing())
            {
               ReferenceFrame frameOfInterest = endEffector.getReferenceFrame();
               OptimizerContactModel contactModel = endEffector.getContactModel();
               if (contactModel instanceof OptimizerCylinderContactModel)
               {
                  frameOfInterest = ((OptimizerCylinderContactModel) contactModel).getCylinderFrame();
               }


               tempPoint.setToZero(frameOfInterest);
               tempPoint.changeFrame(endEffector.getReferenceFrame().getRootFrame());
               q = 0;

               for (int iRhoModel = 0; iRhoModel < contactModel.getSizeInRho(); iRhoModel++)
               {
                  nativeInput.packQrho(iRho, tempVectorMatrix);
                  double rhoOfI = rho.get(iRho);
                  tempSpatialVector.set(centerOfMassFrame, tempVectorMatrix);

                  tempSpatialVector.scale(rhoOfI);
                  tempSpatialVector.changeFrame(frameOfInterest);
                  packYoDoubles(iRho, q, tempSpatialVector, tempPoint);

                  iRho++;
               }

               q = 1;

               for (int iPhiModel = 0; iPhiModel < contactModel.getSizeInPhi(); iPhiModel++)
               {
                  nativeInput.packQphi(phiLocation, tempVectorMatrix);
                  double phiOfI = phi.get(phiLocation);

                  tempSpatialVector.set(centerOfMassFrame, tempVectorMatrix);

                  tempSpatialVector.scale(phiOfI);
                  tempSpatialVector.changeFrame(frameOfInterest);
                  packYoDoubles(phiLocation, q, tempSpatialVector, tempPoint);


                  phiLocation++;
               }
            }
         }
      }
   }

   private void packYoDoubles(int i, int q, SpatialForceVector currentBasisVector, FramePoint localPoint)
   {
      graphicYoDoubles[q][i][X].set(localPoint.getX());
      graphicYoDoubles[q][i][Y].set(localPoint.getY());
      graphicYoDoubles[q][i][Z].set(localPoint.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packAngularPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][i][xx].set(tempVector.getX());
      graphicYoDoubles[q][i][yy].set(tempVector.getY());
      graphicYoDoubles[q][i][zz].set(tempVector.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packLinearPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][i][x].set(tempVector.getX());
      graphicYoDoubles[q][i][y].set(tempVector.getY());
      graphicYoDoubles[q][i][z].set(tempVector.getZ());
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
