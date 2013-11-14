package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

public class CylinderAndPlaneContactSpatialForceVectorCalculator
{
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

   private final DenseMatrix64F qRho;
   private final DenseMatrix64F qPhi;


   public CylinderAndPlaneContactSpatialForceVectorCalculator(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
                                                              DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, int rhoSize, int phiSize)
   {
      int wrenchLength = Wrench.SIZE;

      qRho = new DenseMatrix64F(wrenchLength, rhoSize);
      qPhi = new DenseMatrix64F(wrenchLength, phiSize);

      visualize = dynamicGraphicObjectsListRegistry != null;
      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);
      this.tempSpatialVector = new SpatialForceVector(centerOfMassFrame);

      if (visualize)
      {
         graphicWrenches[0] = new DynamicGraphicVector[rhoSize][2];
         graphicYoDoubles[0] = new DoubleYoVariable[rhoSize][9];

         graphicWrenches[1] = new DynamicGraphicVector[phiSize][2];
         graphicYoDoubles[1] = new DoubleYoVariable[phiSize][9];
         double scaleFactor = 0.01;

         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoLinear = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoAngular = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiLinear = new ArrayList<DynamicGraphicObject>();
         ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiAngular = new ArrayList<DynamicGraphicObject>();

         int q = 0;
         for (int i = 0; i < rhoSize; i++)
         {
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable(name + "rhoOptimizerOutputVectorsElement" + q + i + j, registry);
            }

            double greenLevel = 0.25 * i / (double) rhoSize;
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

         for (int i = 0; i < phiSize; i++)
         {
            double greenLevel = 0.25 * i / (double) phiSize;
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

   public void setQRho(DenseMatrix64F qRho)
   {
      this.qRho.set(qRho);
   }

   public void setQPhi(DenseMatrix64F qPhi)
   {
      this.qPhi.set(qPhi);
   }

   public void computeWrenches(List<? extends EndEffector> endEffectors, DenseMatrix64F rho, DenseMatrix64F phi)
   {
      int iRho = 0;
      int phiLocation = 0;

      for (int i = 0; i < endEffectors.size(); i++)
      {
         EndEffector endEffector = endEffectors.get(i);
         if (endEffector.isLoadBearing())
         {
            tempSum.zero();
            OptimizerContactModel model = endEffector.getContactModel();
            for (int iRhoModel = 0; iRhoModel < model.getRhoSize(); iRhoModel++)
            {
               packQrho(iRho, tempVectorMatrix);
               double rhoOfI = rho.get(iRho);
               
               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVectorMatrix.times(j, rhoOfI);
                  tempSum.add(j, 0, tempVectorMatrix.get(j));
               }

               iRho++;
            }

            for (int iPhiModel = 0; iPhiModel < model.getPhiSize(); iPhiModel++)
            {
               packQphi(phiLocation, tempVectorMatrix);
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
         for (int i = 0; i < endEffectors.size(); i++)
         {
            EndEffector endEffector = endEffectors.get(i);
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

               for (int iRhoModel = 0; iRhoModel < contactModel.getRhoSize(); iRhoModel++)
               {
                  packQrho(iRho, tempVectorMatrix);
                  double rhoOfI = rho.get(iRho);
                  tempSpatialVector.set(centerOfMassFrame, tempVectorMatrix);

                  tempSpatialVector.scale(rhoOfI);
                  tempSpatialVector.changeFrame(frameOfInterest);
                  packYoDoubles(iRho, q, tempSpatialVector, tempPoint);

                  iRho++;
               }

               q = 1;

               for (int iPhiModel = 0; iPhiModel < contactModel.getPhiSize(); iPhiModel++)
               {
                  packQphi(phiLocation, tempVectorMatrix);
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

   public void packQphi(int i, DenseMatrix64F tempVector)
   {
      for (int j = 0; j < SpatialForceVector.SIZE; j++)
      {
         tempVector.set(j, qPhi.get(j, i));
      }
   }

   public void packQrho(int i, DenseMatrix64F tempVector)
   {
      for (int j = 0; j < SpatialForceVector.SIZE; j++)
      {
         tempVector.set(j, qRho.get(j, i));
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
