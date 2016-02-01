package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class CylinderAndPlaneContactSpatialForceVectorCalculator
{
   private final ReferenceFrame centerOfMassFrame;
   private final Map<EndEffector, SpatialForceVector> spatialForceVectors = new LinkedHashMap<EndEffector, SpatialForceVector>();

   private final DenseMatrix64F tempVectorMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F tempSum = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private List<? extends EndEffector> endEffectors;

   private final YoVariableRegistry registry;
   private final YoGraphicVector[][][] graphicWrenches = new YoGraphicVector[2][][];
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

   private final DoubleYoVariable rhoTotal;
   private final DenseMatrix64F rhoMean;
   private final DenseMatrix64F rhoMeanLoadedEndEffectors;

   public CylinderAndPlaneContactSpatialForceVectorCalculator(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           YoGraphicsListRegistry yoGraphicsListRegistry, int rhoSize, int phiSize)
   {
      int wrenchLength = Wrench.SIZE;

      qRho = new DenseMatrix64F(wrenchLength, rhoSize);
      qPhi = new DenseMatrix64F(wrenchLength, phiSize);
      rhoMean = new DenseMatrix64F(rhoSize, 1);
      rhoMeanLoadedEndEffectors = new DenseMatrix64F(4, 1);

      visualize = yoGraphicsListRegistry != null;
      String name = getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);
      this.tempSpatialVector = new SpatialForceVector(centerOfMassFrame);
      
      rhoTotal = new DoubleYoVariable(name + "RhoTotal", registry);

      if (visualize)
      {
         graphicWrenches[0] = new YoGraphicVector[rhoSize][2];
         graphicYoDoubles[0] = new DoubleYoVariable[rhoSize][9];

         graphicWrenches[1] = new YoGraphicVector[phiSize][2];
         graphicYoDoubles[1] = new DoubleYoVariable[phiSize][9];
         double scaleFactor = 0.01;

         ArrayList<YoGraphic> dynamicGraphicVectorsRhoLinear = new ArrayList<YoGraphic>();
         ArrayList<YoGraphic> dynamicGraphicVectorsRhoAngular = new ArrayList<YoGraphic>();
         ArrayList<YoGraphic> dynamicGraphicVectorsPhiLinear = new ArrayList<YoGraphic>();
         ArrayList<YoGraphic> dynamicGraphicVectorsPhiAngular = new ArrayList<YoGraphic>();

         int q = 0;
         for (int i = 0; i < rhoSize; i++)
         {
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable(name + "rhoOptimizerOutputVectorsElement" + q + i + j, registry);
            }

            double greenLevel = 0.25 * i / rhoSize;
            graphicWrenches[q][i][LINEAR] = new YoGraphicVector("RhoGraphicOutput" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsRhoLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new YoGraphicVector("RhoGraphicOutput" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsRhoAngular.add(graphicWrenches[q][i][ANGULAR]);
         }

         q = 1;

         for (int i = 0; i < phiSize; i++)
         {
            double greenLevel = 0.25 * i / phiSize;
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable("rhoOptimizerOutputVectorsElement" + q + i + j, registry);
            }

            graphicWrenches[q][i][LINEAR] = new YoGraphicVector("PhiGraphicOutput" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsPhiLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new YoGraphicVector("PhiGraphicOutput" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsPhiAngular.add(graphicWrenches[q][i][ANGULAR]);

         }

         yoGraphicsListRegistry.registerYoGraphics("OptimizerOutputVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
         yoGraphicsListRegistry.registerYoGraphics("OptimizerOutputVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
         yoGraphicsListRegistry.registerYoGraphics("OptimizerOutputVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
         yoGraphicsListRegistry.registerYoGraphics("OptimizerOutputVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);
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
      this.endEffectors = endEffectors;

      int iRho = 0;
      int phiLocation = 0;

      rhoMean.zero();
      rhoTotal.set(0.0);

      int loadedEndEffectors = 0;
      for (int i = 0; i < endEffectors.size(); i++)
      {
         if (endEffectors.get(i).isLoadBearing())
         {
            loadedEndEffectors++;
         }
      }

      rhoMeanLoadedEndEffectors.reshape(loadedEndEffectors, 1);
      rhoMeanLoadedEndEffectors.zero();

      int loadedEndEffector = 0;
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


            DenseMatrix64F rhosSingleEndEffector = CommonOps.extract(rho, (iRho - model.getRhoSize()), iRho, 0, 1);
            double rhosSum = CommonOps.elementSum(rhosSingleEndEffector);
            double rhosAverage = rhosSum / model.getRhoSize();
            rhoTotal.add(rhosAverage);
            rhoMeanLoadedEndEffectors.set(loadedEndEffector, rhosAverage);

            for (int index = (iRho - model.getRhoSize()); index < iRho; index++)
            {
               rhoMean.set(index, rhosAverage);
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

            loadedEndEffector++;
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

   public void packWRhoPenalizer(DenseMatrix64F wRhoPenalizerToPack, double wRhoMaxPenalty)
   {
      CommonOps.setIdentity(wRhoPenalizerToPack);
      CommonOps.scale(wRhoMaxPenalty, wRhoPenalizerToPack);

      if (endEffectors == null)
      {
         wRhoPenalizerToPack.zero();
         return;
      }

      int iRho = 0;
      for (int i = 0; i < endEffectors.size(); i++)
      {
         EndEffector endEffector = endEffectors.get(i);
         if (endEffector.isLoadBearing())
         {
            double penaltyScaling = 0.0;
            double meanRhoLoaded = rhoMeanLoadedEndEffectors.get(i);

            if (rhoTotal.getDoubleValue() > 1e-3)
               penaltyScaling = Math.max(0.0, 1.0 - meanRhoLoaded / rhoTotal.getDoubleValue());

            // For visualization only
            endEffector.setWRhoPenalizer(wRhoMaxPenalty * penaltyScaling);

            int rhoSize = endEffector.getContactModel().getRhoSize();
            for (int index = 0; index < rhoSize; index++)
            {
               wRhoPenalizerToPack.set(iRho, iRho, wRhoMaxPenalty * penaltyScaling);
               iRho++;
            }
         }
         else
         {
            // For visualization only
            endEffector.setWRhoPenalizer(0.0);
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

   public void packRhoMean(DenseMatrix64F rhoMeanToPack)
   {
      rhoMeanToPack.set(rhoMean);
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
