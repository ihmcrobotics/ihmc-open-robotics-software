package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;



public class CylinderAndPlaneContactMatrixCalculator
{
   private static final boolean DEBUG = false;
   private final ReferenceFrame centerOfMassFrame;
   private final SpatialForceVector[] qRhoVectors;
   private final SpatialForceVector[] qPhiVectors;
   private final BooleanYoVariable debug;
   private final YoGraphicVector[][][] graphicWrenches = new YoGraphicVector[2][][];
   private final DoubleYoVariable[][][] graphicYoDoubles = new DoubleYoVariable[2][][];
   private final FramePoint tempPoint;
   private final FrameVector tempVector;
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
   private ArrayList<YoGraphic> dynamicGraphicVectorsRhoLinear;
   private ArrayList<YoGraphic> dynamicGraphicVectorsRhoAngular;
   private ArrayList<YoGraphic> dynamicGraphicVectorsPhiLinear;
   private ArrayList<YoGraphic> dynamicGraphicVectorsPhiAngular;
   private final boolean visualize;

   private final DenseMatrix64F rhoMin;
   private final DenseMatrix64F qRho;
   private final DenseMatrix64F phiMin;
   private final DenseMatrix64F phiMax;
   private final DenseMatrix64F qPhi;
   private final DenseMatrix64F wRho;
   private final DenseMatrix64F wPhi;


   public CylinderAndPlaneContactMatrixCalculator(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
                                                  YoGraphicsListRegistry yoGraphicsListRegistry, int rhoSize, int phiSize)
   {
      int wrenchLength = Wrench.SIZE;

      qRho = new DenseMatrix64F(wrenchLength, rhoSize);
      qPhi = new DenseMatrix64F(wrenchLength, phiSize);
      rhoMin = new DenseMatrix64F(rhoSize, 1);
      phiMin = new DenseMatrix64F(phiSize, 1);
      phiMax = new DenseMatrix64F(phiSize, 1);
      wRho = new DenseMatrix64F(rhoSize, rhoSize);
      wPhi = new DenseMatrix64F(phiSize, phiSize);

      visualize = yoGraphicsListRegistry != null;
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);

      qRhoVectors = new SpatialForceVector[rhoSize];
      qPhiVectors = new SpatialForceVector[phiSize];

      for (int i = 0; i < rhoSize; i++)
      {
         qRhoVectors[i] = new SpatialForceVector(centerOfMassFrame);
      }

      for (int i = 0; i < phiSize; i++)
      {
         qPhiVectors[i] = new SpatialForceVector(centerOfMassFrame);
      }

      if (visualize)
      {
         graphicWrenches[0] = new YoGraphicVector[rhoSize][2];
         graphicYoDoubles[0] = new DoubleYoVariable[rhoSize][9];
         graphicWrenches[1] = new YoGraphicVector[phiSize][2];
         graphicYoDoubles[1] = new DoubleYoVariable[phiSize][9];
         dynamicGraphicVectorsRhoLinear = new ArrayList<YoGraphic>();
         dynamicGraphicVectorsRhoAngular = new ArrayList<YoGraphic>();
         dynamicGraphicVectorsPhiLinear = new ArrayList<YoGraphic>();
         dynamicGraphicVectorsPhiAngular = new ArrayList<YoGraphic>();

         double scaleFactor = 0.25;


         int q = 0;
         for (int i = 0; i < rhoSize; i++)
         {
            for (int j = 0; j < 9; j++)
            {
               graphicYoDoubles[q][i][j] = new DoubleYoVariable("rhoGraphicVectorElement" + q + i + j, registry);
            }

            double greenLevel = 0.25 * i / (double) rhoSize;
            graphicWrenches[q][i][LINEAR] = new YoGraphicVector("RhoGraphicBasis" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsRhoLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new YoGraphicVector("RhoGraphicBasis" + q + i + "Angular", graphicYoDoubles[q][i][X],
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
               graphicYoDoubles[q][i][j] = new DoubleYoVariable("rhoGraphicVectorElement" + q + i + j, registry);
            }

            graphicWrenches[q][i][LINEAR] = new YoGraphicVector( "PhiGraphicBasis" + q + i + "Linear", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.0, 0.7));
            dynamicGraphicVectorsPhiLinear.add(graphicWrenches[q][i][LINEAR]);
            graphicWrenches[q][i][ANGULAR] = new YoGraphicVector("PhiGraphicBasis" + q + i + "Angular", graphicYoDoubles[q][i][X],
                    graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                    scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.6, 0.7));
            dynamicGraphicVectorsPhiAngular.add(graphicWrenches[q][i][ANGULAR]);

         }

         yoGraphicsListRegistry.registerYoGraphics("rawBasisVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
         yoGraphicsListRegistry.registerYoGraphics("rawBasisVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
         yoGraphicsListRegistry.registerYoGraphics("rawBasisVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
         yoGraphicsListRegistry.registerYoGraphics("rawBasisVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);
      }

      this.debug = new BooleanYoVariable(this.getClass().getSimpleName() + "Debug", registry);
      this.debug.set(DEBUG);

   }

   public void computeMatrices(List<? extends EndEffector> endEffectors)
   {
      int iRho = 0;
      int iPhi = 0;

      rhoMin.zero();
      qRho.zero();
      phiMin.zero();
      phiMax.zero();
      qPhi.zero();

      // Initializing these to identity instead of zero will make sure that there is a unique optimum of 0 for
      // unused rhos and phis, so that there are no NoConvergenceExceptions
      CommonOps.setIdentity(wRho);
      CommonOps.setIdentity(wPhi);

      for (int i = 0; i < endEffectors.size(); i++)
      {
         EndEffector endEffector = endEffectors.get(i);
         if (endEffector.isLoadBearing())
         {
            OptimizerContactModel model = endEffector.getContactModel();

            for (int iRhoModel = 0; iRhoModel < model.getRhoSize(); iRhoModel++)
            {
               SpatialForceVector currentBasisVector = qRhoVectors[iRho];

               setRhoMin(iRho, 0, model.getRhoMin(iRhoModel));
               model.packQRhoBodyFrame(iRhoModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(centerOfMassFrame);
               setQRho(iRho, currentBasisVector);

               wRho.set(iRho, iRho, model.getWRho());

               iRho++;
            }

            for (int iPhiModel = 0; iPhiModel < model.getPhiSize(); iPhiModel++)
            {
               SpatialForceVector currentBasisVector = qPhiVectors[iPhi];

               setPhiMin(iPhi, 0, model.getPhiMin(iPhiModel));
               setPhiMax(iPhi, 0, model.getPhiMax(iPhiModel));
               model.packQPhiBodyFrame(iPhiModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(centerOfMassFrame);
               setQPhi(iPhi, currentBasisVector);

               wPhi.set(iPhi, iPhi, model.getWPhi());

               iPhi++;
            }
         }
      }

      if (visualize)
      {
         iRho = 0;
         iPhi = 0;
         int q = 0;

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
               OptimizerContactModel model = contactModel;

               for (int iRhoModel = 0; iRhoModel < model.getRhoSize(); iRhoModel++)
               {
                  SpatialForceVector currentBasisVector = qRhoVectors[iRho];
                  currentBasisVector.changeFrame(frameOfInterest);
                  packYoDoubles(iRho, q, currentBasisVector, tempPoint);
                  iRho++;
               }

               q = 1;

               for (int iPhiModel = 0; iPhiModel < model.getPhiSize(); iPhiModel++)
               {
                  SpatialForceVector currentBasisVector = qPhiVectors[iPhi];
                  currentBasisVector.changeFrame(frameOfInterest);
                  packYoDoubles(iPhi, q, currentBasisVector, tempPoint);
                  iPhi++;
               }
            }
         }
      }


   }

   private void setRhoMin(int rhoLocation, int i, double rhoMin2)
   {
      rhoMin.set(rhoLocation, i, rhoMin2);
   }

   private void setQRho(int rhoLocation, SpatialForceVector spatialForceVector)
   {
      spatialForceVector.packMatrixColumn(qRho, rhoLocation);
   }

   private void setPhiMin(int phiLocation, int i, double phiMin)
   {
      this.phiMin.set(phiLocation, i, phiMin);
   }

   private void setPhiMax(int phiLocation, int i, double phiMax)
   {
      this.phiMax.set(phiLocation, i, phiMax);
   }

   private void setQPhi(int phiLocation, SpatialForceVector spatialForceVector)
   {
      spatialForceVector.packMatrixColumn(qPhi, phiLocation);
   }

   private void packYoDoubles(int iPhi, int q, SpatialForceVector currentBasisVector, FramePoint localPoint)
   {
      graphicYoDoubles[q][iPhi][X].set(localPoint.getX());
      graphicYoDoubles[q][iPhi][Y].set(localPoint.getY());
      graphicYoDoubles[q][iPhi][Z].set(localPoint.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packAngularPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][iPhi][xx].set(tempVector.getX());
      graphicYoDoubles[q][iPhi][yy].set(tempVector.getY());
      graphicYoDoubles[q][iPhi][zz].set(tempVector.getZ());
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame());
      currentBasisVector.packLinearPart(tempVector);
      tempVector.changeFrame(currentBasisVector.getExpressedInFrame().getRootFrame());
      graphicYoDoubles[q][iPhi][x].set(tempVector.getX());
      graphicYoDoubles[q][iPhi][y].set(tempVector.getY());
      graphicYoDoubles[q][iPhi][z].set(tempVector.getZ());
   }

   public DenseMatrix64F getRhoMin()
   {
      return rhoMin;
   }

   public DenseMatrix64F getQRho()
   {
      return qRho;
   }

   public DenseMatrix64F getPhiMin()
   {
      return phiMin;
   }

   public DenseMatrix64F getPhiMax()
   {
      return phiMax;
   }

   public DenseMatrix64F getQPhi()
   {
      return qPhi;
   }

   public void printIfDebug(String message)
   {
      if (this.debug.getBooleanValue())
      {
         System.out.println(this.getClass().getSimpleName() + ": " + message);
      }
   }

   public DenseMatrix64F getWRho()
   {
      return wRho;
   }

   public DenseMatrix64F getWPhi()
   {
      return wPhi;
   }
}
