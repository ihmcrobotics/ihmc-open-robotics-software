package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;


public class CylinderAndPlaneContactForceOptimizerMatrixCalculator
{
   private static final boolean DEBUG = false;
   private static final int PHISIZE = CylinderAndPlaneContactForceOptimizerNative.phiSize;
   private static final int RHOSIZE = CylinderAndPlaneContactForceOptimizerNative.rhoSize;
   private final ReferenceFrame centerOfMassFrame;
   private final SpatialForceVector[] qRhoVectors;
   private final SpatialForceVector[] qPhiVectors;
   private final BooleanYoVariable debug;
   private final DynamicGraphicVector[][][] graphicWrenches = new DynamicGraphicVector[2][][];
   private final DoubleYoVariable[][][] graphicYoDoubles = new DoubleYoVariable[2][][];
   private final FramePoint tempPoint;
   private final FrameVector tempVector ;
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



   public CylinderAndPlaneContactForceOptimizerMatrixCalculator(String name, ReferenceFrame centerOfMassFrame, YoVariableRegistry registry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);

      qRhoVectors = new SpatialForceVector[RHOSIZE];
      graphicWrenches[0] = new DynamicGraphicVector[RHOSIZE][2];
      graphicYoDoubles[0] = new DoubleYoVariable[RHOSIZE][9];

      qPhiVectors = new SpatialForceVector[PHISIZE];
      graphicWrenches[1] = new DynamicGraphicVector[PHISIZE][2];
      graphicYoDoubles[1] = new DoubleYoVariable[PHISIZE][9];

      double scaleFactor = 0.25;

      ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoLinear = new ArrayList<DynamicGraphicObject>();
      ArrayList<DynamicGraphicObject> dynamicGraphicVectorsRhoAngular = new ArrayList<DynamicGraphicObject>();
      ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiLinear = new ArrayList<DynamicGraphicObject>();
      ArrayList<DynamicGraphicObject> dynamicGraphicVectorsPhiAngular = new ArrayList<DynamicGraphicObject>();

      int q = 0;
      for (int i = 0; i < RHOSIZE; i++)
      {
         qRhoVectors[i] = new SpatialForceVector(centerOfMassFrame);

         for (int j = 0; j < 9; j++)
         {
            graphicYoDoubles[q][i][j] = new DoubleYoVariable(name + "rhoGraphicVectorElement" + q + i + j, registry);
         }

         double greenLevel = 0.25 * i / (double) RHOSIZE;
         graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector(name + "RhoGraphicVector" + q + i + "Linear", graphicYoDoubles[q][i][X],
                 graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                 scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.0, 0.7));
         dynamicGraphicVectorsRhoLinear.add(graphicWrenches[q][i][LINEAR]);
         graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector(name + "RhoGraphicVector" + q + i + "Angular", graphicYoDoubles[q][i][X],
                 graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                 scaleFactor, new YoAppearanceRGBColor(0.5, greenLevel, 0.6, 0.7));
         dynamicGraphicVectorsRhoAngular.add(graphicWrenches[q][i][ANGULAR]);
      }

      q = 1;

      for (int i = 0; i < PHISIZE; i++)
      {
         qPhiVectors[i] = new SpatialForceVector(centerOfMassFrame);
         double greenLevel = 0.25 * i / (double) PHISIZE;
         for (int j = 0; j < 9; j++)
         {
            graphicYoDoubles[q][i][j] = new DoubleYoVariable(name + "rhoGraphicVectorElement" + q + i + j, registry);
         }

         graphicWrenches[q][i][LINEAR] = new DynamicGraphicVector(name + "PhiGraphicVector" + q + i + "Linear", graphicYoDoubles[q][i][X],
                 graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][x], graphicYoDoubles[q][i][y], graphicYoDoubles[q][i][z],
                 scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.0, 0.7));
         dynamicGraphicVectorsPhiLinear.add(graphicWrenches[q][i][LINEAR]);
         graphicWrenches[q][i][ANGULAR] = new DynamicGraphicVector(name + "PhiGraphicVector" + q + i + "Angular", graphicYoDoubles[q][i][X],
                 graphicYoDoubles[q][i][Y], graphicYoDoubles[q][i][Z], graphicYoDoubles[q][i][xx], graphicYoDoubles[q][i][yy], graphicYoDoubles[q][i][zz],
                 scaleFactor, new YoAppearanceRGBColor(1.0, greenLevel, 0.6, 0.7));
         dynamicGraphicVectorsPhiAngular.add(graphicWrenches[q][i][ANGULAR]);

      }

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "RawBasisVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "RawBasisVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "RawBasisVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "RawBasisVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);

      this.debug = new BooleanYoVariable(this.getClass().getSimpleName() + "Debug", registry);
      this.debug.set(DEBUG);

   }

   public void computeAllMatriciesAndPopulateNativeInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput)
   {
      int iRho = 0;
      int iPhi = 0;
      int q = 0;
      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            ReferenceFrame frameOfInterest = endEffector.getReferenceFrame();
            OptimizerContactModel contactModel = endEffector.getContactModel();
            if (contactModel instanceof OptimizerCylinderContactModel)
            {
               frameOfInterest=((OptimizerCylinderContactModel)contactModel).getCylinderFrame();
            }
            
            tempPoint.setToZero(frameOfInterest);
            printIfDebug("tempPoint is :" + tempPoint);
            tempPoint.changeFrame(endEffector.getReferenceFrame().getRootFrame());
            printIfDebug("tempPoint is :" + tempPoint);
            q = 0;
            OptimizerContactModel model = contactModel;
            
            for (int iRhoModel = 0; iRhoModel < model.getSizeInRho(); iRhoModel++)
            {
               SpatialForceVector currentBasisVector = qRhoVectors[iRho];

               nativeInput.setRhoMin(iRho, 0, model.getRhoMin(iRhoModel));
               model.packQRhoBodyFrame(iRhoModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(frameOfInterest);
               printIfDebug("qRhoVectors[" + iRho + "] = " + currentBasisVector);
               packYoDoubles(iRho, q, currentBasisVector, tempPoint);

               currentBasisVector.changeFrame(centerOfMassFrame);
               nativeInput.setQRho(iRho, currentBasisVector);

               iRho++;
            }

            q = 1;

            for (int iPhiModel = 0; iPhiModel < model.getSizeInPhi(); iPhiModel++)
            {
               SpatialForceVector currentBasisVector = qPhiVectors[iPhi];

               nativeInput.setPhiMin(iPhi, 0, model.getPhiMin(iPhiModel));
               nativeInput.setPhiMax(iPhi, 0, model.getPhiMax(iPhiModel));
               model.packQPhiBodyFrame(iPhiModel, currentBasisVector, endEffector.getReferenceFrame());

               currentBasisVector.changeFrame(frameOfInterest);
               packYoDoubles(iPhi, q, currentBasisVector, tempPoint);

               currentBasisVector.changeFrame(centerOfMassFrame);
               nativeInput.setQPhi(iPhi, currentBasisVector);

               iPhi++;
            }
         }
      }
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

   public void printIfDebug(String message)
   {
      if (this.debug.getBooleanValue())
      {
         System.out.println(this.getClass().getSimpleName() + ": " + message);
      }
   }
}
