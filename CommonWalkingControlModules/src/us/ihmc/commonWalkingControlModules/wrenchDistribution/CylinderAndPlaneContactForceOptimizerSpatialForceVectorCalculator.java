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



   public CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator(String name, ReferenceFrame centerOfMassFrame, YoVariableRegistry registry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.tempPoint = new FramePoint(centerOfMassFrame);
      this.tempVector = new FrameVector(centerOfMassFrame);
      
      
      graphicWrenches[0] = new DynamicGraphicVector[RHOSIZE][2];
      graphicYoDoubles[0] = new DoubleYoVariable[RHOSIZE][9];

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

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "OptimizerOutputVectorsRhoLinear ", dynamicGraphicVectorsRhoLinear);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "OptimizerOutputVectorsRhoAngular", dynamicGraphicVectorsRhoAngular);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "OptimizerOutputVectorsPhiLinear ", dynamicGraphicVectorsPhiLinear);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects(name + "OptimizerOutputVectorsPhiAngular", dynamicGraphicVectorsPhiAngular);
      
      
   }

   public void computeAllWrenchesBasedOnNativeOutputAndInput(Collection<? extends EndEffector> endEffectors,
           CylinderAndPlaneContactForceOptimizerNativeInput nativeInput, CylinderAndPlaneContactForceOptimizerNativeOutput nativeOutput)
   {
      int rhoLocation = 0;
      int phiLocation = 0;
      DenseMatrix64F rho = nativeOutput.getRho();
      DenseMatrix64F phi = nativeOutput.getPhi();

      int iRho = 0;
      int iPhi = 0;
      int q = 0;
      
      for (EndEffector endEffector : endEffectors)
      {
         if (endEffector.isLoadBearing())
         {
            tempSum.zero();
            OptimizerContactModel model = endEffector.getContactModel();
            for (int i = 0; i < model.getSizeInRho(); i++)
            {
               nativeInput.packQrho(rhoLocation, tempVectorMatrix);
               double rhoOfI = rho.get(rhoLocation);

               for (int j = 0; j < SpatialForceVector.SIZE; j++)
               {
                  tempVectorMatrix.times(j, rhoOfI);
                  tempSum.add(j, 0, tempVectorMatrix.get(j));
               }

               rhoLocation++;
            }

            for (int i = 0; i < model.getSizeInPhi(); i++)
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
