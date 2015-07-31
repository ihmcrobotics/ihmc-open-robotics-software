package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.utilities.math.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import Jama.Matrix;

public class VirtualChainBuilder
{
   private final ReferenceFrame baseFrame;
   private final ArrayList<ReferenceFrame> referenceFrames;
   private final VirtualChainData virtualChainData;
   private final ArrayList<FrameVector> virtualChainParameterVectors;

   public VirtualChainBuilder(ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames)
   {
      this.baseFrame = baseFrame;

      if (!baseFrame.isZupFrame())
      {
         throw new RuntimeException("!baseFrame.isZupFrame()");
      }

      this.referenceFrames = referenceFrames;

      virtualChainData = new VirtualChainData();
      virtualChainParameterVectors = new ArrayList<FrameVector>();
   }

   public void recordDataPoint(FramePoint2d centerOfMassProjection)
   {
      centerOfMassProjection.checkReferenceFrameMatch(baseFrame);
      virtualChainData.addData(baseFrame, referenceFrames, centerOfMassProjection);
   }

   public int getSolutionRank()
   {
      VirtualChainDataMatrix virtualChainDataMatrix = new VirtualChainDataMatrix(virtualChainData);
      Matrix AMatrix = virtualChainDataMatrix.getAMatrix();

      int rank = AMatrix.rank();

      return rank;
   }


   public boolean estimateVirtualChainParameterVectors()
   {
      // Reset the virtual chain for a new estimation
      virtualChainParameterVectors.clear();

      VirtualChainDataMatrix virtualChainDataMatrix = new VirtualChainDataMatrix(virtualChainData);

      Matrix yMatrix = virtualChainDataMatrix.getYMatrix();
      Matrix AMatrix = virtualChainDataMatrix.getAMatrix();

      int virtualChainDegreesOfFreedom = AMatrix.getColumnDimension() / 3;
      int numberOfPostureRecorded = AMatrix.getRowDimension() / 2;

      if ((double) numberOfPostureRecorded > (double) 3.0 / 2.0 * virtualChainDegreesOfFreedom)
      {
         Matrix pseudoInverse = MatrixTools.pseudoinverse(AMatrix);
         Matrix solution = pseudoInverse.times(yMatrix);

         Matrix errorMatrix = AMatrix.times(solution).minus(yMatrix);
         double[] averageAndMax = computeAverageAndMax(errorMatrix);

         System.out.println("Average error on recorded postures = " + averageAndMax[0]);
         System.out.println("Max error on recorded postures = " + averageAndMax[1]);

         for (int i = 0; i < virtualChainDegreesOfFreedom; i++)
         {
            double x = solution.get(i * 3, 0);
            double y = solution.get(i * 3 + 1, 0);
            double z = solution.get(i * 3 + 2, 0);
            double epsilon = 1e-10;

            if (Math.abs(x) < epsilon)
               x = 0.0;
            if (Math.abs(y) < epsilon)
               y = 0.0;
            if (Math.abs(z) < epsilon)
               z = 0.0;

            FrameVector frameVector = new FrameVector(referenceFrames.get(i), x, y, z);
            virtualChainParameterVectors.add(frameVector);
         }

         return true;
      }
      else
         return false;
   }

   public ArrayList<FrameVector> getVirtualChainParameterVectors()
   {
      return virtualChainParameterVectors;
   }


   private double[] computeAverageAndMax(Matrix matrix)
   {
      int rowDimension = matrix.getRowDimension();
      int columnDimension = matrix.getColumnDimension();

      if (columnDimension != 1)
         throw new RuntimeException();

      double total = 0.0;
      double max = 0.0;
      for (int i = 0; i < rowDimension; i++)
      {
         double absoluteValue = Math.abs(matrix.get(i, 0));
         total = total + absoluteValue;

         if (absoluteValue > max)
            max = absoluteValue;
      }

      double average = total / ((double) rowDimension);

      return new double[] {average, max};
   }


   public void saveVirtualChainParameterVectors(String fileName) throws IOException
   {
      File currentDirectory = new File(".");
      String filePath = currentDirectory.getCanonicalPath()
                        + "/../CommonWalkingControlModules/src/us/ihmc/commonWalkingControlModules/calibration/virtualCoMChain/";

      File file = new File(filePath + fileName + ".vcp");

      BufferedWriter writer = null;

      if (file.exists())
      {
         file.delete();
      }

      try
      {
         writer = new BufferedWriter(new FileWriter(file));
      }
      catch (FileNotFoundException e)
      {
         System.out.println("Unable to write the virtual chain parameters into the file " + file.toString());
      }

      for (FrameVector frameVector : virtualChainParameterVectors)
      {
         writer.write(((Double) frameVector.getX()).toString());
         writer.newLine();
         writer.write(((Double) frameVector.getY()).toString());
         writer.newLine();
         writer.write(((Double) frameVector.getZ()).toString());
         writer.newLine();
      }

      writer.close();
      System.out.println("Virtual chain parameters have been successfully saved into the file " + file.toString());
   }

}

