package us.ihmc.promp;

import us.ihmc.log.LogTools;
import us.ihmc.promp.presets.ProMPInfoMapper;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class ProMPUtil
{
   private static boolean ETC_DIRECTORY_PRINTED = false;
   private static boolean DEMO_DIRECTORY_PRINTED = false;

   public static Path getEtcDirectory()
   {
      Path pathToPrompSourceMain = WorkspacePathTools.inferFilesystemSourceSetDirectory(ProMPUtil.class);
      Path prompProject = Paths.get("/").resolve(pathToPrompSourceMain.subpath(0, pathToPrompSourceMain.getNameCount() - 2));
      Path demosDirectory = prompProject.resolve("etc");
      if (!ETC_DIRECTORY_PRINTED)
      {
         ETC_DIRECTORY_PRINTED = true;
         LogTools.info("ProMP etc directory: {}", demosDirectory);
      }
      return demosDirectory;
   }

   public static Path getDemosDirectory()
   {
      Path demosDirectory = getEtcDirectory().resolve("demos");
      if (!DEMO_DIRECTORY_PRINTED)
      {
         DEMO_DIRECTORY_PRINTED = true;
         LogTools.info("ProMP demos directory: {}", demosDirectory);
      }
      return demosDirectory;
   }

   public static void saveAsCSV(ProMPInfoMapper.EigenMatrixXd dataMatrix, String fileName)
   {
      // Check for null input matrix
      if (dataMatrix == null)
      {
         throw new IllegalArgumentException("Input data matrix is null");
      }
      String fileDirAbs = getEtcDirectory().toString();
      File csvFile = new File(fileDirAbs + fileName);

      try
      {
         // Create PrintWriter for writing to file
         PrintWriter writer = new PrintWriter(csvFile);

         // Write data to file
         for (int i = 0; i < dataMatrix.rows(); i++)
         {
            for (int j = 0; j < dataMatrix.cols(); j++)
            {
               writer.print(dataMatrix.coeff(i, j));
               if (j < dataMatrix.cols() - 1)
               {
                  writer.append(",");
               }
            }
            writer.println();
         }

         // Close PrintWriter
         writer.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Error writing to file: " + e.getMessage());
      }
   }

   public static void printMatrix(ProMPInfoMapper.EigenMatrixXd matrix, String name)
   {
      System.out.println(name);
      for (int row = 0; row < matrix.rows(); row++)
      {
         for (int col = 0; col < matrix.cols(); col++)
         {
            System.out.print(matrix.coeff(row, col) + " ");
         }
         System.out.println();
      }
      System.out.println();
   }

   public static ProMPInfoMapper.EigenMatrixXd concatenateEigenMatrix(ProMPInfoMapper.EigenMatrixXd matrixA, ProMPInfoMapper.EigenMatrixXd matrixB)
   {
      ProMPInfoMapper.EigenMatrixXd matrixC = new ProMPInfoMapper.EigenMatrixXd((int) Math.min(matrixA.rows(), matrixB.rows()),
                                                                                (int) (matrixA.cols() + matrixB.cols()));
      for (int row = 0; row < (int) matrixC.rows(); row++)
      {
         for (int colA = 0; colA < (int) matrixA.cols(); colA++)
         {
            matrixC.apply(row, colA).put(matrixA.coeff(row, colA));
         }
         for (int colB = (int) matrixA.cols(); colB < (int) matrixA.cols() + matrixB.cols(); colB++)
         {
            matrixC.apply(row, colB).put(matrixB.coeff(row, colB - (int) matrixA.cols()));
         }
      }
      return matrixC;
   }

   public static TrajectoryVector concatenateTrajectoryVector(TrajectoryVector vectorA, TrajectoryVector vectorB)
   {
      TrajectoryVector vectorC = new TrajectoryVector();
      for (int row = 0; row < (int) vectorA.size(); row++)
      {
         Trajectory trajectoryRowA = vectorA.get(row);
         Trajectory trajectoryRowB = vectorB.get(row);
         ProMPInfoMapper.EigenMatrixXd rowA = trajectoryRowA.matrix();
         ProMPInfoMapper.EigenMatrixXd rowB = trajectoryRowB.matrix();
         ProMPInfoMapper.EigenMatrixXd rowC = concatenateEigenMatrix(rowA, rowB);
         Trajectory trajectoryRowC = new Trajectory(rowC);
         vectorC.put(trajectoryRowC);
      }
      return vectorC;
   }
}
