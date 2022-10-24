package us.ihmc.promp;

import us.ihmc.promp.presets.ProMPInfoMapper;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class ProMPUtil
{
   public static void saveAsCSV(ProMPInfoMapper.EigenMatrixXd dataMatrix, String fileName)
   {
      List<String[]> dataLines = new ArrayList<>();
      for (int i = 0; i < dataMatrix.rows(); i++)
      {
         String[] stringLine = new String[(int) dataMatrix.cols()];
         for (int j = 0; j < dataMatrix.cols(); j++)
            stringLine[j] = "" + dataMatrix.coeff(i, j);
         dataLines.add(stringLine);
      }
      WorkspaceDirectory fileDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String fileDirAbs = fileDirectory.getDirectoryPath().toAbsolutePath().toString();
      File csvFile = new File(fileDirAbs + fileName);
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         dataLines.stream().map(s -> convertToCSV(s)).forEach(writer::println);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static String convertToCSV(String[] data)
   {
      return Stream.of(data).collect(Collectors.joining(","));
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
}
