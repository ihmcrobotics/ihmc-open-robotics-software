package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.PrintWriter;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ReachabilityMapCSVExporter implements ReachabilityMapFileWriter
{
   @Override
   public void write(File file, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      int dataLength = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel != null)
            dataLength += voxel.getNumberOfReachableRays();
      }
//      float[][] dataMatrix = new float[dataLength][4];
      ArrayList<ArrayList<Float>> dataMatrix = new ArrayList<>();
      List<String[]> dataLines = new ArrayList<>();
      int row = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;

         for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
         {
            VoxelExtraData extraData = voxel.getRayExtraData(rayIndex);

            if (extraData == null)
               continue;
            float x = (float) extraData.getDesiredPosition().getX();
            float y = (float) extraData.getDesiredPosition().getY();
            float z = (float) extraData.getDesiredPosition().getZ();
            float r = (float) voxel.getR();
//            dataMatrix.add(new ArrayList<Float>(Arrays.asList((float)extraData.getDesiredPosition().getX(),
//                       (float)extraData.getDesiredPosition().getY(),(float)extraData.getDesiredPosition().getZ(),(float)voxel.getR())));
            dataLines.add(new String[] {Float.toString(x),Float.toString(y),Float.toString(z),Float.toString(r)});
//            dataMatrix[row][0]=(float) extraData.getDesiredPosition().getX();
//            dataMatrix[row][1]=(float) extraData.getDesiredPosition().getY();
//            dataMatrix[row][2]=(float) extraData.getDesiredPosition().getZ();
//            dataMatrix[row][3]=(float) voxel.getR();
            row++;
         }
      }
      RobotDefinition robotDefinition = robotInformation.getRobotDefinition();
      Path path = FileSystems.getDefault().getPath("").toAbsolutePath();
      String currentPath = path.toString();
      File csvFile = new File(currentPath+robotDefinition.getName()+this.getFileExtension());
      try (PrintWriter pw = new PrintWriter(csvFile)) {
         dataLines.stream()
                  .map(this::convertToCSV)
                  .forEach(pw::println);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   /* create a method for formatting a single line of data represented as an array of Strings */
   private String convertToCSV(String[] data) {
      return Stream.of(data)
                   .map(this::escapeSpecialCharacters)
                   .collect(Collectors.joining(","));
   }

   private String escapeSpecialCharacters(String data) {
      String escapedData = data.replaceAll("\\R", " ");
      if (data.contains(",") || data.contains("\"") || data.contains("'")) {
         data = data.replace("\"", "\"\"");
         escapedData = "\"" + data + "\"";
      }
      return escapedData;
   }

   @Override
   public String getFileExtension()
   {
      return ".csv";
   }

}