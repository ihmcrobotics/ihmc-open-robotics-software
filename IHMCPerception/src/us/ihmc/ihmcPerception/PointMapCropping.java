package us.ihmc.ihmcPerception;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import javax.swing.JFileChooser;
import javax.vecmath.Point3d;

import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundReaderAndWriter;
import us.ihmc.robotics.quadTree.Box;

/**
 * Created by agrabertilton on 1/26/15.
 */
public class PointMapCropping
{
   public static void main(String[] args)
   {
      String filename = "resources/pointListsForTesting/firstFewCinderBlockScans.pointList";

      double minX = -1.0; //5.0f;
      double minY = -3.0; //5.0f;
      double maxX = 1.0; //5.0f;
      double maxY = 4.0; //5.0f;

      Box bounds = new Box(minX, minY, maxX, maxY);

      double maxZ = 0.6;
      int skipPoints = 0;
      int maxNumberOfPoints = 2000000;
      try
      {
         QuadTreeForGroundReaderAndWriter quadTreeForGroundReaderAndWriter = new QuadTreeForGroundReaderAndWriter();
         ArrayList<Point3d> points = quadTreeForGroundReaderAndWriter.readPointsFromFile(filename, skipPoints, maxNumberOfPoints, bounds, maxZ);

         JFileChooser outputChooser = new JFileChooser();
         outputChooser.setDialogTitle("Specify a file to save");
         int returnVal = outputChooser.showSaveDialog(null);
         if (returnVal != JFileChooser.APPROVE_OPTION){
            System.err.println("Can not save here: " + outputChooser.getName());
            return;
         }
         File fileOut = outputChooser.getSelectedFile();

         BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(fileOut)));

         for (Point3d point : points){
            bufferedWriter.write(point.toString() + System.lineSeparator());
         }

         bufferedWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
