package us.ihmc.commonWalkingControlModules.terrain;


import java.io.File;

import javax.media.j3d.Transform3D;
import javax.swing.JFileChooser;
import javax.swing.JFrame;

import com.yobotics.simulationconstructionset.GroundProfile;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.AlternatingSlopesGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.BumpyGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.GroundProfileFromFile;
import com.yobotics.simulationconstructionset.util.ground.ObstacleCourse;
import com.yobotics.simulationconstructionset.util.ground.RandomRockyGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.RollingGroundProfile;

public enum TerrainType
{
   FLAT, ROLLING_HILLS, BUMPY, ROCKY, STEP_UP, ALTERNATING_SLOPES, STEEP_UPSLOPE, MOON, LOADED, COURSE;

   public static final double FLAT_HEIGHT = 0.0;
   
   public static GroundProfile setUpTerrain(TerrainType terrainType)
   {
      return setUpTerrain(terrainType, null);
   }

   public static GroundProfile setUpTerrain(TerrainType terrainType, YoVariableRegistry registry)
   {
      GroundProfile groundProfile = null;
      switch (terrainType)
      {
         case FLAT :
            groundProfile = new FlatGroundProfile(FLAT_HEIGHT);

            break;

         case ROLLING_HILLS :
            groundProfile = new RollingGroundProfile(0.1, 0.3, 0.0, -20.0, 20.0, -20.0, 20.0);

            break;

         case BUMPY :
            groundProfile = new BumpyGroundProfile(0.04, 0.4, 0.1, 0.2, 0.02, 0.35, 0.0, 5.0, -20.0, 20.0, -20.0, 20.0);

            break;

         case ROCKY :
            groundProfile = new RandomRockyGroundProfile(10);

            break;

         case STEP_UP :
            groundProfile = new StepUpGroundProfile(registry);
            break;
            
         case ALTERNATING_SLOPES :
         {
            double xMin = -1.0, xMax = 12.0;
            double yMin = -2.0, yMax = 2.0;
            double[][] xSlopePairs = new double[][]{{1.0, 0.05}, {3.0, 0.0}, {5.0, 0.1}, {7.0, 0.0}, {9.0, 0.1}, {11.0, 0.0}};
            groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
         
            break;
         }
            
         case STEEP_UPSLOPE :
         {
            double xMin = -1.0, xMax = 10.0;
            double yMin = -2.0, yMax = 2.0;
            double[][] xSlopePairs = new double[][]{{1.0, 0.2}, {6.0, 0.0}, {10.0, 0.0}};
            groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
            break;
         }
         
         case COURSE :
            groundProfile = new ObstacleCourse();

            break;

         case MOON :
            groundProfile = new GroundProfileFromFile("GroundModels" + File.separator + "moonNew.asc", new Transform3D());

            break;

         case LOADED :
            JFileChooser fc = new JFileChooser();
            int returnVal = fc.showOpenDialog(new JFrame());

            if (returnVal == JFileChooser.APPROVE_OPTION)
            {
               File file = fc.getSelectedFile();

               // This is where a real application would open the file.
               System.out.println("Opening: " + file.getName());
               groundProfile = new GroundProfileFromFile(file.getAbsolutePath(), new Transform3D());
            }
            else
            {
               System.out.println("Open command cancelled by user.");
               groundProfile = new FlatGroundProfile();
            }



            break;

         default :
            groundProfile = new FlatGroundProfile();
      }

      return groundProfile;
   }


}

