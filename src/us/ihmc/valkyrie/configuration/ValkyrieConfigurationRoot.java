package us.ihmc.valkyrie.configuration;

import us.ihmc.utilities.robotSide.SideDependentList;

public class ValkyrieConfigurationRoot
{
   public static final String[] API_BUILDER_INPUT_FILES = new String[] {"TurbodriverAPI_DRCv3.xml", "TurbodriverAPI_DRCv4_ihmc.xml", "TurbodriverAPI_DRCv4_linear_ihmc.xml", "TurbodriverAPI_DRCv4.xml", "TurbodriverAPI_DRCv4_linear.xml", "FingerAPI_DRCv1.xml",
         "WristAPI_DRCv1.xml", "TurbodriverAPI_DRCv4_bench.xml"};


   public static final SideDependentList<String> FOOT_SENSOR_FILES_BASENAMES = new SideDependentList<>("FT14020", "FT16592");

   public static final boolean VALKYRIE_WITH_ARMS = true;

   public static final String SCHEDULE_FILE = "main_ihmc.yaml";
//   public static final String URDF_FILE = "models/V1_hw_ihmc.urdf";
   public static final String URDF_FILE = "models/V1_hw_reduced_polygon_count_ihmc.urdf";
   public static final String SDF_FILE;
   static
   {
      if (VALKYRIE_WITH_ARMS)
//         SDF_FILE = "models/V1/sdf/V1_sim_shells.sdf";
    	  SDF_FILE = "models/V1/sdf/V1_sim_shells_reduced_polygon_count.sdf";
      else
         SDF_FILE = "models/V1/sdf/V1_sim_shells_no_arms.sdf";
   }

   public static final String BENCH_SCHEDULE_FILE = "main_bench.yaml";
   public static final String BENCH_URDF_FILE = "models/V1_hw_bench.urdf";
   
}
