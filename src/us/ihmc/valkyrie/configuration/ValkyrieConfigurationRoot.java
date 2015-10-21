package us.ihmc.valkyrie.configuration;

import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieConfigurationRoot
{
   public static final String[] API_BUILDER_INPUT_FILES = new String[] {"TurbodriverAPI_DRCv3.xml", "TurbodriverAPI_DRCv4_ihmc.xml", "TurbodriverAPI_DRCv4_linear_ihmc.xml", "TurbodriverAPI_DRCv4.xml", "TurbodriverAPI_DRCv4_linear.xml", "FingerAPI_DRCv1.xml",
         "WristAPI_DRCv1.xml", "TurbodriverAPI_DRCv4_bench.xml"};

   public static final SideDependentList<String> FOOT_SENSOR_FILES_BASENAMES = new SideDependentList<>("FT14020", "FT16592");
   
   public static final String SCHEDULE_FILE = "main_ihmc.yaml";
   
   public static final String URDF_FILE = "models/val_description/urdf/valkyrie_A_hw.urdf";
   public static final String SDF_FILE = "models/val_description/sdf/valkyrie_B_hw.sdf";

   
   public static final boolean VALKYRIE_WITH_ARMS = true;
}
