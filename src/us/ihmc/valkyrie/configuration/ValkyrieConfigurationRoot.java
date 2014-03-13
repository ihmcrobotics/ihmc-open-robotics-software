package us.ihmc.valkyrie.configuration;

import us.ihmc.robotSide.SideDependentList;

public class ValkyrieConfigurationRoot
{
   public static final String[] API_BUILDER_INPUT_FILES = new String[] {"TurbodriverAPI_DRCv3.xml", "TurbodriverAPI_DRCv4.xml", "TurbodriverAPI_DRCv4_linear.xml", "FingerAPI_DRCv1.xml",
         "WristAPI_DRCv1.xml"};

   public static final String REGISTER_BUILDER_FILE_NAME = "registers.xml";

   public static final SideDependentList<String> FOOT_SENSOR_FILES_BASENAMES = new SideDependentList<>("FT14020", "FT14175");

   public static final String SCHEDULE_FILE = "main_sim.yaml";
}
