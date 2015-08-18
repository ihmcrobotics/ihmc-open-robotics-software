package us.ihmc.communication.packets.sensing;


public enum MultisenseTest
{
   DIRECT_FROM_MULTISENSE("/multisense/lidar_points2", MultisenseFrameName.LEFT_CAMERA_OPTICAL_FRAME),
   UNFILERED_CLOUD("/filter_scan_test/unfiltered_cloud_in_head_root", MultisenseFrameName.HEAD_ROOT),
   
   //Near Scan
   FILTER_FOR_NEAR_SCAN("/filter_scan_test/multisense/filtered_cloud", MultisenseFrameName.HEAD_ROOT),
   ASSEMBLED_NEAR_SCAN("/filter_scan_test/assembled_cloud", MultisenseFrameName.HEAD_ROOT),
   
   //QUADTREE
   SHADOW_FILTER_FOR_QUADTREE("/filter_scan_test/multisense/highly_filtered_cloud", MultisenseFrameName.HEAD_ROOT),
   OUTLIER_FILTER_FOR_QUADTREE("/filter_scan_test/multisense/quadtree_cloud", MultisenseFrameName.HEAD_ROOT),
   
   NEAR_SCAN_IN_POINT_CLOUD_DATA_RECEIVER("multisense/filtered_cloud", MultisenseFrameName.WORLD),
   NEAR_SCAN_IN_WORLD_FROM_ROS("multisense/filtered_cloud", MultisenseFrameName.HEAD_ROOT),
   
   NEAR_SCAN_IN_HEAD_FROM_ROS("head/multisense/filtered_cloud", MultisenseFrameName.HEAD_ROOT),
   NEAR_SCAN_IN_U_TORSO_FROM_ROS("utorso/multisense/filtered_cloud", MultisenseFrameName.U_TORSO),
   NEAR_SCAN_IN_M_TORSO_FROM_ROS("mtorso/multisense/filtered_cloud", MultisenseFrameName.M_TORSO),
   NEAR_SCAN_IN_L_TORSO_FROM_ROS("ltorso/multisense/filtered_cloud", MultisenseFrameName.L_TORSO),
   NEAR_SCAN_IN_PELVIS_FROM_ROS( "pelvis/multisense/filtered_cloud", MultisenseFrameName.PELVIS);
   
   public String rosTopic;
   public MultisenseFrameName frame;
   
   MultisenseTest(String rosTopic, MultisenseFrameName frame)
   {
      this.rosTopic = rosTopic;
      this.frame = frame;
   }
   
   public String getRosTopic()
   {
      return rosTopic;
   }

   public MultisenseFrameName getFrame()
   {
      return frame;
   }
   
   public static enum MultisenseFrameName
   {
      HEAD_ROOT, LEFT_CAMERA_OPTICAL_FRAME, WORLD, U_TORSO, M_TORSO, L_TORSO, PELVIS;
   }
}
