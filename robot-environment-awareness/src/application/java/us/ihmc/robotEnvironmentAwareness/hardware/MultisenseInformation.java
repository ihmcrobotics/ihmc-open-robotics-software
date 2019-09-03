package us.ihmc.robotEnvironmentAwareness.hardware;

import boofcv.struct.calib.IntrinsicParameters;

public enum MultisenseInformation
{
   CART, ATLAS;

   public String getAddress()
   {
      switch (this)
      {
      case CART:
         return "http://10.6.192.14:11311";
      case ATLAS:
         return "http://10.7.4.100:11311";
      }
      return null;
   }

   public IntrinsicParameters getIntrinsicParameters()
   {
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
      switch (this)
      {
      case CART:
         intrinsicParameters.setFx(566.8350830078125);
         intrinsicParameters.setFy(566.8350830078125);
         intrinsicParameters.setCx(505.5);
         intrinsicParameters.setCy(260.5);
         break;
      case ATLAS:
         intrinsicParameters.setFx(584.234619140625);
         intrinsicParameters.setFy(584.234619140625);
         intrinsicParameters.setCx(512.0);
         intrinsicParameters.setCy(272.0);
         break;
      }
      return intrinsicParameters;
   }

   public int[] getProjectionOffset()
   {
      int[] offset = new int[2];
      switch (this)
      {
      case CART:
         offset[0] = 7;
         offset[1] = 0;
         break;
      case ATLAS:
         offset[0] = 8;
         offset[1] = 0;
         break;
      }
      return offset;
   }

   public static String getImageTopicName()
   {
      return "/multisense/left/image_rect_color";
   }

   public static String getCameraInfoTopicName()
   {
      return "/multisense/left/camera_info";
   }

   public static String getLidarScanTopicName()
   {
      return "/singleScanAsCloudWithSource";
   }

   public static String getStereoVisionPointCloudTopicName()
   {
      return "/multisense/image_points2_color_world";
   }
}
