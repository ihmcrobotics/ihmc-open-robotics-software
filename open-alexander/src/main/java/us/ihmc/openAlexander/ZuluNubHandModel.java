package us.ihmc.openAlexander;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.handsros2.HandModel;
import us.ihmc.robotics.partNames.HandJointName;

public class ZuluNubHandModel implements HandModel
{
   public static final double CARBON_TUBE_RADIUS = 0.03;
   public static final double CONTROL_FRAME_X_OFFSET = -0.013;

   private static final double SHORT_CONTROL_FRAME_Z_OFFSET = -0.298;
   private static final double LONG_CONTROL_FRAME_Z_OFFSET = -0.413;
   public static final double CONTROL_FRAME_Z_OFFSET = ZuluVersion.SHORT_NUBS ? SHORT_CONTROL_FRAME_Z_OFFSET : LONG_CONTROL_FRAME_Z_OFFSET;

   public static double getControlFrameOffsetZ()
   {
      return CONTROL_FRAME_Z_OFFSET;
   }

   public static Vector3D getElbowToControlFrame()
   {
      return new Vector3D(ZuluNubHandModel.CONTROL_FRAME_X_OFFSET, 0.0, CONTROL_FRAME_Z_OFFSET);
   }

   private final HandJointName[] handJointNames = new HandJointName[0];

   @Override
   public HandJointName[] getHandJointNames()
   {
      return handJointNames;
   }
}

