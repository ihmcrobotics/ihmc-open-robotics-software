package us.ihmc.communication.packets.dataobjects;

import us.ihmc.tools.DocumentedEnum;


/**
 * @author twan
 *         Date: 5/6/13
 */
public enum HighLevelState implements DocumentedEnum<HighLevelState>
{
   WALKING,
   JOINT_POSITION_CONTROL,
   DRIVING,
   DO_NOTHING_BEHAVIOR,
   JOINT_PD_CONTROL,
   INGRESS_EGRESS,
   DIAGNOSTICS,
   MULTI_CONTACT,
   INVERSE_DYNAMICS_JOINT_CONTROL,
   TASKSPACE_POSITION_CONTROL,
   POSE_PLAYBACK;

   public static final HighLevelState[] values = values();
   
   public static final HighLevelState[] rosApiValues = new HighLevelState[]{WALKING, JOINT_POSITION_CONTROL, DO_NOTHING_BEHAVIOR};

   @Override
   public String getDocumentation(HighLevelState var)
   {
      switch(var)
      {
         case WALKING:
            return "whole body force control employing IHMC walking, balance, and manipulation algorithms";
         case JOINT_POSITION_CONTROL:
            return "joint control. NOTE: controller will not attempt to keep the robot balanced";
         case DO_NOTHING_BEHAVIOR:
            return "do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.";
         default:
            return "no documentation available";
      }
   }

   @Override
   public HighLevelState[] getDocumentedValues()
   {
      return rosApiValues;
   }
}
