package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;

public class ChangingAnkleToSoleScriptTransformer extends ScriptTransformer
{
   public ChangingAnkleToSoleScriptTransformer(String scriptDirectoryPath) throws IOException, InterruptedException
   {
      super(scriptDirectoryPath);
   }

   @Override
   public Object transformScriptObject(Object object)
   {
      if (object instanceof FootstepDataListMessage)
      {
         FootstepDataListMessage packetToTransform = (FootstepDataListMessage) object;
         for (FootstepDataMessage footstep : packetToTransform.getDataList())
         {
            transformFootstepFromAnkleToSole(footstep);
         }
      }
      
      if (object instanceof FootstepDataMessage)
      {
         FootstepDataMessage packetToTransform = (FootstepDataMessage) object;
         transformFootstepFromAnkleToSole(packetToTransform);
      }
      
      return object;
   }
   
   private void transformFootstepFromAnkleToSole(FootstepDataMessage footstep)
   {
//      if (footstep.origin == FootstepOrigin.AT_SOLE_FRAME)
//      {
//         return;
//      }
//      else if (footstep.origin == FootstepOrigin.AT_ANKLE_FRAME)
//      {
//         RigidBodyTransform footstepPose = new RigidBodyTransform();
//         footstepPose.setRotation(footstep.getOrientation());
//         footstepPose.setTranslation(footstep.getLocation());
////         PrintTools.info("Before:\n" + footstepPose.toString());
//
//         RigidBodyTransform offsetInAnkleFrame = new RigidBodyTransform();
//         offsetInAnkleFrame.setTranslation(0.025, 0.0, -0.084);
//         footstepPose.multiply(offsetInAnkleFrame);
////         PrintTools.info("After:\n" + footstepPose.toString());
//         
//         footstepPose.getRotation(footstep.getOrientation());
//         footstepPose.getTranslation(footstep.getLocation());
//      }
//      else
//      {
//         PrintTools.error("Unknown reference!");
//      }
   }
   
   public static void main(String[] args) throws IOException, InterruptedException
   {
      ArrayList<String> paths = new ArrayList<>();
      paths.add("../IHMCAvatarInterfaces/resources/scripts/ExerciseAndJUnitScripts");
      
      // TODO: plenty of these scripts are broken.
//      paths.add("../IHMCHumanoidBehaviors/scripts");
//      paths.add("../IHMCHumanoidBehaviors/resources/scripts");
//      paths.add("../IHMCHumanoidBehaviors/resources/us/ihmc/atlas/scripts");
//      paths.add("../IHMCHumanoidBehaviors/resources/us/ihmc/humanoidBehaviors/behaviors/scripts");
//      paths.add("../../IHMCInternalSoftware/IHMCHumanoidOperatorInterface/scripts");
//      paths.add("../../IHMCInternalSoftware/IHMCHumanoidOperatorInterface/scripts/ExerciseAndJUnitScripts");
//      paths.add("../Atlas/scripts");

      int index = 0;
      for (String path : paths)
      {
         System.out.println("Transforming scripts in: " + paths.get(index++));
         new ChangingAnkleToSoleScriptTransformer(path);
      }
   }
}
