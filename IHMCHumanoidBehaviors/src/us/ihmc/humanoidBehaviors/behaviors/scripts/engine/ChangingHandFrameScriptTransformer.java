package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;


import java.io.IOException;
import java.util.ArrayList;

public class ChangingHandFrameScriptTransformer extends ScriptTransformer
{

   public ChangingHandFrameScriptTransformer(String scriptDirectoryPath) throws IOException, InterruptedException
   {
      super(scriptDirectoryPath);
   }

   @Override
   public Object transformScriptObject(Object object)
   {
//      if (object instanceof HandPosePacket)
//      {
//         HandPosePacket packetToTransform = (HandPosePacket) object;
//         if (packetToTransform.orientation == null || packetToTransform.position == null)
//            return object;
//         RigidBodyTransform handpose = new RigidBodyTransform();
//         handpose.setRotation(packetToTransform.getOrientation());
//         handpose.setTranslation(new Vector3d(packetToTransform.position));
//         RigidBodyTransform xOffsetInHandFrame = new RigidBodyTransform();
//         xOffsetInHandFrame.setTranslation(0.16, 0.0, 0.0);
//         handpose.multiply(handpose, xOffsetInHandFrame);
//         handpose.getTranslation(packetToTransform.position);
//      }
      
      return object;
   }
   
   public static void main(String[] args) throws IOException, InterruptedException
   {
      ArrayList<String> paths = new ArrayList<>();
      paths.add("..\\Atlas\\scripts");
      paths.add("..\\DarpaRoboticsChallenge\\scripts");
      paths.add("..\\DarpaRoboticsChallenge\\scriptsSaved");
      paths.add("..\\DarpaRoboticsChallenge\\resources\\scripts\\ExerciseAndJUnitScripts");
      paths.add("..\\IHMCHumanoidBehaviors\\scripts");
      paths.add("..\\IHMCHumanoidBehaviors\\\resources\\scripts");
      paths.add("..\\IHMCHumanoidOperatorInterface\\scripts");
      paths.add("..\\IHMCHumanoidOperatorInterface\\resources\\finalScripts");

      int index = 0;
      for (String path : paths)
      {
         System.out.println("Transforming scripts in: " + paths.get(index++));
         new ChangingHandFrameScriptTransformer(path);
      }
   }
}
