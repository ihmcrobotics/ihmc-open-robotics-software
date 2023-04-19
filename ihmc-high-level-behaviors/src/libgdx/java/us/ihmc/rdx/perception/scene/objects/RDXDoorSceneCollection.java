package us.ihmc.rdx.perception.scene.objects;

/**
 * TODO: Separate the non RDX part into ihmc-perception.
 */
public class RDXDoorSceneCollection
{
   public static RDXSceneObject createPullDoorPanel()
   {
      return new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj", "PullDoorPanel");
   }

   public static RDXSceneObject createPullDoorFrame()
   {
      return new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj", "PullDoorFrame");
   }

   public static RDXSceneObject createPushDoorPanel()
   {
      return new RDXSceneObject("environmentObjects/door/doorPanel/DoorPanel.g3dj", "PushDoorPanel");
   }

   public static RDXSceneObject createPushDoorFrame()
   {
      return new RDXSceneObject("environmentObjects/door/doorFrame/DoorFrame.g3dj","PullDoorFrame");
   }
}
