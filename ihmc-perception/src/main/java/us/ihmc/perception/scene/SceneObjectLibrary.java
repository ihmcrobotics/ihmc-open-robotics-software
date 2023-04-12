package us.ihmc.perception.scene;

/**
 * Use to specify which scene objects a robot is looking for.
 *
 * We can put more specific stuff in here for now, like door specific
 * and even dynamic objects.
 *
 * This is a super high level class, it includes ROS 2 topics,
 * heuristic stuff for certain objects, stuff like that.
 *
 * This class exists so we can support multiple robots detecting the same
 * objects, but also to serve the need we have to define things
 * by hand and construct custom heuristics.
 */
public class SceneObjectLibrary
{
   public static SceneObjectLibrary defaultObjects()
   {
      SceneObjectLibrary sceneObjectLibrary = new SceneObjectLibrary();

      // Add door stuff


      // Add ArUco box

      // Add non-ArUco cup -- detected by neural net

      return sceneObjectLibrary;
   }

}
