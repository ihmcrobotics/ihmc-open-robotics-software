package us.ihmc.gdx.ui.behaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.behaviors.RemoteBehaviorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

import java.util.HashMap;
import java.util.Map;

/**
 * This class constructs a UI for behavior operation.
 */
public class GDXBehaviorUI
{
   public static int PREFERRED_WINDOW_WIDTH = 1750;
   public static int PREFERRED_WINDOW_HEIGHT = 1000;

   private final Messager behaviorMessager;
   private final ROS2Node ros2Node;
   private final Map<String, GDXBehaviorUIInterface> behaviorUIInterfaces = new HashMap<>();
   private final Map<String, Boolean> enabledUIs = new HashMap<>();

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   public static GDXBehaviorUI createInterprocess(GDXBehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, String behaviorModuleAddress)
   {
      return create(behaviorUIRegistry, robotModel, CommunicationMode.INTERPROCESS, CommunicationMode.INTERPROCESS, behaviorModuleAddress, null);
   }

   public static GDXBehaviorUI createIntraprocess(GDXBehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, Messager behaviorSharedMemoryMessager)
   {
      return create(behaviorUIRegistry, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTRAPROCESS, null, behaviorSharedMemoryMessager);
   }

   public static GDXBehaviorUI create(GDXBehaviorUIRegistry behaviorUIRegistry,
                                      DRCRobotModel robotModel,
                                      CommunicationMode ros2CommunicationMode,
                                      CommunicationMode messagerCommunicationMode,
                                      String behaviorModuleAddress,
                                      Messager behaviorSharedMemoryMessager)
   {

      if (messagerCommunicationMode == CommunicationMode.INTRAPROCESS && behaviorSharedMemoryMessager == null)
         throw new RuntimeException("Must pass shared Messager for Messager intraprocess mode.");
      else if (messagerCommunicationMode == CommunicationMode.INTERPROCESS && behaviorModuleAddress == null)
         throw new RuntimeException("Must pass address for Messager interprocess mode.");

      Messager messager = messagerCommunicationMode == CommunicationMode.INTRAPROCESS
            ? behaviorSharedMemoryMessager : RemoteBehaviorInterface.createForUI(behaviorUIRegistry, behaviorModuleAddress);
      return new GDXBehaviorUI(behaviorUIRegistry, messager, robotModel, ros2CommunicationMode.getPubSubImplementation());
   }

   public GDXBehaviorUI(GDXBehaviorUIRegistry behaviorUIRegistry, Messager behaviorMessager, DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this.behaviorMessager = behaviorMessager;

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "behavior_ui");

   }

   public void create(GDX3DSceneManager sceneManager)
   {

   }

   public void renderImGuiWindows(GDX3DSceneManager sceneManager)
   {

   }

   public void closeMessager()
   {
      ExceptionTools.handle(behaviorMessager::closeMessager, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public static void claimEditing(Object claimingEditor)
   {
      if (GDXBehaviorUI.ACTIVE_EDITOR != null)
      {
         throw new RuntimeException("Only one editor may be active at a time.");
      }
      else
      {
         GDXBehaviorUI.ACTIVE_EDITOR = claimingEditor;
         LogTools.debug("editor activated: {}", claimingEditor.getClass().getSimpleName());
      }
   }

   public void destroy()
   {
      for (GDXBehaviorUIInterface behaviorUIInterface : behaviorUIInterfaces.values())
      {
         behaviorUIInterface.destroy();
      }
      ros2Node.destroy();
   }
}
