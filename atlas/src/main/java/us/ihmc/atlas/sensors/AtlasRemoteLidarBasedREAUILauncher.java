package us.ihmc.atlas.sensors;

import com.esotericsoftware.minlog.Log;
import javafx.stage.Stage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;

import static us.ihmc.commons.exception.DefaultExceptionHandler.RUNTIME_EXCEPTION;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.getPrivateNetClassList;

public class AtlasRemoteLidarBasedREAUILauncher
{
   public AtlasRemoteLidarBasedREAUILauncher()
   {

      Log.TRACE();

      JavaFXApplicationCreator.buildJavaFXApplication(this::start);
   }

   public void start(Stage primaryStage)
   {
//      KryoMessager messager = KryoMessager.createTCPClient(REAModuleAPI.API, "poweredge", NetworkPorts.REA_MODULE_UI_PORT, getPrivateNetClassList());
//      ExceptionTools.handle(messager::startMessager, RUNTIME_EXCEPTION);

      KryoMessager messager = KryoMessager.createClient(REAModuleAPI.API, "poweredge", NetworkPorts.REA_MODULE_UI_PORT.getPort(), "KryoREAUI", 1);
      messager.startMessagerBlocking();

      REAUIMessager uiMessager = new REAUIMessager(messager);

      LIDARBasedEnvironmentAwarenessUI remoteUI = ExceptionTools.handle(() -> new LIDARBasedEnvironmentAwarenessUI(uiMessager, primaryStage, false),
                                                                        DefaultExceptionHandler.RUNTIME_EXCEPTION);

//      LIDARBasedEnvironmentAwarenessUI remoteUI = ExceptionTools.handle(() -> LIDARBasedEnvironmentAwarenessUI.creatRemoteUI(primaryStage, "poweredge"),
//                                                                        DefaultExceptionHandler.RUNTIME_EXCEPTION);
      remoteUI.show();
   }

   public static void main(String[] args)
   {
      new AtlasRemoteLidarBasedREAUILauncher();
   }
}
