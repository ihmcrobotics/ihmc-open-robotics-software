package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.asset.AssetConfig;
import com.jme3.renderer.opengl.GLRenderer;
import com.jme3.scene.Node;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeSystem;
import com.jme3.system.lwjgl.LwjglContext;
import com.jme3.texture.plugins.AWTLoader;
import jme3dae.ColladaLoader;
import jme3dae.collada14.ColladaDocumentV14;
import jme3dae.materials.FXBumpMaterialGenerator;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelsContext;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.jMonkeyEngineToolkit.stlLoader.STLLoader;
import us.ihmc.log.LogTools;

import java.util.logging.Level;
import java.util.logging.Logger;

import static java.util.logging.Logger.*;

public class JMEFromScratchEnvironment
{
   private final java.util.logging.Logger[] jmeLoggers = new java.util.logging.Logger[] {getLogger(FXBumpMaterialGenerator.class.getName()),
                                                                                         getLogger(ColladaDocumentV14.class.getName()),
                                                                                         getLogger(GLRenderer.class.getName()),
                                                                                         getLogger(AssetConfig.class.getName()),
                                                                                         getLogger(JmeSystem.class.getName()),
                                                                                         getLogger(LwjglContext.class.getName())};

   private final FunctionalSimpleApplication jme = new FunctionalSimpleApplication();

   private Node zUpNode;

   public JMEFromScratchEnvironment()
   {
      for (Logger jmeLogger : jmeLoggers)
      {
         jmeLogger.setLevel(Level.SEVERE);
      }

      AppSettings appSettings = new AppSettings(true);

      appSettings.setCustomRenderer(PBOAwtPanelsContext.class); //??

      appSettings.setAudioRenderer(null);
      appSettings.setResolution(1100, 800);
      appSettings.setVSync(true);

      jme.setSimpleInitApp(this::simpleInitApp);
      jme.setPauseOnLostFocus(false);
      jme.setShowSettings(false);
      jme.setSettings(appSettings);
      jme.setDisplayFps(true);
      jme.setDisplayStatView(false);

      // AWTPanelsContextManager ?

      jme.start();
   }

   private void simpleInitApp()
   {
      LogTools.info("Hello JME");

//      Thread.currentThread().setUncaughtExceptionHandler(null);
//      Thread.setDefaultUncaughtExceptionHandler(null);

      jme.getAssetManager().registerLoader(AWTLoader.class, "tif");
      jme.getAssetManager().registerLoader(ColladaLoader.class, "dae");
      jme.getAssetManager().registerLoader(STLLoader.class, "stl");
      jme.getAssetManager().registerLoader(MaterialLoader.class, "material");

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      jme.getRootNode().attachChild(zUpNode);
   }

   public static void main(String[] args)
   {
      new JMEFromScratchEnvironment();
   }
}
