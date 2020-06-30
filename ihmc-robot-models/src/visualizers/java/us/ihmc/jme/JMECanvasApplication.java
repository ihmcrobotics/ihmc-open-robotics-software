package us.ihmc.jme;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetConfig;
import com.jme3.asset.TextureKey;
import com.jme3.light.AmbientLight;
import com.jme3.light.PointLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.opengl.GLRenderer;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.lwjgl.LwjglContext;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.SkyFactory;
import javafx.scene.paint.Color;
import jme3dae.ColladaLoader;
import jme3dae.collada14.ColladaDocumentV14;
import jme3dae.materials.FXBumpMaterialGenerator;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.jMonkeyEngineToolkit.stlLoader.STLLoader;

import java.awt.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.util.logging.Logger.getLogger;

public class JMECanvasApplication
{
   private static final boolean DISABLE_LOGGING = false;
   private final java.util.logging.Logger[] jmeLoggers = new java.util.logging.Logger[] {getLogger(FXBumpMaterialGenerator.class.getName()),
                                                                                         getLogger(ColladaDocumentV14.class.getName()),
                                                                                         getLogger(GLRenderer.class.getName()),
                                                                                         getLogger(AssetConfig.class.getName()),
                                                                                         getLogger(JmeSystem.class.getName()),
                                                                                         getLogger(LwjglContext.class.getName())};

   private final FunctionalSimpleApplication jme = new FunctionalSimpleApplication();

   private Node zUpNode;
   private FocusBasedJMECamera customCamera;
   private ViewPort customViewport;

   public JMECanvasApplication()
   {
      if (DISABLE_LOGGING)
      {
         for (Logger jmeLogger : jmeLoggers)
         {
            jmeLogger.setLevel(Level.SEVERE);
         }
      }

      AppSettings appSettings = new AppSettings(true);
      appSettings.setAudioRenderer(null);
      appSettings.setResolution(1100, 800);
      appSettings.setVSync(true);
      appSettings.setSamples(4);

      jme.setSimpleInitApp(this::simpleInitApp);
      jme.setInitialize(this::initialize);
      jme.setSimpleUpdate(this::simpleUpdate);
      jme.setPauseOnLostFocus(false);
      jme.setShowSettings(false);
      jme.setSettings(appSettings);
      jme.setDisplayFps(false);
      jme.setDisplayStatView(false);

      jme.createCanvas();
   }

   private void simpleInitApp()
   {
      Thread.currentThread().setUncaughtExceptionHandler(null);
      Thread.setDefaultUncaughtExceptionHandler(null);

      jme.getAssetManager().registerLoader(AWTLoader.class, "tif");
      jme.getAssetManager().registerLoader(ColladaLoader.class, "dae");
      jme.getAssetManager().registerLoader(STLLoader.class, "stl");
      jme.getAssetManager().registerLoader(MaterialLoader.class, "material");

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      jme.getRootNode().attachChild(zUpNode);

      jme.getFlyByCamera().setEnabled(false);
      jme.getViewPort().setEnabled(false);
      jme.getRenderManager().removeMainView(jme.getViewPort());
      jme.getRenderManager().removeMainView(jme.getGuiViewPort());

      customCamera = new FocusBasedJMECamera(1100, 800, jme.getInputManager(), jme.getAssetManager());
      zUpNode.attachChild(customCamera.getFocusPointSphere());

      customViewport = jme.getRenderManager().createMainView("JMEViewport", customCamera);
      customViewport.attachScene(jme.getRootNode());
      customViewport.setClearFlags(true, true, true);
      customViewport.setBackgroundColor(new ColorRGBA(0.5019608f, 0.5019608f, 0.5019608f, 1.0f));

      RenderState renderState = new RenderState();
      renderState.setDepthWrite(false);
      renderState.setDepthFunc(RenderState.TestFunction.Equal);
      jme.getRenderer().applyRenderState(renderState);
   }

   private void initialize()
   {

   }

   private void simpleUpdate(float tpf)
   {
      customCamera.simpleUpdate(tpf);
   }

   public Canvas getCanvas()
   {
      return ((JmeCanvasContext) jme.getContext()).getCanvas();
   }

   public SimpleApplication getJME()
   {
      return jme;
   }

   public Node getZUpNode()
   {
      return zUpNode;
   }

   public void enqueue(Runnable runnable)
   {
      jme.enqueue(runnable);
   }

   public void createCoordinateFrame(double length)
   {
      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;

      JMEMultiColorMeshBuilder meshBuilder = new JMEMultiColorMeshBuilder();
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), javafx.scene.paint.Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), javafx.scene.paint.Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), javafx.scene.paint.Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), javafx.scene.paint.Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), javafx.scene.paint.Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      Mesh mesh = meshBuilder.generateMesh();

      Geometry geometry = new Geometry("g1", mesh);
      geometry.setMaterial(meshBuilder.generateMaterial(jme.getAssetManager()));

      zUpNode.attachChild(geometry);
   }

   /**
    * Lighting does not work correctly until you call this or another createSky method.
    *
    * @param color constant sky background color
    */
   public void setupConstantColorSky(Color color)
   {
      customViewport.setBackgroundColor(JMEColorConversions.toJMEColor(color));

      double radius = 1000.0;
      Sphere sphere = new Sphere(10, 10, (float) radius, false, false);
      Geometry geometry = new Geometry("meep2", sphere);
      Material material = new Material(jme.getAssetManager(), "Common/MatDefs/Misc/Sky.j3md");
      geometry.setMaterial(material);
      zUpNode.attachChild(geometry);
   }

   public void setupRealisticSky()
   {
      String west = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0005.bmp";
      String east = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0002.bmp";
      String north = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0001.bmp";
      String south = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0004.bmp";
      String up = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0003.bmp";
      String down = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0007.bmp";
      Texture westTex = jme.getAssetManager().loadTexture(new TextureKey(west, true));
      Texture eastTex = jme.getAssetManager().loadTexture(new TextureKey(east, true));
      Texture northTex = jme.getAssetManager().loadTexture(new TextureKey(north, true));
      Texture southTex = jme.getAssetManager().loadTexture(new TextureKey(south, true));
      Texture upTex = jme.getAssetManager().loadTexture(new TextureKey(up, true));
      Texture downTex = jme.getAssetManager().loadTexture(new TextureKey(down, true));
      Spatial sky = SkyFactory.createSky(jme.getAssetManager(), westTex, eastTex, northTex, southTex, upTex, downTex);
      sky.setLocalScale(1000);
      zUpNode.attachChild(sky);
   }

   public void setupPointLighting()
   {
      float ambientValue = 0.7f;
      float pointValue = 0.2f;
      float pointDistance = 1000.0f;

      ColorRGBA ambientColor = new ColorRGBA(ambientValue, ambientValue, ambientValue, 1.0f);
      zUpNode.addLight(new AmbientLight(ambientColor));
      ColorRGBA indoorColor = new ColorRGBA(pointValue, pointValue, pointValue, 1.0f);

      // order seems to be odd: y, z, x (but only when sky added)
      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, pointDistance), indoorColor));
      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, pointDistance), indoorColor));
      zUpNode.addLight(new PointLight(new Vector3f(-pointDistance, pointDistance, -pointDistance), indoorColor));
      zUpNode.addLight(new PointLight(new Vector3f(pointDistance, pointDistance, -pointDistance), indoorColor));
   }

   public void setupDirectionalLighting()
   {

   }
}