package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.asset.AssetConfig;
import com.jme3.asset.TextureKey;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.opengl.GLRenderer;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.lwjgl.LwjglContext;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.SkyFactory;
import jme3dae.ColladaLoader;
import jme3dae.collada14.ColladaDocumentV14;
import jme3dae.materials.FXBumpMaterialGenerator;
import jme3tools.optimize.GeometryBatchFactory;
import us.ihmc.jMonkeyEngineToolkit.jme.JMECamera;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.jMonkeyEngineToolkit.stlLoader.STLLoader;
import us.ihmc.log.LogTools;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.util.logging.Logger.*;

public class JMEInSwingWindowEnvironment
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
   private Node floor;
   private DirectionalLight primaryLight;
   private AmbientLight ambientLight;
   private ArrayList<DirectionalLight> lights = new ArrayList<>();

   public JMEInSwingWindowEnvironment()
   {
      if (DISABLE_LOGGING)
      {
         for (Logger jmeLogger : jmeLoggers)
         {
            jmeLogger.setLevel(Level.SEVERE);
         }
      }

      AppSettings appSettings = new AppSettings(true);

//      appSettings.setCustomRenderer(PBOAwtPanelsContext.class); //??

      appSettings.setAudioRenderer(null);
      appSettings.setResolution(1100, 800);
      appSettings.setVSync(true);

      jme.setSimpleInitApp(this::simpleInitApp);
      jme.setInitialize(this::initialize);
      jme.setPauseOnLostFocus(false);
      jme.setShowSettings(false);
      jme.setSettings(appSettings);
      jme.setDisplayFps(true);
      jme.setDisplayStatView(false);

      jme.createCanvas();


//      jmeCanvasContext.getCanvas().setPreferredSize(new Dimension(1600, 600));

      // AWTPanelsContextManager ?

      jme.startCanvas();
//      jme.start();


   }

   private void simpleInitApp()
   {
      LogTools.info("Hello JME");

      Thread.currentThread().setUncaughtExceptionHandler(null);
      Thread.setDefaultUncaughtExceptionHandler(null);

      jme.getAssetManager().registerLoader(AWTLoader.class, "tif");
      jme.getAssetManager().registerLoader(ColladaLoader.class, "dae");
      jme.getAssetManager().registerLoader(STLLoader.class, "stl");
      jme.getAssetManager().registerLoader(MaterialLoader.class, "material");

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      jme.getRootNode().attachChild(zUpNode);

      setupLighting();


      jme.getFlyByCamera().setEnabled(false);
      jme.getViewPort().setEnabled(false);
      jme.getRenderManager().removeMainView(jme.getViewPort());
      jme.getRenderManager().removeMainView(jme.getGuiViewPort());

      CustomJMECamera customCamera = new CustomJMECamera(1100, 800);
//      JMECamera customCamera = new JMECamera(1100, 800);

//      Camera customCamera = new Camera(1100, 800);
//      customCamera.setFrustumPerspective(45f, (float)customCamera.getWidth() / customCamera.getHeight(), 1f, 1000f);
//      customCamera.setLocation(new Vector3f(0f, 0f, 10f));
//      customCamera.lookAt(new Vector3f(0f, 0f, 0f), Vector3f.UNIT_Y);

      ViewPort customViewport = jme.getRenderManager().createMainView("JMEViewport", customCamera);
      customViewport.attachScene(jme.getRootNode());
      customViewport.setClearFlags(true, true, true);

      setUpGrid();
      setupSky();

      JFrame frame = new JFrame("JME");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Container contentPane = frame.getContentPane();
      contentPane.setLayout(new BorderLayout());

      JPanel centerPanel = new JPanel(new BorderLayout());
      centerPanel.setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
      centerPanel.setPreferredSize(new Dimension(1100, 800));
      centerPanel.add(((JmeCanvasContext) jme.getContext()).getCanvas(), BorderLayout.CENTER);
      contentPane.add(centerPanel, BorderLayout.CENTER);

      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);

//      jme.getContext()
   }

   private void initialize()
   {

   }

   private void setupSky()
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
      jme.getRootNode().attachChild(sky);
   }

   private void setupLighting()
   {
      // Note: Something gets transformed, so that z is x, and y is z, and x is y.

      Vector3f fromTheTop = new Vector3f(0.0f, -1.0f, 0.0f).normalizeLocal();
      Vector3f fromTheFront = new Vector3f(0.0f, 0.0f, -1.0f).normalizeLocal();
      Vector3f fromTheSide = new Vector3f(-1.0f, 0.0f, 0.0f).normalizeLocal();

      Vector3f fromTheBack = new Vector3f(fromTheFront).negate().normalizeLocal();
      Vector3f fromTheOtherSide = new Vector3f(fromTheSide).negate().normalizeLocal();
      Vector3f fromTheBottom = new Vector3f(fromTheTop).negate().normalizeLocal();

      Vector3f fromTheTopFront = fromTheTop.add(fromTheFront).normalizeLocal();
      Vector3f fromTheFrontSide = fromTheFront.add(fromTheSide).normalizeLocal();
      Vector3f fromTheBackOtherSide = fromTheBack.add(fromTheOtherSide).normalizeLocal();
      Vector3f fromTheBackBottom = fromTheBack.add(fromTheBottom).normalizeLocal();

      primaryLight = new DirectionalLight();
      primaryLight.setColor(ColorRGBA.White.mult(0.5f));
      primaryLight.setDirection(fromTheTopFront);
      jme.getRootNode().addLight(primaryLight);
      lights.add(primaryLight);

      // TODO: Ambient light could be brighter, but OBJ model files seem to just get white in ambient light, rather than their material.
      ambientLight = new AmbientLight();
      ambientLight.setColor(ColorRGBA.White.mult(0.2f));
      jme.getRootNode().addLight(ambientLight);

      addDirectionalLight(ColorRGBA.White.mult(0.35f), fromTheFrontSide);
      addDirectionalLight(ColorRGBA.White.mult(0.3f), fromTheBackOtherSide);
      addDirectionalLight(ColorRGBA.White.mult(0.28f), fromTheBackBottom);
      addDirectionalLight(ColorRGBA.White.mult(0.32f), fromTheOtherSide);
      addDirectionalLight(ColorRGBA.White.mult(0.35f), fromTheSide);

      jme.getRenderManager().setPreferredLightMode(TechniqueDef.LightMode.SinglePass);

      jme.getRootNode().setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
      zUpNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
   }

   private void addDirectionalLight(ColorRGBA color, Vector3f direction)
   {

      if (lights.isEmpty())
      {
         primaryLight.setColor(color);
         primaryLight.setDirection(direction);
         lights.add(primaryLight);
         jme.getRootNode().addLight(primaryLight);
      }
      else
      {
         DirectionalLight light = new DirectionalLight();
         light.setColor(color);
         light.setDirection(direction.normalizeLocal());
         lights.add(light);
         jme.getRootNode().addLight(light);
      }
   }

   private void setUpGrid()
   {
      Node grid = new Node();
      float size = 200;
      float height = 0.0025f;
      float width = 0.02f;

      // draw base Axis Lines
      for (int x = new Float(-size / 2).intValue(); x < new Float(size / 2).intValue(); x++)
      {
         Vector3f centerBox2 = new Vector3f(x, 0, -height - 0.01f);
         Box box2 = new Box(0.0f, 0.0f, 0.0f);
         box2.updateGeometry(centerBox2, width, size / 2, height);

         Geometry line = new Geometry("Box2", box2);
         Material mat2 = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
         if (x == 0)
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.DarkGray);
         }
         else
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.Gray);
         }

         line.setShadowMode(RenderQueue.ShadowMode.Off);
         line.setMaterial(mat2);
         grid.attachChild(line);
      }

      for (float y = new Float(-size / 2).intValue(); y < new Float(size / 2).intValue(); y++)
      {
         Vector3f centerBox2 = new Vector3f(0, y, -height - 0.01f);
         Box box2 = new Box(0.0f, 0.0f, 0.0f);
         box2.updateGeometry(centerBox2, size / 2, width, height);

         Geometry line = new Geometry("Box2", box2);
         Material mat2 = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
         if (y == 0)
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.DarkGray);
         }
         else
         {
            mat2.setBoolean("UseMaterialColors", true);

            mat2.setColor("Diffuse", ColorRGBA.Gray);
         }

         line.setMaterial(mat2);
         line.setShadowMode(RenderQueue.ShadowMode.Off);
         grid.attachChild(line);
      }

      GeometryBatchFactory.optimize(grid, true);
      zUpNode.attachChild(grid);
   }

   private void makeFloor()
   {
      Vector3f center = new Vector3f(0.0f, 0.0f, -0.25f);
      Box quad = new Box(0.0f, 0.0f, 0.0f);
      quad.updateGeometry(center, 100.0f, 100.0f, 0.25f);
      floor = new Node("floor Nodes");
      Node floorObject = new Node("floor object");
      Geometry floorGeometry = new Geometry("level-Floor", quad);
      floorGeometry.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_Z));

      Material mat = new Material(jme.getAssetManager(), "Common/MatDefs/Light/Lighting.j3md");
      mat.setBoolean("UseMaterialColors", true);

      mat.setColor("Diffuse", new ColorRGBA(1.0f, 1.0f, 1.0f, 0.0f));

      floorGeometry.setShadowMode(RenderQueue.ShadowMode.Receive);

      mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      floorGeometry.setQueueBucket(RenderQueue.Bucket.Transparent);
      floorGeometry.setMaterial(mat);
      floorObject.attachChild(floorGeometry);
      floor.attachChild(floorObject);

//      attachFloor(floor);
   }

   public static void main(String[] args)
   {
      new JMEInSwingWindowEnvironment();
   }
}
