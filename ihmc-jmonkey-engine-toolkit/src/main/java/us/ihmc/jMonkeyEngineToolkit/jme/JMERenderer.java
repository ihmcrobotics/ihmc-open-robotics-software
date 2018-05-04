package us.ihmc.jMonkeyEngineToolkit.jme;

import java.applet.Applet;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GraphicsDevice;
import java.awt.Window;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import java.net.URL;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.swing.JComponent;
import javax.swing.RepaintManager;

import com.google.common.collect.HashBiMap;
import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetConfig;
import com.jme3.asset.AssetManager;
import com.jme3.asset.TextureKey;
import com.jme3.audio.AudioContext;
import com.jme3.input.InputManager;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.light.Light;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.profile.AppStep;
import com.jme3.renderer.opengl.GLRenderer;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.ogre.MaterialLoader;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.system.awt.AwtPanelsContext;
import com.jme3.system.lwjgl.LwjglContext;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.SkyFactory;
import com.jme3.util.SkyFactory.EnvMapType;

import jme3dae.ColladaLoader;
import jme3dae.collada14.ColladaDocumentV14;
import jme3dae.materials.FXBumpMaterialGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DSpotLight;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListenerHolder;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DInterface;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListenerHolder;
import us.ihmc.graphicsDescription.input.mouse.MouseListener;
import us.ihmc.graphicsDescription.input.mouse.MouseListenerHolder;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.GPULidarListener;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DBackgroundScaleMode;
import us.ihmc.jMonkeyEngineToolkit.Updatable;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.input.SelectedListenerHolder;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEViewportAdapter.ViewportType;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanel;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelListener;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelsContext;
import us.ihmc.jMonkeyEngineToolkit.jme.contextManager.AWTPanelsContextManager;
import us.ihmc.jMonkeyEngineToolkit.jme.contextManager.CanvasContextManager;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.JMEGPULidar;
import us.ihmc.jMonkeyEngineToolkit.jme.terrain.JMEHeightMapTerrain;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMENodeTools;
import us.ihmc.jMonkeyEngineToolkit.stlLoader.STLLoader;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class JMERenderer extends SimpleApplication implements Graphics3DAdapter, PBOAwtPanelListener
{
   /**
    * Some bullshit because JME is flooding the JAVA logger and has no option to deactivate it rather than changing the logger level.
    * It is not desirable to change the level the root logger though, it'll prevent other loggers to display useful information.
    * The annoyance here is to find all JME loggers to keep a reference to each of them so any changes will remain permanent.
    */
   private final Logger[] jmeLoggers = new Logger[] {Logger.getLogger(FXBumpMaterialGenerator.class.getName()),
         Logger.getLogger(ColladaDocumentV14.class.getName()), Logger.getLogger(GLRenderer.class.getName()), Logger.getLogger(AssetConfig.class.getName()),
         Logger.getLogger(JmeSystem.class.getName()), Logger.getLogger(LwjglContext.class.getName())};

   public enum RenderType
   {
      CANVAS, AWTPANELS
   }
   
   public enum SkyboxToUse
   {
      JME_MOUNTAINS, BLUE_SKY, DUSK_SKY;
   }
   
   public static final SkyboxToUse skyboxToUse = SkyboxToUse.BLUE_SKY;

   public final static boolean USE_PBO = true;
   public static boolean USE_GPU_LIDAR_PARALLEL_SCENE = false; // Disable: this is super slow
   public final static boolean DEBUG_GPU_LIDAR_PARALLEL_SCENE = false;
   private final RenderType renderType;
   public static boolean tickUpdated = false;

   private final Object repaintNotifier = new Object(); // If we aren't rendering at a continuous FPS, this condition is used for waking up the renderer if a repaint is required
   private boolean lazyRendering = true; // If lazy rendering is true, we render only when window repaint is needed
   private int lazyRendersToPerform = 10; // How many render loops to do after a lazy render wake-up call - some events such as resize or startup need multiple passes to properly reinstate the image

   static
   {
      Path scsCachePath = PathTools.systemTemporaryDirectory().resolve("SCSCache");

      FileTools.ensureDirectoryExists(scsCachePath, DefaultExceptionHandler.PRINT_STACKTRACE);

      System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + scsCachePath.toString());
      NativeLibraryLoader.setCustomExtractionFolder(scsCachePath.toString());
   }

   private final Object loadingStatus = new Object();
   private final Object graphicsConch = new Object();

   private JMEContextManager contextManager;

   private ArrayList<JMEViewportAdapter> viewportAdapters = new ArrayList<JMEViewportAdapter>();

   private HashBiMap<Graphics3DNode, JMEGraphics3DNode> jmeGraphicsNodes = HashBiMap.create();
   private Collection<JMEGraphics3DNode> jmeGraphicsNodesListView = jmeGraphicsNodes.values();
   
   private HashBiMap<Graphics3DSpotLight, JMESpotLight> jmeSpotLights = HashBiMap.create();
   private Collection<JMESpotLight> jmeSpotLightsView = jmeSpotLights.values();

   private boolean isTerrainVisible = true;

   private Mouse3DInterface mouse3DJoystick;

   private SelectedListenerHolder selectedListenerHolder = new SelectedListenerHolder();
   private KeyListenerHolder keyListenerHolder = new KeyListenerHolder();
   private MouseListenerHolder mouseListenerHolder = new MouseListenerHolder();
   private Mouse3DListenerHolder mouse3DListenerHolder = new Mouse3DListenerHolder();

   private Node rootJoint;
   private Node terrain;
   private Node zUpNode;

   private ArrayList<DirectionalLight> lights = new ArrayList<>(); 
   private ArrayList<JMEGPULidar> gpuLidars = new ArrayList<>();
   private ArrayList<PBOAwtPanel> pboAwtPanels;
   private DirectionalLight primaryLight;
   private CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private ArrayList<Updatable> updatables = new ArrayList<Updatable>(); // things we want to move automatically

   private Spatial sky = null;
   private HeightMap heightMap = null;
   private AppearanceDefinition terrainAppearance = null;
   
   public JMERenderer(RenderType renderType)
   {
      this(renderType, null);
   }
   
   public JMERenderer(RenderType renderType, Mouse3DInterface mouse3dJoystick)
   {
      super();
      this.renderType = renderType;
      this.mouse3DJoystick = mouse3dJoystick;

      changeJMELoggerLevelToSevere();

      if (renderType == RenderType.AWTPANELS)
      {
         initializeAWTPanels();
      }
      else
      {
         initializeCanvas();
      }

      synchronized (loadingStatus)
      {
         try
         {
            loadingStatus.wait();
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException("Loading interrupted");
         }
      }
      

   }

   private void changeJMELoggerLevelToSevere()
   {
      for (Logger jmeLogger : jmeLoggers)
         jmeLogger.setLevel(Level.SEVERE);
   }

   private void notifyRepaint(int rendersToPerform)
   {
      synchronized (repaintNotifier)
      {
         // Multiply by 2 because of double buffering - we want to repaint both buffers
         lazyRendersToPerform = Math.max(lazyRendersToPerform, rendersToPerform * 2);
         repaintNotifier.notifyAll();
      }
   }

   private void notifyRepaint()
   {
      notifyRepaint(1);
   }

   @Override
   @Deprecated
   public Node getRootNode()
   {
      // should be using zUpNode now so that there is no axis flip flopping going on.
      return super.getRootNode();
   }

   public JMEGraphics3DNode addNodesRecursively(Graphics3DNode graphics3dNode, Node parentNode)
   {
      synchronized (graphicsConch)
      {
         JMEGraphics3DNode jmeNode = new JMEGraphics3DNode(graphics3dNode, getAssetManager(), this, closeableAndDisposableRegistry);

         if (rootJoint == null)
         {
            rootJoint = jmeNode;
         }

         Graphics3DNodeType nodeType = graphics3dNode.getNodeType();

         jmeNode.setType(nodeType);

         jmeGraphicsNodes.put(graphics3dNode, jmeNode);
         parentNode.attachChild(jmeNode);

         for (Graphics3DNode child : graphics3dNode.getChildrenNodes())
         {
            addNodesRecursively(child, jmeNode);
         }

         notifyRepaint();

         return jmeNode;
      }
   }

   @Override
   public void addRootNode(final Graphics3DNode rootNode)
   {
      enqueue(new Callable<JMEGraphics3DNode>()
      {
         @Override
         public JMEGraphics3DNode call() throws Exception
         {
            synchronized (graphicsConch)
            {
               JMEGraphics3DNode node = addNodesRecursively(rootNode, zUpNode);
               jmeGraphicsNodes.put(rootNode, node);

               return node;
            }
         }
      });
      notifyRepaint();
   }

   public void registerViewport(JMEViewportAdapter viewportAdapter)
   {
      viewportAdapters.add(viewportAdapter);
   }

   @Override
   public ViewportAdapter createNewViewport(GraphicsDevice graphicsDevice, boolean isMainViewport, boolean isOffScreen)
   {
      if (isMainViewport)
      {
         for (JMEViewportAdapter jmeViewportAdapter : viewportAdapters)
         {
            if (jmeViewportAdapter.isMainViewport())
            {
               throw new RuntimeException("Can only have one main viewport");
            }
         }
      }

      JMEViewportAdapter newViewport = new JMEViewportAdapter(this, rootNode, isMainViewport, isOffScreen ? ViewportType.OFFSCREEN : ViewportType.CANVAS, false,
            Color.LIGHT_GRAY, false);
      notifyRepaint();

      return newViewport;
   }

   @Override
   public void closeViewport(ViewportAdapter viewport)
   {
      ((JMEViewportAdapter) viewport).closeViewportAdapter();
      while (viewportAdapters.remove(viewport))
      {
      }

      notifyRepaint();
   }

   @Override
   public Object getGraphicsConch()
   {
      return graphicsConch;
   }

   public Canvas getCanvas()
   {
      if (renderType != RenderType.CANVAS)
      {
         throw new RuntimeException("Cannot get canvas if rendertype not is canvas");
      }

      JmeCanvasContext ctx = (JmeCanvasContext) getContext();

      return ctx.getCanvas();
   }

   private void initializeCanvas()
   {
      AppSettings appSettings = new AppSettings(true);
      appSettings.setWidth(1200);
      appSettings.setHeight(600);
      appSettings.setAudioRenderer(null);
      appSettings.setVSync(true);
      setSettings(appSettings);

      setPauseOnLostFocus(false);

      createCanvas();
      JmeCanvasContext ctx = (JmeCanvasContext) getContext();
      ctx.setSystemListener(this);
      Dimension dim = new Dimension(1200, 600);
      ctx.getCanvas().setPreferredSize(dim);

      contextManager = new CanvasContextManager(this);
      addRepaintListeners(ctx.getCanvas());

      startCanvas();
   }

   private void initializeAWTPanels()
   {
      AppSettings appSettings = new AppSettings(true);

      if (USE_PBO)
      {
         appSettings.setCustomRenderer(PBOAwtPanelsContext.class);
      }
      else
      {
         appSettings.setCustomRenderer(AwtPanelsContext.class);
      }

      /**
       * Extracts natives due bug described in
       * http://jmonkeyengine.org/forum/topic/setting-audio-renderer-to-null-results-in-a-failure-to-load-lwjgl-native-library
       *
       * Remove when loading native libraries is fixed by JMonkeyEngine upstream
       */

      /*
       * try { Natives.extractNativeLibs(JmeSystem.getPlatform(), appSettings);
       * } catch (IOException e) { throw new
       * RuntimeException("Cannot load native libs"); }
       */
      appSettings.setAudioRenderer(null);
      appSettings.setFrameRate(30);

      setSettings(appSettings);

      setShowSettings(false);
      setPauseOnLostFocus(false);

      contextManager = new AWTPanelsContextManager(this);

      start();

      if (getContext() instanceof PBOAwtPanelsContext)
      {
         if (DEBUG_GPU_LIDAR_PARALLEL_SCENE)
         {
            System.out.println(PrintTools.DEBUG + "Adding " + PBOAwtPanelsContext.class.getSimpleName() + " listener.");
         }
         ((PBOAwtPanelsContext) getContext()).addPBOAwtPanelListener(this);
         pboAwtPanels = ((PBOAwtPanelsContext) getContext()).getPanelList();
      }
      else
      {
         System.out.println(PrintTools.DEBUG + "Context is not of type " + PBOAwtPanelsContext.class.getSimpleName());
      }
   }

   private void recursivelyRemoveNodesFromMap(Graphics3DNode rootNode)
   {
      synchronized (graphicsConch)
      {
         for (Graphics3DNode child : rootNode.getChildrenNodes())
         {
            recursivelyRemoveNodesFromMap(child);
         }

         jmeGraphicsNodes.remove(rootNode);
      }
   }

   @Override
   public void removeRootNode(final Graphics3DNode rootNode)
   {
      enqueue(new Callable<JMEGraphics3DNode>()
      {
         @Override
         public JMEGraphics3DNode call() throws Exception
         {
            synchronized (graphicsConch)
            {
               JMEGraphics3DNode node = jmeGraphicsNodes.get(rootNode);
               if (node != null)
               {
                  node.removeFromParent();
                  recursivelyRemoveNodesFromMap(rootNode);
               }

               notifyRepaint();

               return node;
            }
         }
      });
   }

   public static void setupAssetManger(AssetManager assetManager)
   {
      assetManager.registerLoader(AWTLoader.class, "tif");
      assetManager.registerLoader(ColladaLoader.class, "dae");
      assetManager.registerLoader(STLLoader.class, "stl");
      assetManager.registerLoader(MaterialLoader.class, "material");
   }

   private void disableMainViewport()
   {
      flyCam.setEnabled(false);
      flyCam = null;
      viewPort.setEnabled(false);
      renderManager.removeMainView(viewPort);
      renderManager.removeMainView(guiViewPort);
   }

   private AmbientLight ambientLight;

   public void setAmbientLightBrightness(float brightness)
   {
      ambientLight.setColor(ColorRGBA.White.mult(brightness));
      notifyRepaint();
   }
   
   @Override
   public void setAmbientLight(Color ambient)
   {
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {

            ColorRGBA jmeAmbient = JMEDataTypeUtils.colorToColorRGBA(ambient);
            ambientLight.setColor(jmeAmbient);
            notifyRepaint();
            return null;
         }
      });
   }

   private void setupLighting()
   {
      primaryLight = new DirectionalLight();
      primaryLight.setColor(ColorRGBA.White.mult(0.5f));
      primaryLight.setDirection(new Vector3f(-0.1f, -1.0f, -0.2f).normalizeLocal());
      rootNode.addLight(primaryLight);
      lights.add(primaryLight);

      ambientLight = new AmbientLight();
      ambientLight.setColor(ColorRGBA.White.mult(.8f)); //1.3f));
      rootNode.addLight(ambientLight);

      addDirectionalLight(ColorRGBA.White.mult(0.1f), new Vector3f(1.0f, -0.0f, -0.5f).normalizeLocal());

      addDirectionalLight(ColorRGBA.White.mult(0.4f), new Vector3f(0.0f, -1.0f, 0.0f).normalizeLocal());
      
      renderManager.setPreferredLightMode(TechniqueDef.LightMode.SinglePass);

      rootNode.setShadowMode(ShadowMode.CastAndReceive);
      zUpNode.setShadowMode(ShadowMode.CastAndReceive);
   }

   private void addDirectionalLight(ColorRGBA color, Vector3f direction)
   {
      
      if(lights.size() > 1)
      {
         DirectionalLight light = new DirectionalLight();
         light.setColor(color);
         light.setDirection(direction.normalizeLocal());
         lights.add(light);
         rootNode.addLight(light);         
      }
      else
      {
         primaryLight.setColor(color);
         primaryLight.setDirection(direction);
         lights.add(primaryLight);
         rootNode.addLight(primaryLight);
      }
   }


   private void deleteSky()
   {
      if(sky != null)
      {
         rootNode.detachChild(sky);
         sky = null;
      }
   }
   
   private void updateSky()
   {
      sky.setLocalScale(1000);
      rootNode.attachChild(sky);
      notifyRepaint();

   }
   
   public void setupSky(String skyBox)
   {
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            deleteSky();
            sky = SkyFactory.createSky(assetManager, skyBox, EnvMapType.CubeMap);
            updateSky();
            return null;
         }
      });
   }
   
   public void setupSky(String west, String east, String north, String south, String up, String down)
   {
      
      Texture westTex = assetManager.loadTexture(new TextureKey(west, true));
      Texture eastTex = assetManager.loadTexture(new TextureKey(east, true));
      Texture northTex = assetManager.loadTexture(new TextureKey(north, true));
      Texture southTex = assetManager.loadTexture(new TextureKey(south, true));
      Texture upTex = assetManager.loadTexture(new TextureKey(up, true));
      Texture downTex = assetManager.loadTexture(new TextureKey(down, true));
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            deleteSky();
            sky = SkyFactory.createSky(assetManager, westTex, eastTex, northTex, southTex, upTex, downTex);
       
            updateSky();
            return null;
         }
      });
   }

   
   @Override
   public void setupSky()
   {
      try
      {
         if (skyboxToUse == SkyboxToUse.BLUE_SKY)
         {
            String west = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0005.bmp";
            String east = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0002.bmp";
            String north ="Textures/Sky/Bright/skyboxsun25degtest/skyrender0001.bmp";
            String south ="Textures/Sky/Bright/skyboxsun25degtest/skyrender0004.bmp";
            String up =   "Textures/Sky/Bright/skyboxsun25degtest/skyrender0003.bmp";
            String down = "Textures/Sky/Bright/skyboxsun25degtest/skyrender0007.bmp";
            setupSky(west, east, north, south, up, down);
         }
         else if (skyboxToUse == SkyboxToUse.DUSK_SKY)
         {
            String west = "Textures/Sky/Bright/skyboxsun45deg/skyrender0005.bmp";
            String east = "Textures/Sky/Bright/skyboxsun45deg/skyrender0002.bmp";
            String north ="Textures/Sky/Bright/skyboxsun45deg/skyrender0001.bmp";
            String south ="Textures/Sky/Bright/skyboxsun45deg/skyrender0004.bmp";
            String up =   "Textures/Sky/Bright/skyboxsun45deg/skyrender0003.bmp";
            String down = "Textures/Sky/Bright/skyboxsun45deg/skyrender0006.bmp";
            setupSky(west, east, north, south, up, down);
         }
         else if (skyboxToUse == SkyboxToUse.JME_MOUNTAINS)
         {
            setupSky("Textures/Sky/Bright/BrightSky.dds");
         }

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void simpleInitApp()
   {
      setupExceptionHandling();

      setupAssetManger(assetManager);

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      rootNode.attachChild(zUpNode);

      setupLighting();
      disableMainViewport();

      contextManager.initialize();

      // setupSky();

      synchronized (loadingStatus)
      {
         loadingStatus.notify();
      }

      setDisplayFps(true);
      setDisplayStatView(false);
   }

   private void setupExceptionHandling()
   {
      Thread.currentThread().setUncaughtExceptionHandler(null);
      Thread.setDefaultUncaughtExceptionHandler(null);
   }

   int count = 0;

   @Override
   public void simpleUpdate(float tpf)
   {
      if (alreadyClosing)
         return;

      synchronized (graphicsConch)
      {
         if (alreadyClosing)
            return;

         for (JMEGraphics3DNode jmeGraphicsNode : jmeGraphicsNodesListView)
         {
            jmeGraphicsNode.update();
         }
         
         updateSpotLights();
         
         updateCameras();
      }

      if (count > 1000 && !tickUpdated)
      {
         tickUpdated = true;
      }

      count++;

      updateGraphics(tpf);
   }

   /**
    * Checks refreshFlags of the root node. If there is no need to refresh the scene,
    * no rendering is performed in lazy rendering mode.
    * @return true if the scene needs re-rendering
    */
   private boolean shouldRepaint()
   {
      // We have to use reflection because JME does not provide a getter
//      try
//      {
//         Field field = Spatial.class.getDeclaredField("refreshFlags");
//         field.setAccessible(true);
//         int refresh = field.getInt(rootNode);
//         return refresh != 0;
//      }
//      catch (Exception ex)
      {
         return true; // In case of exceptions render always
      }
   }

   /**
    * We override {@link SimpleApplication#update()} with our custom implementation that does not
    * render anything unless necessary.
    */
   @Override
   public void update()
   {
      if (alreadyClosing)
         return;

      if (prof != null)
         prof.appStep(AppStep.BeginFrame);

      applicationUpdate(); // makes sure to execute AppTasks
      if (speed == 0 || paused)
      {
         return;
      }

      float tpf = timer.getTimePerFrame() * speed;

      // update states
      if (prof != null)
         prof.appStep(AppStep.StateManagerUpdate);
      stateManager.update(tpf);

      // simple update and root node
      simpleUpdate(tpf);
      if (shouldRepaint())
         lazyRendersToPerform = Math.max(lazyRendersToPerform, 2); // Render to both front and back buffer

      if (prof != null)
         prof.appStep(AppStep.SpatialUpdate);
      rootNode.updateLogicalState(tpf);
      guiNode.updateLogicalState(tpf);

      rootNode.updateGeometricState();
      guiNode.updateGeometricState();

      // render states
      if (prof != null)
         prof.appStep(AppStep.StateManagerRender);
      stateManager.render(renderManager);

      if (prof != null)
         prof.appStep(AppStep.RenderFrame);

      // Do not render anything unless necessary
      if (!lazyRendering || lazyRendersToPerform > 0)
      {
         renderManager.render(tpf, context.isRenderable());
         lazyRendersToPerform = Math.max(0, lazyRendersToPerform - 1);
      }
      simpleRender(renderManager);
      stateManager.postRender();

      if (prof != null)
         prof.appStep(AppStep.EndFrame);
   }

   /**
    * This is copied verbatim from {@link Application#update()} because we override
    * the {@link #update()} method and this is the only way to call the original code.
    */
   public void applicationUpdate()
   {
      // Make sure the audio renderer is available to callables
      AudioContext.setAudioRenderer(audioRenderer);

      if (prof != null)
         prof.appStep(AppStep.QueuedTasks);
      runQueuedTasks();

      if (speed == 0 || paused)
         return;

      timer.update();

      if (inputEnabled)
      {
         if (prof != null)
            prof.appStep(AppStep.ProcessInput);
         inputManager.update(timer.getTimePerFrame());
      }

      if (audioRenderer != null)
      {
         if (prof != null)
            prof.appStep(AppStep.ProcessAudio);
         audioRenderer.update(timer.getTimePerFrame());
      }

      // user code here..
   }

   private void updateSpotLights()
   {
      for (JMESpotLight spotLight : jmeSpotLightsView)
      {
         spotLight.update();
      }
   }
   
   private synchronized void updateCameras()
   {
      if (alreadyClosing)
         return;

      for (JMEViewportAdapter viewportAdapter : viewportAdapters)
      {
         viewportAdapter.updateCamera();
      }
   }

   @Override
   public void setGroundVisible(final boolean isVisible)
   {
      enqueue(new Callable<Boolean>()
      {
         @Override
         public Boolean call() throws Exception
         {
            if (isVisible != isTerrainVisible)
            {
               if (isVisible)
               {
                  zUpNode.attachChild(terrain);
               }
               else
               {
                  terrain.removeFromParent();
               }

               isTerrainVisible = isVisible;
            }

            return isVisible;
         }
      });
      notifyRepaint();
   }

   private void repaintTerrain()
   {
      enqueue(new Callable<JMEHeightMapTerrain>()
      {
         @Override
         public JMEHeightMapTerrain call() throws Exception
         {
            if (terrain != null)
            {
               terrain.removeFromParent();
            }

            JMEHeightMapTerrain jmeTerrain = new JMEHeightMapTerrain(heightMap, assetManager, terrainAppearance);
            terrain = jmeTerrain.getTerrain();
            terrain.setShadowMode(ShadowMode.Receive);

            if (isTerrainVisible)
            {
               zUpNode.attachChild(terrain);
            }

            return jmeTerrain;
         }
      });
      notifyRepaint();

   }
   
   @Override
   public void setHeightMap(final HeightMap heightMap)
   {
      this.heightMap = heightMap;
      repaintTerrain();
   }

   public HashBiMap<Graphics3DNode, JMEGraphics3DNode> getJmeGraphicsNodes()
   {
      return jmeGraphicsNodes;
   }

   @Override
   public void addSelectedListener(SelectedListener selectedListener)
   {
      this.selectedListenerHolder.addSelectedListener(selectedListener);
   }

   public Node getZUpNode()
   {
      return zUpNode;
   }

   public void setTransparentNodesScaleToZero()
   {
      for (Node transparentNode : getVisualizationNodes())
      {
         if (transparentNode instanceof JMEGraphics3DNode)
         {
            ((JMEGraphics3DNode) transparentNode).getGraphics3DNode().getTransform().setScale(0);
         }
         else
         {
            transparentNode.setLocalScale(0);
         }
      }
      notifyRepaint();
   }

   public void setTransparentNodesScaleToOne()
   {
      for (Node transparentNode : getVisualizationNodes())
      {
         if (transparentNode instanceof JMEGraphics3DNode)
         {
            ((JMEGraphics3DNode) transparentNode).getGraphics3DNode().getTransform().setScale(1);
         }
         else
         {
            transparentNode.setLocalScale(1);
         }
      }
      notifyRepaint();
   }

   @Override
   public void isShowing(PBOAwtPanel pboAwtPanel)
   {
      PrintTools.info(this, "A " + pboAwtPanel.getClass().getSimpleName() + " showed on screen.");

      addRepaintListeners(pboAwtPanel);

      if (!gpuLidars.isEmpty())
      {
         updateGPULidarScenes();
      }
   }

   public void addRepaintListeners(final Component panel)
   {
      if (alreadyClosing)
         return;
      if (panel == null)
         return;

      panel.addMouseListener(new MouseAdapter()
      {
         @Override
         public void mousePressed(MouseEvent e)
         {
            notifyRepaint();
         }
      });

      panel.addMouseMotionListener(new MouseAdapter()
      {
         @Override
         public void mouseDragged(MouseEvent e)
         {
            notifyRepaint();
         }
      });

      panel.addKeyListener(new KeyAdapter()
      {
         @Override
         public void keyPressed(KeyEvent e)
         {
            notifyRepaint();
         }
      });

      panel.addComponentListener(new ComponentAdapter()
      {
         @Override
         public void componentResized(ComponentEvent e)
         {
            notifyRepaint(3);
         }

         @Override
         public void componentMoved(ComponentEvent e)
         {
            notifyRepaint();
         }
      });

      RepaintManager.setCurrentManager(new NewRepaintManager(this, panel, closeableAndDisposableRegistry));
   }

   private static class NewRepaintManager extends RepaintManager implements CloseableAndDisposable
   {
      private RepaintManager oldManager;
      private Component panel;
      private JMERenderer jmeRender;

      public NewRepaintManager(JMERenderer jmeRender, Component panel, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
      {
         oldManager = RepaintManager.currentManager(panel);
         this.panel = panel;

         if (closeableAndDisposableRegistry != null)
         {
            closeableAndDisposableRegistry.registerCloseableAndDisposable(this);
         }
      }

      @Override
      public void addDirtyRegion(Applet applet, int x, int y, int w, int h)
      {
         if (oldManager == null)
            return;
         oldManager.addDirtyRegion(applet, x, y, w, h);
      }

      @Override
      public void addDirtyRegion(Window window, int x, int y, int w, int h)
      {
         if (oldManager == null)
            return;
         oldManager.addDirtyRegion(window, x, y, w, h);
      }

      @Override
      public synchronized void addInvalidComponent(JComponent invalidComponent)
      {
         if (oldManager == null)
            return;
         oldManager.addInvalidComponent(invalidComponent);
      }

      @Override
      public void addDirtyRegion(JComponent c, int x, int y, int w, int h)
      {
         if (oldManager == null)
            return;
         oldManager.addDirtyRegion(c, x, y, w, h);

         if (jmeRender == null)
            return;
         if (c == panel)
            jmeRender.notifyRepaint();
      }

      @Override
      public void closeAndDispose()
      {
         // Have to do this messy stuff since Swing has all these static calls, like RepaintManager.setCurrentManager() and the SwingThread never dies...
         if (oldManager != null)
         {
            // TODO: Something is messed up with reseting the repaintManager. Someone should look into
            // what is going on here and clean this up.
            // My sense right now is that every time we make a new JMERenderer, it recursively makes 
            // a new repaint manager...
            //            RepaintManager.setCurrentManager(oldManager);
            //            oldManager = null;
            panel = null;
            jmeRender = null;
         }
      }
   }

   @Override
   public void isCreated(PBOAwtPanel pboAwtPanel)
   {
      if (DEBUG_GPU_LIDAR_PARALLEL_SCENE)
      {
         PrintTools.debug(this, "Creating " + pboAwtPanel.getClass().getSimpleName());
      }
   }

   public void updateGPULidarScenes()
   {
      if (!pboAwtPanels.isEmpty())
      {
         Stopwatch timer = new Stopwatch().start();

         for (JMEGPULidar gpuLidar : gpuLidars)
         {
            gpuLidar.updateViewPortScenes();
         }

         notifyRepaint();
         PrintTools.info(this, "GPULidar scene updated. Took " + MathTools.roundToSignificantFigures(timer.totalElapsed(), 2) + " s");
      }
   }

   public void syncSubsceneToMain(Node subscene)
   {
      syncSubsceneToMain(getZUpNode(), subscene, subscene);
   }

   private void syncSubsceneToMain(Node main, Node sub, Node subsceneRoot)
   {
      if (alreadyClosing)
         return;

      sub.setLocalTransform(main.getLocalTransform());

      for (Spatial child : main.getChildren())
      {
         if (child.getName() == null)
            continue;

         Spatial subchild = findChildWithName(sub, child.getName());

         if (subchild == null && !JMENodeTools.isVisualization(child))
         {
            if (child instanceof Node)
            {
               subchild = cloneNode((Node) child);
               sub.attachChild(subchild);
            }
            else
            {
               subchild = child.clone(false);
               sub.attachChild(subchild);
            }
         }

         if (subchild != null)
         {
            if (child instanceof Node)
            {
               syncSubsceneToMain((Node) child, (Node) subchild, subsceneRoot);
            }
            else
            {
               subchild.setLocalTransform(child.getLocalTransform());
            }
         }
      }

      subsceneRoot.updateGeometricState();
      notifyRepaint();
   }

   private Spatial findChildWithName(Node node, String name)
   {
      if (name == null)
         return null;

      for (Spatial child : node.getChildren())
      {
         if (child != null && child.getName() != null && child.getName().equals(name))
            return child;
      }

      return null;
   }

   public Node cloneScene()
   {
      Node clonedScene = cloneNode(getZUpNode());

      return clonedScene;
   }

   private Node cloneNode(Node node)
   {
      Node clone = node.clone(false);

      ArrayList<Spatial> newChildren = new ArrayList<>();
      for (Spatial child : clone.getChildren())
      {
         if (child instanceof Node && child.getName() != null)
         {
            newChildren.add(cloneNode((Node) child));
         }
         else
         {
            newChildren.add(child);
         }
      }

      clone.detachAllChildren();

      for (Spatial newChild : newChildren)
      {
         clone.attachChild(newChild);
      }

      clone.updateGeometricState();

      return clone;
   }

   public Node cloneSceneWithoutVisualizations()
   {
      Node clonedScene = cloneScene();

      removeVisualizations(clonedScene, 0, clonedScene);

      return clonedScene;
   }

   /**
    * Returns a count of the number of nodes removed. Used for debugging, etc.
    */
   private long removeVisualizations(Node node, long num, Node rootNode)
   {
      for (Spatial child : node.getChildren())
      {
         if (child instanceof Node)
         {
            if (JMENodeTools.isVisualization(child))
            {
               child.removeFromParent();
               num++;
            }
            else
            {
               removeVisualizations((Node) child, num, rootNode);
            }
         }
      }

      rootNode.updateGeometricState();
      notifyRepaint();

      return num;
   }

   public List<Node> getVisualizationNodes()
   {
      List<Node> flattenedNodeList = getAllNodesInSceneAsList();
      List<Node> transparentNodeList = new ArrayList<>();

      for (Node node : flattenedNodeList)
      {
         if (JMENodeTools.isVisualization(node))
         {
            transparentNodeList.add(node);
         }
      }

      return transparentNodeList;
   }

   public long getNumberOfNodesInScene()
   {
      return getNumberOfNodesInScene(getZUpNode());
   }

   private long getNumberOfNodesInScene(Node root)
   {
      long count = 1;

      for (Spatial spatial : root.getChildren())
      {
         if (spatial instanceof Node)
         {
            count += getNumberOfNodesInScene((Node) spatial);
         }
      }

      return count;
   }

   public List<Node> getAllNodesInSceneAsList()
   {
      return getAllNodesInSceneAsList(getZUpNode(), new ArrayList<Node>());
   }

   private List<Node> getAllNodesInSceneAsList(Node root, List<Node> list)
   {
      list.add(root);

      for (Spatial spatial : root.getChildren())
      {
         if (spatial instanceof Node)
         {
            getAllNodesInSceneAsList((Node) spatial, list);
         }
      }

      return list;
   }

   @Override
   public void setBackgroundColor(final MutableColor color)
   {
      //      enqueue(new Callable<Object>()
      //      {
      //         @Override
      //         public Object call() throws Exception
      //         {
      //            for (JMEViewportAdapter viewportAdapter : viewportAdapters)
      //            {
      //               viewportAdapter.getViewPort().setBackgroundColor(new ColorRGBA(color.x, color.y, color.z, 1.0f));
      //            }
      //            
      //            return null;
      //         }
      //      });
   }

   @Override
   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
      System.err.println(getClass().getSimpleName() + ": setBackgroundImage not implemented.");

      // TODO
   }

   @Override
   public void setGroundAppearance(AppearanceDefinition app)
   {
      terrainAppearance = app;
      if(heightMap != null)
      {
         repaintTerrain();
      }
   }

   @Override
   public void addKeyListener(KeyListener keyListener)
   {
      keyListenerHolder.addKeyListener(keyListener);
   }

   public SelectedListenerHolder getSelectedListenerHolder()
   {
      return selectedListenerHolder;
   }

   public KeyListenerHolder getKeyListenerHolder()
   {
      return keyListenerHolder;
   }

   @Override
   public void addMouseListener(MouseListener mouseListener)
   {
      mouseListenerHolder.addMouseListener(mouseListener);
   }

   public MouseListenerHolder getMouseListenerHolder()
   {
      return mouseListenerHolder;
   }

   @Override
   public void addMouse3DListener(Mouse3DListener mouse3DListener)
   {
      mouse3DListenerHolder.addMouse3DListener(mouse3DListener);
   }

   public Mouse3DListenerHolder getMouse3DListenerHolder()
   {
      return mouse3DListenerHolder;
   }

   public Mouse3DInterface getMouse3DJoystick()
   {
      return mouse3DJoystick;
   }

   public DirectionalLight getPrimaryLight()
   {
      return primaryLight;
   }

   @Override
   public InputManager getInputManager()
   {
      return inputManager;
   }

   @Override
   public void freezeFrame(final Graphics3DNode rootJoint)
   {
      enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            JMEGraphics3DNode node = jmeGraphicsNodes.get(rootJoint);
            Node freezedNode = node.clone(true);
            zUpNode.attachChild(freezedNode);

            return null;
         }
      });
      notifyRepaint();
   }

   private boolean alreadyClosing = false;

   @Override
   public void closeAndDispose()
   {
      if (alreadyClosing)
         return;
      alreadyClosing = true;
      notifyRepaint();

      stop();

      // Things in jme SimpleApplication:
      rootNode = null;
      guiNode = null;
      fpsText = null;
      guiFont = null;
      flyCam = null;

      if (closeableAndDisposableRegistry != null)
      {
         closeableAndDisposableRegistry.closeAndDispose();
         closeableAndDisposableRegistry = null;
      }

      if (viewportAdapters != null)
      {
         for (JMEViewportAdapter viewportAdapter : viewportAdapters)
         {
            viewportAdapter.closeAndDispose();
         }
         viewportAdapters.clear();
         viewportAdapters = null;
      }

      if (contextManager != null)
      {
         contextManager.closeAndDispose();
         contextManager = null;
      }


      if (jmeGraphicsNodes != null)
      {
         synchronized (graphicsConch)
         {
            jmeGraphicsNodes.clear();
            jmeGraphicsNodes = null;
         }
      }

      if (jmeGraphicsNodesListView != null)
      {
         synchronized (graphicsConch)
         {
            jmeGraphicsNodesListView.clear();
            jmeGraphicsNodesListView = null;
         }
      }

      selectedListenerHolder = null;
      keyListenerHolder = null;
      mouseListenerHolder = null;
      mouse3DListenerHolder = null;

      if(mouse3DJoystick != null)
      {
         mouse3DJoystick.stopPolling();
         mouse3DJoystick = null;
      }

      rootJoint = null;
      terrain = null;
      zUpNode = null;

      if (gpuLidars != null)
      {
         for (JMEGPULidar jmegpuLidar : gpuLidars)
         {
            jmegpuLidar.cleanup();
         }
         gpuLidars.clear();
         gpuLidars = null;
      }

      if (pboAwtPanels != null)
      {
         for (PBOAwtPanel pboAwtPanel : pboAwtPanels)
         {
            pboAwtPanel.closeAndDispose();
         }

         pboAwtPanels.clear();
         pboAwtPanels = null;
      }

      primaryLight = null;
   }

   @Override
   public JMEContextManager getContextManager()
   {
      return contextManager;
   }


   public RenderType getRenderType()
   {
      return renderType;
   }

   public boolean isGroundVisible()
   {
      return isTerrainVisible;
   }

   @Override
   public JMEGPULidar createGPULidar(int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      JMEGPULidar gpuLidar = new JMEGPULidar(this, pointsPerSweep, scanHeight, fieldOfView, minRange, maxRange);

      gpuLidars.add(gpuLidar);
      updateGPULidarScenes();

      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      JMEGPULidar gpuLidar = createGPULidar(pointsPerSweep, scanHeight, fieldOfView, minRange, maxRange);
      gpuLidar.addGPULidarListener(listener);

      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, LidarScanParameters lidarScanParameters)
   {
      JMEGPULidar gpuLidar = createGPULidar(listener, lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getScanHeight(),
            lidarScanParameters.getFieldOfView(), lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());

      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(LidarScanParameters lidarScanParameters)
   {
      JMEGPULidar gpuLidar = createGPULidar(lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getScanHeight(), lidarScanParameters.getFieldOfView(),
            lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());

      return gpuLidar;
   }

   @Override
   public void play()
   {
      notifyRepaint();
      this.lazyRendering = false;
   }

   @Override
   public void pause()
   {
      this.lazyRendering = true;
   }

   protected synchronized void updateGraphics(float tpf)
   {
      for (Updatable updatable : updatables)
      {
         updatable.simpleUpdate(tpf);
      }
   }

   public synchronized void registerUpdatable(final Updatable updatable)
   {
      enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            updatables.add(updatable);

            return null;
         }
      });
   }

   public synchronized void removeUpdatable(final Updatable updatable)
   {
      enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            updatables.remove(updatable);

            return null;
         }
      });
   }

   @Override
   public void addDirectionalLight(Color color, Vector3D direction)
   {
      Vector3f jmeDirection = JMEDataTypeUtils.vecMathTuple3dToJMEVector3f(direction);
      JMEGeometryUtils.transformFromZupToJMECoordinates(jmeDirection);
      
      ColorRGBA jmeColor = JMEDataTypeUtils.colorToColorRGBA(color);
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            addDirectionalLight(jmeColor, jmeDirection);
            return null;
         }
      });
   }

   @Override
   public void clearDirectionalLights()
   {
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            for(Light light : lights)
            {
               rootNode.removeLight(light);
            }
            lights.clear();
            return null;
         }
      });
   }

   @Override
   public void addSpotLight(Graphics3DSpotLight spotLight)
   {
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            if(jmeSpotLights.containsKey(spotLight))
            {
               throw new RuntimeException("Spotlight is already added to graphic subsystem");
            }
            
            JMESpotLight jmeSpotLight = new JMESpotLight(spotLight);
            jmeSpotLights.put(spotLight, jmeSpotLight);
            rootNode.addLight(jmeSpotLight); 
            
            return null;
         }
      });
   }

   @Override
   public void removeSpotLight(Graphics3DSpotLight spotLight)
   {
      enqueue(new Callable<Object>()
      {

         @Override
         public Object call() throws Exception
         {
            JMESpotLight spotLightToRemove = jmeSpotLights.remove(spotLight);
            if(spotLightToRemove != null)
            {
               rootNode.removeLight(spotLightToRemove);
            }
            
            
            return null;
         }
      });
   }
}
