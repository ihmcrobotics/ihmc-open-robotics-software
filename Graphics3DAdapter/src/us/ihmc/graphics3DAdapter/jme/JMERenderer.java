package us.ihmc.graphics3DAdapter.jme;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GraphicsDevice;
import java.io.File;
import java.net.URL;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.vecmath.Color3f;

import org.jmonkeyengine.scene.plugins.ogre.MaterialLoader;

import jme3dae.ColladaLoader;
import us.ihmc.graphics3DAdapter.GPULidarListener;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.Graphics3DBackgroundScaleMode;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.input.SelectedListener;
import us.ihmc.graphics3DAdapter.input.SelectedListenerHolder;
import us.ihmc.graphics3DAdapter.jme.JMEViewportAdapter.ViewportType;
import us.ihmc.graphics3DAdapter.jme.context.PBOAwtPanel;
import us.ihmc.graphics3DAdapter.jme.context.PBOAwtPanelListener;
import us.ihmc.graphics3DAdapter.jme.context.PBOAwtPanelsContext;
import us.ihmc.graphics3DAdapter.jme.contextManager.AWTPanelsContextManager;
import us.ihmc.graphics3DAdapter.jme.contextManager.CanvasContextManager;
import us.ihmc.graphics3DAdapter.jme.lidar.JMEGPULidar;
import us.ihmc.graphics3DAdapter.jme.terrain.JMEHeightMapTerrain;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.graphics3DAdapter.jme.util.JMENodeTools;
import us.ihmc.graphics3DAdapter.stlLoader.STLLoader;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.tools.FormattingTools;
import us.ihmc.utilities.inputDevices.keyboard.KeyListener;
import us.ihmc.utilities.inputDevices.keyboard.KeyListenerHolder;
import us.ihmc.utilities.inputDevices.mouse.MouseListener;
import us.ihmc.utilities.inputDevices.mouse.MouseListenerHolder;
import us.ihmc.utilities.inputDevices.mouse3DJoystick.Mouse3DJoystick;
import us.ihmc.utilities.inputDevices.mouse3DJoystick.Mouse3DListener;
import us.ihmc.utilities.inputDevices.mouse3DJoystick.Mouse3DListenerHolder;
import us.ihmc.utilities.io.files.FileTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.utilities.time.Timer;

import com.google.common.collect.HashBiMap;
import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetManager;
import com.jme3.input.InputManager;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.system.awt.AwtPanelsContext;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.SkyFactory;

public class JMERenderer extends SimpleApplication implements Graphics3DAdapter, PBOAwtPanelListener
{
   public enum RenderType {CANVAS, AWTPANELS}

   public final static boolean USE_PBO = true;
   public final static boolean USE_GPU_LIDAR_PARALLEL_SCENE = false; // Disable: this is super slow
   public final static boolean DEBUG_GPU_LIDAR_PARALLEL_SCENE = false;
   private final RenderType renderType;
   public static boolean tickUpdated = false;

   static
   {
      Path scsCachePath = FileTools.getTemporaryDirectoryPath().resolve("SCSCache");
      
      FileTools.ensureDirectoryExists(scsCachePath);

      System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + scsCachePath.toString());
      NativeLibraryLoader.setCustomExtractionFolder(scsCachePath.toString());
   }
   
   private final Object loadingStatus = new Object();
   private final Object graphicsConch = new Object();

   private JMEContextManager contextManager;
   private JMEAssetLocator assetLocator;

   private ArrayList<JMEViewportAdapter> viewportAdapters = new ArrayList<JMEViewportAdapter>();

   private HashBiMap<Graphics3DNode, JMEGraphics3DNode> jmeGraphicsNodes = HashBiMap.create();
   private Collection<JMEGraphics3DNode> jmeGraphicsNodesListView = jmeGraphicsNodes.values();

   private boolean isTerrainVisible = true;

   private Mouse3DJoystick mouse3DJoystick = new Mouse3DJoystick();
   
   private SelectedListenerHolder selectedListenerHolder = new SelectedListenerHolder();
   private KeyListenerHolder keyListenerHolder = new KeyListenerHolder();
   private MouseListenerHolder mouseListenerHolder = new MouseListenerHolder();
   private Mouse3DListenerHolder mouse3DListenerHolder = new Mouse3DListenerHolder();

   private Node rootJoint;
   private Node terrain;
   private Node zUpNode;
   
   private ArrayList<JMEGPULidar> gpuLidars = new ArrayList<>();
   private ArrayList<PBOAwtPanel> pboAwtPanels;

   private DirectionalLight primaryLight;

   public JMERenderer(RenderType renderType)
   {
      super();
      this.renderType = renderType;
      Logger.getLogger("").setLevel(Level.SEVERE);

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
         JMEGraphics3DNode jmeNode = new JMEGraphics3DNode(graphics3dNode, assetLocator, this);
   
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
         
         return jmeNode;
      }
   }

   public void addRootNode(final Graphics3DNode rootNode)
   {
      enqueue(new Callable<JMEGraphics3DNode>()
      {
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
   }
   
   public void registerViewport(JMEViewportAdapter viewportAdapter)
   {
      viewportAdapters.add(viewportAdapter);
   }

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
      
      JMEViewportAdapter newViewport = new JMEViewportAdapter(this, rootNode, isMainViewport, isOffScreen ? ViewportType.OFFSCREEN : ViewportType.CANVAS, false, Color.LIGHT_GRAY);
      
      return newViewport;
   }

   public void closeViewport(ViewportAdapter viewport)
   {
      ((JMEViewportAdapter) viewport).closeViewportAdapter();
      while (viewportAdapters.remove(viewport))
      ;
   }

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
       *  try
       * {
       *   Natives.extractNativeLibs(JmeSystem.getPlatform(), appSettings);
       * }
       * catch (IOException e)
       * {
       *   throw new RuntimeException("Cannot load native libs");
       * }
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

   public void removeRootNode(final Graphics3DNode rootNode)
   {
      enqueue(new Callable<JMEGraphics3DNode>()
      {
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
   }

   private void setupLighting()
   {
      primaryLight = new DirectionalLight();
      primaryLight.setColor(ColorRGBA.White.mult(0.5f));
      primaryLight.setDirection(new Vector3f(-0.1f, -1.0f, -0.2f).normalizeLocal());
      rootNode.addLight(primaryLight);
      
      ambientLight = new AmbientLight();
      ambientLight.setColor(ColorRGBA.White.mult(.8f)); //1.3f));
      rootNode.addLight(ambientLight);
      
      DirectionalLight primaryLight2 = new DirectionalLight();
      primaryLight2.setColor(ColorRGBA.White.mult(0.1f));
      primaryLight2.setDirection(new Vector3f(1.0f, -0.0f, -0.5f).normalizeLocal());
      rootNode.addLight(primaryLight2);
      
      DirectionalLight primaryLight3 = new DirectionalLight();
      primaryLight3.setColor(ColorRGBA.White.mult(0.4f));
      primaryLight3.setDirection(new Vector3f(0.0f, -1.0f, 0.0f).normalizeLocal());
      rootNode.addLight(primaryLight3);
      
      renderManager.setPreferredLightMode(TechniqueDef.LightMode.SinglePass);
      
      rootNode.setShadowMode(ShadowMode.CastAndReceive);
      zUpNode.setShadowMode(ShadowMode.CastAndReceive);
   }

   public void setupSky()
   {
      Spatial sky = SkyFactory.createSky(assetManager, "Textures/Sky/Bright/BrightSky.dds", false);
      sky.setLocalScale(1000);
      rootNode.attachChild(sky);
   }

   @Override
   public void simpleInitApp()
   {
      setupExceptionHandling();

      setupAssetManger(assetManager);

      zUpNode = new Node("zUpNode");
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      rootNode.attachChild(zUpNode);
      assetLocator = new JMEAssetLocator(assetManager);

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
      if (alreadyClosing) return;
      
      synchronized (graphicsConch)
      {
         if (alreadyClosing) return;

         for (JMEGraphics3DNode jmeGraphicsNode : jmeGraphicsNodesListView)
         {
            jmeGraphicsNode.update();
         }
      }

      if (count > 1000&&!tickUpdated)
      {
         tickUpdated = true;
      }

      updateCameras();
      count++;
   }

   private void updateCameras()
   {
      if (alreadyClosing) return;

      for (JMEViewportAdapter viewportAdapter : viewportAdapters)
      {
         viewportAdapter.updateCamera();
      }
   }

   public void setGroundVisible(final boolean isVisible)
   {
      enqueue(new Callable<Boolean>()
      {
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
   }

   public void setHeightMap(final HeightMap heightMap)
   {
      enqueue(new Callable<JMEHeightMapTerrain>()
      {
         public JMEHeightMapTerrain call() throws Exception
         {
            if (terrain != null)
            {
               terrain.removeFromParent();
            }

            JMEHeightMapTerrain jmeTerrain = new JMEHeightMapTerrain(heightMap, assetManager);
            terrain = jmeTerrain.getTerrain();
            terrain.setShadowMode(ShadowMode.Receive);

            if (isTerrainVisible)
            {
               zUpNode.attachChild(terrain);
            }

            return jmeTerrain;
         }
      });
   }

   public HashBiMap<Graphics3DNode, JMEGraphics3DNode> getJmeGraphicsNodes()
   {
      return jmeGraphicsNodes;
   }

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
   }
   
   @Override
   public void isShowing(PBOAwtPanel pboAwtPanel)
   {
      PrintTools.info(this, "A " + pboAwtPanel.getClass().getSimpleName() + " showed on screen.");
      
      if (!gpuLidars.isEmpty())
      {
         updateGPULidarScenes();
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
         Timer timer = new Timer().start();

         for (JMEGPULidar gpuLidar : gpuLidars)
         {
            gpuLidar.updateViewPortScenes();
         }

         PrintTools.info(this, "GPULidar scene updated. Took " + FormattingTools.roundToSignificantFigures(timer.totalElapsed(), 2) + " s");
      }
   }

   public void syncSubsceneToMain(Node subscene)
   {
      syncSubsceneToMain(getZUpNode(), subscene, subscene);
   }
   
   private void syncSubsceneToMain(Node main, Node sub, Node subsceneRoot)
   {
      if (alreadyClosing) return;
      
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

   public void setBackgroundColor(final Color3f color)
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

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
      System.err.println(getClass().getSimpleName() + ": setBackgroundImage not implemented.");

      // TODO
   }

   public void setGroundAppearance(AppearanceDefinition app)
   {
      System.err.println(getClass().getSimpleName() + ": setGroundAppearance not implemented.");

      // TODO
   }

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

   public void addMouseListener(MouseListener mouseListener)
   {
      mouseListenerHolder.addMouseListener(mouseListener);
   }

   public MouseListenerHolder getMouseListenerHolder()
   {
      return mouseListenerHolder;
   }
   
   public void addMouse3DListener(Mouse3DListener mouse3DListener)
   {
      mouse3DListenerHolder.addMouse3DListener(mouse3DListener);
   }

   public Mouse3DListenerHolder getMouse3DListenerHolder()
   {
      return mouse3DListenerHolder;
   }
   
   public Mouse3DJoystick getMouse3DJoystick()
   {
      return mouse3DJoystick;
   }

   public DirectionalLight getPrimaryLight()
   {
      return primaryLight;
   }

   public InputManager getInputManager()
   {
      return inputManager;
   }

   public void freezeFrame(final Graphics3DNode rootJoint)
   {
      enqueue(new Callable<Object>()
      {
         public Object call() throws Exception
         {
            JMEGraphics3DNode node = jmeGraphicsNodes.get(rootJoint);
            Node freezedNode = node.clone(true);
            zUpNode.attachChild(freezedNode);

            return null;
         }
      });
   }

   private boolean alreadyClosing = false;
   
   public void closeAndDispose()
   {
      if (alreadyClosing) return;
      alreadyClosing = true;
      
      stop();
      
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
      
      assetLocator = null;
      
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
      
      mouse3DJoystick.stopPolling();

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
         for (PBOAwtPanel  pboAwtPanel : pboAwtPanels)
         {
            pboAwtPanel.closeAndDispose();
         }
         
         pboAwtPanels.clear();
         pboAwtPanels = null;
      }
      
      primaryLight = null;
   }

   public JMEContextManager getContextManager()
   {
      return contextManager;
   }

   public JMEAssetLocator getAssetLocator()
   {
      return assetLocator;
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
   public JMEGPULidar createGPULidar(int pointsPerSweep, double fieldOfView, double minRange, double maxRange)
   {
      JMEGPULidar gpuLidar = new JMEGPULidar(this, pointsPerSweep, fieldOfView, minRange, maxRange);
      
      gpuLidars.add(gpuLidar);
      updateGPULidarScenes();
   
      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, int pointsPerSweep, double fieldOfView, double minRange, double maxRange)
   {
      JMEGPULidar gpuLidar = createGPULidar(pointsPerSweep, fieldOfView, minRange, maxRange);
      gpuLidar.addGPULidarListener(listener);

      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, LidarScanParameters lidarScanParameters)
   {
      JMEGPULidar gpuLidar = createGPULidar(listener, lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getFieldOfView(),
                                lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());

      return gpuLidar;
   }

   @Override
   public JMEGPULidar createGPULidar(LidarScanParameters lidarScanParameters)
   {
      JMEGPULidar gpuLidar = createGPULidar(lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getFieldOfView(), lidarScanParameters.getMinRange(),
                                lidarScanParameters.getMaxRange());

      return gpuLidar;
   }
}
