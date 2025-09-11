package us.ihmc.rdx.perception;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.apache.logging.log4j.core.util.ExecutorServices;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.unitree.robotModel.H1RobotModel;
import us.ihmc.unitree.robotModel.H1Version;
import us.ihmc.zed.global.zed;

import java.io.File;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.*;

/**
 * SVO player + SCS-2 log robot visualizer.
 * - Plays SVO (color, depth, optional point cloud)
 * - Loads an SCS-2/YoVariables log and renders the robot moving
 * - Attaches the point cloud to the head camera frame (left-eye offset tunable)
 */
public class RDXZEDSVOYoVariablePlayer
{
   // ---------- Defaults / UI ----------
   //   private static final String DEFAULT_SVO = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20250902_134118_H1ZEDXMini.svo2").toAbsolutePath().toString();
   private static final String DEFAULT_SVO = "/opt/ihmc/LogData/H1/Arghya/20250902_133905_H1HardwareControlProcess/perception/20250902_134118_H1ZEDXMini.svo2";
   // Robot log:
   private static final String DEFAULT_LOG = "/opt/ihmc/LogData/H1/Arghya/20250902_133905_H1HardwareControlProcess/robotData.log";

   // --- SVO UI
   private final ImString svoPath = new ImString(DEFAULT_SVO, 1024);
   private final ImBoolean hasLoadedSVO = new ImBoolean(false);
   private final ImInt selectedModelIdx = new ImInt(0);
   private final String[] zedModels = new String[] { "ZED_2", "ZED_MINI", "ZED_X_MINI" };
   private final ImInt selectedDepthModeIdx = new ImInt(0);
   private final String[] depthModes = new String[] { "PERFORMANCE", "QUALITY", "NEURAL" };
   private final ImBoolean loopPlayback = new ImBoolean(true);
   private final ImBoolean isPlaying = new ImBoolean(false);
   private final ImBoolean useSvoFPS = new ImBoolean(true);
   private final ImFloat fallbackFPS = new ImFloat(30.0f);
   private double effectiveFPS = 30.0;
   private int totalFrames = 0;
   private int currentFrame = 0;

   // --- Robot Log UI
   private final ImString logPath = new ImString(DEFAULT_LOG, 1024);
   private final ImBoolean hasLoadedLog = new ImBoolean(false);
   private final ImBoolean showRobot = new ImBoolean(true);
   private final ImBoolean attachCloudToHead = new ImBoolean(true);
   private final ImFloat leftEyeYOffset = new ImFloat(0.0f); // meters; tune if you don't read ZED calib here

   // ---------- ROS2 / Sensor ----------
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private ZEDSVOPlaybackSensor zedPlaybackSensor;

   // ---------- Images & visualization ----------
   private RawImage colorImage;
   private RawImage depthImage;

   private final RDXOpenCVVideoVisualizer colorImageVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "ZED Color", false);
   private final RDXOpenCVVideoVisualizer depthImageVisualizer = new RDXOpenCVVideoVisualizer("ZED Depth", "ZED Depth", false);
   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);
   private final ImBoolean renderZEDPointCloud = new ImBoolean(true);

   // A tiny scene marker so something 3D is visible
   private ModelInstance centroidBall;
   private final ImBoolean renderCentroid = new ImBoolean(false);

   // ---------- UI / app ----------
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());
   private final ExecutorService grabExecutor = Executors.newSingleThreadExecutor();
   private volatile boolean grabCompletedSinceLastRender = false;
   private volatile boolean inFlight = false;

   // Playback timing accumulator (render-thread driven)
   private double timeAccumulatorSec = 0.0;

   // ---------- SCS-2 / Robot ----------
   private RDXSCS2LogSession scs2Session;
   private HumanoidReferenceFrames referenceFrames;
   private FullHumanoidRobotModelWrapper logRobotModel;
   private FullHumanoidRobotModel vizRobotModel;
   private RDXMultiBodyGraphic robotGraphic;
   private final List<OneDoFJointBasics> vizOneDoFJoints = new ArrayList<>();
   private ReferenceFrame zedLeftEyeFrame;

   private RDXZEDSVOYoVariablePlayer()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Closer"));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            // 1) Create visualizers first
            colorImageVisualizer.create();
            depthImageVisualizer.create();
            zedPointCloudVisualizer.create();

            colorImageVisualizer.setActive(true);
            depthImageVisualizer.setActive(true);
            zedPointCloudVisualizer.setActive(true);

            // Register panels BEFORE baseUI.create()
            baseUI.getImGuiPanelManager().addPanel("SVO Player", this::renderSvoControls);
            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);
            baseUI.getImGuiPanelManager().addPanel("Robot Log", this::renderRobotLogPanel);
            baseUI.getImGuiPanelManager().addPanel(colorImageVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(depthImageVisualizer.getPanel());

            // Scene stuff (renderables)
            centroidBall = RDXModelBuilder.createSphere(0.03f, Color.SKY);
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool, sceneLevels) ->
                                                           {
                                                              if (renderZEDPointCloud.get())
                                                                 zedPointCloudVisualizer.getRenderables(renderables, pool, sceneLevels);
                                                           });

            // Robot graphic holder; created on-demand after log is loaded
            robotGraphic = new RDXMultiBodyGraphic("Robot");
            // we create() and add to scene only after the robot model is ready

            // build UI
            baseUI.create();
         }

         @Override
         public void render()
         {
            // --- SVO playback driver ---
            final float dt = Gdx.graphics.getDeltaTime();
            if (isPlaying.get() && hasLoadedSVO.get() && effectiveFPS > 1e-3)
            {
               timeAccumulatorSec += dt;
               final double step = 1.0 / effectiveFPS;
               while (timeAccumulatorSec >= step)
               {
                  timeAccumulatorSec -= step;
                  if (!inFlight)
                     stepToNextFrameAsync();
               }
            }

            // --- SCS-2 log update (advances to next state) ---
            if (hasLoadedLog.get())
            {
               scs2Session.update(); // advances internal time; we use current state right after
               updateRobotFromLog();
               if (attachCloudToHead.get())
                  updatePointCloudPoseFromHeadCamera();
            }

            // If the async grab finished, update the visualizers on the render thread
            if (grabCompletedSinceLastRender)
            {
               grabCompletedSinceLastRender = false;
               updateImageVisualizers();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         // ---------- ImGui panels ----------
         private void renderSvoControls()
         {
            ImGui.inputText("SVO path", svoPath);
            ImGui.combo("ZED Model", selectedModelIdx, zedModels);
            ImGui.combo("Depth Mode", selectedDepthModeIdx, depthModes);

            if (ImGui.button(hasLoadedSVO.get() ? "Reload SVO" : "Load SVO"))
               loadSVO(svoPath.get());

            ImGui.separator();

            if (hasLoadedSVO.get())
            {
               if (ImGui.button(isPlaying.get() ? "Pause" : "Play"))
               {
                  if (isPlaying.get()) pause(); else play();
               }
               ImGui.sameLine();
               if (ImGui.button("Restart"))
                  restart();

               ImGui.checkbox("Loop", loopPlayback);

               ImGui.separator();
               ImGui.checkbox("Use SVO FPS", useSvoFPS);
               if (!useSvoFPS.get())
                  ImGui.sliderFloat("Fallback FPS", fallbackFPS.getData(), 1.0f, 120.0f);

               ImGui.separator();
               ImGui.checkbox("Show Point Cloud", renderZEDPointCloud);
               ImGui.checkbox("Render Centroid", renderCentroid);

               ImGui.separator();
               ImGui.text(String.format("Frame %d / %d  |  Playback %.2f FPS",
                                        currentFrame, Math.max(0, totalFrames - 1), effectiveFPS));
            }
            else
            {
               ImGui.textDisabled("Load an SVO to enable playback controls.");
            }
         }

         private void renderOptions()
         {
            ImGui.checkbox("Show Point Cloud", renderZEDPointCloud);
            ImGui.checkbox("Render Centroid", renderCentroid);
            ImGui.sliderFloat("Left-eye Y offset (m)", leftEyeYOffset.getData(), -0.2f, 0.2f);
            ImGui.checkbox("Attach cloud to head camera", attachCloudToHead);
            ImGui.text(String.format("SVO Frame %d / %d | FPS %.2f",
                                     currentFrame, Math.max(0, totalFrames - 1), effectiveFPS));
         }

         private void renderRobotLogPanel()
         {
            ImGui.inputText("Log path", logPath);
            if (ImGui.button(hasLoadedLog.get() ? "Reload Log" : "Load Log"))
               loadRobotLog(logPath.get());

            if (!hasLoadedLog.get())
            {
               ImGui.textDisabled("Load an SCS-2/YoVariables log to enable robot playback.");
               return;
            }

            ImGui.separator();
            ImGui.checkbox("Show Robot", showRobot);
            if (robotGraphic != null)
               robotGraphic.setActive(showRobot.get());

            ImGui.text("Robot playback follows the log (scs2Session.update()).");
            ImGui.text("If you need hard sync, add a seek-to-time using your session API.");
         }

         @Override
         public void dispose()
         {
            close();
            baseUI.dispose();
         }
      });
   }

   // -------------------- SVO controls --------------------
   private void play()
   {
      if (!hasLoadedSVO.get()) return;
      isPlaying.set(true);
   }

   private void pause()
   {
      isPlaying.set(false);
   }

   private void loadSVO(String path)
   {
      File f = new File(path);
      if (!f.exists() || !f.isFile())
      {
         LogTools.error("SVO path invalid: " + path);
         return;
      }

      // Cleanup previous
      releaseImages();
      if (zedPlaybackSensor != null)
      {
         zedPlaybackSensor.close();
         zedPlaybackSensor = null;
      }

      // Select ZED model
      ZEDModelData model = switch (selectedModelIdx.get()) {
         case 0 -> ZEDModelData.ZED_2;
         case 1 -> ZEDModelData.ZED_MINI;
         case 2 -> ZEDModelData.ZED_X_MINI;
         default -> ZEDModelData.ZED_2;
      };

      // Select depth mode
      int depthMode = switch (selectedDepthModeIdx.get())
      {
         case 1 -> zed.SL_DEPTH_MODE_QUALITY;
         case 2 -> zed.SL_DEPTH_MODE_NEURAL;
         default -> zed.SL_DEPTH_MODE_PERFORMANCE;
      };

      try
      {
         zedPlaybackSensor = new ZEDSVOPlaybackSensor(ros2Helper, 0, model, depthMode, path);
         zedPlaybackSensor.useTrackedPose(false);

         // Probe once to get metadata like length (and first images)
         zedPlaybackSensor.run(true);
         try { zedPlaybackSensor.waitForGrab(); }
         catch (InterruptedException ie) { Thread.currentThread().interrupt(); LogTools.warn("Initial grab interrupted"); }
         finally { zedPlaybackSensor.run(false); }

         totalFrames = zedPlaybackSensor.getLength();
         currentFrame = 0;
         hasLoadedSVO.set(true);
         isPlaying.set(false);
         timeAccumulatorSec = 0.0;

         // FPS policy
         effectiveFPS = resolveSvoFPS(zedPlaybackSensor, fallbackFPS.get(), useSvoFPS.get());

         // Display first frame immediately
         grabExactFrameAsync(0);

         LogTools.info("Loaded SVO: " + path + " | frames=" + totalFrames + " | fps=" + effectiveFPS);
      }
      catch (Exception e)
      {
         LogTools.error("Failed to load SVO: " + e.getMessage());
         hasLoadedSVO.set(false);
      }
   }

   private void restart()
   {
      if (!hasLoadedSVO.get()) return;
      isPlaying.set(false);
      timeAccumulatorSec = 0.0;
      grabExactFrameAsync(0);
   }

   private void stepToNextFrameAsync()
   {
      if (!hasLoadedSVO.get()) return;

      int next = currentFrame + 1;
      if (next >= totalFrames)
      {
         if (loopPlayback.get())
            next = 0;
         else
         {
            next = totalFrames - 1;
            isPlaying.set(false);
         }
      }
      grabExactFrameAsync(next);
   }

   private void grabExactFrameAsync(int frameIdx)
   {
      if (zedPlaybackSensor == null || inFlight) return;

      inFlight = true;
      grabExecutor.submit(() ->
                          {
                             try
                             {
                                grabExactFrame(frameIdx);
                                currentFrame = frameIdx;
                                grabCompletedSinceLastRender = true;
                             }
                             catch (Throwable t)
                             {
                                LogTools.error("Grab failed: " + t.getMessage());
                             }
                             finally
                             {
                                inFlight = false;
                             }
                          });
   }

   private void grabExactFrame(int frameToGrab)
   {
      // Position and grab exactly one frame (non-streaming step)
      zedPlaybackSensor.setCurrentPosition(frameToGrab);
      zedPlaybackSensor.run(true);
      try { zedPlaybackSensor.waitForGrab(); } catch (InterruptedException ignored) {}
      zedPlaybackSensor.run(false);

      // Release previous
      releaseImages();

      // Acquire images from the playback sensor
      colorImage = zedPlaybackSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      depthImage = zedPlaybackSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
   }

   private void updateImageVisualizers()
   {
      if (colorImage != null)
      {
         colorImageVisualizer.updateImageDimensions(colorImage.getWidth(), colorImage.getHeight());
         colorImage.getPixelFormat().convertToRGBA(colorImage.getCpuImageMat(), colorImageVisualizer.getRGBA8Mat());
         colorImageVisualizer.update();
      }

      if (depthImage != null)
      {
         depthImageVisualizer.updateImageDimensions(depthImage.getWidth(), depthImage.getHeight());
         Mat grayDepth = new Mat();
         opencv_core.normalize(depthImage.getCpuImageMat(), grayDepth, 0.0, 255.0,
                               opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, null);
         opencv_imgproc.cvtColor(grayDepth, depthImageVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2BGRA);
         depthImageVisualizer.update();
         grayDepth.close();
      }

      if (renderZEDPointCloud.get() && colorImage != null && depthImage != null)
      {
         zedPointCloudVisualizer.setColorImage(colorImage);
         zedPointCloudVisualizer.setDepthImage(depthImage);
         zedPointCloudVisualizer.update();
      }

      if (renderCentroid.get())
         centroidBall.transform.setTranslation(0.1f, 0.0f, 0.05f);
   }

   private void releaseImages()
   {
      if (colorImage != null) { colorImage.release(); colorImage = null; }
      if (depthImage != null) { depthImage.release(); depthImage = null; }
   }

   // -------------------- Robot log (Option A) --------------------
   private void loadRobotLog(String path)
   {
      File f = new File(path);
      if (!f.exists() || !f.isFile())
      {
         LogTools.error("Log path invalid: " + path);
         return;
      }

      try
      {
         // Start session
         scs2Session = new RDXSCS2LogSession(baseUI);
         scs2Session.startSession(path, null);

         // Detect H1 version from the log's SDF (like your sample)
         Path logDir = scs2Session.getSession().getLogDataReader().getLogDirectory().toPath();
         boolean leftGripper = Files.readString(logDir.resolve("model.sdf"), StandardCharsets.UTF_8)
                                    .contains("left_base");
         H1Version robotVersion = leftGripper ? H1Version.FULL_ROBOT : H1Version.HANDLESS;

         // Create a concrete robot model to render
         H1RobotModel robotModel = new H1RobotModel(robotVersion);
         vizRobotModel = robotModel.createFullRobotModel();

         // Build the wrapper for the log state
         boolean enforceUniqueReferenceFrames = false;
         logRobotModel = new FullHumanoidRobotModelWrapper(scs2Session.getRobots().get(0).getRootBody().getRigidBody(),
                                                           robotModel.getRobotDefinition(),
                                                           robotModel.getJointMap(),
                                                           enforceUniqueReferenceFrames);

         // Reference frames used to mount the cloud to the head camera
         referenceFrames = new HumanoidReferenceFrames(logRobotModel, robotModel.getSensorInformation());

         // Map joints by name (with CrossFourBar fallback like your sample)
         vizOneDoFJoints.clear();
         for (OneDoFJointBasics logJoint : logRobotModel.getOneDoFJoints())
         {
            OneDoFJointBasics target = vizRobotModel.getOneDoFJointByName(logJoint.getName());
            if (target != null)
            {
               vizOneDoFJoints.add(target);
            }
            else
            {
               // Look inside cross-four-bar joints for the loop revolute joints
               for (OneDoFJointBasics j : vizRobotModel.getOneDoFJoints())
               {
                  if (j instanceof CrossFourBarJoint cf)
                  {
                     for (RevoluteJointBasics loop : cf.getFourBarFunction().getLoopJoints())
                        if (loop.getName().equals(logJoint.getName()))
                           vizOneDoFJoints.add(loop);
                  }
               }
            }
         }

         // Build the robot graphic and add to the scene
         robotGraphic.create();
         robotGraphic.loadRobotModelAndGraphics(robotModel.getRobotDefinition(), vizRobotModel.getElevator());
         robotGraphic.setActive(true);
         baseUI.getPrimaryScene().addRenderableProvider(robotGraphic);

         hasLoadedLog.set(true);
         LogTools.info("Loaded SCS-2 log: " + path);
      }
      catch (Exception e)
      {
         LogTools.error("Failed to load robot log: " + e.getMessage());
         hasLoadedLog.set(false);
      }
   }

   /** Pulls the current state from the log and pushes into the viz model; call each render after scs2Session.update(). */
   private void updateRobotFromLog()
   {
      if (!hasLoadedLog.get() || vizRobotModel == null || logRobotModel == null)
         return;

      // Copy joints
      OneDoFJointBasics[] src = logRobotModel.getOneDoFJoints();
      int n = Math.min(src.length, vizOneDoFJoints.size());
      for (int i = 0; i < n; i++)
         vizOneDoFJoints.get(i).setQ(src[i].getQ());

      // Copy root pose
      vizRobotModel.getRootJoint().getJointPose().set(logRobotModel.getRootJoint().getJointPose());

      // Update frames & graphic
      vizRobotModel.updateFrames();
      if (referenceFrames != null)
         referenceFrames.updateFrames();

      if (robotGraphic != null && robotGraphic.isRobotLoaded())
      {
         robotGraphic.update();
         robotGraphic.setOpacity(0.8f);
         robotGraphic.setActive(showRobot.get());
      }
   }

   /** Computes the world pose of the left-eye camera and applies it to the point cloud visualizer (via reflection). */
   private void updatePointCloudPoseFromHeadCamera()
   {
      if (referenceFrames == null || vizRobotModel == null)
         return;

      // Build the left-eye frame once
      if (zedLeftEyeFrame == null)
      {
         // Parent camera frame from HumanoidReferenceFrames (same one the sample uses)
         ReferenceFrame cameraCenter = referenceFrames.getExperimentalCameraFrame();
         // Left-eye offset: (+Y is left in standard H1 frames; you may invert if needed)
         RigidBodyTransform leftEyeToCenter = new RigidBodyTransform(new Quaternion(),
                                                                     new Vector3D(0.0, leftEyeYOffset.get(), 0.0));
         zedLeftEyeFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("ZED Left Eye",
                                                                                             cameraCenter,
                                                                                             leftEyeToCenter);
      }

      // Pose of the left eye in world
      RigidBodyTransform leftEyeToWorld = new RigidBodyTransform();
      zedLeftEyeFrame.getTransformToDesiredFrame(leftEyeToWorld, ReferenceFrame.getWorldFrame());

      // Push to the point cloud visualizer if it supports it
      setPointCloudPoseIfSupported(leftEyeToWorld);
   }

   /** Tries common method names to set the point cloud pose without introducing a hard dependency. */
   private void setPointCloudPoseIfSupported(RigidBodyTransform poseInWorld)
   {
      if (zedPointCloudVisualizer == null || poseInWorld == null) return;

      // Try a few method names that exist in different IHMC code versions
      List<String> candidateMethods = Arrays.asList(
            "setPoseInWorld",
            "setSensorFrameToWorld",
            "setTransformToWorld",
            "setWorldPose"
      );
      for (String name : candidateMethods)
      {
         try
         {
            Method m = zedPointCloudVisualizer.getClass().getMethod(name, RigidBodyTransform.class);
            m.invoke(zedPointCloudVisualizer, poseInWorld);
            return;
         }
         catch (Exception ignored) { /* try next */ }
      }
      // If we get here, your visualizer doesn't expose a pose setter. You can
      // add one or wrap the renderable provider to apply a transform.
   }

   // -------------------- Shutdown --------------------
   private void close()
   {
      ExecutorServices.shutdown(grabExecutor, 2, TimeUnit.SECONDS, getClass().getSimpleName());

      releaseImages();

      if (zedPlaybackSensor != null)
      {
         zedPlaybackSensor.close();
         zedPlaybackSensor = null;
      }

      if (robotGraphic != null)
         robotGraphic.destroy();

      if (scs2Session != null)
         scs2Session.destroy(baseUI);

      colorImageVisualizer.destroy();
      depthImageVisualizer.destroy();
      zedPointCloudVisualizer.destroy();

      if (ros2Node != null)
         ros2Node.destroy();
   }

   // ----- Utility: try to read SVO FPS via reflection; fall back if unavailable -----
   private static double resolveSvoFPS(Object playbackSensor, double fallback, boolean useSvo)
   {
      if (!useSvo || playbackSensor == null) return fallback;
      String[] methodNames = { "getFPS", "getFps", "getSvoFPS", "getFrameRate" };
      for (String name : methodNames)
      {
         try
         {
            Method m = playbackSensor.getClass().getMethod(name);
            Object v = m.invoke(playbackSensor);
            if (v instanceof Number)
            {
               double fps = ((Number) v).doubleValue();
               if (fps > 0.5 && fps < 500.0) return fps;
            }
         }
         catch (ReflectiveOperationException ignored) { }
      }
      return fallback;
   }

   public static void main(String[] args)
   {
      new RDXZEDSVOYoVariablePlayer();
   }
}
