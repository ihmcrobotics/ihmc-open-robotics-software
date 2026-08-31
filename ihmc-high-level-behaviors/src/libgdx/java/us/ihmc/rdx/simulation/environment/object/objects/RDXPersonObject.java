package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.Animation;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.g3d.utils.AnimationController;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import us.ihmc.behaviors.simulation.RigidBodySceneObjectDefinitions;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * RenderPeople Manuel (photoreal scan, original UVs) weighted to a Mixamo skeleton so
 * the crowd can idle / walk / point / stop. Shirt colour swaps the scan atlas per instance.
 */
public class RDXPersonObject extends RDXEnvironmentObject
{
   public static final String NAME = "Person";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXPersonObject.class);

   public static final String CLIP_IDLE = "idle";
   public static final String CLIP_WALK = "walk";
   public static final String CLIP_POINT = "point";
   public static final String CLIP_POINT_LEFT = "pointLeft";
   public static final String CLIP_STOP = "stop";
   /**
    * Mixamo point / stop clips raise, hold, then lower back to idle. The last frame is rest, so a
    * hold freezes at this fraction of the clip (about 1.25 s of a 2.5 s clip).
    */
   public static final float GESTURE_HOLD_FRACTION = 0.50f;

   private static final String SHIRT_TEXTURE_DIRECTORY = "environmentObjects/person/";

   private static final List<RDXPersonObject> LIVE = new CopyOnWriteArrayList<>();
   private static final Map<String, Texture> SHIRT_TEXTURES = new ConcurrentHashMap<>();

   private AnimationController animation;
   private String playingClip = "";
   private boolean gesturePlaying;
   private boolean freezeGesture;
   private float freezeTime;
   private double lastPlacedX = Double.NaN;
   private double lastPlacedY = Double.NaN;
   private double lastPlacedYaw = Double.NaN;

   public RDXPersonObject()
   {
      this(NAME, FACTORY);
   }

   public RDXPersonObject(String name)
   {
      this(name, FACTORY);
   }

   /** {@code modelFilePath} is ignored — every person uses the poseable Mixamo GLB. */
   public RDXPersonObject(String name, String modelFilePath)
   {
      this(name, FACTORY);
   }

   protected RDXPersonObject(String name, RDXEnvironmentObjectFactory factory)
   {
      this(name, factory, true);
   }

   protected RDXPersonObject(String name, RDXEnvironmentObjectFactory factory, String ignoredModelFilePath)
   {
      this(name, factory, true);
   }

   private RDXPersonObject(String name, RDXEnvironmentObjectFactory factory, boolean ignored)
   {
      super(name, factory);
      String modelPath = RigidBodySceneObjectDefinitions.PERSON_ANIMATED_MODEL_FILE_PATH;
      LogTools.info("Person '{}' loading poseable model {}", name, modelPath);
      Model realisticModel = RDXModelLoader.load(modelPath);
      setRealisticModel(realisticModel);
      isolateInstanceMaterials();
      applyShirtTexture(shirtTextureForName(name));
      // Mixamo glTF is Y-up / +Z-forward. Roll so they stand on Z, then yaw so yaw=0 faces +X.
      getRealisticModelOffset().appendRollRotation(Math.PI / 2.0);
      getRealisticModelOffset().prependYawRotation(Math.PI / 2.0);

      double sizeX = 0.5;
      double sizeY = 1.0;
      double sizeZ = 1.8;
      setMass(80.0f);
      getCollisionShapeOffset().getTranslation().add(0.0, 0.0, sizeZ / 2.0);
      getBoundingSphere().setRadius(5.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.DarkGray());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color);
                        });
      setCollisionGeometryObject(collisionBox);
      startIdle();
      LIVE.add(this);
   }

   public static List<RDXPersonObject> livePeople()
   {
      return LIVE;
   }

   public static RDXPersonObject findLive(String nameContains)
   {
      if (nameContains == null || nameContains.isBlank())
         return LIVE.isEmpty() ? null : LIVE.get(0);
      String needle = nameContains.toLowerCase(Locale.ROOT);
      for (RDXPersonObject person : LIVE)
      {
         if (person.getName().toLowerCase(Locale.ROOT).contains(needle))
            return person;
      }
      return null;
   }

   public static RDXPersonObject closestTo(double x, double y)
   {
      RDXPersonObject best = null;
      double bestDistance = Double.POSITIVE_INFINITY;
      for (RDXPersonObject person : LIVE)
      {
         double distance = Math.hypot(person.worldX() - x, person.worldY() - y);
         if (distance < bestDistance)
         {
            best = person;
            bestDistance = distance;
         }
      }
      return best;
   }

   public double worldX()
   {
      return getObjectTransform().getTranslationX();
   }

   public double worldY()
   {
      return getObjectTransform().getTranslationY();
   }

   public double worldZ()
   {
      return getObjectTransform().getTranslationZ();
   }

   /** World yaw of the body +X (facing). */
   public double bodyYaw()
   {
      return getObjectTransform().getRotation().getYaw();
   }

   public boolean isPointingLeft()
   {
      return isPointClip(playingClip, true);
   }

   public boolean isPointingRight()
   {
      return isPointClip(playingClip, false);
   }

   /** Both-hands-up halt clip, facing the robot. */
   public boolean isStopping()
   {
      return isStopClip(playingClip);
   }

   static boolean isStopClip(String clip)
   {
      if (clip == null || clip.isBlank())
         return false;
      String id = clip.toLowerCase(Locale.ROOT);
      int sep = Math.max(id.lastIndexOf('|'), Math.max(id.lastIndexOf('/'), id.lastIndexOf(':')));
      String leaf = sep >= 0 ? id.substring(sep + 1) : id;
      return leaf.equals(CLIP_STOP) || leaf.equals("halt");
   }

   static boolean isPointClip(String clip, boolean left)
   {
      if (clip == null || clip.isBlank())
         return false;
      String id = clip.toLowerCase(Locale.ROOT);
      if (left)
         return id.contains("pointleft") || id.equals(CLIP_POINT_LEFT);
      return id.contains("point") && !id.contains("pointleft");
   }

   /** Turn in place so the body faces {@code (x, y)} without changing the current clip. */
   public void faceToward(double x, double y)
   {
      Pose3D pose = new Pose3D();
      pose.getPosition().set(worldX(), worldY(), getObjectTransform().getTranslationZ());
      pose.getOrientation().setYawPitchRoll(Math.atan2(y - worldY(), x - worldX()), 0.0, 0.0);
      super.setPoseInWorld(pose);
      rememberPlacedPose(pose.getX(), pose.getY(), pose.getYaw());
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (animation != null && Gdx.graphics != null)
         tickAnimation(Gdx.graphics.getDeltaTime());
      super.getRealRenderables(renderables, pool);
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      boolean moved = Double.isFinite(lastPlacedX)
                      && Math.hypot(poseInWorld.getX() - lastPlacedX, poseInWorld.getY() - lastPlacedY) > 0.003;
      super.setPoseInWorld(poseInWorld);
      rememberPlacedPose(poseInWorld.getX(), poseInWorld.getY(), poseInWorld.getYaw());
      if (!gesturePlaying)
         playLoop(moved ? CLIP_WALK : CLIP_IDLE);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      super.setPositionInWorld(positionInWorld);
      lastPlacedX = positionInWorld.getX();
      lastPlacedY = positionInWorld.getY();
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      super.setTransformToWorld(transformToWorld);
      rememberPlacedPose(transformToWorld.getTranslationX(),
                         transformToWorld.getTranslationY(),
                         transformToWorld.getRotation().getYaw());
   }

   private void rememberPlacedPose(double x, double y, double yaw)
   {
      lastPlacedX = x;
      lastPlacedY = y;
      lastPlacedYaw = yaw;
   }

   public void playLoop(String clip)
   {
      play(clip, -1);
   }

   /** Hold this clip, or drop back to idle if it is already held. */
   public void toggleHoldGesture(String clip)
   {
      String id = resolveClip(clip);
      if (animation == null || id == null)
         return;
      if (id.equals(playingClip) && gesturePlaying)
      {
         releaseGesture();
         return;
      }
      holdGesture(clip);
   }

   /**
    * Snap to the mid-clip hold and keep applying it. Mixamo point / stop clips lower back to idle
    * at the end, so freezing the last frame dropped the arms before a pose model could read them.
    */
   public void holdGesture(String clip)
   {
      String id = resolveClip(clip);
      if (animation == null || id == null)
         return;
      gesturePlaying = true;
      if (id.equals(playingClip) && freezeGesture)
         return;
      playingClip = id;
      animation.paused = false;
      animation.setAnimation(id, -1);
      freezeTime = holdTimeSeconds(animation.current == null ? 2.5f : animation.current.duration);
      freezeGesture = true;
      applyFrozenGesture();
   }

   /** Mid-clip time where Mixamo point / stop clips hold the raised pose. */
   public static float holdTimeSeconds(float clipDuration)
   {
      if (!Float.isFinite(clipDuration) || clipDuration <= 0.0f)
         return 1.25f;
      return clipDuration * GESTURE_HOLD_FRACTION;
   }

   /** Play once, then return to idle. Used for stop / point. */
   public void playGesture(String clip)
   {
      String id = resolveClip(clip);
      if (animation == null || id == null)
         return;
      clearFreeze();
      gesturePlaying = true;
      playingClip = id;
      animation.paused = false;
      animation.setAnimation(id, 1, new AnimationController.AnimationListener()
      {
         @Override
         public void onEnd(AnimationController.AnimationDesc animation)
         {
            gesturePlaying = false;
            playLoop(CLIP_IDLE);
         }

         @Override
         public void onLoop(AnimationController.AnimationDesc animation)
         {
         }
      });
   }

   public String playingClip()
   {
      return playingClip;
   }

   /** Drop a held stop / point and loop idle again. */
   public void releaseGesture()
   {
      gesturePlaying = false;
      clearFreeze();
      if (animation != null)
         animation.paused = false;
      playingClip = "";
      playLoop(CLIP_IDLE);
   }

   /**
    * World position of a Mixamo node after the current clip frame. Used to project a COCO pose
    * into the robot camera so gestures are classified, not read from the clip name.
    * <p>
    * {@code node.globalTransform} is relative to the model root, so it carries neither the Y-up to
    * Z-up offset nor where the person stands and which way they face. Composing the instance
    * transform is what makes a raised arm point the same way in the world as it does on screen.
    */
   public boolean worldPointOfNode(String nodeId, Point3D out)
   {
      if (out == null || nodeId == null || nodeId.isBlank())
         return false;
      RDXModelInstance instance = getRealisticModelInstance();
      if (instance == null)
         return false;
      if (animation != null)
         tickAnimation(0.0f);
      instance.calculateTransforms();
      Node node = findSkeletonNode(instance, nodeId);
      if (node == null)
         return false;
      Matrix4 nodeToWorld = new Matrix4(instance.transform).mul(node.globalTransform);
      Vector3 translation = new Vector3();
      nodeToWorld.getTranslation(translation);
      if (!Float.isFinite(translation.x) || !Float.isFinite(translation.y) || !Float.isFinite(translation.z))
         return false;
      LibGDXTools.toEuclid(translation, out);
      return true;
   }

   private void startIdle()
   {
      RDXModelInstance instance = getRealisticModelInstance();
      if (instance == null || instance.animations == null || instance.animations.size == 0)
      {
         LogTools.warn("Person model has no animations — standing pose only");
         return;
      }
      animation = new AnimationController(instance);
      List<String> names = new ArrayList<>();
      for (Animation clip : instance.animations)
         names.add(clip.id);
      LogTools.info("Person animations: {}", names);
      playLoop(CLIP_IDLE);
   }

   private void play(String clip, int loopCount)
   {
      String id = resolveClip(clip);
      if (animation == null || id == null || id.equals(playingClip))
         return;
      clearFreeze();
      playingClip = id;
      animation.paused = false;
      animation.setAnimation(id, loopCount);
   }

   private void clearFreeze()
   {
      freezeGesture = false;
      freezeTime = 0.0f;
   }

   /** Advance the clip, or keep applying the mid-clip hold so the arms stay raised. */
   private void tickAnimation(float delta)
   {
      if (animation == null)
         return;
      if (freezeGesture && animation.current != null)
      {
         animation.current.loopCount = -1;
         if (animation.current.time >= freezeTime)
         {
            applyFrozenGesture();
            return;
         }
      }
      animation.paused = false;
      animation.update(delta);
      if (freezeGesture && animation.current != null && animation.current.time >= freezeTime)
         applyFrozenGesture();
   }

   private void applyFrozenGesture()
   {
      if (animation == null || animation.current == null)
         return;
      animation.paused = false;
      animation.current.loopCount = -1;
      animation.current.time = freezeTime;
      animation.update(0.0f);
   }

   private static Node findSkeletonNode(RDXModelInstance instance, String nodeId)
   {
      Node node = instance.getNode(nodeId, true);
      if (node != null)
         return node;
      node = instance.getNode(nodeId, true, true);
      if (node != null)
         return node;
      String leaf = nodeId;
      int sep = Math.max(nodeId.lastIndexOf(':'), Math.max(nodeId.lastIndexOf('/'), nodeId.lastIndexOf('.')));
      if (sep >= 0)
         leaf = nodeId.substring(sep + 1);
      return findNodeByLeaf(instance.nodes, leaf);
   }

   private static Node findNodeByLeaf(Iterable<Node> nodes, String leaf)
   {
      if (nodes == null || leaf == null || leaf.isBlank())
         return null;
      String needle = leaf.toLowerCase(Locale.ROOT);
      for (Node node : nodes)
      {
         String id = node.id;
         if (id != null)
         {
            String lower = id.toLowerCase(Locale.ROOT);
            if (lower.equals(needle) || lower.endsWith(":" + needle) || lower.endsWith("." + needle) || lower.endsWith(needle))
               return node;
         }
         Node child = findNodeByLeaf(node.getChildren(), leaf);
         if (child != null)
            return child;
      }
      return null;
   }

   private String resolveClip(String wanted)
   {
      RDXModelInstance instance = getRealisticModelInstance();
      if (instance == null || wanted == null)
         return null;
      String needle = wanted.toLowerCase(Locale.ROOT);
      for (Animation clip : instance.animations)
      {
         if (clip.id != null && clip.id.equalsIgnoreCase(wanted))
            return clip.id;
      }
      for (Animation clip : instance.animations)
      {
         if (clip.id != null && clip.id.toLowerCase(Locale.ROOT).contains(needle))
            return clip.id;
      }
      return null;
   }

   private void isolateInstanceMaterials()
   {
      RDXModelInstance instance = getRealisticModelInstance();
      if (instance == null)
         return;
      Array<Material> copies = new Array<>();
      for (Material material : instance.materials)
         copies.add(material.copy());
      for (int i = 0; i < instance.materials.size; i++)
      {
         Material original = instance.materials.get(i);
         Material copy = copies.get(i);
         replaceMaterial(instance.nodes, original, copy);
         instance.materials.set(i, copy);
      }
   }

   private static void replaceMaterial(Iterable<Node> nodes, Material from, Material to)
   {
      for (Node node : nodes)
      {
         if (node.parts != null)
         {
            for (NodePart part : node.parts)
            {
               if (part.material == from)
                  part.material = to;
            }
         }
         if (node.hasChildren())
            replaceMaterial(node.getChildren(), from, to);
      }
   }

   private void applyShirtTexture(String texturePath)
   {
      RDXModelInstance instance = getRealisticModelInstance();
      if (instance == null || texturePath == null)
         return;
      FileHandle file = Gdx.files.internal(texturePath);
      if (!file.exists())
      {
         LogTools.warn("Shirt texture '{}' not found, keeping the original skin", texturePath);
         return;
      }
      Texture texture = SHIRT_TEXTURES.computeIfAbsent(texturePath, path -> new Texture(Gdx.files.internal(path), true));
      texture.setFilter(Texture.TextureFilter.MipMapLinearLinear, Texture.TextureFilter.Linear);
      for (Material material : instance.materials)
      {
         material.set(PBRTextureAttribute.createBaseColorTexture(texture));
         material.set(PBRColorAttribute.createBaseColorFactor(Color.WHITE));
      }
   }

   /** Recoloured copies of the scan's own atlas; {@code null} keeps the grey shirt he was scanned in. */
   static String shirtTextureForName(String name)
   {
      String token = name == null ? "" : name.toLowerCase(Locale.ROOT);
      for (String colour : new String[] {"orange", "green", "blue", "yellow", "red"})
      {
         if (token.contains(colour))
            return SHIRT_TEXTURE_DIRECTORY + "personShirt" + Character.toUpperCase(colour.charAt(0)) + colour.substring(1) + ".jpg";
      }
      if (token.contains("black"))
         return SHIRT_TEXTURE_DIRECTORY + "personShirtBlackPants.jpg";
      return null;
   }
}
