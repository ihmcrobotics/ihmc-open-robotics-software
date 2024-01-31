package us.ihmc.rdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Attribute;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.interaction.StepCheckIsPointInsideAlgorithm;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.Function;

public class RDXSCS2SimpleObject
{
   protected static final HashMap<String, AtomicInteger> OBJECT_INDEXES = new HashMap<>();

   protected String titleCasedName;
   protected final String pascalCasedName;
   protected final int objectIndex;
   protected RDXSCS2EnvironmentObjectFactory factory;
   protected Model realisticModel;
   protected RDXModelInstance realisticModelInstance;
   protected RDXModelInstance collisionModelInstance;
   protected final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
   protected RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
   protected RigidBodyTransform realisticModelOffset = new RigidBodyTransform();
   protected Sphere3D boundingSphere = new Sphere3D(1.0);
   protected Shape3DBasics collisionGeometryObject;
   protected Function<Point3DReadOnly, Boolean> isPointInside;
   protected Model collisionMesh;
   protected final RigidBodyTransform tempTransform = new RigidBodyTransform();
   protected final Matrix4 tempGDXTransform = new Matrix4();
   protected final RigidBodyTransform placementTransform = new RigidBodyTransform();
   protected final ReferenceFrame placementFrame;
   protected final FramePose3D placementFramePose = new FramePose3D();
   protected ReferenceFrame realisticModelFrame;
   protected ReferenceFrame collisionModelFrame;
   protected boolean isSelected = false;

   protected Attribute originalColor;

   public RDXSCS2SimpleObject(String titleCasedName, RDXSCS2EnvironmentObjectFactory factory)
   {
      this.titleCasedName = titleCasedName;
      this.factory = factory;
      pascalCasedName = FormattingTools.titleToKebabCase(titleCasedName);

      AtomicInteger atomicIndex = OBJECT_INDEXES.get(pascalCasedName);
      if (atomicIndex == null)
      {
         atomicIndex = new AtomicInteger(0);
         OBJECT_INDEXES.put(pascalCasedName, atomicIndex);
      }
      objectIndex = atomicIndex.getAndIncrement();
      placementFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "PlacementFrame" + objectIndex,
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       placementTransform);
      realisticModelFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "RealisticModelFrame" + objectIndex,
                                                                                            placementFrame,
                                                                                            realisticModelOffset);
      collisionModelFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "CollisionModelFrame" + objectIndex,
                                                                                            placementFrame,
                                                                                            collisionShapeOffset);
   }

   public RDXSCS2SimpleObject(String titleCasedName)
   {
      this.titleCasedName = titleCasedName;
      pascalCasedName = FormattingTools.titleToKebabCase(titleCasedName);

      AtomicInteger atomicIndex = OBJECT_INDEXES.get(pascalCasedName);
      if (atomicIndex == null)
      {
         atomicIndex = new AtomicInteger(0);
         OBJECT_INDEXES.put(pascalCasedName, atomicIndex);
      }
      objectIndex = atomicIndex.getAndIncrement();
      placementFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "PlacementFrame" + objectIndex,
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       placementTransform);
      realisticModelFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "RealisticModelFrame" + objectIndex,
                                                                                            placementFrame,
                                                                                            realisticModelOffset);
      collisionModelFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "CollisionModelFrame" + objectIndex,
                                                                                            placementFrame,
                                                                                            collisionShapeOffset);
   }

   public void setRealisticModel(Model realisticModel)
   {
      this.realisticModel = realisticModel;
      realisticModelInstance = new RDXModelInstance(realisticModel);
      this.originalColor = realisticModelInstance.materials.get(0).get(ColorAttribute.Diffuse);
   }

   public void setCollisionModel(Model collisionGraphic)
   {
      this.collisionMesh = collisionGraphic;
      collisionModelInstance = new RDXModelInstance(collisionGraphic);
   }

   public void setCollisionModel(Consumer<RDXMultiColorMeshBuilder> meshBuilderConsumer)
   {
      Model collisionGraphic = RDXModelBuilder.buildModel(meshBuilderConsumer, pascalCasedName + "CollisionModel" + getObjectIndex());
      LibGDXTools.setOpacity(collisionGraphic, 0.4f);
      setCollisionModel(collisionGraphic);
   }

   public void setCollisionGeometryObject(Shape3DBasics collisionGeometryObject)
   {
      this.collisionGeometryObject = collisionGeometryObject;
      isPointInside = collisionGeometryObject::isPointInside;

      if (collisionMesh == null)
      {
         setCollisionModel(meshBuilder ->
         {
            Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
            if (collisionGeometryObject instanceof Box3D)
            {
               Box3D box3D = (Box3D) collisionGeometryObject;
               meshBuilder.addBox((float) box3D.getSizeX(), (float) box3D.getSizeY(), (float) box3D.getSizeZ(), color);
               meshBuilder.addMultiLineBox(box3D.getVertices(), 0.01, color); // so we can see it better
            }
            else if (collisionGeometryObject instanceof Sphere3D)
            {
               Sphere3D sphere3D = (Sphere3D) collisionGeometryObject;
               meshBuilder.addSphere((float) sphere3D.getRadius(), color);
            }
         });
      }
   }

   public void setPointIsInsideAlgorithm(Function<Point3DReadOnly, Boolean> isPointInside)
   {
      this.isPointInside = isPointInside;
   }

   /**
    * If we are colliding spheres or boxes, this is overkill. Maybe make this a class that the complicated
    * objects can instantiate? or they can override this method...
    */
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      stepCheckIsPointInsideAlgorithm.update(boundingSphere.getRadius(), boundingSphere.getPosition());
      return !Double.isNaN(stepCheckIsPointInsideAlgorithm.intersect(pickRay, 100, isPointInside, intersectionToPack, false));
   }

   public void setSelected(boolean selected)
   {
      isSelected = selected;
   }

   public void setRawIsSelected(boolean selected)
   {
      isSelected = selected;
   }

   public RigidBodyTransform getCollisionShapeOffset()
   {
      return collisionShapeOffset;
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (realisticModelInstance != null)
         realisticModelInstance.getRenderables(renderables, pool);
   }

   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (collisionModelInstance != null)
         collisionModelInstance.getRenderables(renderables, pool);
   }

   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      placementTransform.getTranslation().set(positionInWorld);
      updateRenderablesPoses();
   }

   public void setPoseInWorld(Pose3D poseInWorld)
   {
      poseInWorld.get(placementTransform);
      updateRenderablesPoses();
   }

   public void setTransformToWorld(Matrix4 transformToWorld)
   {
      LibGDXTools.toEuclid(transformToWorld, tempTransform);
      setTransformToWorld(tempTransform);
   }

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      placementTransform.set(transformToWorld);
      updateRenderablesPoses();
   }

   public void updateRenderablesPoses()
   {
      placementFrame.update();
      realisticModelFrame.update();
      collisionModelFrame.update();

      placementFramePose.setFromReferenceFrame(realisticModelFrame);
      LibGDXTools.toLibGDX(placementFramePose, tempTransform, realisticModelInstance.transform);

      placementFramePose.setFromReferenceFrame(collisionModelFrame);
      LibGDXTools.toLibGDX(placementFramePose, tempTransform, collisionModelInstance.transform);
      if (collisionGeometryObject.getPose() == null)
      {
         if (collisionGeometryObject instanceof Sphere3D)
         {
            Sphere3D sphere = (Sphere3D) collisionGeometryObject;
            sphere.getPosition().set(placementFramePose.getPosition());
         }
      }
      else
      {
         collisionGeometryObject.getPose().set(placementFramePose);
      }
      boundingSphere.getPosition().set(placementFramePose.getPosition());
   }

   public RigidBodyTransform getObjectTransform()
   {
      return placementTransform;
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public String getPascalCasedName()
   {
      return pascalCasedName;
   }

   public int getObjectIndex()
   {
      return objectIndex;
   }

   public RDXSCS2EnvironmentObjectFactory getFactory()
   {
      return factory;
   }

   public RigidBodyTransform getRealisticModelOffset()
   {
      return realisticModelOffset;
   }

   public Sphere3D getBoundingSphere()
   {
      return boundingSphere;
   }

   public boolean getIsSelected()
   {
      return isSelected;
   }

   public RDXModelInstance getRealisticModelInstance()
   {
      return realisticModelInstance;
   }

   public void setCollisionModelColor(ColorAttribute color, float transparency)
   {
      this.collisionModelInstance.materials.get(0).set(color);
      this.collisionModelInstance.setOpacity(transparency);
   }
}
