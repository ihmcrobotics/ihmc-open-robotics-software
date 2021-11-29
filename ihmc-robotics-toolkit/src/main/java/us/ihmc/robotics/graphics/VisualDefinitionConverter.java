package us.ihmc.robotics.graphics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.graphicsDescription.instructions.ArcTorusGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CapsuleGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.ConeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CylinderGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.EllipsoidGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.ExtrudedPolygonGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.HemiEllipsoidGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.PyramidCubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.SphereGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.TruncatedConeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.WedgeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Torus3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.geometry.TruncatedCone3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class VisualDefinitionConverter
{
   public static Graphics3DObject toGraphics3DObject(Collection<? extends VisualDefinition> source)
   {
      Graphics3DObject output = new Graphics3DObject();
      for (VisualDefinition visualDefinition : source)
         output.combine(toGraphics3DObject(visualDefinition));
      return output;
   }

   public static Graphics3DObject toGraphics3DObject(VisualDefinition source)
   {
      if (source == null)
         return null;

      Graphics3DObject output = new Graphics3DObject();
      AffineTransformDefinition originPose = source.getOriginPose();

      if (originPose.hasTranslation())
      {
         output.translate(originPose.getTranslation());
      }

      if (originPose.hasLinearTransform())
      {
         LinearTransform3D linearTransform = originPose.getLinearTransform();

         if (linearTransform.isRotationMatrix())
         {
            output.rotate(linearTransform.getAsQuaternion());
         }
         else
         {
            if (!linearTransform.getPreScaleQuaternion().isZeroOrientation())
               output.rotate(linearTransform.getPreScaleQuaternion());
            output.scale(linearTransform.getScaleVector());
            if (!linearTransform.getPostScaleQuaternion().isZeroOrientation())
               output.rotate(linearTransform.getPostScaleQuaternion());
         }
      }

      List<Graphics3DPrimitiveInstruction> instructions = toGraphics3DPrimitiveInstruction(source.getGeometryDefinition());
      if (instructions == null || instructions.isEmpty())
         return null;

      for (Graphics3DPrimitiveInstruction instruction : instructions)
      {
         if (instruction instanceof Graphics3DInstruction)
            ((Graphics3DInstruction) instruction).setAppearance(toAppearanceDefinition(source.getMaterialDefinition()));
         output.addInstruction(instruction);
      }
      return output;
   }

   public static List<Graphics3DPrimitiveInstruction> toGraphics3DPrimitiveInstruction(GeometryDefinition source)
   {
      if (source == null)
         return null;

      if (source instanceof ArcTorus3DDefinition)
      {
         ArcTorus3DDefinition arcTorus = (ArcTorus3DDefinition) source;
         return Collections.singletonList(new ArcTorusGraphics3DInstruction(arcTorus.getStartAngle(),
                                                                            arcTorus.getEndAngle(),
                                                                            arcTorus.getMajorRadius(),
                                                                            arcTorus.getMinorRadius(),
                                                                            arcTorus.getResolution()));
      }
      else if (source instanceof Box3DDefinition)
      {
         Box3DDefinition box = (Box3DDefinition) source;
         return Collections.singletonList(new CubeGraphics3DInstruction(box.getSizeX(), box.getSizeY(), box.getSizeZ(), box.isCentered()));
      }
      else if (source instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsule = (Capsule3DDefinition) source;
         return Collections.singletonList(new CapsuleGraphics3DInstruction(capsule.getLength(),
                                                                           capsule.getRadiusX(),
                                                                           capsule.getRadiusY(),
                                                                           capsule.getRadiusZ(),
                                                                           capsule.getResolution()));
      }
      else if (source instanceof Cone3DDefinition)
      {
         Cone3DDefinition cone = (Cone3DDefinition) source;
         return Collections.singletonList(new ConeGraphics3DInstruction(cone.getHeight(), cone.getRadius(), cone.getResolution()));
      }
      else if (source instanceof ConvexPolytope3DDefinition)
      {
         return null; // TODO Not sure here
      }
      else if (source instanceof Cylinder3DDefinition)
      {
         Cylinder3DDefinition cylinder = (Cylinder3DDefinition) source; // FIXME Handle the offset along the cylinder's axis.
         return Collections.singletonList(new CylinderGraphics3DInstruction(cylinder.getRadius(), cylinder.getLength(), cylinder.getResolution()));
      }
      else if (source instanceof Ellipsoid3DDefinition)
      {
         Ellipsoid3DDefinition ellipsoid = (Ellipsoid3DDefinition) source;
         return Collections.singletonList(new EllipsoidGraphics3DInstruction(ellipsoid.getRadiusX(),
                                                                             ellipsoid.getRadiusY(),
                                                                             ellipsoid.getRadiusZ(),
                                                                             ellipsoid.getResolution()));
      }
      else if (source instanceof ExtrudedPolygon2DDefinition)
      {
         ExtrudedPolygon2DDefinition polygon = (ExtrudedPolygon2DDefinition) source; // FIXME handle the case that bottom-z is not 0
         return Collections.singletonList(new ExtrudedPolygonGraphics3DInstruction(polygon.getPolygonVertices(), polygon.getTopZ() - polygon.getBottomZ()));
      }
      else if (source instanceof ExtrusionDefinition)
      {
         return null; // FIXME implement me
      }
      else if (source instanceof HemiEllipsoid3DDefinition)
      {
         HemiEllipsoid3DDefinition hemiEllipsoid = (HemiEllipsoid3DDefinition) source;
         return Collections.singletonList(new HemiEllipsoidGraphics3DInstruction(hemiEllipsoid.getRadiusX(),
                                                                                 hemiEllipsoid.getRadiusY(),
                                                                                 hemiEllipsoid.getRadiusZ(),
                                                                                 hemiEllipsoid.getResolution()));
      }
      else if (source instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition model = (ModelFileGeometryDefinition) source;
         List<Graphics3DPrimitiveInstruction> output = new ArrayList<>();
         if (model.getScale() != null)
            output.add(new Graphics3DScaleInstruction(model.getScale()));
         if (model.getSubmeshes() == null || model.getSubmeshes().isEmpty())
            output.add(new Graphics3DAddModelFileInstruction(model.getFileName(), null, model.getResourceDirectories(), model.getResourceClassLoader()));
         else
            output.add(new Graphics3DAddModelFileInstruction(model.getFileName(),
                                                             model.getSubmeshes().get(0).getName(),
                                                             model.getSubmeshes().get(0).getCenter(),
                                                             null,
                                                             model.getResourceDirectories(),
                                                             model.getResourceClassLoader()));
         return output;
      }
      else if (source instanceof PyramidBox3DDefinition)
      {
         PyramidBox3DDefinition pyramidBox = (PyramidBox3DDefinition) source;
         return Collections.singletonList(new PyramidCubeGraphics3DInstruction(pyramidBox.getBoxSizeX(),
                                                                               pyramidBox.getBoxSizeY(),
                                                                               pyramidBox.getBoxSizeZ(),
                                                                               pyramidBox.getPyramidHeight()));
      }
      else if (source instanceof Ramp3DDefinition)
      {
         Ramp3DDefinition ramp = (Ramp3DDefinition) source; // FIXME The origin might not be the same.
         return Collections.singletonList(new WedgeGraphics3DInstruction(ramp.getSizeX(), ramp.getSizeY(), ramp.getSizeZ()));
      }
      else if (source instanceof Sphere3DDefinition)
      {
         Sphere3DDefinition sphere = (Sphere3DDefinition) source;
         return Collections.singletonList(new SphereGraphics3DInstruction(sphere.getRadius(), sphere.getResolution()));
      }
      else if (source instanceof Torus3DDefinition)
      {
         Torus3DDefinition torus = (Torus3DDefinition) source;
         return Collections.singletonList(new ArcTorusGraphics3DInstruction(0,
                                                                            2.0 * Math.PI,
                                                                            torus.getMajorRadius(),
                                                                            torus.getMinorRadius(),
                                                                            torus.getResolution()));
      }
      else if (source instanceof TriangleMesh3DDefinition)
      {
         TriangleMesh3DDefinition mesh = (TriangleMesh3DDefinition) source;
         return Collections.singletonList(new Graphics3DAddMeshDataInstruction(new MeshDataHolder(mesh.getVertices(),
                                                                                                  mesh.getTextures() == null ? null
                                                                                                        : Arrays.stream(mesh.getTextures())
                                                                                                                .map(t -> new TexCoord2f(t.getX32(),
                                                                                                                                         t.getY32()))
                                                                                                                .toArray(TexCoord2f[]::new),
                                                                                                  mesh.getTriangleIndices(),
                                                                                                  mesh.getNormals()),
                                                                               null));
      }
      else if (source instanceof TruncatedCone3DDefinition)
      {
         TruncatedCone3DDefinition cone = (TruncatedCone3DDefinition) source;
         return Collections.singletonList(new TruncatedConeGraphics3DInstruction(cone.getHeight(),
                                                                                 cone.getBaseRadiusX(),
                                                                                 cone.getBaseRadiusY(),
                                                                                 cone.getTopRadiusX(),
                                                                                 cone.getTopRadiusY(),
                                                                                 cone.getResolution()));
      }
      else
      {
         throw new IllegalArgumentException("Unsupported geometry type: " + source.getClass().getName());
      }
   }

   public static AppearanceDefinition toAppearanceDefinition(MaterialDefinition source)
   {
      if (source == null)
         return null;

      ColorDefinition diffuseColor = source.getDiffuseColor();
      ColorDefinition specularColor = source.getSpecularColor();
      ColorDefinition ambientColor = source.getAmbientColor();

      if (diffuseColor != null)
      {
         if (specularColor != null && ambientColor != null)
         {
            YoAppearanceMaterial output = new YoAppearanceMaterial();
            output.setDiffuseColor((float) diffuseColor.getRed(), (float) diffuseColor.getGreen(), (float) diffuseColor.getBlue());
            output.setSpecularColor((float) specularColor.getRed(), (float) specularColor.getGreen(), (float) specularColor.getBlue());
            output.setAmbientColor((float) ambientColor.getRed(), (float) ambientColor.getGreen(), (float) ambientColor.getBlue());
            output.setShininess((float) source.getShininess());
            return output;
         }
         else
         {
            return new YoAppearanceRGBColor(diffuseColor.getRed(), diffuseColor.getGreen(), diffuseColor.getBlue(), 1.0 - diffuseColor.getAlpha());
         }
      }
      else
      {
         TextureDefinition diffuseMap = source.getDiffuseMap();

         if (diffuseMap == null)
            return null;

         if (diffuseMap.getFilename() != null)
            return new YoAppearanceTexture(diffuseMap.getFilename());
         if (diffuseMap.getFileURL() != null)
            return new YoAppearanceTexture(diffuseMap.getFileURL().toExternalForm());
         if (diffuseMap.getImage() != null)
            return new YoAppearanceTexture(diffuseMap.getImage());
         return null;
      }
   }
}
