package us.ihmc.robotbuilder.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.CubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddHeightMapInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.PrimitiveGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.SphereGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.jMonkeyEngineToolkit.graphics.Graphics3DInstructionExecutor;

/**
 * Converts a {@link us.ihmc.graphicsDescription.Graphics3DObject} to a {@link MeshView}
 * which can be used in a JavaFX Scene.
 */
public class JavaFX3DInstructionExecutor extends Graphics3DInstructionExecutor {
    private TriangleMesh outputMesh = new TriangleMesh();
    private Transform outputRotation = new Affine(), outputScale = new Affine(), outputTranslation = new Affine();
    private Material outputMaterial = new PhongMaterial(Color.GRAY);

    public JavaFX3DInstructionExecutor(ArrayList<Graphics3DPrimitiveInstruction> instructions) {
        setUpGraphicsFromDefinition(instructions);
    }

    @Override
    protected void doAddMeshDataInstruction(Graphics3DAddMeshDataInstruction graphics3DAddMeshData) {
        graphics3DAddMeshData.getMeshData().getVertices();
        outputMesh = interpretMeshData(graphics3DAddMeshData.getMeshData());
        outputMaterial = convertMaterial(graphics3DAddMeshData.getAppearance());
    }

    @Override
    protected void doAddHeightMapInstruction(Graphics3DAddHeightMapInstruction graphics3DAddHeightMap) {
        // not implemented yet
    }

    @Override
    protected void doAddExtrusionInstruction(Graphics3DAddExtrusionInstruction graphics3DAddText) {
        // not implemented yet
    }

    @Override
    protected void doAddModelFileInstruction(Graphics3DAddModelFileInstruction graphics3DAddModelFile) {
        // not implemented yet
    }

    @Override
    protected void doIdentityInstruction() {

    }

    @Override
    protected void doRotateInstruction(Graphics3DRotateInstruction rot) {
        Matrix3d mat = rot.getRotationMatrix();
        outputRotation = new Affine(new double[] {
                mat.getM00(), mat.getM01(), mat.getM02(), 0,
                mat.getM10(), mat.getM11(), mat.getM12(), 0,
                mat.getM20(), mat.getM21(), mat.getM22(), 0,
                0, 0, 0, 1
        }, MatrixType.MT_3D_4x4, 0);
    }

    @Override
    protected void doScaleInstruction(Graphics3DScaleInstruction graphics3DScale) {
        Vector3d scale = graphics3DScale.getScaleFactor();
        outputScale = new Scale(scale.x, scale.y, scale.z);
    }

    @Override
    protected void doTranslateInstruction(Graphics3DTranslateInstruction graphics3DTranslate) {
        Vector3d t = graphics3DTranslate.getTranslation();
        outputTranslation = new Translate(t.x, t.y, t.z);
    }

    public Node getResult() {
        MeshView result = new MeshView();
        result.setMesh(outputMesh);
        result.getTransforms().addAll(outputRotation, outputScale, outputTranslation);
        result.setMaterial(outputMaterial);
        //result.setDrawMode(DrawMode.LINE);
        return result;
    }

    private static Material convertMaterial(AppearanceDefinition appearance) {
        Color color = new Color(appearance.getColor().getX(), appearance.getColor().getY(), appearance.getColor().getZ(), appearance.getTransparency());
        PhongMaterial res = new PhongMaterial();
        res.setDiffuseColor(color);
        res.setSpecularColor(Color.WHITE);
        return res;
    }

    private static TriangleMesh interpretMeshData(MeshDataHolder meshData)
    {
        Point3f[] vertices = meshData.getVertices();
        TexCoord2f[] textureCoords = meshData.getTexturePoints();
        int[] triangleIndices = meshData.getTriangleIndices();
        Vector3f[] normals = meshData.getVertexNormals();

        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
        int[] indices = Arrays.stream(triangleIndices).flatMap(x -> IntStream.of(x, x, x)).toArray();
        mesh.getFaces().addAll(indices);

        float[] vertexBuffer = Arrays.stream(vertices).flatMap(v -> Stream.of(v.x, v.y, v.z)).collect(FloatArrayCollector.create());
        mesh.getPoints().addAll(vertexBuffer);

        float[] texCoordBuffer = Arrays.stream(textureCoords).flatMap(v -> Stream.of(v.x, v.y)).collect(FloatArrayCollector.create());
        mesh.getTexCoords().addAll(texCoordBuffer);

        float[] normalBuffer = Arrays.stream(normals).flatMap(n -> Stream.of(n.x, n.y, n.z)).collect(FloatArrayCollector.create());
        mesh.getPoints().addAll(normalBuffer);

        return mesh;
    }

   @Override
   protected void doAddPrimitiveInstruction(PrimitiveGraphics3DInstruction primitiveInstruction)
   {
      if (primitiveInstruction instanceof CubeGraphics3DInstruction)
      {
         CubeGraphics3DInstruction cubeInstruction = (CubeGraphics3DInstruction) primitiveInstruction;

         MeshDataHolder meshData = MeshDataGenerator.Cube(cubeInstruction.getLength(), cubeInstruction.getWidth(), cubeInstruction.getHeight(),
                                                          cubeInstruction.getCenteredInTheCenter(), cubeInstruction.getTextureFaces());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, cubeInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else if (primitiveInstruction instanceof SphereGraphics3DInstruction)
      {
         SphereGraphics3DInstruction sphereInstruction = (SphereGraphics3DInstruction) primitiveInstruction;
         
         MeshDataHolder meshData = MeshDataGenerator.Sphere(sphereInstruction.getRadius(), sphereInstruction.getResolution(), sphereInstruction.getResolution());
         Graphics3DAddMeshDataInstruction meshDataInstruction = Graphics3DObject.createMeshDataInstruction(meshData, sphereInstruction.getAppearance());

         doAddMeshDataInstruction(meshDataInstruction);
      }
      else
      {
         throw new RuntimeException("Need to support that primitive type! primitiveInstruction = " + primitiveInstruction);
      }
      
   }
}
