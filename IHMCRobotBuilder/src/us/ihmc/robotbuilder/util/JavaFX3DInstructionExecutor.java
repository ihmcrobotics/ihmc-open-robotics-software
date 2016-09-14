package us.ihmc.robotbuilder.util;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import javafx.scene.transform.*;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DInstructionExecutor;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.instructions.*;
import us.ihmc.graphics3DAdapter.graphics.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.primitives.Graphics3DTranslateInstruction;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Converts a {@link us.ihmc.graphics3DAdapter.graphics.Graphics3DObject} to a {@link MeshView}
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
        int[] polygonIndices = meshData.getPolygonIndices();
        int[] pointsPerPolygonCount = meshData.getPolygonStripCounts();

        ArrayList<Integer> triangleIndices = new ArrayList<>();

        int polygonIndicesStart = 0;
        for (int pointsForThisPolygon : pointsPerPolygonCount)
        {
            int[] polygon = new int[pointsForThisPolygon];
            System.arraycopy(polygonIndices, polygonIndicesStart, polygon, 0, pointsForThisPolygon);

            int[] splitIntoTriangles = splitPolygonIntoTriangles(polygon);

            for (int i : splitIntoTriangles)
            {
                triangleIndices.add(i);
            }

            polygonIndicesStart += pointsForThisPolygon;
        }

        int[] indices = triangleIndices.stream().flatMapToInt(x -> IntStream.of(x, x)).toArray();

        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_TEXCOORD);
        mesh.getFaces().addAll(indices);

        float[] vertexBuffer = Arrays.stream(vertices).flatMap(v -> Stream.of(v.x, v.y, v.z)).collect(FloatArrayCollector.create());
        mesh.getPoints().addAll(vertexBuffer);

        float[] texCoordBuffer = Arrays.stream(textureCoords).flatMap(v -> Stream.of(v.x, v.y)).collect(FloatArrayCollector.create());
        mesh.getTexCoords().addAll(texCoordBuffer);

        return mesh;
    }

    private static int[] splitPolygonIntoTriangles(int[] polygonIndices)
    {
        if(polygonIndices.length <= 3)
            return polygonIndices;

        // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
        int[] ret = new int[3 * (polygonIndices.length - 2)];
        int i = 0;
        for(int j = 2; j < polygonIndices.length; j++)
        {
            ret[i++] = polygonIndices[0];
            ret[i++] = polygonIndices[j-1];
            ret[i++] = polygonIndices[j];
        }

        return ret;
    }
}
