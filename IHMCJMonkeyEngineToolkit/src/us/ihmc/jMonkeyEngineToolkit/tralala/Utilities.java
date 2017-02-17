package us.ihmc.jMonkeyEngineToolkit.tralala;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Line;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.Texture;
import com.jme3.util.BufferUtils;

import jme3tools.optimize.GeometryBatchFactory;

public final class Utilities
{
   /**
    * We force character to not be able to exceed rotation limits. Because it
    * will turn the world upside down.
    */
   public static final float MIN_ANGLE_X = -89 * FastMath.DEG_TO_RAD;
   public static final float MAX_ANGLE_X = 89 * FastMath.DEG_TO_RAD;
   public static final Quad IDENTITY_QUAD = new Quad(1, 1);
   public static final String NEW_LINE_CHARACTER = System.getProperty("line.separator");

   public static Node createBillboard(Material mat, float radius)
   {
      Node sunGeom = new Node();
      Geometry sun = new Geometry("", new Quad(radius, radius));
      sun.setMaterial(mat);
      sun.setQueueBucket(Bucket.Transparent);
      sunGeom.attachChild(sun);
      sun.move(-radius / 2f, -radius / 2f, -radius / 2f);
      BillboardControl bc = new BillboardControl();
      sunGeom.addControl(bc);
      return sunGeom;
   }
   
   public static Material getUnshadedMaterial(String texturePath, ColorRGBA color, AssetManager assetManager)
   {
      return getUnshadedMaterial(assetManager.loadTexture(texturePath),color, BlendMode.Off, assetManager);
   }
   
   public static Material getUnshadedMaterial(Texture texture, ColorRGBA color, BlendMode mode, AssetManager assetManager)
   {
      Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      if (texture != null) mat.setTexture("ColorMap", texture);
      if (color != null) mat.setColor("Color", color);
      if (mode != BlendMode.Off)
      {
         mat.getAdditionalRenderState().setBlendMode(mode);
         mat.setTransparent(true);
      }
      return mat;
   }

   public static Material getLightingMaterial(String texturePath, String normalPath, String parallaxPath, float shiness, AssetManager assetManager)
   {
      Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      mat.setTexture("DiffuseMap", assetManager.loadTexture(texturePath));
      if (normalPath != null)   mat.setTexture("NormalMap", assetManager.loadTexture(normalPath));
      if (parallaxPath != null) mat.setTexture("ParallaxMap", assetManager.loadTexture(parallaxPath));
      if (shiness >= 0) mat.setFloat("Shininess", shiness); // [0,128]
      return mat;
   }

   public static void printErrorLine()
   {
      System.err.println(" at " + Thread.currentThread().getStackTrace()[2]);
   }

   public static List<File> getSubfiles(File file)
   {
      return getSubfiles(file, new ArrayList<File>(80));
   }

   public static List<File> getSubfiles(File file, List<File> fileList)
   {
      File[] files = file.listFiles();
      for (File f : files)
      {
         if (f.isFile()) fileList.add(f);
         else getSubfiles(f, fileList);
      }
      return fileList;
   }

   public static boolean isBlank(String str)
   {
      if (str == null) return true;
      if (str.isEmpty()) return true;
      for (char c : str.toCharArray()) if (!Character.isWhitespace(c)) return false;
      return true;
   }

   public static InputStream StringToInputStream(String s)
   {
      try
      {
         return new ByteArrayInputStream(s.getBytes("UTF-8"));
      }
      catch (UnsupportedEncodingException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static void appendSpaces(StringBuilder sb, int numberOfSpaces)
   {
      for (int i = 0; i < numberOfSpaces; i++)
      {
         sb.append(' ');
      }
   }

   public static String IpToString(InetSocketAddress address)
   {
      String s = address.getAddress().getHostAddress() + ":" + address.getPort();
      return s;
   }

   public static InetAddress getMyIpAddress()
   {
      try
      {
         return InetAddress.getLocalHost();
      }
      catch (UnknownHostException ex)
      {
         ex.printStackTrace();
         return null;
      }
   }

   public static void sleep(int millis)
   {
      try
      {
         Thread.sleep(millis);
      }
      catch (InterruptedException ex)
      {
         ex.printStackTrace();
      }
   }

   public static String getPrefix(String name)
   {
      return name.substring(0, name.indexOf('.'));
   }

   /* Get the extension of a file.
    *
    * Code taken from java tutorial : http://download.oracle.com/javase/tutorial/uiswing/components/filechooser.html#filters
    */
   public static String getExtension(File f)
   {
      String ext = null;
      String s = f.getName();
      int i = s.lastIndexOf('.');

      if (i > 0 && i < s.length() - 1)
      {
         ext = s.substring(i + 1).toLowerCase();
      }
      return ext;
   }

   ///-----------------------------------------------------------------------------------
   //Converts a file into an array of string
   public static ArrayList<String> readFile(File file)
   {
      return readFile(file, "UTF8");
   }

   ///-----------------------------------------------------------------------------------
   //Converts a file into an array of string
   public static ArrayList<String> readFile(File file, String encoding)
   {
      ArrayList<String> lines = new ArrayList<String>(50);
      if (!file.exists())
      {
         throw new IllegalStateException("Unable to find File " + file + " bye");
      }//if
      String line;
      boolean end = false;
      try
      {
         BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file), encoding));
         while (!end)
         {
            line = br.readLine();
            if (line == null)
            {
               end = true;
            }
            else
            {
               lines.add(line);
            }
         }//while
         br.close();
      }//try
      catch (IOException ioe)
      {
         System.err.println("Unable to open file");
         System.exit(-1);
      }
      return lines;
   }//readFile

   public static String readFileAsString(String file)
   {
      return readFileAsString(new File(file), "UTF8");
   }

   public static String readFileAsString(File file)
   {
      return readFileAsString(file, "UTF8");
   }

   public static String readFileAsString(File file, String encoding)
   {
      StringBuilder sb = new StringBuilder();
      ArrayList<String> lines = readFile(file, encoding);
      for (String line : lines)
      {
         sb.append(line).append(NEW_LINE_CHARACTER);
      }
      return new String(sb);
   }

   public static void saveFile(File file, String whatToSave)
   {
      saveFile(file, whatToSave, "UTF8");
   }

   public static void saveFile(File file, String whatToSave, String encoding)
   {
      if (file.isDirectory())
      {
         throw new IllegalArgumentException("File " + file + " should not be a directory");
      }

      try
      {
         BufferedWriter output = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), encoding));
         output.write(whatToSave);
         output.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   /**
       * Distance formula from Koen Samyn :
       * http://knol.google.com/k/plane-equation-in-3d#
       * <p/>
       * @param pointToBeMeasured the Point you want to find its distance from
       * plane.
       * @param anyPointOfPlane any plane's point e.g plane's origin
       * @param planeNormal the plane's Normal at the "anyPointOfPlane"
       */
   public static float getDistanceOfPointFromPlane(Vector3f pointToBeMeasured, Vector3f anyPointOfPlane, Vector3f planeNormal)
   {
      return pointToBeMeasured.subtract(anyPointOfPlane).dot(planeNormal);
   }

   /**
    * Symmetry formula from Koen Samyn :  http://knol.google.com/k/mirroring-a-point-on-a-3d-plane#
    * Same info on wikipedia : http://en.wikipedia.org/wiki/Reflection_%28mathematics%29
    * 
    * @param pointToBeMirrored the Point you want to find its symmetric
    * @param planeOrigin  the planeOrigin e.g (0,0,0)
    * @param planeNormal  the planeNormal is a vector that points upward from the plane e.g Vector3f.UNIT_Z
    */
   public static Vector3f getSummetricPosition(Vector3f pointToBeMirrored, Vector3f planeOrigin, Vector3f planeNormal)
   {
      if (!planeNormal.isUnitVector())
         throw new IllegalArgumentException("planeNormal " + planeNormal.toString() + " is not a unit vector");
      Vector3f symmetricPoint = pointToBeMirrored.subtract(planeNormal.mult(2 * getDistanceOfPointFromPlane(pointToBeMirrored, planeOrigin, planeNormal)));
      return symmetricPoint;
   }

   public static Vector3f getPositionOnPlane(Vector3f pointToBeCentered, Vector3f planeOrigin, Vector3f planeNormal)
   {
      if (!planeNormal.isUnitVector())
         throw new IllegalArgumentException("planeNormal " + planeNormal.toString() + " is not a unit vector");
      Vector3f centerPoint = pointToBeCentered.subtract(planeNormal.mult(getDistanceOfPointFromPlane(pointToBeCentered, planeOrigin, planeNormal)));
      return centerPoint;
   }

   public static float[] merge(float[] f1, float[] f2)
   {
      if (f1 == null) return f2;
      if (f2 == null) return f1;
      float[] f = new float[f1.length + f2.length];
      System.arraycopy(f1, 0, f, 0, f1.length);
      System.arraycopy(f2, 0, f, f1.length, f2.length);
      return f;
   }

   public static short[] merge(short[] f1, short[] f2)
   {
      if (f1 == null) return f2;
      if (f2 == null) return f1;
      short[] f = new short[f1.length + f2.length];
      System.arraycopy(f1, 0, f, 0, f1.length);
      System.arraycopy(f2, 0, f, f1.length, f2.length);
      return f;
   }

   public static byte[] merge(byte[] f1, byte[] f2)
   {
      if (f1 == null) return f2;
      if (f2 == null) return f1;
      byte[] f = new byte[f1.length + f2.length];
      System.arraycopy(f1, 0, f, 0, f1.length);
      System.arraycopy(f2, 0, f, f1.length, f2.length);
      return f;
   }

   public static FloatBuffer merge(FloatBuffer f1, FloatBuffer f2)
   {
      if (f1 == null) return f2;
      if (f2 == null) return f1;
      return BufferUtils.createFloatBuffer(merge(BufferUtils.getFloatArray(f1), BufferUtils.getFloatArray(f2)));
   }

   public static ShortBuffer merge(ShortBuffer f1, ShortBuffer f2)
   {
      if (f1 == null) return f2;
      if (f2 == null) return f1;
      return BufferUtils.createShortBuffer(merge(getShortArray(f1), getShortArray(f2)));
   }

   public static Type getTexCoordType(int i)
   {
      switch (i)
      {
         case 0:
            return Type.TexCoord;
         case 1:
            return Type.TexCoord2;
         case 2:
            return Type.TexCoord3;
         case 3:
            return Type.TexCoord4;
         case 4:
            return Type.TexCoord5;
         case 5:
            return Type.TexCoord6;
         case 6:
            return Type.TexCoord7;
         case 7:
            return Type.TexCoord8;
         default:
            throw new IllegalArgumentException("The specified tex coord type not found");
      }
   }

   public static Vector3f getVector3FromArray(float[] array, int i)
   {
      return new Vector3f(array[i * 3], array[i * 3 + 1], array[i * 3 + 2]);
   }

   public static Vector3f getVector3FromArray(List<Float> array, int i)
   {
      return new Vector3f(array.get(i * 3), array.get(i * 3 + 1), array.get(i * 3 + 2));
   }

   public static Vector2f getVector2FromArray(float[] array, int i)
   {
      return new Vector2f(array[i * 2], array[i * 2 + 1]);
   }

   public static Vector2f getVector2FromArray(List<Float> array, int i)
   {
      return new Vector2f(array.get(i * 2), array.get(i * 2 + 1));
   }

   public static Vector2f[] getVector2FromArray(List<Float>[] array, int i)
   {
      Vector2f[] returned = new Vector2f[array.length];
      for (int j = 0; j < array.length; j++) returned[j] = getVector2FromArray(array[j], i);
      return returned;
   }

   public static Quaternion getQuaternionFromArray(List<Float> array, int i)
   {
      return new Quaternion(array.get(i * 4), array.get(i * 4 + 1), array.get(i * 4 + 2), array.get(i * 4 + 3));
   }

   public static ArrayList<Byte> getAsList(byte[] array)
   {
      ArrayList<Byte> resultArray = new ArrayList<Byte>();
      for (byte s : array)
      {
         resultArray.add(s);
      }
      return resultArray;
   }

   public static ArrayList<Short> getAsList(short[] array)
   {
      ArrayList<Short> resultArray = new ArrayList<Short>();
      for (short s : array)
      {
         resultArray.add(s);
      }
      return resultArray;
   }

   public static ArrayList<Float> getAsList(float[] array)
   {
      ArrayList<Float> resultArray = new ArrayList<Float>();
      for (float f : array)
      {
         resultArray.add(f);
      }
      return resultArray;
   }

   @SuppressWarnings("unchecked")
   public static ArrayList<Float>[] getAsList(float[][] array)
   {
      ArrayList<Float>[] resultArray = new ArrayList[array.length];
      for (int i = 0; i < array.length; i++)
      {
         resultArray[i] = new ArrayList<Float>();
         for (float f : array[i])
         {
            resultArray[i].add(f);
         }
      }
      return resultArray;
   }

   public static byte[] toByteArray(Collection<Byte> array)
   {
      byte[] resultArray = new byte[array.size()];
      int i = 0;
      for (byte s : array)
      {
         resultArray[i++] = s;
      }
      return resultArray;
   }

   public static short[] toShortArray(Collection<Short> array)
   {
      short[] resultArray = new short[array.size()];
      int i = 0;
      for (short s : array)
      {
         resultArray[i++] = s;
      }
      return resultArray;
   }

   public static float[] toFloatArray(Collection<Float> array)
   {
      float[] resultArray = new float[array.size()];
      int i = 0;
      for (float s : array)
      {
         resultArray[i++] = s;
      }
      return resultArray;
   }

   public static float[][] toFloatArray(Collection<Float>[] array)
   {
      float[][] resultArray = new float[array.length][];
      int i = 0;
      for (int j = 0; j < array.length; j++)
      {
         resultArray[j] = new float[array[j].size()];
         for (float s : array[j])
         {
            resultArray[j][i++] = s;
         }
      }
      return resultArray;
   }

   public static void addInList(Quaternion p1, List<Float> array)
   {
      array.add(p1.getX());
      array.add(p1.getY());
      array.add(p1.getZ());
      array.add(p1.getW());
   }

   public static void addInList(Vector3f p1, List<Float> array)
   {
      array.add(p1.x);
      array.add(p1.y);
      array.add(p1.z);
   }

   public static void addInList(Vector2f p1, List<Float> array)
   {
      array.add(p1.x);
      array.add(p1.y);
   }

   public static void setInArray(Vector3f p1, float[] array, int i)
   {
      array[i * 3] = p1.x;
      array[i * 3 + 1] = p1.y;
      array[i * 3 + 2] = p1.z;
   }

   public static void setInArray(Vector2f p1, float[] array, int i)
   {
      array[i * 2] = p1.x;
      array[i * 2 + 1] = p1.y;
   }
   
   public static Geometry createPoint(Material mat, Vector3f position)
   {
      return createLine(mat,position,position.add(new Vector3f(0,0.5f,0)));
   }

   public static Plane createPlane(Vector3f planeOrigin, Vector3f planeNormal)
   {
      return new Plane(planeOrigin, planeNormal.dot(planeOrigin));
   }

   public static Geometry createArrow(Material mat, Vector3f pos, Vector3f dir)
   {
      Arrow arrow = new Arrow(dir.subtract(pos));
      arrow.setLineWidth(4); // make arrow thicker
      Geometry arrowGeom = new Geometry("Arrow", arrow);
      arrowGeom.setMaterial(mat);
      arrowGeom.setLocalTranslation(pos);
      return arrowGeom;
   }

   public static Geometry createLine(Material mat, Vector3f startPosition, Vector3f endPosition)
   {
      return createLine("Line", mat, startPosition, endPosition);
   }

   public static Geometry createLine(String name, Material mat, Vector3f startPosition, Vector3f endPosition)
   {
      Line line = new Line(startPosition, endPosition);
      Geometry lineGeom = new Geometry(name, line);
      lineGeom.setMaterial(mat);
      return lineGeom;
   }

   /**Lower left corner is at (0,0,0) and top right corner is at (x,y,z)*/
   public static Box createLowerLeftBox(float x, float y, float z)
   {
      return new Box(new Vector3f(x / 2f, y / 2f, z / 2f), x / 2f, y / 2f, z / 2f);
   }

   public static Geometry createHorizontalQuad(float x, float y, float z, float width, float length)
   {
      Quad quad = IDENTITY_QUAD;
      if (width != 1 && length != 1) quad = new Quad(width, length);

      Geometry tile = new Geometry("Quad", quad);
      tile.rotate(FastMath.HALF_PI, 0, FastMath.PI);
      tile.move(x + width, y, z);
      return tile;
   }

   public static Node createDebugNormals(Mesh mesh, Material mat, float scale)
   {
      Node debugNormals = new Node();

      VertexBuffer vertex = mesh.getBuffer(Type.Position);
      float[] vertexArray = BufferUtils.getFloatArray((FloatBuffer) vertex.getData());

      VertexBuffer normals = mesh.getBuffer(Type.Normal);
      if (normals == null) return debugNormals;
      float[] normalArray = BufferUtils.getFloatArray((FloatBuffer) normals.getData());

      for (int i = 0; i < vertexArray.length; i += 3)
      {
         Vector3f p1 = new Vector3f(vertexArray[i], vertexArray[i + 1], vertexArray[i + 2]);
         Vector3f n1 = new Vector3f(normalArray[i], normalArray[i + 1], normalArray[i + 2]);

         debugNormals.attachChild(createLine("DebugShowNormalsLine", mat, p1, p1.add(n1.mult(scale))));
      }

      GeometryBatchFactory.optimize(debugNormals);
      return debugNormals;
   }

   public static Node createSelectionCircle(Material mat, float scale, int points)
   {
      Node circle = new Node();
      for (int i = 0; i < points; i++)
      {
         float x0 = scale * FastMath.cos(i * 2 * FastMath.PI / points);
         float y0 = scale * FastMath.sin(i * 2 * FastMath.PI / points);
         float x1 = scale * FastMath.cos((i + 1) * 2 * FastMath.PI / points);
         float y1 = scale * FastMath.sin((i + 1) * 2 * FastMath.PI / points);

         circle.attachChild(createLine("Circle's Line", mat, new Vector3f(x0, 0, y0), new Vector3f(x1, 0, y1)));
      }
      GeometryBatchFactory.optimize(circle);
      return circle;
   }

   public static Node createWorldGrid(Material mat, int xi, int yi)
   {
      return createWorldGrid(mat, mat, xi, yi, true);
   }

   /**Debug Lines*/
   public static Node createWorldGrid(Material mat, Material axisMat, int xi, int yi, boolean centered)
   {
      Node worldGrid = new Node();

      Material materialToUse;
      //Horizontial Lines
      for (int i = 0; i <= yi; i++)
      {
         if (i == yi / 2) materialToUse = axisMat;
         else materialToUse = mat;

         if (!centered) worldGrid.attachChild(createLine("GridLineX" + i, materialToUse, new Vector3f(0, 0, i), new Vector3f(xi, 0, i)));
         else worldGrid.attachChild(createLine("GridLineX" + i, materialToUse, new Vector3f(-xi / 2f, 0, -yi / 2f + i), new Vector3f(xi / 2f, 0, -yi / 2f + i)));
      }

      //Vertical Lines
      for (int i = 0; i <= xi; i++)
      {
         if (i == xi / 2) materialToUse = axisMat;
         else materialToUse = mat;

         if (!centered) worldGrid.attachChild(createLine("GridLineY" + i, materialToUse, new Vector3f(i, 0, 0), new Vector3f(i, 0, yi)));
         else worldGrid.attachChild(createLine("GridLineY" + i, materialToUse, new Vector3f(-xi / 2f + i, 0, -yi / 2f), new Vector3f(-xi / 2f + i, 0, yi / 2f)));
      }
      GeometryBatchFactory.optimize(worldGrid);

      return worldGrid;
   }

   /**
    * Create a new short[] array and populate it with the given ShortBuffer's
    * contents.
    *
    * @param buff
    *            the ShortBuffer to read from
    * @return a new short array populated from the ShortBuffer
    */
   public static short[] getShortArray(ShortBuffer buff)
   {
      if (buff == null)
         return null;
      buff.clear();
      short[] inds = new short[buff.limit()];
      for (int x = 0; x < inds.length; x++)
      {
         inds[x] = buff.get();
      }
      return inds;
   }

   /**
    * Create a new byte[] array and populate it with the given ByteBuffer's
    * contents.
    *
    * @param buff
    *            the ByteBuffer to read from
    * @return a new byte array populated from the ByteBuffer
    */
   public static byte[] getByteArray(ByteBuffer buff)
   {
      if (buff == null)
         return null;
      buff.clear();
      byte[] inds = new byte[buff.limit()];
      for (int x = 0; x < inds.length; x++)
      {
         inds[x] = buff.get();
      }
      return inds;
   }
   
   /** Converts local cordinates "(dx,dy,dz)" to world coordinates based on Vector3D "rotation".
    * <a href="http://jerome.jouvie.free.fr/OpenGl/Tutorials/Tutorial26.php"> More info : </a>
    *
    * This kind of deplacement generaly used for player movement (in shooting game ...).
    * The x rotation is 'ignored' for the calculation of the deplacement.
    * This result that if you look upward and you want to move forward,
    * the deplacement is calculated like if your were parallely to the ground.
    */
   public static Vector3f localToWorldCoordinatesIgnoreHeight(float dx, float dy, float dz, float rotationX, float rotationY)
   {
      //Don't calculate for nothing ...
      if (dx == 0.0f & dy == 0.0f && dz == 0.0f)
         return new Vector3f();

      double xRot = -rotationX;
      double yRot = -rotationY;

      //Calculate the formula
      float x = (float) (dx * Math.cos(yRot) + 0 - dz * Math.sin(yRot));
      float y = (float) (0 + dy * Math.cos(xRot) + 0);
      float z = (float) (dx * Math.sin(yRot) + 0 + dz * Math.cos(yRot));

      //Return the vector expressed in the global axis system
      return new Vector3f(x, y, z);
   }//localToWorldCoordinatesIgnoreHeight
}//Utilities