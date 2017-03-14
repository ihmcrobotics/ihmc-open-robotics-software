package us.ihmc.ihmcPerception.objectDetector;

import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.caffe.Caffe;
import org.bytedeco.javacpp.caffe.FloatBlob;
import org.bytedeco.javacpp.caffe.FloatBlobVector;
import org.bytedeco.javacpp.caffe.FloatNet;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.time.Stopwatch;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.logging.Level;

import static org.bytedeco.javacpp.caffe.TEST;

/**
 *
 */
public class ValveDetector
{
   private static int NETWORK_OUTPUT_WIDTH;
   private static int NETWORK_OUTPUT_HEIGHT;
   private static int NETWORK_INPUT_WIDTH;
   private static int NETWORK_INPUT_HEIGHT;

   private static FloatNet caffe_net;
   private static final Object caffeNetLock = new Object();
   private static int caffe_gpu = -1;

   private static void setCaffeMode(int gpu)
   {
      // Set device id and mode
      if (gpu >= 0)
      {
         Caffe.SetDevice(gpu);
         Caffe.set_mode(Caffe.GPU);
      }
      else
      {
         Caffe.set_mode(Caffe.CPU);
      }
   }

   private static void initialize() throws Exception
   {
      if (caffe_net != null)
         return;

      String gpuStr = System.getProperty("gpu");
      if (gpuStr == null)
         gpuStr = "-1";
      int gpu = -1;
      try
      {
         gpu = Integer.parseInt(gpuStr);
      } catch (Exception ex)
      {
         // keep -1 (cpu)
      }

      String model, weights;
      try
      {
         String tempDir = System.getProperty("java.io.tmpdir");
         model = exportResource("/valvenet/deploy.prototxt", tempDir);
         weights = exportResource("/valvenet/snapshot_iter_1761.caffemodel", tempDir);

         String[] cudaLibs = {"libcublas.so.7.5", "libcudart.so.7.5", "libcurand.so.7.5"};
         File cudaLibsDir = Loader.getTempDir();
         for (String cudaLib : cudaLibs)
         {
            if (exportResource("/" + cudaLib, cudaLibsDir.getAbsolutePath()).isEmpty())
               PrintTools.warn("Could not extract " + cudaLib + " to " + cudaLibsDir.getAbsolutePath()
                                    + ". If CUDA 7.5 is not installed, object detection will likely fail with UnsatisfiedLinkError.");
            else
               PrintTools.debug("Extracted " + cudaLib + " to " + cudaLibsDir);
         }

         System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + cudaLibsDir.getAbsolutePath());

      }
      catch (Exception ex)
      {
         PrintTools.error(Level.SEVERE, "Could not load weights: " + ex.getMessage());
         throw ex;
      }

      if (model.length() == 0)
      {
         throw new Exception("Need a model definition to score.");
      }
      if (weights.length() == 0)
      {
         throw new Exception("Need model weights to score.");
      }

      setCaffeMode(gpu);
      if (gpu < 0)
         PrintTools.info("Use CPU.");
      else
         PrintTools.info("Use GPU with device ID " + gpu);
      caffe_gpu = gpu;

      // Instantiate the caffe net.
      caffe_net = new FloatNet(model, TEST);
      caffe_net.CopyTrainedLayersFrom(weights);
      NETWORK_OUTPUT_WIDTH = caffe_net.output_blobs().get(0).shape(3);
      NETWORK_OUTPUT_HEIGHT = caffe_net.output_blobs().get(0).shape(2);
      NETWORK_INPUT_WIDTH = caffe_net.input_blobs().get(0).shape(3);
      NETWORK_INPUT_HEIGHT = caffe_net.input_blobs().get(0).shape(2);
   }

   ValveDetector() throws Exception
   {
      initialize();
   }

   /**
    * Export a resource embedded into a Jar file to the local file path.
    *
    * @param resourceName ie.: "/SmartLibrary.dll"
    * @return The path to the exported resource
    * @throws Exception if extraction fails
    */
   private static String exportResource(String resourceName, String outDir) throws Exception
   {
      InputStream stream = null;
      OutputStream resStreamOut = null;
      try
      {
         stream = ValveDetector.class
               .getResourceAsStream(resourceName);
         if (stream == null)
         {
            throw new Exception("Cannot get resource \"" + resourceName + "\" from Jar file.");
         }

         int readBytes;
         byte[] buffer = new byte[4096];
         String resourceFileName = new File(resourceName).getName();
         File outFile = new File(outDir + "/" + resourceFileName);

         resStreamOut = new FileOutputStream(outFile.getAbsoluteFile());
         while ((readBytes = stream.read(buffer)) > 0)
         {
            resStreamOut.write(buffer, 0, readBytes);
         }
         return outFile.getAbsolutePath();
      } finally
      {
         if (stream != null)
            stream.close();
         if (resStreamOut != null)
            resStreamOut.close();
      }
   }

   public Pair<List<Rectangle>, HeatMap> detect(BufferedImage image)
   {
      Stopwatch timer = new Stopwatch();
      timer.start();

      setCaffeMode(caffe_gpu);
      PreprocessedImage processedImage = processImage(image, NETWORK_INPUT_WIDTH, NETWORK_INPUT_HEIGHT);
      FloatPointer pointer = new FloatPointer(processedImage.data.length);
      pointer.put(processedImage.data);

      float[] output;
      synchronized (caffeNetLock)
      {
         caffe_net.input_blobs().get(0).set_cpu_data(pointer);
         output = transferNetworkOutputToJava(caffe_net.Forward());
      }
      HeatMap outputMap = new HeatMap(0, 0, NETWORK_OUTPUT_WIDTH, NETWORK_OUTPUT_HEIGHT);
      System.arraycopy(output, 0, outputMap.data, 0, output.length);

      final float outputScaleX = outputMap.w / (float)NETWORK_INPUT_WIDTH;
      final float outputScaleY = outputMap.h / (float)NETWORK_INPUT_HEIGHT;
      List<Component> components = findComponents(binarize(outputMap));
      List<Rectangle> result = new ArrayList<>();
      for (Component component : components)
      {
         float inputScaleX = NETWORK_INPUT_WIDTH / (float) image.getWidth();
         float inputScaleY = NETWORK_INPUT_HEIGHT / (float) image.getHeight();
         result.add(componentBound(component, inputScaleX, inputScaleY, outputScaleX, outputScaleY));
      }

      System.out.println("Total detection time: " + timer.lap());
      return Pair.of(result, outputMap);
   }

   private float[] transferNetworkOutputToJava(FloatBlobVector output)
   {
      FloatBlob outputLayer = output.get(0);
      FloatPointer data = outputLayer.cpu_data();
      float[] networkOutput = new float[outputLayer.count()];
      data.get(networkOutput);
      return networkOutput;
   }

   static class HeatMap
   {
      final float[] data;
      final int offsetX, offsetY;
      final int w, h;

      private HeatMap(int offsetX, int offsetY, int w, int h)
      {
         this.offsetX = offsetX;
         this.offsetY = offsetY;
         this.w = w;
         this.h = h;
         data = new float[w * h];
      }

   }

   @SuppressWarnings("unused") private static BufferedImage imageFromArray(float[] data, int w, int h)
   {
      BufferedImage result = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
      float min = data[0], max = data[0];
      for (float pixel : data)
      {
         min = Math.min(min, pixel);
         max = Math.max(max, pixel);
      }
      int i = 0;
      float norm = (max - min) == 0 ? 1 : 1 / (max - min);
      for (int y = 0; y < h; y++)
      {
         for (int x = 0; x < w; x++)
         {

            float val = (data[i] - min) * norm;
            result.setRGB(x, y, new Color(val, val, val).getRGB());
            i++;
         }
      }
      return result;
   }

   private static PreprocessedImage processImage(BufferedImage image, int targetW, int targetH)
   {
      if (image.getWidth() != targetW || image.getHeight() != targetH)
      {
         image = scaleImage(image, new Dimension(targetW, targetH));
      }

      float[] output = new float[image.getWidth() * image.getHeight() * 3];
      int n = image.getWidth() * image.getHeight();
      for (int y = 0; y < image.getHeight(); y++)
      {
         for (int x = 0; x < image.getWidth(); x++)
         {
            int rgb = image.getRGB(x, y);
            int a = (rgb >> 24) & 0xFF;
            int r = (rgb >> 16) & 0xFF;
            int g = (rgb >> 8) & 0xFF;
            int b = rgb & 0xFF;
            if (a == 0)
               r = g = b = 127;
            int index = y * image.getWidth() + x;
            output[index] = b;
            output[n + index] = g;
            output[2 * n + index] = r;
         }
      }

      normalizeData(output);

      return new PreprocessedImage(output);
   }

   private static void normalizeData(float[] data)
   {
      double avg = 0, avg2 = 0;
      double invN = 1.0 / data.length;
      for (float val : data)
      {
         avg += val * invN;
         avg2 += val * val * invN;
      }

      double var = avg2 - avg * avg;
      double stddev = Math.sqrt(var);
      float avgf = (float)avg;
      float invStdDev = (float)(1 / stddev);

      float expectedMean = 127;
      float expectedStdDev = 255 / 3.0f;

      for (int i = 0; i < data.length; i++)
      {
         float val = data[i];
         val = ((val - avgf) * invStdDev) * expectedStdDev + expectedMean;
         val = Math.max(0, Math.min(255, val));
         data[i] = val;
      }
   }

   private static BufferedImage scaleImage(BufferedImage img, Dimension targetDimensions)
   {
      // create the scaled version of the image with the specified border
      BufferedImage newImg = new BufferedImage(targetDimensions.width, targetDimensions.height, BufferedImage.TYPE_INT_ARGB);
      Graphics2D gr = (Graphics2D) newImg.getGraphics();
      gr.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);
      gr.drawImage(img, 0, 0, newImg.getWidth(), newImg.getHeight(), null);
      gr.dispose();
      return newImg;
   }

   private static Rectangle componentBound(Component component, float inputScaleX, float inputScaleY, float outputScaleX, float outputScaleY)
   {
      int tx1 = convertOutputXCoordToInputXCoord(component.minX, outputScaleX, inputScaleX);
      int tx2 = convertOutputXCoordToInputXCoord(component.maxX + 1, outputScaleX, inputScaleX);
      int ty1 = convertOutputYCoordToInputYCoord(component.minY, outputScaleY, inputScaleY);
      int ty2 = convertOutputYCoordToInputYCoord(component.maxY + 1, outputScaleY, inputScaleY);
      return new Rectangle(tx1, ty1, tx2 - tx1, ty2 - ty1);
   }

   private static int convertOutputXCoordToInputXCoord(float xcoord, float outputScaleX, float inputScaleX)
   {
      return (int) ((xcoord / outputScaleX) / inputScaleX);
   }

   private static int convertOutputYCoordToInputYCoord(float ycoord, float outputScaleY, float inputScaleY)
   {
      return (int) ((ycoord / outputScaleY) / inputScaleY);
   }

   private static boolean[][] binarize(HeatMap networkOutput)
   {
      int w = networkOutput.w;
      int h = networkOutput.h;
      boolean[][] result = new boolean[h][w];
      for (int y = 0; y < h; y++)
      {
         for (int x = 0; x < w; x++)
         {
            float val = networkOutput.data[y * w + x];
            result[y][x] = val > 0.8f;
         }
      }

      for (int y = 0; y < h; y++)
      {
         for (int x = 0; x < w; x++)
         {
            if (result[y][x])
               continue;
            for (int i = -1; i <= 1; i++)
            {
               for (int j = -1; j <= 1; j++)
               {
                  if (x + i >= 0 && x + i < w && y + j >= 0 && y + j < h && result[y][x])
                     result[y][x] = networkOutput.data[y * w + x] >= 0.1;
               }
            }
         }
      }

      return result;
   }

   private static List<Component> findComponents(boolean[][] thresholdedPixels)
   {
      ArrayList<Component> result = new ArrayList<>();
      int w = thresholdedPixels[0].length;
      int h = thresholdedPixels.length;
      boolean[][] visited = new boolean[h][w];
      LinkedList<int[]> activePixels = new LinkedList<>();
      for (int y = 0; y < h; y++)
      {
         for (int x = 0; x < w; x++)
         {
            if (thresholdedPixels[y][x])
               activePixels.add(new int[] {x, y});
         }
      }

      while (!activePixels.isEmpty())
      {
         Queue<int[]> queue = new LinkedList<>();
         queue.add(activePixels.poll());
         List<int[]> currentComponent = new ArrayList<>();
         while (!queue.isEmpty())
         {
            int[] coord = queue.poll();
            int x = coord[0], y = coord[1];
            if (x < 0 || y < 0 || y >= h || x >= w || !thresholdedPixels[y][x] || visited[y][x])
               continue;
            currentComponent.add(coord);
            visited[y][x] = true;
            for (int i = -1; i <= 1; i++)
            {
               for (int j = -1; j <= 1; j++)
               {
                  queue.add(new int[] {x + i, y + j});
               }
            }
         }

         if (currentComponent.size() < 12)
            continue;

         result.add(new Component(currentComponent));
      }

      return result;
   }

   private static class PreprocessedImage
   {
      private final float[] data;

      private PreprocessedImage(float[] data)
      {
         this.data = data;
      }
   }

   private static class Component
   {
      int minX, minY, maxX, maxY;
      final List<int[]> pixels;

      Component(List<int[]> pixels)
      {
         minY = minX = Integer.MAX_VALUE;
         maxY = maxX = Integer.MIN_VALUE;
         for (int[] pixel : pixels)
         {
            int x = pixel[0], y = pixel[1];
            minX = Math.min(x, minX);
            maxX = Math.max(x, maxX);
            minY = Math.min(y, minY);
            maxY = Math.max(y, maxY);
         }

         this.pixels = pixels;
      }
   }
}
