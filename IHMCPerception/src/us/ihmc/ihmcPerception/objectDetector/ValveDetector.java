package us.ihmc.ihmcPerception.objectDetector;

import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.caffe;
import org.bytedeco.javacpp.caffe.Caffe;
import org.bytedeco.javacpp.caffe.FloatBlob;
import org.bytedeco.javacpp.caffe.FloatBlobVector;
import org.bytedeco.javacpp.caffe.FloatNet;
import us.ihmc.tools.time.Timer;

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
import java.util.logging.Logger;

import static org.bytedeco.javacpp.caffe.TEST;

/**
 *
 */
public class ValveDetector
{
   private static final Logger logger = Logger.getLogger(caffe.class.getSimpleName());
   private static int NETWORK_OUTPUT_WIDTH;
   private static int NETWORK_OUTPUT_HEIGHT;
   private static int NETWORK_INPUT_WIDTH;
   private static int NETWORK_INPUT_HEIGHT;

   private static FloatNet caffe_net;
   private static final Object caffeNetLock = new Object();

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
               logger.warning("Could not extract " + cudaLib + " to " + cudaLibsDir.getAbsolutePath()
                                    + ". If CUDA 7.5 is not installed, object detection will likely fail with UnsatisfiedLinkError.");
            else
               logger.fine("Extracted " + cudaLib + " to " + cudaLibsDir);
         }

         System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + cudaLibsDir.getAbsolutePath());

      }
      catch (Exception ex)
      {
         logger.log(Level.SEVERE, "Could not load weights: " + ex.getMessage());
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

      // Set device id and mode
      if (gpu >= 0)
      {
         logger.info("Use GPU with device ID " + gpu);
         Caffe.SetDevice(gpu);
         Caffe.set_mode(Caffe.GPU);
      }
      else
      {
         logger.info("Use CPU.");
         Caffe.set_mode(Caffe.CPU);
      }
      // Instantiate the caffe net.
      caffe_net = new FloatNet(model, TEST);
      caffe_net.CopyTrainedLayersFrom(weights);
      NETWORK_OUTPUT_WIDTH = caffe_net.output_blobs().get(0).shape(3);
      NETWORK_OUTPUT_HEIGHT = caffe_net.output_blobs().get(0).shape(2);
      NETWORK_INPUT_WIDTH = caffe_net.input_blobs().get(0).shape(3);
      NETWORK_INPUT_HEIGHT = caffe_net.input_blobs().get(0).shape(2);
   }

   public ValveDetector() throws Exception
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
               .getResourceAsStream(resourceName);//note that each / is a directory down in the "jar tree" been the jar the root of the tree
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
      Timer timer = new Timer();
      timer.start();
      Rectangle[] crops = createZoomedInCrops(image.getWidth(), image.getHeight());
      HeatMap[] heatMaps = detectInCrops(image, crops);
      HeatMap outputMap = combineCropHeatMaps(heatMaps);

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

   private HeatMap combineCropHeatMaps(HeatMap[] heatMaps)
   {
      int outW = 0, outH = 0;
      for (HeatMap heatMap : heatMaps)
      {
         outW = Math.max(outW, heatMap.offsetX + heatMap.w);
         outH = Math.max(outH, heatMap.offsetY + heatMap.h);
      }

      HeatMap result = new HeatMap(0, 0, outW, outH);
      for (HeatMap heatMap : heatMaps)
      {
         result.merge(heatMap, heatMap.offsetX, heatMap.offsetY);
      }

      return result;
   }

   private Rectangle[] createZoomedInCrops(int width, int height)
   {
      // We split the image in 5 crops - topleft, topright, bottomleft, bottomright and center (center overlaps with the others)
      // We then merge the outputs. This creates a super-resolution output, as if the image was zoomed in 2x, allowing
      // for a better detection of distant objects paying only small performance penalty as the crops are batched and processed
      // at once.

      int halfWidth = width / 2;
      int halfHeight = height / 2;
      Rectangle[] crops = new Rectangle[5];
      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            crops[i * 2 + j] = new Rectangle(i * halfWidth, j * halfHeight, halfWidth, halfHeight);
         }
      }
      crops[4] = new Rectangle(halfWidth - halfWidth / 2, halfHeight - halfHeight / 2, halfWidth, halfHeight);
      return crops;
   }

   private HeatMap[] detectInCrops(BufferedImage image, Rectangle[] crops)
   {
      int inputPixels = NETWORK_INPUT_WIDTH * NETWORK_INPUT_HEIGHT;
      int channels = 3;
      Timer timer = new Timer();
      timer.start();
      float[] processedData = new float[inputPixels * channels * crops.length];
      PreprocessedImage[] processedImages = new PreprocessedImage[crops.length];
      for (int i = 0; i < crops.length; i++)
      {
         Rectangle crop = crops[i];
         BufferedImage cropped = image.getSubimage(crop.x, crop.y, crop.width, crop.height);
         processedImages[i] = processImage(cropped, NETWORK_INPUT_WIDTH, NETWORK_INPUT_HEIGHT);
         System.arraycopy(processedImages[i].data, 0, processedData, i * inputPixels * channels, processedImages[i].data.length);
      }

      FloatPointer pointer = new FloatPointer(processedData.length);
      pointer.put(processedData);
      System.out.println("Image preparation took: " + timer.lap());

      float[] output;
      synchronized (caffeNetLock)
      {
         caffe_net.input_blobs().get(0).set_cpu_data(pointer);
         output = transferNetworkOutputToJava(caffe_net.Forward());
      }

      System.out.println("DetectNet took " + timer.lap());

      HeatMap[] result = new HeatMap[crops.length];
      for (int i = 0; i < result.length; i++)
      {
         int offsetX = (int)(crops[i].x * processedImages[i].scaleX * NETWORK_OUTPUT_WIDTH / NETWORK_INPUT_WIDTH);
         int offsetY = (int)(crops[i].y * processedImages[i].scaleY * NETWORK_OUTPUT_HEIGHT / NETWORK_INPUT_HEIGHT);
         result[i] = new HeatMap(offsetX, offsetY, NETWORK_OUTPUT_WIDTH, NETWORK_OUTPUT_HEIGHT);
         System.arraycopy(output, i * NETWORK_OUTPUT_WIDTH * NETWORK_OUTPUT_HEIGHT, result[i].data, 0, result[i].data.length);
      }

      System.out.println("HeatMap generation took" + timer.lap());

      return result;
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

      void merge(HeatMap other, int ox, int oy)
      {
         for (int y = 0; y < other.h; y++)
         {
            for (int x = 0; x < other.w; x++)
            {
               if (oy + y < 0 || oy + y >= h || ox + x < 0 || ox + x >= w)
                  continue;
               int dataIndex = w * (oy + y) + (ox + x);
               data[dataIndex] = Math.max(data[dataIndex], other.data[y * other.w + x]);
            }
         }
      }
   }

   private static BufferedImage imageFromArray(float[] data, int w, int h)
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
      float sx = targetW / (float) image.getWidth();
      float sy = targetH / (float) image.getHeight();

      image = normalizeBrightnessAndContrast(image);
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
            output[index] = b / 1.0f;
            output[n + index] = g / 1.0f;
            output[2 * n + index] = r / 1.0f;
         }
      }

      return new PreprocessedImage(output, sx, sy);
   }

   private static BufferedImage normalizeBrightnessAndContrast(BufferedImage source)
   {
      double avg = 0, avg2 = 0;
      int n = source.getWidth() * source.getHeight();
      for (int y = 0; y < source.getHeight(); y++)
      {
         for (int x = 0; x < source.getWidth(); x++)
         {
            int rgb = source.getRGB(x, y);
            int r = (rgb >> 16) & 0xFF;
            int g = (rgb >> 8) & 0xFF;
            int b = rgb & 0xFF;
            double gray = (r + g + b) / 3.0;
            avg += gray / n;
            avg2 += gray * gray / n;
         }
      }

      double var = avg2 - avg * avg;
      double stddev = Math.sqrt(var);

      double expectedMean = 127;
      double expectedStdDev = 255 / 3.0;

      BufferedImage result = new BufferedImage(source.getWidth(), source.getHeight(), BufferedImage.TYPE_INT_ARGB);
      for (int y = 0; y < source.getHeight(); y++)
      {
         for (int x = 0; x < source.getWidth(); x++)
         {
            int rgb = source.getRGB(x, y);
            double r = (rgb >> 16) & 0xFF;
            double g = (rgb >> 8) & 0xFF;
            double b = rgb & 0xFF;
            r = ((r - avg) / stddev) * expectedStdDev + expectedMean;
            g = ((g - avg) / stddev) * expectedStdDev + expectedMean;
            b = ((b - avg) / stddev) * expectedStdDev + expectedMean;
            r = Math.min(255, Math.max(0, r));
            g = Math.min(255, Math.max(0, g));
            b = Math.min(255, Math.max(0, b));
            int ir = (int) r << 16;
            int ig = (int) g << 8;
            int ib = (int) b;
            result.setRGB(x, y, 0xff000000 | ir | ig | ib);
         }
      }
      return result;
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
      private final float scaleX, scaleY;

      private PreprocessedImage(float[] data, float scaleX, float scaleY)
      {
         this.data = data;
         this.scaleX = scaleX;
         this.scaleY = scaleY;
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
