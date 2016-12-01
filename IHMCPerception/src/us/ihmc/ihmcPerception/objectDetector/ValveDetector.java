package us.ihmc.ihmcPerception.objectDetector;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.caffe;
import org.bytedeco.javacpp.caffe.Caffe;
import org.bytedeco.javacpp.caffe.FloatBlob;
import org.bytedeco.javacpp.caffe.FloatBlobVector;
import org.bytedeco.javacpp.caffe.FloatNet;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
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
public class ValveDetector {
    private static final Logger logger = Logger.getLogger(caffe.class.getSimpleName());

    private final FloatNet caffe_net;

    public ValveDetector() throws Exception
    {
        int gpu = -1;

        String model, weights;
        try {
            model = exportResource("/valvenet/deploy.prototxt");
            weights = exportResource("/valvenet/snapshot_iter_1761.caffemodel");
        } catch (Exception ex) {
            logger.log(Level.SEVERE, "Could not load weights: " + ex.getMessage());
            throw ex;
        }

        if (model.length() == 0) {
            throw new Exception("Need a model definition to score.");
        }
        if (weights.length() == 0) {
            throw new Exception("Need model weights to score.");
        }

        // Set device id and mode
        if (gpu >= 0) {
            logger.info("Use GPU with device ID " + gpu);
            Caffe.SetDevice(gpu);
            Caffe.set_mode(Caffe.GPU);
        } else {
            logger.info("Use CPU.");
            Caffe.set_mode(Caffe.CPU);
        }
        // Instantiate the caffe net.
        caffe_net = new FloatNet(model, TEST);
        caffe_net.CopyTrainedLayersFrom(weights);
    }

    /**
     * Export a resource embedded into a Jar file to the local file path.
     *
     * @param resourceName ie.: "/SmartLibrary.dll"
     * @return The path to the exported resource
     * @throws Exception if extraction fails
     */
    private static String exportResource(String resourceName) throws Exception {
        InputStream stream = null;
        OutputStream resStreamOut = null;
        try {
            stream = ValveDetector.class.getResourceAsStream(resourceName);//note that each / is a directory down in the "jar tree" been the jar the root of the tree
            if(stream == null) {
                throw new Exception("Cannot get resource \"" + resourceName + "\" from Jar file.");
            }

            int readBytes;
            byte[] buffer = new byte[4096];
            String tempDir = System.getProperty("java.io.tmpdir");
            String resourceFileName = new File(resourceName).getName();
            File outFile = new File(tempDir + "/" + resourceFileName);

            resStreamOut = new FileOutputStream(outFile.getAbsoluteFile());
            while ((readBytes = stream.read(buffer)) > 0) {
                resStreamOut.write(buffer, 0, readBytes);
            }
            return outFile.getAbsolutePath();
        } finally {
            if (stream != null)
                stream.close();
            if (resStreamOut != null)
                resStreamOut.close();
        }
    }

    public List<Rectangle> detect(BufferedImage image)
    {
        IntPointer shape = caffe_net.blob_by_name("data").shape();
        PreprocessedImage processed = processImage(image, shape.get(3), shape.get(2));
        caffe_net.blob_by_name("data").set_cpu_data(processed.data);

        FloatBlobVector output = caffe_net.Forward();
        FloatBlob outputLayer = output.get(0);
        FloatPointer data = outputLayer.cpu_data();
        float[] networkOutput = new float[outputLayer.count()];
        data.get(networkOutput);

        List<Component> components = findComponents(binarize(networkOutput, 64, 32));
        List<Rectangle> result = new ArrayList<>();
        for (Component component : components) {
            result.add(componentBound(component, processed, 64.0f / 1024, 32.0f / 512));
        }
        return result;
    }

    private static BufferedImage imageFromArray(float[] data, int w, int h)
    {
        BufferedImage result = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
        float min = data[0], max = data[0];
        for (float pixel : data) {
            min = Math.min(min, pixel);
            max = Math.max(max, pixel);
        }
        int i = 0;
        float norm = (max - min) == 0 ? 1 : 1 / (max - min);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {

                float val = (data[i] - min) * norm;
                result.setRGB(x, y, new Color(val, val, val).getRGB());
                i++;
            }
        }
        return result;
    }

    private static PreprocessedImage processImage(BufferedImage image, int targetW, int targetH)
    {
        BufferedImage padded = image, scaled = image;
        if (image.getWidth() != targetW || image.getHeight() != targetH)
        {
            Dimension scaledDim = getScaledDimension(new Dimension(image.getWidth(), image.getHeight()), new Dimension(targetW, targetH));
            scaled = scaleImageAspect(image, scaledDim);
            if (scaled.getWidth() != targetW || scaled.getHeight() != targetH)
            {
                padded = padImage(scaled, targetW, targetH, Color.BLACK);
            } else {
                padded = scaled;
            }
        }

        int tx = targetW / 2 - scaled.getWidth() / 2;
        int ty = targetH / 2 - scaled.getHeight() / 2;
        float sx = scaled.getWidth() / (float) image.getWidth();
        float sy = scaled.getHeight() / (float) image.getHeight();

        float[] output = new float[padded.getWidth() * padded.getHeight() * 3];
        int n = padded.getWidth() * padded.getHeight();
        for (int y = 0; y < padded.getHeight(); y++) {
            for (int x = 0; x < padded.getWidth(); x++) {
                int rgb = padded.getRGB(x, y);
                int a = (rgb >> 24) & 0xFF;
                int r = (rgb >> 16) & 0xFF;
                int g = (rgb >> 8) & 0xFF;
                int b = rgb & 0xFF;
                if (a == 0)
                    r = g = b = 127;
                int index = y * padded.getWidth() + x;
                output[index] = b / 1.0f;
                output[n + index] = g / 1.0f;
                output[2 * n + index] = r / 1.0f;
            }
        }

        FloatPointer pointer = new FloatPointer(output.length);
        pointer.put(output);
        return new PreprocessedImage(pointer, tx, ty, sx, sy);
    }

    private static Dimension getScaledDimension(Dimension imgSize, Dimension boundary) {

        int img_width = imgSize.width;
        int img_height = imgSize.height;
        if (img_width > img_height) {
            // need to scale based on width
            float aspect_ratio = (float) img_height / (float) img_width;
            img_width = boundary.width;
            img_height = (int) ((float) img_width * aspect_ratio);
        } else {
            // need to scale based on width
            float aspect_ratio = (float) img_width / (float) img_height;
            img_height = boundary.height;
            img_width = (int) ((float) img_height * aspect_ratio);
        }

        // first check if we need to scale width
        final int original_width = img_width;
        final int original_height = img_height;
        if (img_width > boundary.width) {
            //scale width to fit
            img_width = boundary.width;
            //scale height to maintain aspect ratio
            img_height = (img_width * original_height) / original_width;
        }

        // then check if we need to scale even with the new height
        if (img_height > boundary.height) {
            //scale height to fit instead
            img_height = boundary.height;
            //scale width to maintain aspect ratio
            img_width = (img_height * original_width) / original_height;
        }

        return new Dimension(img_width, img_height);
    }

    private static BufferedImage scaleImageAspect(BufferedImage img, Dimension targetDimensions) {
        // create the scaled version of the image with the specified border
        BufferedImage newImg = new BufferedImage(targetDimensions.width, targetDimensions.height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D gr = (Graphics2D) newImg.getGraphics();
        gr.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        gr.drawImage(img, 0, 0, newImg.getWidth(), newImg.getHeight(), null);
        gr.dispose();
        return newImg;
    }

    private static BufferedImage padImage(BufferedImage img, int desiredW, int desiredH, Color color) {
        // create the scaled version of the image with the specified border
        BufferedImage new_img = new BufferedImage(desiredW, desiredH, BufferedImage.TYPE_INT_ARGB);
        Graphics2D gr = (Graphics2D) new_img.getGraphics();
        gr.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);
        assert(desiredW / 2 - img.getWidth() / 2 >= 0);
        assert(desiredH / 2 - img.getHeight() / 2 >= 0);
        gr.drawImage(img, desiredW / 2 - img.getWidth() / 2, desiredH / 2 - img.getHeight() / 2, img.getWidth(), img.getHeight(), null);
        gr.dispose();
        return new_img;
    }

    private static Rectangle componentBound(Component component, PreprocessedImage preprocessedImage, float outputScaleX, float outputScaleY)
    {
        int tx1 = (int)((component.minX / outputScaleX - preprocessedImage.shiftX) / preprocessedImage.scaleX);
        int tx2 = (int)(((component.maxX + 1) / outputScaleX - preprocessedImage.shiftX) / preprocessedImage.scaleX);
        int ty1 = (int)((component.minY / outputScaleY - preprocessedImage.shiftY) / preprocessedImage.scaleY);
        int ty2 = (int)(((component.maxY + 1) / outputScaleY - preprocessedImage.shiftY) / preprocessedImage.scaleY);
        return new Rectangle(tx1, ty1, tx2 - tx1, ty2 - ty1);
    }

    private static boolean[][] binarize(float[] networkOutput, int w, int h)
    {
        float min = networkOutput[0], max = networkOutput[0];
        for (float pixel : networkOutput) {
            min = Math.min(min, pixel);
            max = Math.max(max, pixel);
        }
        float norm = (max - min) == 0 ? 1 : 1 / (max - min);

        boolean[][] result = new boolean[h][w];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                float val = networkOutput[y * w + x];
                result[y][x] = val > 0.1f && (val / norm) > 0.4f;
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
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                if (thresholdedPixels[y][x])
                    activePixels.add(new int[]{x, y});
            }
        }

        while (!activePixels.isEmpty()) {
            Queue<int[]> queue = new LinkedList<>();
            queue.add(activePixels.poll());
            List<int[]> currentComponent = new ArrayList<>();
            while (!queue.isEmpty()) {
                int[] coord = queue.poll();
                int x = coord[0], y = coord[1];
                if (!thresholdedPixels[y][x] || visited[y][x] || x < 0 || y < 0 || y >= h || x >= w)
                    continue;
                currentComponent.add(coord);
                visited[y][x] = true;
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        queue.add(new int[]{x + i, y + j});
                    }
                }
            }

            if (currentComponent.size() < 9)
                continue;

            result.add(new Component(currentComponent));
        }

        return result;
    }

    private static class PreprocessedImage {
        private final FloatPointer data;
        private final float shiftX, shiftY, scaleX, scaleY;

        private PreprocessedImage(FloatPointer data, float shiftX, float shiftY, float scaleX, float scaleY) {
            this.data = data;
            this.shiftX = shiftX;
            this.shiftY = shiftY;
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
            for (int[] pixel : pixels) {
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
