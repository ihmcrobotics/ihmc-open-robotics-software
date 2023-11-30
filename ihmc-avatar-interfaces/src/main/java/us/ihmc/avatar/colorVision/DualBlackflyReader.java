package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.util.concurrent.atomic.AtomicReference;

public class DualBlackflyReader
{
    private static final String BLACKFLY_SERIAL_LEFT = "22206802";
    private static final String BLACKFLY_SERIAL_RIGHT = "22206798";

    private SpinnakerBlackflyManager spinnakerBlackflyManager;
    private final SideDependentList<SpinnakerBlackflyReaderThread> spinnakerBlackflyReaderThreads = new SideDependentList<>();

    private volatile boolean running;

    public void start() throws Exception
    {
        if (running)
        {
            throw new RuntimeException("Already started");
        }

        running = true;

        try
        {
            spinnakerBlackflyManager = new SpinnakerBlackflyManager();

            SpinnakerBlackflyReaderThread leftSpinnakerBlackflyReaderThread = new SpinnakerBlackflyReaderThread(RobotSide.LEFT,
                                                                                                                spinnakerBlackflyManager.createSpinnakerBlackfly(
                                                                                                                      BLACKFLY_SERIAL_LEFT));
            SpinnakerBlackflyReaderThread rightSpinnakerBlackflyReaderThread = new SpinnakerBlackflyReaderThread(RobotSide.RIGHT,
                                                                                                                 spinnakerBlackflyManager.createSpinnakerBlackfly(
                                                                                                                       BLACKFLY_SERIAL_RIGHT));

            spinnakerBlackflyReaderThreads.put(RobotSide.LEFT, leftSpinnakerBlackflyReaderThread);
            spinnakerBlackflyReaderThreads.put(RobotSide.RIGHT, rightSpinnakerBlackflyReaderThread);

            for (SpinnakerBlackflyReaderThread spinnakerBlackflyReaderThread : spinnakerBlackflyReaderThreads)
            {
                spinnakerBlackflyReaderThread.start();
            }
        }
        catch (Exception exception)
        {
            running = false;

            throw exception;
        }
    }

    public void stop()
    {
        if (running)
        {
            for (SpinnakerBlackflyReaderThread spinnakerBlackflyReaderThread : spinnakerBlackflyReaderThreads)
            {
                try
                {
                    spinnakerBlackflyReaderThread.join();
                }
                catch (InterruptedException e)
                {
                    LogTools.error(e);
                }
            }
        }

        running = false;

        spinnakerBlackflyReaderThreads.clear();
        spinnakerBlackflyManager.destroy();
    }

    public SideDependentList<SpinnakerBlackflyReaderThread> getSpinnakerBlackflyReaderThreads()
    {
        return spinnakerBlackflyReaderThreads;
    }

    public boolean isRunning()
    {
        return running;
    }

    public class SpinnakerBlackflyReaderThread extends Thread
    {
        private final SpinnakerBlackfly spinnakerBlackfly;
        private final AtomicReference<BytePointer> latestImageDataPointer = new AtomicReference<>();
        private int width;
        private int height;
        private Object notify;
        private FrequencyStatisticPrinter flFrequencyStatisticPrinter = new FrequencyStatisticPrinter();

        public SpinnakerBlackflyReaderThread(RobotSide robotSide, SpinnakerBlackfly spinnakerBlackfly)
        {
            super("SpinnakerBlackflyReaderThread" + robotSide.getPascalCaseName());
            this.spinnakerBlackfly = spinnakerBlackfly;
        }

        @Override
        public void run()
        {
            latestImageDataPointer.set(new BytePointer());

            while (running)
            {
                spinImage spinImage = new spinImage();

                spinnakerBlackfly.getNextImage(spinImage);

                int width = spinnakerBlackfly.getWidth(spinImage);
                int height = spinnakerBlackfly.getHeight(spinImage);
                this.width = width;
                this.height = height;
                int imageFrameSize = width * height;

                BytePointer spinImageData = new BytePointer(imageFrameSize);
                spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

                flFrequencyStatisticPrinter.ping();

                latestImageDataPointer.set(spinImageData);

                if (notify != null)
                {
                    synchronized (notify)
                    {
                        notify.notifyAll();
                    }
                }

                spinnakerBlackfly.releaseImage(spinImage);
            }

            spinnakerBlackfly.stopAcquiringImages();
        }

        public int getWidth()
        {
            return width;
        }

        public int getHeight()
        {
            return height;
        }

        public AtomicReference<BytePointer> getLatestImageDataPointer()
        {
            return latestImageDataPointer;
        }

        public void setNotify(Object notify)
        {
            if (this.notify != null)
            {
                throw new RuntimeException("Notify already set");
            }
            this.notify = notify;
        }
    }

    public static void main(String[] args)
    {
        DualBlackflyReader dualBlackflyReader = new DualBlackflyReader();

        try
        {
            dualBlackflyReader.start();
        }
        catch (Exception e)
        {
            return;
        }

        Runtime.getRuntime().addShutdownHook(new Thread(dualBlackflyReader::stop));

        ThreadTools.sleepForever();
    }
}
