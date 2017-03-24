package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * Describes the job of a {@link YoGraphic}. When the user creates one, the {@link YoGraphic} is assumed 
 * to be writing in the {@link YoVariable}s.
 * When the graphic is created from {@link YoGraphicFactory}, it is assumed that the graphic is used as a
 * {@link RemoteYoGraphic} and thus does only read the {@link YoVariable}s.
 * 
 * <p>
 * Differentiating the {@link YoGraphic} according to their job allows to implement simple 
 * synchronization between a {@code WRITER} and a {@code READER} to ensure proper rendering.
 * 
 * @author Sylvain
 *
 */
enum YoGraphicJob
{
   /** The YoGraphic is the one processing user data and updating the YoVariables. */
   WRITER,
   /** The YoGraphic reads the YoVariable to create the meshes only. */
   READER
}