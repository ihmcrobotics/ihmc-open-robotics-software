package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreMissingTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public class EuclidFrameMissingTools
{
   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static void extractNormalPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FixedFrameTuple3DBasics inputNormalPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis, inputNormalPartToPack);
      EuclidCoreMissingTools.extractNormalPart(input, normalAxis, inputNormalPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    * @throws ReferenceFrameMismatchException if {@code input} and {@code normalAxis} are not expressed
    *                                         in the same reference frame.
    */
   public static void extractNormalPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FrameTuple3DBasics inputNormalPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis);
      inputNormalPartToPack.setReferenceFrame(input.getReferenceFrame());
      EuclidCoreMissingTools.extractNormalPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputNormalPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                     the tuple to extract the tangential part of. Not modified.
    * @param normalAxis                the normal vector. It is normalized internally if needed. Not
    *                                  modified.
    * @param inputTangentialPartToPack the tuple used to store the tangential part of the input.
    *                                  Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static void extractTangentialPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FixedFrameTuple3DBasics inputTangentialPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis, inputTangentialPartToPack);
      EuclidCoreMissingTools.extractTangentialPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputTangentialPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                     the tuple to extract the tangential part of. Not modified.
    * @param normalAxis                the normal vector. It is normalized internally if needed. Not
    *                                  modified.
    * @param inputTangentialPartToPack the tuple used to store the tangential part of the input.
    *                                  Modified.
    * @throws ReferenceFrameMismatchException if {@code input} and {@code normalAxis} are not expressed
    *                                         in the same reference frame.
    */
   public static void extractTangentialPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FrameTuple3DBasics inputTangentialPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis);
      inputTangentialPartToPack.setReferenceFrame(input.getReferenceFrame());
      EuclidCoreMissingTools.extractTangentialPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputTangentialPartToPack);
   }
}
